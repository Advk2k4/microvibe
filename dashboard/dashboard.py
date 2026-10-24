#!/usr/bin/env python3
"""MicroVibe dashboard: live serial view, baseline learning, z-score alerts, recording and replay.

Live:    python3 dashboard.py [--port /dev/cu.usbmodemXXXX]
Replay:  python3 dashboard.py --replay sessions/session_X.csv [--speed 4]
Batch:   python3 dashboard.py --replay sessions/session_X.csv --headless   (prints states/alerts)

Every device line (R/S/E/#) and every dashboard action (H lines) goes into the session CSV.
Replay feeds that file through the same Engine, so charts, states and alerts come out identical.
"""
import argparse, collections, glob, math, os, queue, sys, threading, time

import numpy as np

FS = 1000                 # piezo sample rate (firmware FS_HZ)
BLADES = 3
TACH_MIN_SWING = 40       # ADC counts; below this the photoresistor sees no usable blade chopping
BASELINE_S = 30           # seconds of healthy running to learn
SUSTAIN_S = 3             # a fault must persist this long before FAULT
CLEAR_S = 5               # seconds back under the warning level before FAULT clears
MIN_FREQ = 5              # Hz; ignore DC / slow drift when picking the dominant frequency

# Alert rules: z = (x - baseline mean) / baseline sd, per feature.
# Set from measurements in bench/RESULTS.md (see README "Alert tuning"); not guessed.
ALERTS = {
    # feature: (z_warn, z_fault, direction)  direction +1 = only rises alert, -1 = only drops, 0 = both
    "rms":      (3.0, 5.0, +1),
    "p2p":      (3.0, 5.0, +1),
    "amp_1x":   (3.0, 5.0, +1),
    "slip":     (3.0, 5.0, +1),
}
FAULT_KIND = {"rms": "vibration", "p2p": "vibration", "amp_1x": "imbalance (1x rotation)",
              "slip": "load / brake drag"}

# Thermistor: 10k NTC from 3V3 to GPIO 2, 10k from GPIO 2 to GND. B=3950 is the usual kit value,
# NOT measured on this part, so temperature is approximate; the trend (dT/dt) is what we use.
THERM_B, THERM_R0, THERM_T0, THERM_RFIX = 3950.0, 10000.0, 298.15, 10000.0


def therm_c(adc):
    if adc <= 0 or adc >= 4095:
        return float("nan")
    r = THERM_RFIX * (4095.0 / adc - 1.0)          # NTC on the high side
    return 1.0 / (1.0 / THERM_T0 + math.log(r / THERM_R0) / THERM_B) - 273.15


class Engine:
    """Turns protocol lines into 1 s feature windows, baseline, z-scores and a state machine."""

    def __init__(self):
        self.samples = collections.OrderedDict()   # seq -> np.array block
        self.windows = []                          # list of feature dicts, one per S line
        self.events = collections.deque(maxlen=8)
        self.baseline = None                       # feature -> (mean, sd)
        self.learning = None                       # list of windows being collected
        self.state = "IDLE"
        self.cause = ""
        self.over = 0
        self.under = 0
        self.last_spectrum = (np.array([]), np.array([]))
        self.seq = 0
        self.transitions = []                      # (t_s, state, cause)

    # ---- input ----
    def feed(self, line):
        line = line.strip()
        if not line:
            return
        f = line.split(",")
        kind = f[0]
        try:
            if kind == "R":
                seq = int(f[1])
                self.samples[seq] = np.array(f[2:], dtype=float)
                self.seq = seq + len(f) - 2
                while len(self.samples) > 200:
                    self.samples.popitem(last=False)
            elif kind == "S":
                self._second(f)
            elif kind == "E":
                self._event(f[2])
            elif kind == "H":
                self._event(f[2])
        except (ValueError, IndexError):
            pass                                   # partial line at start of a serial read

    def _event(self, text):
        t = self.seq / FS
        self.events.append(f"{t:7.1f}s {text}")
        if text in ("BTN_LEARN", "LEARN"):
            self.learning = []
            self.baseline = None
            self._set("LEARNING", "")
        elif text in ("BTN_RESET", "RESET"):
            self.over = self.under = 0
            self._set("HEALTHY" if self.baseline else "IDLE", "")

    def _set(self, state, cause):
        if state != self.state:
            self.state, self.cause = state, cause
            self.transitions.append((self.seq / FS, state, cause))

    # ---- per-second features ----
    def _second(self, f):
        seq_end, duty, killed, brake = int(f[1]), int(f[3]), int(f[4]), int(f[5])
        pulses, tmin, tmax, tilt, therm = (int(x) for x in f[6:11])
        x = self._window(seq_end - FS, seq_end)
        w = {"t": seq_end / FS, "duty": duty, "killed": killed, "brake": brake, "tilt": tilt,
             "tach_swing": tmax - tmin, "temp": therm_c(therm), "dropped": int(f[13]) if len(f) > 13 else 0}
        w["rpm"] = pulses * 60.0 / BLADES if tmax - tmin >= TACH_MIN_SWING else float("nan")
        if x is not None:
            x = x - x.mean()
            w["rms"] = float(np.sqrt(np.mean(x ** 2)))
            w["p2p"] = float(x.max() - x.min())
            spec = np.abs(np.fft.rfft(x * np.hanning(len(x)))) * 2 / len(x)
            freqs = np.fft.rfftfreq(len(x), 1 / FS)
            band = freqs >= MIN_FREQ
            i = int(np.argmax(spec[band]))
            w["dom_hz"] = float(freqs[band][i])
            w["dom_amp"] = float(spec[band][i])
            rot = w["rpm"] / 60.0
            w["amp_1x"] = float(spec[np.abs(freqs - rot) <= 2].max()) if rot == rot and rot >= MIN_FREQ else float("nan")
            self.last_spectrum = (freqs, spec)
        else:
            for k in ("rms", "p2p", "dom_hz", "dom_amp", "amp_1x"):
                w[k] = float("nan")
        # slip: measured RPM per unit of commanded duty, relative to the baseline ratio
        b = self.baseline
        if b and "rpm_per_duty" in b and duty > 0 and w["rpm"] == w["rpm"]:
            w["slip"] = 1.0 - (w["rpm"] / duty) / b["rpm_per_duty"][0]
        else:
            w["slip"] = float("nan")
        w["dTdt"] = self._dtdt(w)
        self.windows.append(w)
        self._update_state(w)

    def _window(self, a, b):
        parts = [(s, v) for s, v in self.samples.items() if s + len(v) > a and s < b]
        if not parts:
            return None
        x = np.concatenate([v[max(0, a - s):max(0, b - s)] for s, v in parts])
        return x if len(x) >= 0.9 * FS else None   # need >= 90% of the second (dropped blocks)

    def _dtdt(self, w):
        pts = [(v["t"], v["temp"]) for v in self.windows[-29:]] + [(w["t"], w["temp"])]
        pts = [(t, c) for t, c in pts if c == c]
        if len(pts) < 10:
            return float("nan")
        t, c = np.array(pts).T
        return float(np.polyfit(t, c, 1)[0] * 60.0)    # degC per minute

    # ---- baseline + state machine ----
    def _running(self, w):
        return w["duty"] > 0 and not w["killed"]

    def _update_state(self, w):
        if self.learning is not None:
            if self._running(w) and not w["brake"] and w["rms"] == w["rms"]:
                self.learning.append(w)
            if len(self.learning) >= BASELINE_S:
                self._finish_baseline()
            return
        if not self.baseline:
            return
        if not self._running(w):
            self.over = self.under = 0
            self._set("STOPPED", "kill switch" if w["killed"] else "duty 0")
            return
        z = self.zscores(w)
        w["z"] = z
        warn = [k for k, (v, _) in z.items() if v >= ALERTS[k][0]]
        fault = [k for k, (v, _) in z.items() if v >= ALERTS[k][1]]
        w["zmax"] = max((v for v, _ in z.values()), default=0.0)
        if fault:
            self.over += 1
            self.under = 0
        elif warn:
            self.over = 0
            self.under = 0
        else:
            self.over = 0
            self.under += 1
        if self.state == "STOPPED":
            self._set("HEALTHY", "")
        if self.over >= SUSTAIN_S:
            k = max(fault, key=lambda k: z[k][0] / ALERTS[k][1])
            self._set("FAULT", f"{FAULT_KIND[k]} ({k} z={z[k][0]:.1f})")
        elif self.state == "FAULT":
            if self.under >= CLEAR_S:
                self._set("HEALTHY", "")
        elif warn:
            k = max(warn, key=lambda k: z[k][0] / ALERTS[k][0])
            self._set("WARNING", f"{FAULT_KIND[k]} ({k} z={z[k][0]:.1f})")
        elif self.under >= SUSTAIN_S:
            self._set("HEALTHY", "")

    def zscores(self, w):
        out = {}
        for k, (_, _, direction) in ALERTS.items():
            if k not in self.baseline or w.get(k) != w.get(k):
                continue
            m, sd = self.baseline[k]
            z = (w[k] - m) / sd
            z = z if direction > 0 else (-z if direction < 0 else abs(z))
            out[k] = (z, w[k])
        return out

    def _finish_baseline(self):
        ws = self.learning
        self.learning = None
        b = {}
        for k in ("rms", "p2p", "amp_1x", "dom_hz", "tilt", "rpm"):
            v = np.array([w[k] for w in ws if w[k] == w[k]], dtype=float)
            if len(v) >= 10:
                # sd floor: 2% of the mean, so a perfectly steady feature can't divide by ~0
                b[k] = (float(v.mean()), float(max(v.std(ddof=1), 0.02 * abs(v.mean()), 1e-6)))
        rpd = np.array([w["rpm"] / w["duty"] for w in ws if w["rpm"] == w["rpm"]], dtype=float)
        if len(rpd) >= 10:
            b["rpm_per_duty"] = (float(rpd.mean()), float(rpd.std(ddof=1)))
            slips = 1.0 - rpd / rpd.mean()
            b["slip"] = (0.0, float(max(slips.std(ddof=1), 0.01)))
        self.baseline = b
        self.over = self.under = 0
        self._set("HEALTHY", "")
        self.events.append(f"{self.seq / FS:7.1f}s BASELINE " +
                           " ".join(f"{k}={m:.1f}±{s:.1f}" for k, (m, s) in b.items() if k != "rpm_per_duty"))


# ---------------- sources ----------------
def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") +
                   glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("COM*"))
    if not ports:
        sys.exit("No serial port found. Plug in the ESP32 or pass --port, or use --replay.")
    return ports[0]


class LiveSource:
    def __init__(self, port, outdir):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        os.makedirs(outdir, exist_ok=True)
        self.path = os.path.join(outdir, time.strftime("session_%Y%m%d_%H%M%S.csv"))
        self.log = open(self.path, "w")
        self.lock = threading.Lock()
        self.q = queue.Queue()
        self.engine_seq = lambda: 0
        threading.Thread(target=self._read, daemon=True).start()
        print(f"Recording to {self.path}")

    def _read(self):
        buf = b""
        while True:
            buf += self.ser.read(4096)
            *lines, buf = buf.split(b"\n")
            with self.lock:
                for raw in lines:
                    s = raw.decode("ascii", "replace").strip()
                    if s:
                        self.log.write(s + "\n")
                        self.q.put(s)
                self.log.flush()

    def send(self, cmd, note=None):
        self.ser.write((cmd + "\n").encode())
        h = f"H,{self.engine_seq()},{note or 'CMD_' + cmd}"
        with self.lock:
            self.log.write(h + "\n")
            self.q.put(h)


class ReplaySource:
    def __init__(self, path, speed):
        self.q = queue.Queue()
        self.path = path
        self.done = False
        threading.Thread(target=self._play, args=(path, speed), daemon=True).start()

    def _play(self, path, speed):
        with open(path) as fh:
            for line in fh:
                self.q.put(line)
                if line.startswith("S,") and speed > 0:
                    time.sleep(1.0 / speed)
        self.done = True

    def send(self, cmd, note=None):
        pass                                        # no hardware in replay


# ---------------- UI ----------------
STATE_COLORS = {"HEALTHY": "#3B6D11", "WARNING": "#BA7517", "FAULT": "#A32D2D",
                "LEARNING": "#185FA5", "STOPPED": "#5F5E5A", "IDLE": "#5F5E5A"}


def run_ui(engine, src, replay):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button

    fig = plt.figure(figsize=(14, 8.5))
    fig.canvas.manager.set_window_title("MicroVibe" + (" — REPLAY " + os.path.basename(src.path) if replay else ""))
    gs = fig.add_gridspec(4, 3, height_ratios=[1.1, 2, 2, 2], hspace=0.55, wspace=0.28,
                          left=0.06, right=0.98, top=0.97, bottom=0.11)
    ax_state = fig.add_subplot(gs[0, :2]); ax_state.axis("off")
    ax_info = fig.add_subplot(gs[0:2, 2]); ax_info.axis("off")
    ax_vib = fig.add_subplot(gs[1, :2])
    ax_spec = fig.add_subplot(gs[2, 2])
    ax_rpm = fig.add_subplot(gs[2, :2])
    ax_z = fig.add_subplot(gs[3, :2])
    ax_temp = fig.add_subplot(gs[3, 2])

    state_txt = ax_state.text(0.0, 0.55, "IDLE", fontsize=40, weight="bold", va="center")
    cause_txt = ax_state.text(0.0, 0.0, "", fontsize=13, va="center")
    info_txt = ax_info.text(0.0, 1.02, "", fontsize=8, va="top", family="monospace")

    def cmd(c, note=None):
        return lambda _e: src.send(c, note)

    def learn(_e):
        src.send("X0", "LEARN")

    def reset(_e):
        src.send("B0")
        src.send("G", "RESET")

    buttons = []
    specs = [("KILL", "#A32D2D", cmd("K", "KILL")), ("Start / Go", "#3B6D11", cmd("G")),
             ("Brake ON", "#BA7517", cmd("B1")), ("Brake OFF", "#888780", cmd("B0")),
             ("Learn baseline", "#185FA5", learn), ("Reset", "#5F5E5A", reset)]
    for i, (label, color, fn) in enumerate(specs):
        bax = fig.add_axes([0.06 + i * 0.155, 0.015, 0.14, 0.055])
        b = Button(bax, label, color=color, hovercolor=color)
        b.label.set_color("white"); b.label.set_fontsize(12 if label != "KILL" else 15)
        if not replay:
            b.on_clicked(fn)
        buttons.append(b)
    if replay:
        fig.text(0.5, 0.075, "REPLAY — no hardware connected; buttons disabled, states recomputed from the file",
                 ha="center", fontsize=10, color="#5F5E5A")

    last_state = [None]

    def update(_frame):
        n = 0
        while n < 400:
            try:
                line = src.q.get_nowait()
            except queue.Empty:
                break
            engine.feed(line)
            n += 1
        if engine.state != last_state[0]:
            last_state[0] = engine.state
            code = {"HEALTHY": "X0", "WARNING": "X1", "FAULT": "X2"}.get(engine.state)
            if code and not replay:
                src.ser.write((code + "\n").encode())
        state_txt.set_text(engine.state)
        state_txt.set_color(STATE_COLORS.get(engine.state, "black"))
        cause_txt.set_text(engine.cause or ("learning baseline: %d/%d s" % (len(engine.learning), BASELINE_S)
                                            if engine.learning is not None else ""))
        W = engine.windows[-120:]
        if not W:
            return []
        t = np.array([w["t"] for w in W])
        get = lambda k: np.array([w.get(k, float("nan")) for w in W], dtype=float)

        ax_vib.clear(); ax_vib.set_title("Piezo vibration per 1 s window (raw ADC counts, relative)", fontsize=10)
        ax_vib.plot(t, get("rms"), label="RMS", color="#534AB7")
        ax_vib.plot(t, get("amp_1x"), label="1x rotation amplitude", color="#D85A30")
        if engine.baseline and "rms" in engine.baseline:
            m, s = engine.baseline["rms"]
            ax_vib.axhspan(m - s, m + s, color="#534AB7", alpha=0.12, label="baseline RMS ±1sd")
        ax_vib.legend(loc="upper left", fontsize=8); ax_vib.set_xlabel("s")

        ax_rpm.clear(); ax_rpm.set_title("Tach RPM (photoresistor) and commanded duty", fontsize=10)
        ax_rpm.plot(t, get("rpm"), color="#0F6E56", label="RPM")
        ax_rpm.set_ylabel("RPM")
        d = ax_rpm.twinx() if not hasattr(update, "rpm_tw") else update.rpm_tw
        update.rpm_tw = d
        d.clear(); d.step(t, get("duty"), color="#888780", where="post", label="duty"); d.set_ylim(0, 270)
        d.set_ylabel("duty /255")
        brk = get("brake")
        for i in np.where(brk > 0)[0]:
            ax_rpm.axvspan(t[i] - 0.5, t[i] + 0.5, color="#BA7517", alpha=0.15, lw=0)

        ax_z.clear(); ax_z.set_title("Anomaly score: max z over alert features", fontsize=10)
        ax_z.plot(t, get("zmax"), color="#A32D2D")
        zw = min(v[0] for v in ALERTS.values()); zf = min(v[1] for v in ALERTS.values())
        ax_z.axhline(zw, ls="--", color="#BA7517", lw=1); ax_z.axhline(zf, ls="--", color="#A32D2D", lw=1)
        ax_z.set_xlabel("s")

        freqs, spec = engine.last_spectrum
        ax_spec.clear(); ax_spec.set_title("Latest 1 s piezo spectrum (FFT in Python)", fontsize=10)
        if len(freqs):
            ax_spec.plot(freqs, spec, color="#534AB7", lw=0.8); ax_spec.set_xlim(0, 500)
            r = W[-1]["rpm"] / 60.0
            if r == r:
                for mult, c in ((1, "#D85A30"), (BLADES, "#0F6E56")):
                    ax_spec.axvline(r * mult, color=c, ls=":", lw=1)
            ax_spec.set_xlabel("Hz")

        ax_temp.clear(); ax_temp.set_title("Thermistor (approx. °C, B=3950 assumed)", fontsize=10)
        ax_temp.plot(t, get("temp"), color="#993C1D"); ax_temp.set_xlabel("s")

        w = W[-1]
        z = w.get("z", {})
        lines = [f"t           {w['t']:8.1f} s", f"duty        {w['duty']:8d}",
                 f"killed      {w['killed']:8d}", f"brake       {w['brake']:8d}",
                 f"RMS         {w['rms']:8.2f}", f"p2p         {w['p2p']:8.1f}",
                 f"dominant    {w['dom_hz']:8.1f} Hz", f"1x amp      {w['amp_1x']:8.2f}",
                 f"RPM         {w['rpm']:8.0f}" + ("" if w['rpm'] == w['rpm'] else f"  (tach swing {w['tach_swing']})"),
                 f"slip        {w['slip']*100 if w['slip'] == w['slip'] else float('nan'):8.1f} %",
                 f"tilt edges  {w['tilt']:8d} /s", f"temp        {w['temp']:8.1f} C", f"dT/dt       {w['dTdt']:8.2f} C/min",
                 f"dropped blk {w['dropped']:8d}"]
        lines += ["z " + "  ".join(f"{k}={v:.1f}" for k, (v, _) in z.items())]
        lines += ["events:"] + [e[:44] for e in list(engine.events)[-5:]]
        info_txt.set_text("\n".join(lines))
        return []

    anim = FuncAnimation(fig, update, interval=250, cache_frame_data=False)
    fig._anim = anim
    plt.show()


def headless(engine, path):
    with open(path) as fh:
        for line in fh:
            engine.feed(line)
    print(f"{os.path.basename(path)}: {len(engine.windows)} s of data")
    if engine.baseline:
        print("baseline: " + ", ".join(f"{k}={m:.2f}±{s:.2f}" for k, (m, s) in engine.baseline.items()))
    for t, state, cause in engine.transitions:
        print(f"  {t:8.1f}s  {state:<9} {cause}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--replay", help="session CSV to replay (no hardware needed)")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier (0 = as fast as possible)")
    ap.add_argument("--headless", action="store_true", help="with --replay: print states/alerts, no window")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sessions"))
    a = ap.parse_args()
    engine = Engine()
    if a.replay:
        if a.headless:
            return headless(engine, a.replay)
        run_ui(engine, ReplaySource(a.replay, a.speed), replay=True)
    else:
        src = LiveSource(a.port or find_port(), a.out)
        src.engine_seq = lambda: engine.seq
        run_ui(engine, src, replay=False)


if __name__ == "__main__":
    main()
