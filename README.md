# MicroVibe

**A predictive-maintenance fault lab built from a $30 starter kit.** A small DC motor and fan on an
ESP32-S3 learns a healthy baseline of vibration, RPM and load, then flags injected faults on a live
dashboard with a kill switch. Industrial vibration-monitoring rigs cost thousands. This one uses kit
parts, and its vibration sensor is a **speaker run backwards**. Every result in this README was
measured on this rig, and the measurements that did *not* work are reported too.

Hackachusetts 2026 (Mass AI Coalition), Robotics & hardware track. Solo build.

> **Judges: no hardware needed.** `python3 dashboard/dashboard.py --replay sessions/<demo>.csv`
> replays a recorded session through the same detection code that ran live.
> See [Replay without hardware](#replay-without-hardware).

## How it works

```
 ESP32-S3 (Arduino)                         Laptop (Python)
 ─────────────────                          ───────────────
 sampler task, 1 kHz ── raw vibration ──►   1 s windows: RMS, peak-to-peak,
 tach / tilt counts  ── 1 s stats  ──────►  FFT dominant frequency, RPM, slip
 L293D PWM motor     ◄── kill / duty ─────  30 s baseline → z-scores →
 servo brake         ◄── brake on/off ────  HEALTHY / WARNING / FAULT (3 s sustained)
                                            records every line → sessions/*.csv
```

- The ESP32 does **no** feature computation and no FFT. It samples and streams raw ADC values.
  Everything else happens in Python on those streamed samples, and the same samples are what
  the replay file contains.
- Faults:
  - **Load / brake drag**: a servo arm presses on the fan hub when the dashboard tells it to.
    Detected through RPM slip (measured RPM per unit of commanded PWM, relative to the baseline).
  - **Rotor imbalance**: a lump of tape on one blade, applied by hand. See [Results](#results)
    for what the sensor could and could not see.
- **Kill switch**: a dashboard button sets PWM to 0 and latches until Go or Reset.

## Hardware (all from a LAFVIN ESP32-S3 starter kit)

| Part | Role |
|---|---|
| ESP32-S3-WROOM (8 MB PSRAM), Arduino core 3.3.12 | Sampling, motor/servo control, serial stream |
| L293D + DC motor + 3-blade fan | The machine under test |
| Speaker (run backwards) | Vibration pickup. The motor is taped down on top of it |
| LED + photoresistor | Reflective tachometer: each passing blade bounces light back, 3 pulses per turn |
| Kit hobby servo | Brake fault injector, pressing on the side of the fan hub |
| Tilt switch | Crude second vibration channel (counts chatter edges) |

**What it does not have:** no accelerometer, no microphone, no calibrated sensor of any kind.

### Honest sensor notes
- **The speaker is uncalibrated and relative.** Its coil turns cone motion into a voltage. The coil
  is a DC path to ground and the ESP32 ADC cannot read negative voltages, so the signal is
  effectively half-wave rectified. With no capacitor in the kit, it cannot be biased to mid-rail.
  Units are raw 12-bit ADC counts, not g or mm/s.
- **There is no anti-alias filter.** Content above 500 Hz folds into the 0–500 Hz band at 1 kHz
  sampling.
- **The first vibration sensor was the kit's passive buzzer**, run the same way. It was used for
  Milestone 1 and the first recorded session (below). Then its leg broke off flush, and v1 uses the
  speaker instead.
- **The tilt switch is crude.** Its edge count has a standard deviation larger than its mean, so it is
  shown but never drives alerts.
- **The temperature channel is not wired in v1.** The firmware and dashboard support a 10k NTC on
  GPIO 2, but that is cut for time.
- **This is a toy fan on a breadboard, not an industrial motor.** The detection method is the real
  one (baseline, z-score, sustained-fault state machine). The physical scale is not.

## Wiring / pin map

The L293D straddles the breadboard centre channel, with its notch (pin 1) top-left. All grounds go to
one rail.

| ESP32-S3 GPIO | Connects to | Status |
|---|---|---|
| 5 | L293D pin 1 (EN1), PWM 20 kHz, 8-bit | wired |
| 6 / 7 | L293D pin 2 (IN1) / pin 7 (IN2): HIGH/LOW = forward | wired |
| 5V pin | L293D pin 8 (+Vmotor) and pin 16 (+V logic); servo red | wired |
| GND | L293D pins 4, 5, 12, 13; everything else | wired |
| — | L293D pins 3 / 6 → motor leads; pins 9, 10, 15 → GND; 11, 14 empty | wired |
| 1 (ADC) | Speaker lead A, plus 10k to GND; speaker lead B → GND | wired |
| 4 | Tilt switch to GND (INPUT_PULLUP) | wired |
| 10 (ADC) | Photoresistor from 3V3, plus 10k to GND. LED: 3V3 → 220 Ω → LED → GND | wired |
| 13 | Servo signal (50 Hz, 500–2500 µs) | wired |
| 2 (ADC) | Thermistor, 10k divider | in firmware, not wired |
| 8 / 9 | LCD1602 I2C SDA / SCL (boot runs an I2C scan) | in firmware, not wired |
| 14 | Active buzzer (fault alarm) | in firmware, not wired |
| 15 / 16 / 17 | RGB LED R / G / B | in firmware, not wired |
| 11 (ADC2, WiFi off) | Speed pot (enable with `P1`) | in firmware, not wired |
| 12 / 18 / 21 | 74HC595 DS / SHCP / STCP → LED bar graph | in firmware, not wired |
| 38 / 39 / 40 | Buttons Learn / Inject / Reset (INPUT_PULLUP) | in firmware, not wired |
| 41 / 42 / 47 / 48 | SD card CS / SCK / MOSI / MISO | in firmware, not wired |

Pins that were avoided: the strapping pins 0, 3, 45 and 46, and GPIO 33–37 (used by the octal PSRAM).

## Run it

### Firmware
Arduino IDE, or `arduino-cli`. Board: ESP32S3 Dev Module, **USB CDC On Boot: Enabled**, PSRAM: OPI.
```
arduino-cli compile --upload -p /dev/cu.usbmodemXXXX \
  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=opi firmware/microvibe
```
The motor starts at duty 180/255 on boot. If the board stays in download mode after flashing, tap EN.

### Dashboard (live)
```
python3 -m pip install -r dashboard/requirements.txt
python3 dashboard/dashboard.py            # auto-detects the port, or pass --port
```
Close the Arduino Serial Monitor first, because only one program can hold the port. Then:
1. Click **Learn baseline** and leave the rig alone for 30 s.
2. Inject a fault: a tape lump on a blade, or **Brake ON**.
3. Use **KILL** to stop the motor and **Reset** to clear the state.

Every session is recorded to `sessions/session_YYYYMMDD_HHMMSS.csv`.

## Replay without hardware
```
python3 dashboard/dashboard.py --replay sessions/<file>.csv              # charts, real time
python3 dashboard/dashboard.py --replay sessions/<file>.csv --speed 5    # 5x faster
python3 dashboard/dashboard.py --replay sessions/<file>.csv --headless   # print states/alerts only
```
The session file contains the exact lines the ESP32 sent, plus `H` lines for dashboard actions such as
Learn, Kill, Brake and Reset. Replay pushes them through the same `Engine` class, so the baseline,
z-scores and state transitions are recomputed, not stored.

## Serial protocol (115200, CSV lines; same format live, on SD and in replay files)

| Line | Fields | Rate |
|---|---|---|
| `R` | `R,seq,v1,…,v50`: raw vibration ADC samples; `seq` = sample index of `v1` (1 kHz clock) | 20/s |
| `S` | `S,seq_end,t_ms,duty,killed,brake,tach_pulses,tach_min,tach_max,tilt_edges,therm_adc,pot_adc,dropped_blocks` | 1/s |
| `E` | `E,t_ms,text`: device events (KILL, GO, BRAKE_ON, BTN_*, …) | on event |
| `H` | `H,seq,text`: dashboard actions (written by the dashboard only) | on event |
| `#` | Boot info: reset reason, I2C scan, SD status | boot |

Commands to the ESP32 (one per line):
- `D<0-255>`: set duty.
- `K`: kill.
- `G`: go.
- `B1` / `B0`: brake on / off.
- `Vb<deg>` / `Vr<deg>`: set the brake / release angle.
- `P1` / `P0`: pot control on / off.
- `X0` / `X1` / `X2`: tell the device the dashboard state (drives the alarm outputs).

## Detection

Per 1 s window (1000 samples):
- RMS and peak-to-peak (after removing the mean).
- Hann-windowed FFT: dominant frequency at 5 Hz or above, and the amplitude at the 1× rotation
  frequency from the tach.
- RPM = tach pulses × 60 / 3 blades, used only if the photoresistor swing is at least 40 counts.
- Slip = 1 − (RPM / duty) ÷ baseline(RPM / duty).

The baseline is 30 s of healthy running, giving a mean and standard deviation per feature. Alerts are
z-scores against that baseline:
- **WARNING** when a feature passes its warning z.
- **FAULT** only when a feature stays above its fault z for **3 consecutive seconds**.
- The state returns to HEALTHY after 5 s back under the warning level.
- Alerts are ignored for 4 s after a start or duty change. The measured spin-up transient is up to 3 s.

Thresholds are in `ALERTS` in `dashboard/dashboard.py`. How they were chosen from the data is in
[Results](#results).

## Results

All measured on this rig on 2026-10-24. Details are in [`bench/RESULTS.md`](bench/RESULTS.md).

**Milestone 1 (passive buzzer, 10k shunt, fast sampling, duty 180):**

| | piezo p2p | tilt edges/s |
|---|---|---|
| Healthy (21 s) | 356.3 ± 160.6 | 4.29 ± 6.94 |
| Tape lump on one blade (10 s) | 467.5 ± 174.9 | 6.60 ± 6.54 |

The motor running is obvious: RMS was about 8.5× the motor-off level. The imbalance is not obvious:
RMS rose only 3%, and the p2p spread is wider than the change.

**First recorded session, 1 kHz stream** (`sessions/2026-10-24_10k_shunt_lump_test.csv`):

| | RMS (counts) |
|---|---|
| Healthy | 8.94 ± 1.94 |
| 1 lump | 8.54 ± 1.86 |
| 2 lumps | 9.09 ± 1.82 |

No 10 Hz band from 5 to 500 Hz moved by more than 0.5 sd. **The tape-lump imbalance was not
detectable with the buzzer front end.**

**Speaker pickup, quick check:** motor off RMS 5.31, motor on RMS 15.67 (about 3×).

<!-- FINAL SESSION RESULTS: filled in from the final recording -->

## Limitations
- One cheap, uncalibrated, half-wave-rectified vibration pickup with no anti-alias filter. Amplitudes
  are relative.
- Small-imbalance detection did not work with the buzzer front end (data above).
- The motor runs from the board's USB 5 V through an L293D, which drops about 2 V, so the speed range
  is limited. Motor plus servo current on USB can brown out the board.
- The photoresistor responds in about 10–50 ms, so tach readings at high RPM are unreliable. RPM is
  only reported when the light swing is clear.
- The baseline is per session and per duty setting. There is no long-term drift model.

## Future work
- A real accelerometer (for example an ADXL345 over SPI) at several kHz, with an anti-alias filter.
- An AC-coupled, mid-rail-biased pickup, so the full waveform is captured.
- An optical slot sensor or Hall-effect tach instead of a photoresistor.
- Run the feature and anomaly model on-device once it is validated offline.

## Repo layout
```
bench/        benchtestA/B/C (original bench sketches), m1_baseline, diag_l293d, RESULTS.md
firmware/     microvibe/microvibe.ino: the v1 firmware
dashboard/    dashboard.py (live + replay), requirements.txt
sessions/     recorded sessions (replayable)
```
