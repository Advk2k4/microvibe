# Bench results

All numbers measured on this rig with `bench/m1_baseline` on 2026-10-24. Fixed duty 180/255 at
20 kHz PWM. The motor lies on its side directly on the passive buzzer (face up in the breadboard),
taped down. FAULT = a small lump of tape on one of the 3 fan blades.

Every value is in raw 12-bit ADC counts per 1 s line. Each 1 s line averages ten 100 ms windows.
The piezo is uncalibrated, so these are relative numbers, not g or mm/s.

| Condition | n (s) | piezo p2p mean ± sd | piezo RMS mean ± sd | tilt edges/s mean ± sd |
|---|---|---|---|---|
| Motor off (floor) | 33 | 101.5 ± 61.1 | 5.43 ± 6.81 | not recorded |
| HEALTHY | 21 | 356.3 ± 160.6 | 46.14 ± 2.05 | 4.29 ± 6.94 |
| FAULT (tape lump) | 10 | 467.5 ± 174.9 | 47.54 ± 1.59 | 6.60 ± 6.54 |

## The four Milestone 1 numbers
- Piezo p2p: HEALTHY 356.3, FAULT 467.5 counts.
- Tilt: HEALTHY 4.29, FAULT 6.60 edges/s.

## What they mean
- **Running vs stopped is obvious.** Running RMS is about 8.5 times the motor-off floor (46 vs 5.4).
  The buzzer-as-piezo sees the motor.
- **Imbalance is NOT obvious in time-domain amplitude.** RMS rises by only 1.4 counts (+3%), which is
  0.7 healthy standard deviations. p2p rises by 111 counts, but its sd is about 160–175. Welch t is
  about 2.1 for RMS and 1.7 for p2p, with only 10 fault seconds.
- **Tilt is very noisy** (sd larger than the mean). It is a crude second channel and should not
  drive alerts alone.
- Next step: imbalance should show up as energy at the shaft rotation frequency (1x RPM), not as more
  total energy. The full firmware therefore streams raw piezo samples at 1 kHz, so that the dashboard
  can check this with an FFT in Python. Alert thresholds come from that measurement.
- Earlier, when the motor was not actually spinning (bad ground), 20 kHz PWM alone raised p2p_mean
  from about 52 to about 68. Part of the running signal is electrical coupling, not vibration.

# Session test: 1 kHz raw stream, 10k shunt (`sessions/2026-10-24_10k_shunt_lump_test.csv`)

`firmware/microvibe`, duty 180, raw piezo at 1 kHz, per-second windows (settle seconds excluded).

| Condition | n (s) | RMS mean ± sd (counts) |
|---|---|---|
| Motor off | 15 | 3.38 ± 1.45 |
| Healthy | 28 | 8.94 ± 1.94 |
| 1 tape lump | 47 | 8.54 ± 1.86 |
| 2 tape lumps | 32 | 9.09 ± 1.82 |

- Running vs off is still clear (about 2.6x RMS).
- **The imbalance is not detectable with this front end.** No 10 Hz band from 5 to 500 Hz shifts by
  more than z = 0.5 between healthy and 1 or 2 lumps, and there is no rotation peak in any condition.
- Cause not established. One hypothesis: if the buzzer is a piezo disc (roughly 10–30 nF), the 10k
  shunt makes a high-pass filter with a cutoff of about 0.5–1.6 kHz, removing shaft-rate content.
  But many kit "passive buzzers" are electromagnetic (a coil of about 16 Ω), and a 10k shunt would not
  filter those. The buzzer type was not verified (no multimeter). There is also no anti-alias filter
  before the 1 kHz sampling, and the motor sits on a breadboard, not a rigid mount.
- After this session the buzzer's GND leg broke, so the shunt-value experiment was not run on it.
- The restart spin-up transient produced 3 s of high RMS z-scores. The dashboard now ignores alerts for
  4 s after a start or duty change (SETTLE_S).

# Final session: speaker pickup, manual brake, tape lump (`sessions/2026-10-24_final_brake_lump.csv`)

Same firmware, duty 180. The vibration pickup is now the kit speaker (the buzzer's leg broke), with
the motor taped on it. Brake = a pencil eraser pressed by hand on the side of the fan hub. The servo
is not wired, so the dashboard's Brake ON/OFF clicks are only time markers in the file.

"Speed line" = the largest FFT peak between 20 and 80 Hz in each 1 s window.

| Segment (session time) | n (s) | speed line Hz | line amplitude | RMS |
|---|---|---|---|---|
| Baseline learn (1021–1050) | 29 | 56.48 ± 0.51 | 2.31 ± 0.25 | 9.44 ± 1.01 |
| Healthy (1051–1072) | 21 | 56.95 ± 0.38 | 2.27 ± 0.27 | 9.36 ± 1.37 |
| Brake 1, eraser pressing (1086–1115) | 29 | 40.66 ± 7.91 (min 22) | 1.83 ± 0.42 | 9.26 ± 1.01* |
| Released (1118–1131) | 13 | 56.77 ± 0.44 | 2.35 ± 0.24 | 8.63 ± 1.22 |
| Brake 2 (1132–1163) | 31 | 44.90 ± 10.81 (min 20) | 1.63 ± 0.54 | 9.20 ± 1.20 |
| Released (1166–1180) | 14 | 57.21 ± 0.43 | 2.26 ± 0.15 | 9.05 ± 1.10 |
| Tape lump on one blade (1193–1234) | 41 | 38.46 ± 16.82 (20–80) | 1.02 ± 0.50 | 8.78 ± 1.35 |

\* RMS for brake 1 covers the whole marker window (1073–1115). The eraser only made contact at about
1086 s; the line stayed at 56 Hz until then.

- **RMS and p2p do not respond to either fault.** Vibration loudness is not the signal.
- **The ~56.5 Hz speed line does respond.** The brake pulls it down to 20–50 Hz as the motor slows,
  and it returns to 56–57 Hz on release. The tape lump breaks it up: its frequency wanders from 20 to
  80 Hz and its amplitude drops by 56%.
- Without a working tach, it is unknown whether 56.5 Hz is the 1x rotation rate (about 3390 RPM) or
  the 3-blade pass rate (about 1130 RPM). It is called a "speed line" because it moves with load.
- Alert rule: z-score of line frequency (both directions) and line amplitude (drop only) against
  the 30 s baseline. WARNING at |z| ≥ 3, FAULT at |z| ≥ 5 sustained for 3 s. sd floored at 2% of the
  mean (1.13 Hz for the line). Healthy segments never left 56–58 Hz, so they stayed below z = 1.4.
- Replay of this session (`--headless`):
  - brake 1 → WARNING 1086 s, FAULT 1088 s
  - brake 2 → FAULT 1137 s
  - tape lump → FAULT 1195 s (motor restarted at 1189 s, so the 4 s settle ended at 1193 s)
  - **No WARNING or FAULT in any healthy segment.**
- **Caveat:** the thresholds were chosen from and checked on this same session. That is one session on
  one rig, not a held-out validation. The earlier buzzer session, run through the same rules, gives
  only brief WARNINGs and no FAULT, consistent with the buzzer not seeing the lump.
