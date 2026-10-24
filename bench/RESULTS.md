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
