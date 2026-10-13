# Changelog

The version is shown in the window title.

## 2.0
- Each delay line keeps its own Start / End / Step and lock-in TC / Sens; switching brings them back (press Set to apply TC / Sens).
- The travel of the selected delay line is shown under the scan fields; Start / End turn red as soon as the range leaves it.
- "Start = here" / "End = here" fill the fields from the current delay-line position.
- Version in the title bar; `MOKE_SETTINGS` redirects the settings file (used by the tests).

## 1.9
- Short plain-language connection errors; details behind the status light.
## 1.8
- Switching the delay line while connected swaps only the delay line.
## 1.7
- Fix: USB enumeration no longer runs while devices are being opened ("Device Not Opened" / "Access is denied").
## 1.6
- Two-row top bar, single Connect / Disconnect button, status lights, failure popup and log.
## 1.5
- Pump delay line on its own SHOT-602 (own COM port).
## 1.4
- Live USB state, 2D mapping only on the mapping stage, pump travel limits.
## 1.3
- Pump delay manual control; per-delay-line scan range.
## 1.2
- Dual-delay scan (main + pump).
## 1.1
- Up to three delay-line profiles with a selector in the top bar, each with its own calibration.
## 1.0
- Time scan and 2D mapping with one delay line, SHOT-602 XZ stage and LI5645 lock-in.
