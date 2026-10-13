# TR-MOKE / TR-MOFE control GUI

A Tkinter desktop app for time-resolved magneto-optical pump–probe measurements, in reflection (TR-MOKE, Kerr effect)
or in transmission (TR-MOFE, Faraday effect):
sweep an optical delay line, read a lock-in amplifier, and map a sample on an XZ stage.
It was written for one particular setup, so treat it as a starting point and adapt the drivers to your hardware.

## What it does
- **Time scan**: sweep the delay line in µm or ps, average, plot X / Y / R / θ live, save CSV + NPZ.
- **2D mapping**: raster an XZ stage at a fixed delay and show the intensity map as it fills in.
- **Several delay lines**: keep up to three delay-line profiles (serial, calibration, travel, last scan range,
  lock-in TC / Sens) and switch between them from the top bar; only the delay line is swapped while connected.
- **Second (pump) delay line** on its own stage controller, scanned together with the main one, alone, or held fixed.
- **Safety**: travel limits are checked before anything moves; Start / End turn red as soon as a range leaves the
  travel; USB enumeration never runs while a device is being opened.
- **Mock mode** for every device, so the whole UI can be tried and tested without hardware.

## Hardware it talks to
| Device | Interface | Library |
|---|---|---|
| Thorlabs brushless DC controller (BBD30x) + DDS delay stage | USB (FTDI D2XX) | `pylablib` (Kinesis) |
| SIGMA KOKI SHOT-602 stage controller (XZ stage, pump delay) | RS-232 | `pyserial` |
| NF Corporation LI5645 lock-in amplifier | GPIB | `pyvisa` |

## Install
Python 3.10+ (tested with 3.12 on Windows and 3.9 / 3.14 in mock mode on macOS).

```
pip install numpy matplotlib pyserial pyvisa pylablib
```

## Configure
Copy `moke_gui_config.example.json` to `moke_gui_config.json` next to `moke_gui.py` and set your ports,
GPIB address and delay-line serial. Everything else (calibration, scan ranges, delay-line profiles) is edited in
the app and saved to `moke_gui_settings.json` (or to the path in the `MOKE_SETTINGS` environment variable).

## Run
```
python moke_gui.py          # Windows: double-click MOKE_GUI.bat (console log) or MOKE_GUI.vbs (no console)
```
Pick **Mock** in the Mode box to try it without hardware. Only one program can own a Kinesis controller at a
time, so close any other control software first.

## Tests
```
python run_tests.py
```
Runs every `test_*.py` in mock mode with a temporary settings file and a time limit per test.

## Before measuring
Check direction and travel of each stage with the hardware connected but the beam blocked, and set the travel
limits in Settings. The authors take no responsibility for damage to equipment.

## Extending it
The code is meant as a starting point for other pump–probe or scanning setups written in plain Python, without
LabVIEW licenses.
- Devices covered now: Thorlabs Kinesis brushless delay stages, SIGMA KOKI SHOT-602, NF LI5645 (GPIB).
- Other instruments can be added through PyVISA, pyserial, pylablib or vendor SDKs; each device has a mock class,
  so new code can be tested without hardware.
- Workflows (delay scans, 2D maps, dual-delay scans), travel checks, calibration profiles and logging are kept
  separate from the device classes, so they can be reused.

Issues and pull requests are welcome.

## 日本語
時間分解の磁気光学ポンプ・プローブ測定（TR-MOKE / TR-MOFE）用の、Python で書いた装置制御 GUI です。
遅延ステージ、ステージコントローラ、ロックインアンプをまとめて制御し、LabVIEW のライセンスは不要です。
装置がなくても Mock モードで動作を確認できます。

## License
MIT
