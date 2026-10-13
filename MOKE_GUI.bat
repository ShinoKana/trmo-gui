@echo off
REM ============================================================
REM  TR-MOKE control GUI launcher
REM  Double-click to open the control panel.
REM  All output + any error is written to moke_gui.log next to this file,
REM  so it can be inspected later (also over SSH).
REM ============================================================
cd /d "%~dp0"
set "LOG=%~dp0moke_gui.log"

REM Make the science packages visible to ANY account. They live in _pypackages
REM next to this launcher, so every Windows account on the PC can import them
REM without a per-user install.
set "PYTHONPATH=%~dp0_pypackages;%PYTHONPATH%"

REM Prefer the system Python 3.12.
set "PY=C:\Program Files\Python312\python.exe"
if not exist "%PY%" set "PY=python"

echo ==================================================== > "%LOG%"
echo Launch: %DATE% %TIME%   account=%USERNAME%   python="%PY%" >> "%LOG%"
echo ==================================================== >> "%LOG%"

REM --- dependency check: fail fast with a clear message ---
"%PY%" -c "import numpy,matplotlib,pyvisa,pylablib,tkinter" >> "%LOG%" 2>&1
if errorlevel 1 (
  echo.
  echo *** Missing Python packages for account "%USERNAME%". ***
  echo *** The GUI needs numpy/matplotlib/pyvisa/pylablib and they are not visible ***
  echo *** to this account's Python. Fixes: ***
  echo ***   - run this as the account where they are installed, OR ***
  echo ***   - install them:  "%PY%" -m pip install numpy matplotlib pyvisa pylablib ***
  echo.
  echo ---- moke_gui.log ----
  type "%LOG%"
  echo.
  pause
  exit /b 1
)

REM --- launch the GUI, capturing all output/errors to the log ---
"%PY%" moke_gui.py >> "%LOG%" 2>&1
if errorlevel 1 (
  echo.
  echo *** The GUI exited with an error. Details below and in "%LOG%". ***
  echo.
  type "%LOG%"
  echo.
  pause
)
