@echo off
rem Installs the FM-1 wavetable firmware over USB, with Windows' own MIDI: nothing else to install but Python.
set PY=
py -3 -c "pass" >/dev/null 2>&1 && set PY=py -3
if not defined PY ( python -c "pass" >/dev/null 2>&1 && set PY=python )
if not defined PY (
  echo Python was not found. Install it from python.org ^(tick "Add python.exe to PATH"^), then run this again.
  echo.
  pause
  exit /b 1
)
echo Connect the FM-1 by USB and switch it on.
echo Close any other music software that might be using its MIDI port.
echo.
%PY% "%~dp0fm1_install.py" "%~dp0fm1-wavetable.fwsc"
echo.
pause
