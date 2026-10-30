@echo off
rem Installs the FM-1 wavetable firmware over USB.
rem Needs Python 3.10 to 3.13 (python.org): the MIDI library has no ready-made build for newer versions yet.
set PY=
for %%V in (3.13 3.12 3.11 3.10) do (
  if not defined PY ( py -%%V -c "pass" >nul 2>&1 && set PY=py -%%V )
)
if not defined PY (
  echo Could not find Python 3.13, 3.12, 3.11 or 3.10.
  echo Install Python 3.13 from python.org ^(tick "Add python.exe to PATH"^), then run this again.
  echo.
  pause
  exit /b 1
)
echo Using %PY%
echo Setting up (first time only takes a minute)...
%PY% -m pip install --quiet --disable-pip-version-check --only-binary=:all: mido python-rtmidi
if errorlevel 1 (
  echo Setup failed: see the message above.
  pause
  exit /b 1
)
echo.
echo Connect the FM-1 by USB and switch it on.
%PY% "%~dp0fm1_install.py" "%~dp0fm1-wavetable.fwsc"
echo.
pause
