@echo off
rem Installs the FM-1 wavetable firmware over USB. Needs Python 3 (python.org, tick "Add python.exe to PATH").
echo Setting up (first time only takes a minute)...
py -m pip install --quiet --disable-pip-version-check mido python-rtmidi || python -m pip install --quiet --disable-pip-version-check mido python-rtmidi
echo.
echo Connect the FM-1 by USB and switch it on.
py "%~dp0fm1_install.py" "%~dp0fm1-wavetable.fwsc" || python "%~dp0fm1_install.py" "%~dp0fm1-wavetable.fwsc"
echo.
pause
