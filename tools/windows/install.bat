@echo off
rem Installs the FM-1 wavetable firmware over USB, with Windows' own MIDI: nothing else to install but Python.
rem Each version is asked for by name first: a stale py/python default (an uninstalled Python) is skipped.
set PY=
for %%V in (3.14 3.13 3.12 3.11 3.10 3.9 3.8) do (
  if not defined PY ( py -%%V -c "pass" >nul 2>&1 && set PY=py -%%V )
)
if not defined PY ( py -3 -c "pass" >nul 2>&1 && set PY=py -3 )
if not defined PY ( python -c "pass" >nul 2>&1 && set PY=python )
if not defined PY (
  echo Python was not found. Install it from python.org ^(tick "Add python.exe to PATH"^), then run this again.
  echo.
  pause
  exit /b 1
)
echo Using %PY%
echo Connect the FM-1 by USB and switch it on.
echo Close any other music software that might be using its MIDI port.
echo.
%PY% "%~dp0fm1_install.py" "%~dp0arcturus-s6.fwsc"
echo.
pause
