@echo off
rem Starts the MavLTE GCS agent on the command line, with the [gcs] section of mavrelay.ini next to
rem this file. (MavLTE.pyw is the same agent with a window.)
rem Then connect Mission Planner with UDP (port 14550) or TCP (127.0.0.1, port 5760).
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 mavrelay.py gcs --config mavrelay.ini
) else (
    python mavrelay.py gcs --config mavrelay.ini
)
pause
