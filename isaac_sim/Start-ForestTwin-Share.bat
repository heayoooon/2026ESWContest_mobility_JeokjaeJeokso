@echo off
setlocal
rem Truck twin with browser streaming (WebRTC, one viewer) and the remote API on port 8211.
rem Python: env ISAAC_PY, else %USERPROFILE%\anaconda3\envs\env_isaacsim42\python.exe
set OMNI_KIT_ACCEPT_EULA=YES
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
if "%ISAAC_PY%"=="" set "ISAAC_PY=%USERPROFILE%\anaconda3\envs\env_isaacsim42\python.exe"
cd /d "%~dp0"
"%ISAAC_PY%" scripts\run_forest_twin.py --share
if errorlevel 1 pause
