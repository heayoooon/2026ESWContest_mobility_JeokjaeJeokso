@echo off
setlocal
rem Truck twin on the forest loop, local window.
rem Python: env ISAAC_PY, else %USERPROFILE%\anaconda3\envs\env_isaacsim42\python.exe
set OMNI_KIT_ACCEPT_EULA=YES
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
if "%ISAAC_PY%"=="" set "ISAAC_PY=%USERPROFILE%\anaconda3\envs\env_isaacsim42\python.exe"
cd /d "%~dp0"
"%ISAAC_PY%" scripts\run_forest_twin.py 
if errorlevel 1 pause
