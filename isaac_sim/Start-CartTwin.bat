@echo off
setlocal
rem Hand-cart version of the twin.
rem Python: env ISAAC_PY, else %USERPROFILE%\anaconda3\envs\env_isaacsim42\python.exe
set OMNI_KIT_ACCEPT_EULA=YES
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
if "%ISAAC_PY%"=="" set "ISAAC_PY=%USERPROFILE%\anaconda3\envs\env_isaacsim42\python.exe"
cd /d "%~dp0"
"%ISAAC_PY%" scripts\run_forest_twin.py --vehicle cart
if errorlevel 1 pause
