@echo off
setlocal
cd /d "%~dp0.."

cmake --preset vs2022-release
if errorlevel 1 goto :fail

cmake --build build/vs2022-release --config Release
if errorlevel 1 goto :fail

echo.
echo Build finished.
exit /b 0

:fail
echo.
echo Build failed.
pause
exit /b 1
