@echo off
setlocal
cd /d "%~dp0.."
set "PATH=D:\develop\opencv\build\x64\vc16\bin;D:\develop\onnxruntime\lib;%PATH%"
set "EXE=%~dp0..\build\vs2022-release\Release\predict_example_main.exe"
set "CONFIG=%~dp0..\config\standalone_yolo_big_blue.yaml"

if not exist "%EXE%" call "%~dp0build_release.bat"
if errorlevel 1 goto :fail
if not exist "%EXE%" goto :fail
if not exist "%CONFIG%" goto :missing_config

"%EXE%" --config "%CONFIG%"
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo Program exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:missing_config
echo.
echo Missing config file: %CONFIG%
pause
exit /b 1

:fail
echo.
echo Build failed or executable is missing.
pause
exit /b 1
