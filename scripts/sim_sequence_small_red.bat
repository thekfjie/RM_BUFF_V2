@echo off
setlocal
cd /d "%~dp0.."

set "SEQUENCE_DIR=%~1"
set "FREQ=%~2"
if "%FREQ%"=="" set "FREQ=30"

if "%SEQUENCE_DIR%"=="" goto :usage

if not defined OpenCV_BIN_DIR set "OpenCV_BIN_DIR=D:\develop\opencv\build\x64\vc16\bin"
if not defined ONNXRUNTIME_LIB_DIR set "ONNXRUNTIME_LIB_DIR=D:\develop\onnxruntime\lib"
if exist "%OpenCV_BIN_DIR%" set "PATH=%OpenCV_BIN_DIR%;%PATH%"
if exist "%ONNXRUNTIME_LIB_DIR%" set "PATH=%ONNXRUNTIME_LIB_DIR%;%PATH%"
set "EXE=%~dp0..\build\vs2022-release\Release\predict_example_main.exe"
set "CONFIG=%~dp0..\config\lab\standalone_yolo_small_red_sim_sequence.yaml"

if not exist "%EXE%" call "%~dp0build_release.bat"
if errorlevel 1 goto :fail
if not exist "%EXE%" goto :fail
if not exist "%CONFIG%" goto :missing_config
if not exist "%SEQUENCE_DIR%" goto :missing_sequence

echo.
echo Running BUFF_V2.1 on image sequence:
echo %SEQUENCE_DIR%
echo Frame rate hint: %FREQ%
echo.

"%EXE%" --config "%CONFIG%" --video "%SEQUENCE_DIR%" --freq %FREQ%
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo Program exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:usage
echo Usage:
echo   sim_sequence_small_red.bat ^<sequence_dir^> [freq]
echo.
echo Example:
echo   sim_sequence_small_red.bat ^<repo-root^>\..\at_vision_simulator_noros\captures\session_0001 30
pause
exit /b 1

:missing_config
echo.
echo Missing config file:
echo %CONFIG%
pause
exit /b 1

:missing_sequence
echo.
echo Missing sequence directory:
echo %SEQUENCE_DIR%
pause
exit /b 1

:fail
echo.
echo Build failed or executable is missing.
pause
exit /b 1
