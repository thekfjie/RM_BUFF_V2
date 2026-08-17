@echo off
setlocal
cd /d "%~dp0.."

set "PATH=D:\develop\opencv\build\x64\vc16\bin;D:\develop\onnxruntime\lib;%PATH%"
set "ROOT=E:\RM\rm_vision"
set "MODEL=%~dp0..\models\best.onnx"
set "EXE=%~dp0..\build\vs2022-release\Release\yolo_video_test.exe"

if "%~1"=="" goto :usage

if not exist "%EXE%" call "%~dp0..\scripts\build_release.bat"
if errorlevel 1 goto :fail
if not exist "%EXE%" goto :fail
if not exist "%MODEL%" goto :missing_model

"%EXE%" --python-root "%ROOT%" --model "%MODEL%" %*
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo Program exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:usage
echo Usage:
echo   run_yolo_video_test.bat [--parameter ^<parameter.yaml^> ^| --video ^<video^>] [extra yolo_video_test args]
echo.
echo Example:
echo   run_yolo_video_test.bat --parameter E:\RM\rm_vision\examples\example_for_prediction\8_dark_blue_small\parameter.yaml --output E:\RM\rm_vision\RM_BUFF_V2.1\tests\output\small_blue_full.avi
pause
exit /b 1

:missing_model
echo.
echo Missing ONNX model: %MODEL%
pause
exit /b 1

:fail
echo.
echo Build failed or executable is missing.
pause
exit /b 1
