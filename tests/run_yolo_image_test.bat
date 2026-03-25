@echo off
setlocal
cd /d "%~dp0.."

set "PATH=D:\develop\opencv\build\x64\vc16\bin;D:\develop\onnxruntime\lib;%PATH%"
set "MODEL=%~dp0..\models\best.onnx"
set "EXE=%~dp0..\build\vs2022-release\Release\yolo_image_test.exe"

if "%~1"=="" goto :usage

if not exist "%EXE%" call "%~dp0..\scripts\build_release.bat"
if errorlevel 1 goto :fail
if not exist "%EXE%" goto :fail
if not exist "%MODEL%" goto :missing_model

"%EXE%" --model "%MODEL%" %*
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo Program exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:usage
echo Usage:
echo   run_yolo_image_test.bat --image ^<image^> [--label ^<label.txt^>] [--output ^<compare.jpg^>] [--conf 0.25] [--show]
echo.
echo Example:
echo   run_yolo_image_test.bat --image E:\RM\buff_dataset\images\train\004b0247-558.jpg --show
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
