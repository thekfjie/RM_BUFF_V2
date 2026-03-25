@echo off
setlocal
cd /d "%~dp0.."
set "PATH=D:\develop\opencv\build\x64\vc16\bin;D:\develop\onnxruntime\lib;%PATH%"
set "ROOT=E:\RM\rm_vision"
set "PARAM=%ROOT%\examples\example_for_prediction\9_dark_red_small\parameter.yaml"
set "ONNX=%~dp0..\models\best.onnx"
set "EXE=%~dp0..\build\vs2022-release\Release\predict_example_main.exe"

if not exist "%EXE%" call "%~dp0build_release.bat"
if errorlevel 1 goto :fail
if not exist "%EXE%" goto :fail
if not exist "%PARAM%" goto :missing_param
if not exist "%ONNX%" goto :missing_model

"%EXE%" --python-root "%ROOT%" --parameter "%PARAM%" --mode small --color red --detector yolo --onnx "%ONNX%"
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo Program exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:missing_param
echo Missing parameter file: %PARAM%
pause
exit /b 1

:missing_model
echo Missing ONNX model: %ONNX%
pause
exit /b 1

:fail
echo Build failed or executable is missing.
pause
exit /b 1
