@echo off
setlocal
call "%~dp0ros2_sim_0_env.bat"
if errorlevel 1 goto :fail

if not exist "%VSDEVCMD_BAT%" goto :missing_vs
if not exist "%ROS2_SETUP%" goto :missing_ros
if not exist "%AT_VISION_SIM_DIR%\Cargo.toml" goto :missing_sim

call "%VSDEVCMD_BAT%" -arch=amd64 -host_arch=amd64 >nul
if errorlevel 1 goto :fail
call "%ROS2_SETUP%"
if errorlevel 1 goto :fail

if not "%LIBCLANG_PATH%"=="" if exist "%LIBCLANG_PATH%" set "PATH=%LIBCLANG_PATH%;%PATH%"
if not "%OpenCV_BIN_DIR%"=="" if exist "%OpenCV_BIN_DIR%" set "PATH=%OpenCV_BIN_DIR%;%PATH%"
if not "%ONNXRUNTIME_LIB_DIR%"=="" if exist "%ONNXRUNTIME_LIB_DIR%" set "PATH=%ONNXRUNTIME_LIB_DIR%;%PATH%"
set "SIMULATOR_EXE=%CARGO_TARGET_DIR%\release\daedalus.exe"
set "NEED_BUILD=0"

cd /d "%AT_VISION_SIM_DIR%"
if not exist "%SIMULATOR_EXE%" (
    set "NEED_BUILD=1"
) else (
    powershell -NoProfile -Command "$exe = Get-Item '%SIMULATOR_EXE%'; $src = Get-ChildItem '%AT_VISION_SIM_DIR%' -Recurse -File -Include *.rs,Cargo.toml,Cargo.lock; if (($src | Measure-Object LastWriteTime -Maximum).Maximum -gt $exe.LastWriteTime) { exit 10 } else { exit 0 }"
    if errorlevel 10 set "NEED_BUILD=1"
)

if "%NEED_BUILD%"=="1" (
    echo.
    echo Simulator sources changed, rebuilding release binary...
    cargo build --release
    if errorlevel 1 goto :fail
)

echo.
echo Launching at_vision_simulator...
echo Keep the simulator camera in Robot view for /image_raw bring-up.
echo ROS2 setup : %ROS2_SETUP%
echo VS DevCmd  : %VSDEVCMD_BAT%
echo libclang   : %LIBCLANG_PATH%
echo Binary     : %SIMULATOR_EXE%
echo.

"%SIMULATOR_EXE%"
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo at_vision_simulator exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:missing_vs
echo.
echo Missing Visual Studio developer environment script:
echo %VSDEVCMD_BAT%
pause
exit /b 1

:missing_ros
echo.
echo Missing ROS 2 setup file:
echo %ROS2_SETUP%
pause
exit /b 1

:missing_sim
echo.
echo Missing simulator directory:
echo %AT_VISION_SIM_DIR%
pause
exit /b 1

:fail
echo.
echo Failed to start at_vision_simulator.
pause
exit /b 1
