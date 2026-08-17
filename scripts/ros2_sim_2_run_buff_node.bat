@echo off
setlocal
call "%~dp0ros2_sim_0_env.bat"
if errorlevel 1 goto :fail

if not exist "%ROS2_SETUP%" goto :missing_ros
if not exist "%RM_WS%\install\setup.bat" goto :missing_install
if not exist "%BUFF_PARAMS_FILE%" goto :missing_params
if not exist "%RM_WS%\install\lib\rm_buff_tracker\buff_node.exe" goto :missing_exe

call "%ROS2_SETUP%"
if errorlevel 1 goto :fail
call "%RM_WS%\install\setup.bat"
if errorlevel 1 goto :fail

if exist "%OpenCV_BIN_DIR%" set "PATH=%OpenCV_BIN_DIR%;%PATH%"
if exist "%ONNXRUNTIME_LIB_DIR%" set "PATH=%ONNXRUNTIME_LIB_DIR%;%PATH%"
if not "%LIBCLANG_PATH%"=="" if exist "%LIBCLANG_PATH%" set "PATH=%LIBCLANG_PATH%;%PATH%"

set "BUFF_NODE_EXE=%RM_WS%\install\lib\rm_buff_tracker\buff_node.exe"
set "NS_ARGS="
if not "%BUFF_NAMESPACE%"=="" set "NS_ARGS=-r __ns:=/%BUFF_NAMESPACE%"

cd /d "%RM_BUFF_SOURCE%"
echo.
echo Launching buff_node...
echo Image topic: %BUFF_IMAGE_TOPIC%
if "%BUFF_NAMESPACE%"=="" (
    echo Namespace : ^<empty^>
) else (
    echo Namespace : %BUFF_NAMESPACE%
)
echo Params    : %BUFF_PARAMS_FILE%
echo.

"%BUFF_NODE_EXE%" --ros-args --params-file "%BUFF_PARAMS_FILE%" -r ~/image_raw:=%BUFF_IMAGE_TOPIC% %NS_ARGS%
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo buff_node exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:missing_ros
echo.
echo Missing ROS 2 setup file:
echo %ROS2_SETUP%
pause
exit /b 1

:missing_install
echo.
echo Missing workspace install setup:
echo %RM_WS%\install\setup.bat
echo Run ros2_sim_1_build_rm_buff_tracker.bat first.
pause
exit /b 1

:missing_params
echo.
echo Missing ROS 2 params file:
echo %BUFF_PARAMS_FILE%
pause
exit /b 1

:missing_exe
echo.
echo Missing buff_node executable:
echo %RM_WS%\install\lib\rm_buff_tracker\buff_node.exe
echo Run ros2_sim_1_build_rm_buff_tracker.bat first.
pause
exit /b 1

:fail
echo.
echo Failed to start buff_node.
pause
exit /b 1
