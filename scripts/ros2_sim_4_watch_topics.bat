@echo off
setlocal
call "%~dp0ros2_sim_0_env.bat"
if errorlevel 1 goto :fail

if not exist "%ROS2_SETUP%" goto :missing_ros
if not exist "%RM_WS%\install\setup.bat" goto :missing_install
if not exist "%RM_WS%\install\lib\rm_buff_tracker\topic_probe.exe" goto :missing_probe

call "%ROS2_SETUP%"
if errorlevel 1 goto :fail
call "%RM_WS%\install\setup.bat"
if errorlevel 1 goto :fail

set "NODE_TOPIC_PREFIX=/buff_tracker_node"
if not "%BUFF_NAMESPACE%"=="" set "NODE_TOPIC_PREFIX=/%BUFF_NAMESPACE%/buff_tracker_node"

echo.
echo Watching image/prediction/debug topics with topic_probe...
echo image       : %BUFF_IMAGE_TOPIC%
echo debug_image : %NODE_TOPIC_PREFIX%/debug_image
echo debug_state : %NODE_TOPIC_PREFIX%/debug_state
echo prediction  : %NODE_TOPIC_PREFIX%/prediction
echo.
"%RM_WS%\install\lib\rm_buff_tracker\topic_probe.exe" "%NODE_TOPIC_PREFIX%" "%BUFF_IMAGE_TOPIC%"
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo topic_probe exited with code %EXITCODE%.
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

:missing_probe
echo.
echo Missing topic_probe executable:
echo %RM_WS%\install\lib\rm_buff_tracker\topic_probe.exe
echo Run ros2_sim_1_build_rm_buff_tracker.bat first.
pause
exit /b 1

:fail
echo.
echo Failed to inspect ROS 2 topics.
pause
exit /b 1
