@echo off
setlocal
call "%~dp0ros2_sim_1_build_rm_buff_tracker.bat"
if errorlevel 1 goto :fail

start "RM Buff Node" cmd /k call "%~dp0ros2_sim_2_run_buff_node.bat"
timeout /t 2 /nobreak >nul
start "AT Vision Simulator" cmd /k call "%~dp0ros2_sim_3_run_at_vision_simulator.bat"
timeout /t 2 /nobreak >nul
start "ROS2 Topic Watch" cmd /k call "%~dp0ros2_sim_4_watch_topics.bat"

echo.
echo Started three windows: buff_node, simulator, and topic watch.
exit /b 0

:fail
echo.
echo Failed to start the full ROS 2 bring-up workflow.
pause
exit /b 1
