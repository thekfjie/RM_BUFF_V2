@echo off
setlocal
call "%~dp0ros2_sim_0_env.bat"
if errorlevel 1 goto :fail

if not exist "%VSDEVCMD_BAT%" goto :missing_vs
if not exist "%ROS2_SETUP%" goto :missing_ros
if not exist "%RM_BUFF_SOURCE%\package.xml" goto :missing_source
if not exist "%OpenCV_DIR%" goto :missing_opencv
where "%PYTHON_EXE%" >nul 2>nul
if errorlevel 1 if not exist "%PYTHON_EXE%" goto :missing_python

if not exist "%RM_WS%" mkdir "%RM_WS%"
if errorlevel 1 goto :fail

where colcon >nul 2>nul
if errorlevel 1 (
    if exist "%COLCON_EXE%" for %%I in ("%COLCON_EXE%") do set "PATH=%%~dpI;%PATH%"
    where colcon >nul 2>nul
    if errorlevel 1 goto :missing_colcon
)

call "%VSDEVCMD_BAT%" -arch=amd64 -host_arch=amd64 >nul
if errorlevel 1 goto :fail
call "%ROS2_SETUP%"
if errorlevel 1 goto :fail

cd /d "%RM_WS%"
echo.
echo Workspace : %RM_WS%
echo Source    : %RM_BUFF_SOURCE%
echo Params    : %BUFF_PARAMS_FILE%
echo.

if exist "%ONNXRUNTIME_DIR%\include\onnxruntime_cxx_api.h" (
    colcon build --merge-install --cmake-force-configure --base-paths "%RM_BUFF_SOURCE%" --packages-select rm_buff_tracker --cmake-args "-DOpenCV_DIR=%OpenCV_DIR%" "-DONNXRUNTIME_DIR=%ONNXRUNTIME_DIR%" "-DPYTHON_EXECUTABLE=%PYTHON_EXE%" "-DPython3_EXECUTABLE=%PYTHON_EXE%"
) else (
    echo ONNXRUNTIME_DIR not found, build will fall back to OpenCV DNN.
    colcon build --merge-install --cmake-force-configure --base-paths "%RM_BUFF_SOURCE%" --packages-select rm_buff_tracker --cmake-args "-DOpenCV_DIR=%OpenCV_DIR%" "-DPYTHON_EXECUTABLE=%PYTHON_EXE%" "-DPython3_EXECUTABLE=%PYTHON_EXE%"
)
if errorlevel 1 goto :fail

echo.
echo ROS 2 build finished.
exit /b 0

:missing_colcon
echo.
echo Missing colcon executable.
echo Install it with:
echo python -m pip install --user colcon-common-extensions
pause
exit /b 1

:missing_python
echo.
echo Missing Python executable:
echo %PYTHON_EXE%
pause
exit /b 1

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

:missing_source
echo.
echo Missing rm_buff_tracker source package:
echo %RM_BUFF_SOURCE%
pause
exit /b 1

:missing_opencv
echo.
echo Missing OpenCV_DIR:
echo %OpenCV_DIR%
pause
exit /b 1

:fail
echo.
echo ROS 2 build failed.
pause
exit /b 1
