@echo off
setlocal

rem Edit these defaults before first use, or set them in the parent shell.
for %%I in ("%~dp0..") do set "RM_BUFF_SOURCE=%%~fI"
for %%I in ("%RM_BUFF_SOURCE%\..\..") do set "RM_ROOT=%%~fI"
if not defined ROS2_SETUP set "ROS2_SETUP=D:\develop\ros2-humble-20260220-windows-release-amd64\ros2-windows\setup.bat"
if not defined VSDEVCMD_BAT set "VSDEVCMD_BAT=D:\develop\vs2022\Community\Common7\Tools\VsDevCmd.bat"
if not defined RM_WS set "RM_WS=%RM_ROOT%\rm_ws"

rem CMake configure path for OpenCV.
if not defined OpenCV_DIR set "OpenCV_DIR=D:\develop\opencv\build"
rem Runtime DLL path for OpenCV.
if not defined OpenCV_BIN_DIR set "OpenCV_BIN_DIR=D:\develop\opencv\build\x64\vc16\bin"

rem ONNX Runtime is optional. Leave these as-is if you want the OpenCV DNN fallback.
if not defined ONNXRUNTIME_DIR set "ONNXRUNTIME_DIR=D:\develop\onnxruntime"
if not defined ONNXRUNTIME_LIB_DIR set "ONNXRUNTIME_LIB_DIR=D:\develop\onnxruntime\lib"

if not defined AT_VISION_SIM_DIR set "AT_VISION_SIM_DIR=%RM_ROOT%\at_vision_simulator-master"
if not defined PYTHON_EXE set "PYTHON_EXE=python"
if not defined CARGO_HOME set "CARGO_HOME=%USERPROFILE%\.cargo"
if not defined RUSTUP_HOME set "RUSTUP_HOME=%USERPROFILE%\.rustup"
if not defined CARGO_TARGET_DIR set "CARGO_TARGET_DIR=%RM_ROOT%\cargo_target_ros2"
if not defined COLCON_EXE set "COLCON_EXE=%APPDATA%\Python\Python310\Scripts\colcon.exe"

rem Windows ROS2 + Rust bindgen needs libclang plus MSVC-compatible clang args.
set "LIBCLANG_PATH=D:\msys64\mingw64\bin"
set "BINDGEN_EXTRA_CLANG_ARGS=--target=x86_64-pc-windows-msvc -fms-extensions -fdeclspec -D_Check_return_="

set "BUFF_PARAMS_FILE=%RM_BUFF_SOURCE%\config\lab\buff_node_simulator.yaml"
set "BUFF_IMAGE_TOPIC=/image_raw"
set "BUFF_NAMESPACE="

endlocal & (
    set "RM_BUFF_SOURCE=%RM_BUFF_SOURCE%"
    set "ROS2_SETUP=%ROS2_SETUP%"
    set "VSDEVCMD_BAT=%VSDEVCMD_BAT%"
    set "RM_WS=%RM_WS%"
    set "OpenCV_DIR=%OpenCV_DIR%"
    set "OpenCV_BIN_DIR=%OpenCV_BIN_DIR%"
    set "ONNXRUNTIME_DIR=%ONNXRUNTIME_DIR%"
    set "ONNXRUNTIME_LIB_DIR=%ONNXRUNTIME_LIB_DIR%"
    set "AT_VISION_SIM_DIR=%AT_VISION_SIM_DIR%"
    set "PYTHON_EXE=%PYTHON_EXE%"
    set "CARGO_HOME=%CARGO_HOME%"
    set "RUSTUP_HOME=%RUSTUP_HOME%"
    set "CARGO_TARGET_DIR=%CARGO_TARGET_DIR%"
    set "COLCON_EXE=%COLCON_EXE%"
    set "LIBCLANG_PATH=%LIBCLANG_PATH%"
    set "BINDGEN_EXTRA_CLANG_ARGS=%BINDGEN_EXTRA_CLANG_ARGS%"
    set "BUFF_PARAMS_FILE=%BUFF_PARAMS_FILE%"
    set "BUFF_IMAGE_TOPIC=%BUFF_IMAGE_TOPIC%"
    set "BUFF_NAMESPACE=%BUFF_NAMESPACE%"
)
exit /b 0
