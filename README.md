# RM_BUFF_V2.1

RoboMaster 能量机关离线复刻、调参和验证仓库。

当前版本主线已经包含：

- 传统小符 / 大符预测流程
- YOLO26 ONNX 初始化 / 重锁辅助
- 单图 / 视频调试工具
- Windows + VS2022 + OpenCV 的离线运行脚本

## 目录

- `src/core/`: 算法核心
- `src/standalone/`: 离线入口
- `src/ros2/`: ROS 2 封装
- `scripts/`: 常用构建与运行脚本
- `tests/`: YOLO 单图 / 视频调试工具
- `docs/`: 详细说明文档

## 构建

```bat
cd /d E:\RM\rm_vision\RM_BUFF_V2.1
cmake --preset vs2022-release
cmake --build build/vs2022-release --config Release
```

## 常用入口

- `build/vs2022-release/Release/predict_example_main.exe`
- `build/vs2022-release/Release/yolo_video_test.exe`
- `build/vs2022-release/Release/yolo_image_test.exe`

## 详细文档

- `docs/README.md`
- `tests/README.md`
