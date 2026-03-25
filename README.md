# RM_BUFF_V2.2

RoboMaster 能量机关识别、跟踪、预测与调试仓库。

当前 2.2 主线的定位是：

- 保留传统方法作为主跟踪链路
- 引入 YOLO26 ONNX 做初始化和重锁
- 同时支持 Windows 离线视频调试和 ROS 2 上机接图

## 2.2 的核心变化

- YOLO26 模型接入完成，支持 ONNX Runtime / OpenCV DNN
- 主流程修正为“YOLO 初始化/重锁 + HSV/F_BuffTracker 持续跟踪”
- 修正关键点顺序，当前数据集按 `kp3 = R`、`kp1/kp2/kp4/kp5 = 扇叶`
- 推理前处理改成更贴近 Ultralytics 的 `letterbox`，解决关键点和 `R` 偏移
- 新增单图和视频调试工具，能直接检查模型框、关键点和推导出的 `R box`
- ROS 2 节点已对齐 2.2 主流程，支持上机接图、预测输出和调试图输出

## 目录

- `src/core/`
  算法核心，包括传统跟踪、YOLO 检测、预测和补偿
- `src/standalone/`
  离线视频入口
- `src/ros2/`
  ROS 2 节点入口
- `config/`
  默认参数和 ROS 2 参数样例
- `launch/`
  ROS 2 启动文件
- `scripts/`
  Windows 下常用构建与运行脚本
- `tests/`
  YOLO 单图 / 视频检查工具
- `models/`
  模型文件
- `docs/`
  更细的设计和开发文档

## 本地离线构建

Windows + VS2022 + OpenCV：

```bat
cd /d E:\RM\rm_vision\RM_BUFF_V2.1
cmake --preset vs2022-release
cmake --build build/vs2022-release --config Release
```

常用可执行文件：

- `build/vs2022-release/Release/predict_example_main.exe`
- `build/vs2022-release/Release/yolo_video_test.exe`
- `build/vs2022-release/Release/yolo_image_test.exe`

## 常用离线调试

纯模型视频调试：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\tests\run_yolo_video_test.bat ^
  --video "E:\RM\rm_vision\examples\example_for_prediction\10_dark_red_small_near\dark_red_small_near.MP4" ^
  --output "E:\RM\rm_vision\RM_BUFF_V2.1\tests\output\dark_red_small_near.avi" ^
  --show
```

纯模型单图对比：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\tests\run_yolo_image_test.bat ^
  --image E:\RM\buff_dataset\images\train\004b0247-558.jpg ^
  --show
```

## ROS 2 上机形态

### 输入

- 订阅：`~/image_raw`
  相机图像，推荐 `bgr8` / `rgb8`

### 输出

- 发布：`~/prediction`
  类型：`geometry_msgs/msg/PointStamped`
  含义：当前补偿后的图像坐标点，`x/y` 为像素坐标
- 发布：`~/debug_state`
  类型：`std_msgs/msg/Float64MultiArray`
  当前字段顺序：
  `found, prediction_ready, lost_frames, observed_angle, raw_angle, delta_angle, compensated_delta, pred_x, pred_y, comp_x, comp_y, r_cx, r_cy, fan_cx, fan_cy, radius`
- 发布：`~/debug_image`
  类型：`sensor_msgs/msg/Image`
  含义：带 `R` 框、扇叶框、预测点和状态文本的调试图

### 当前 ROS 2 策略

- `detector_type=yolo`
  使用 YOLO 做初始化和重锁，正常帧由传统跟踪器接管
- `detector_type=hsv`
  仅用于你已经知道固定 ROI 的情况，需要在参数里填写 `static_r_roi` 和 `static_fan_roi`

## 机器人上怎么接

### 1. 放进 ROS 2 工作空间

建议目录：

```text
~/rm_ws/src/rm_buff_tracker
```

也就是把当前仓库整个放进去，包括：

- `src/`
- `config/`
- `launch/`
- `models/best.onnx`

### 2. 准备依赖

你至少需要：

- ROS 2
- OpenCV
- ONNX Runtime

如果机器人上使用 ONNX Runtime，确保构建时能找到：

- `ONNXRUNTIME_DIR`

### 3. 编译

在 ROS 2 工作空间下：

```bash
cd ~/rm_ws
colcon build --packages-select rm_buff_tracker --cmake-args -DONNXRUNTIME_DIR=/path/to/onnxruntime
source install/setup.bash
```

### 4. 启动相机

确保相机节点已经在发布图像，比如：

- `/camera/image_raw`
- `/hik/image_raw`
- `/mv/image_raw`

### 5. 启动 buff 节点

推荐用参数文件 + remap：

```bash
ros2 launch rm_buff_tracker buff_node.launch.py \
  params_file:=/home/rm/rm_ws/src/rm_buff_tracker/config/buff_node.yaml \
  image_topic:=/camera/image_raw
```

如果你的图像话题不是 `/camera/image_raw`，只改 `image_topic` 即可。

如果你直接用安装目录里的默认参数文件，也可以：

```bash
ros2 launch rm_buff_tracker buff_node.launch.py image_topic:=/camera/image_raw
```

## 上机前需要改什么

你至少要确认下面这些参数：

- `color`
  目标颜色，`red` 或 `blue`
- `mode`
  `small` 或 `big`
- `detector_type`
  上机建议用 `yolo`
- `onnx_model_path`
  机器人上模型的真实绝对路径
- `freq`
  相机和算法实际运行频率
- `delta_t`
  预测时间
- `enable_compensation`
  是否打开弹道补偿

ROS 2 参数样例已经给在：

- `config/buff_node.yaml`

## 机器人上怎么调试

建议按这个顺序：

1. 先确认图像流正常
   看 `~/image_raw` 是否稳定、有时间戳、有画面
2. 再确认模型能加载
   看节点启动日志里是否报 ONNX 加载失败
3. 再看 YOLO 初始化是否成功
   观察日志里是否出现 `YOLO init locked target`
4. 再看传统跟踪是否接住
   看 `~/debug_image` 里 `R` 框和扇叶框是否稳定
5. 最后看预测点是否稳定
   观察 `~/prediction` 是否连续、是否落在合理位置

推荐同时看这三个话题：

- `~/debug_image`
- `~/debug_state`
- `~/prediction`

常用调试命令可以直接这样跑：

```bash
ros2 topic hz /buff_tracker_node/prediction
ros2 topic echo /buff_tracker_node/debug_state
ros2 run rqt_image_view rqt_image_view
```

如果你给节点加了 namespace，比如 `buff`，对应话题就会变成：

```text
/buff/prediction
/buff/debug_state
/buff/debug_image
```

## 当前需要你自己再接的部分

这套 2.2 目前输出的是图像坐标，不是最终云台 yaw/pitch。

如果你要接到机器人整套链路，后面通常还需要：

- 相机内参 / 外参
- 图像点到角度或三维点的映射
- 与云台控制节点的接口
- 发射控制联动

也就是说，当前 `buff_node` 已经能完成：

- 接图
- 自动初始化 / 重锁
- 跟踪
- 预测
- 调试输出

但还没有直接变成“可发弹控制量”。

## 文档入口

- `docs/README.md`
- `tests/README.md`
