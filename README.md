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
- ROS 2 节点已对齐 2.2 主流程，支持上机接图、预测输出、`BuffTarget` 目标状态和调试图输出
- 新增实验性空间通道：Detector 与 Tracker 解耦，支持 CameraInfo、yaw/pitch、TF 和短时丢失外推
- 大符预测改为真实时间戳速度样本、固定 ω 网格搜索、内点重拟合和正弦速度解析积分
- PnP 使用 IPPE 多候选解，并按重投影误差与跨帧位姿连续性选择结果

## 已知现象

- `[YOLO]大符蓝` 在开头几秒里，`mask__` 黑白调试窗口可能会短暂出现重复候选框，主框也可能有轻微跳变。
- 这段现象主要发生在 `YOLO 初始化 -> HSV/F_BuffTracker 接管` 的过渡期，来源是 HSV 二值图里的候选轮廓临时分裂/粘连，不是 YOLO 持续多目标检出。
- 当前实测里，这个现象只出现在开头几秒，后续会自行稳定；如果没有触发持续丢失、频繁重锁或后段预测漂移，通常不需要专门修改。
- 如果后续它开始影响主流程，再优先检查对应场景 `parameter.yaml` 里的 HSV 上下限、`kernel`、`insideRate`、`outsideRate`。

## 目录

- `src/core/`
  算法核心，包括传统跟踪、YOLO 检测、预测和补偿
- `src/standalone/`
  离线视频、图片序列和调参入口
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

Windows + VS2022 + OpenCV。ONNX Runtime 可选；未找到时使用 OpenCV DNN：

```bat
cd /d <repo>
cmake --preset vs2022-release
cmake --build build/vs2022-release --config Release
ctest --test-dir build/vs2022-release -C Release --output-on-failure
```

预设中的依赖路径是本机默认值，也可以通过 `OpenCV_DIR`、`ONNXRUNTIME_DIR` 环境变量或 `CMakeUserPresets.json` 覆盖。

常用可执行文件：

- `build/vs2022-release/Release/predict_example_main.exe`
- `build/vs2022-release/Release/yolo_video_test.exe`
- `build/vs2022-release/Release/yolo_image_test.exe`
- `build/vs2022-release/Release/camera_geometry_test.exe`

## 常用离线调试

纯模型视频调试：

```bat
tests\run_yolo_video_test.bat ^
  --video "<dataset>\dark_red_small_near.MP4" ^
  --output "tests\output\dark_red_small_near.avi" ^
  --show
```

纯模型单图对比：

```bat
tests\run_yolo_image_test.bat ^
  --image "<dataset>\sample.jpg" ^
  --show
```

## ROS 2 上机形态

项目保留两种 ROS 2 形态：单节点兼容链路适合先验证原有算法；独立空间通道用于继续接相机标定、TF 和上层控制。

### 单节点兼容链路

#### 输入

- 订阅：`~/image_raw`
  相机图像，推荐 `bgr8` / `rgb8`

#### 输出

- 发布：`~/prediction`
  类型：`geometry_msgs/msg/PointStamped`
  含义：当前补偿后的图像坐标点，`x/y` 为像素坐标
- 发布：`~/target`
  类型：`rm_buff_tracker/msg/BuffTarget`
  含义：能量机关专用目标状态，包含跟踪状态、预测状态、R 中心、扇叶中心、预测点、半径、角度和角速度；当前仍为图像平面状态，不直接驱动串口
- 发布：`~/debug_state`
  类型：`std_msgs/msg/Float64MultiArray`
  当前字段顺序：
  `found, prediction_ready, lost_frames, observed_angle, raw_angle, delta_angle, compensated_delta, angular_velocity, pred_x, pred_y, comp_x, comp_y, r_cx, r_cy, fan_cx, fan_cy, radius`
- 发布：`~/debug_image`
  类型：`sensor_msgs/msg/Image`
  含义：带 `R` 框、扇叶框、预测点和状态文本的调试图

#### 当前 ROS 2 策略

- `detector_type=yolo`
  使用 YOLO 做初始化和重锁，正常帧由传统跟踪器接管
- `detector_type=hsv`
  仅用于你已经知道固定 ROI 的情况，需要在参数里填写 `static_r_roi` 和 `static_fan_roi`

### 独立空间通道（实验性）

```text
/camera/image_raw + /camera_info
  -> buff_detector_node
  -> /buff/detector/observation (BuffObservation)
  -> buff_tracker_node + TF
  -> /buff/tracker/target (BuffTargetState)
```

Detector 的图像回调只覆盖保存最新帧，后台工作线程负责检测和预测，从而避免旧帧排队。Tracker 负责坐标变换、速度平滑和短时丢失外推。

```bash
ros2 launch rm_buff_tracker buff_spatial_channel.launch.py \
  params_file:=/home/rm/rm_ws/src/rm_buff_tracker/config/lab/buff_spatial_channel.yaml \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  target_frame:=odom
```

注意：示例配置默认关闭 PnP。此时空间位置是把像素射线延伸到配置的 `target_distance`，属于固定距离近似，不是真实深度；实车使用前必须补齐真实相机标定、与当前 5 关键点模型严格对应的目标物理点和外参验证。IPPE 双解选择已经接入；启用后用 PnP 提供真实距离、用预测像素提供未来发射方向，不能直接套用其它 9 关键点模型的点序和尺寸。

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

单节点链路至少需要：

- ROS 2
- OpenCV
- ONNX Runtime（可选；缺失时回退 OpenCV DNN）

空间通道还需要 `tf2`、`tf2_ros`、`tf2_geometry_msgs`。

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
  params_file:=/home/rm/rm_ws/src/rm_buff_tracker/config/lab/buff_node_lab.yaml \
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
  模型路径，支持相对路径或绝对路径；推荐优先写 `models/best.onnx`
- `freq`
  相机和算法实际运行频率
- `delta_t`
  预测时间
- `big_fit_*`
  大符鲁棒拟合参数：ω 搜索步数、样本窗口、最小内点数、内点阈值、物理参数范围和观测断流阈值。算法使用图像时间戳，不再依赖每帧严格等间隔；`freq` 只作为无时间戳入口的回退值
- `enable_compensation`
  是否把弹丸飞行、通信和云台延迟并入预测时域；大符会对正弦速度进行完整积分，而不是使用瞬时速度乘延迟

ROS 2 参数样例已经给在：

- `config/lab/buff_node_lab.yaml`

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
- `~/target`

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

兼容单节点 `buff_node` 目前仍输出图像坐标，不是最终云台控制量。独立空间通道已经能发布 yaw/pitch 和空间目标状态，但默认深度仍是固定距离近似。

如果你要接到机器人整套链路，后面通常还需要：

- 实机相机内参、畸变和外参验证
- 能量机关物理尺寸与 PnP 点序验证
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

## 算法参考

- 浙江大学 Hello World 战队 [HWauto_buff2026](https://github.com/IC-Alan/HWauto_buff2026)：时间戳差分速度、固定 ω 后线性求解、IPPE 平面双解和正弦速度积分思路。当前仓库按自身 5 关键点模型和 ROS2 架构重新实现，没有直接复用对方 TensorRT 模型与物理点序

## License

源代码按 [MIT License](LICENSE) 发布。`models/best.onnx` 的训练数据来源与第三方再分发条款尚未完整记录；独立分发权重前请先核对相应许可，详见 `models/README.md`。
