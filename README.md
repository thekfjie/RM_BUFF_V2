# RoboMaster 能量机关视觉

本仓库提供大小符识别、跟踪、预测，以及与队伍步兵装甲板自瞄共用的 ROS 2 串口入口。ROS 包名为 `rm_buff_tracker`。日常联调先使用图像和目标状态话题；只有相机标定、坐标变换、下位机协议和现场安全检查完成后，才启用串口控制输出。

## 系统分工

```text
相机图像 + CameraInfo + 相机外参
  -> buff_detector_node：目标选择、相位估计、未来击打点预测
  -> buff_tracker_node：采集时刻 TF、坐标转换、超时状态
  -> /buff/tracker/target (BuffTargetState)
  -> buff_serial_bridge：有效性判断、角度转换、模式选择
  -> USART1 / 步兵下位机：云台执行

装甲板自瞄 /tracker/target (auto_aim_interfaces/Target)
  -> 同一个 buff_serial_bridge -> 同一个 USART1
```

串口桥是**唯一**打开视觉串口的节点，同时订阅两条目标话题；部署它以后不再同时运行原 `rm_serial_driver`。`active_mode` 只有两种取值：

| `active_mode` | 目标入口 | 发给下位机 | 预测归属 |
|---|---|---|---|
| `armor` | `/tracker/target` | `0xA5` 装甲板状态 | 保留原下位机装甲板外推 |
| `buff` | `/buff/tracker/target` | `0xA6` 大小符绝对角 | 上位机预测一次，下位机不再外推 |

切换模式不需要重启节点或更换下位机串口，旧目标会先被撤销。大小符算法内部另有 `small` / `big` 两种能量机关配置；这是检测器参数，**不等于**串口的 `armor` / `buff` 模式，修改后重启检测器。下位机仍由遥控器决定是否接管云台；本仓库不发送开火命令。

## 开始部署

推荐 Ubuntu 22.04、ROS 2 Humble 和 C++17 工具链。准备 OpenCV、ONNX Runtime C/C++ CPU SDK、`serial_driver`、`auto_aim_interfaces`、相机驱动及相机标定；ROS 依赖交给 rosdep。`auto_aim_interfaces` 是队伍装甲板工程的消息包，**不在本仓库内**。模型文件为 `models/best.onnx`。环境安装、架构适配与完整命令见 [Ubuntu 部署手册](docs/DEPLOYMENT_UBUNTU22_HUMBLE.md)。

在 ROS 工作空间中各放一份 `rm_buff_tracker` 和 `auto_aim_interfaces`，并准备好 `ONNXRUNTIME_DIR`：

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/rm_ws"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-up-to rm_buff_tracker \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DONNXRUNTIME_DIR="$ONNXRUNTIME_DIR"
source install/setup.bash
ros2 pkg executables rm_buff_tracker
```

构建结果应包含 `buff_detector_node`、`buff_tracker_node` 和 `buff_serial_bridge`。桥未生成时，检查 `serial_driver`、`auto_aim_interfaces`、`std_srvs`、`visualization_msgs` 是否找到；不能拿旧 `install/` 目录的节点与新消息接口混用。

下位机使用 [步兵固件](https://github.com/An-OrangeMan/2026-RoboMaster-Infantry) 时，需要由固件维护者审阅并应用本仓库的[协议补丁](integration/infantry/0001-Add-guarded-BUFF-aim-command-to-infantry-controller.patch)。两套 Keil 工程各有自己的 `application/` 源码，烧录前确认实际使用哪一套；上位机和下位机必须使用配套协议。补丁应用方式、48 字节 `0xA6` 布局和接管条件见[串口接口文档](docs/BUFF_INFANTRY_SERIAL.md)。

## 上机顺序

1. 启动相机并核对 `/camera/image_raw`、`/camera_info` 的分辨率、frame_id 和采集时间。准备真实的 `gimbal_link -> camera_optical_frame` 静态外参；话题不同则在启动参数中指定实际名称。
2. 停止原 `rm_serial_driver`，启动统一串口桥；桥通过固件反馈发布 `odom -> gimbal_link`，默认 `enable_output: false`，只读反馈、不发送控制帧。
3. 配置目标色 `red` / `blue` 和能量机关类型 `small` / `big`，启动空间通道，检查目标状态、TF、深度、预测时间与实际帧率。
4. 在无弹丸、受限云台行程下核对方向和遥控器接管。确认配套固件、标定与目标有效性后，在现场配置副本中将桥的 `enable_output` 显式设为 `true`，重启桥；再测超时撤销及模式切换。

启动前从 `config/lab/` 复制现场配置，不直接改仓库示例。`color`、`mode`、PnP 物理点和串口设备路径按实际设备填写：

```bash
cd "$HOME/rm_ws/src/rm_buff_tracker"
cp -n config/lab/buff_serial_bridge.yaml config/match/buff_serial_local.yaml
cp -n config/lab/buff_spatial_channel.yaml config/match/buff_spatial_local.yaml
```

启动示例；相机话题需换成现场实际值：

```bash
ros2 launch rm_buff_tracker buff_serial_bridge.launch.py \
  params_file:="$HOME/rm_ws/src/rm_buff_tracker/config/match/buff_serial_local.yaml"

ros2 launch rm_buff_tracker buff_spatial_channel.launch.py \
  params_file:="$HOME/rm_ws/src/rm_buff_tracker/config/match/buff_spatial_local.yaml" \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera_info \
  target_frame:=odom
```

桥的 `device_name` 必须对应唯一打开的实际设备；默认 `/dev/ttyACM0`、115200/8N1。模式在线切换：

```bash
ros2 param set /buff_serial_bridge active_mode armor  # 打装甲板
ros2 param set /buff_serial_bridge active_mode buff   # 打大小符
ros2 param get /buff_serial_bridge active_mode
```

装甲板模式由队伍原装甲板节点继续发布 `/tracker/target`。桥复用原装甲板字段、颜色反馈与重置服务；大小符模式只使用有效的未来点，不重复做装甲板预测。两个目标入口可以保持订阅，但同一时刻只会发送所选模式的命令。

## 有效性和调参

默认空间配置 `enable_pnp: false`，固定 `target_distance: 7.0` 仅是射线距离示例；默认串口配置 `require_pnp: true` 且 `enable_output: false`。因此**默认配置不会产生有效的大小符云台命令**。要启用真实深度，先测量与本仓库五关键点模型对应的四个扇叶平面物理点，并验证内参、畸变及外参。不要套用其它战队九关键点模型的尺寸或点序。

```bash
ros2 topic echo /buff/tracker/target --once
ros2 topic hz /buff/tracker/target
ros2 run tf2_ros tf2_echo odom camera_optical_frame
```

重点检查 `tracking`、`tracker_state`、`prediction_ready`、`tf_ready`、`pnp_ready`、`prediction_horizon` 和采集帧年龄。`TEMP_LOST` 及超时状态不能作为有效命令；桥也会拒绝过期帧、过期反馈和超出角度限幅的目标。大符采用时间戳速度样本及固定角频率搜索拟合，小符使用相位卡尔曼滤波；参数与消息定义见[视觉接口约定](docs/VISUAL_INTERFACE.md)。

预测时域和弹速、通信及云台延迟需要实测；当前大小符接口**尚未**完成枪口偏置、重力弹道和发射许可闭环。串口联通不等于已经能命中，目标 Linux 编译、通信、实车追踪和击打效果都须逐项验证。

## 离线开发

Windows 开发可用 VS2022、OpenCV 和 `CMakePresets.json`，预设路径按本机修改；ONNX Runtime 未找到时会回退到 OpenCV DNN，但实机推荐先验证 ONNX Runtime：

```bat
cmake --preset vs2022-release
cmake --build build/vs2022-release --config Release
ctest --test-dir build/vs2022-release -C Release --output-on-failure
```

离线入口 `predict_example_main`、`yolo_image_test`、`yolo_video_test` 可用于图像和录像核对；兼容节点 `buff_node` 输出像素坐标与调试图，**不是**串口控制入口。图片、录像脚本见[测试说明](tests/README.md)，各参数配置见[配置说明](config/README.md)。

## 资料与许可

- [完整环境与部署步骤](docs/DEPLOYMENT_UBUNTU22_HUMBLE.md)
- [视觉消息、坐标与预测时域](docs/VISUAL_INTERFACE.md)
- [步兵串口协议、固件补丁与模式切换](docs/BUFF_INFANTRY_SERIAL.md)
- [文档索引](docs/README.md)

本项目基于 [RM_Buff_Tracker_GUT](https://github.com/DH13768095744/RM_Buff_Tracker_GUT) 优化与扩展。感谢 [vision-code](https://github.com/pcpengchang/vision-code)、[HWauto_buff2026](https://github.com/IC-Alan/HWauto_buff2026)、[rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim) 等开源项目的分享。源码按 [MIT License](LICENSE) 发布；模型权重说明见 [models/README.md](models/README.md)。
