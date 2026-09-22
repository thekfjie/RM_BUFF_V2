# 上位机部署：Ubuntu 22.04 / ROS 2 Humble

整理日期：2026-09-22。本文给出目标机准备步骤，不代表这次已经在 Linux 上编译、运行或验证实车。源码目前经过静态检查；消息接口有变更，旧 install 目录不能直接混用。

## 1. 要准备什么

推荐基线是原生 Ubuntu 22.04 64 位、ROS 2 Humble、系统 GCC/CMake/OpenCV，以及 ONNX Runtime C/C++ CPU SDK。普通工控机选择 x64 包；ARM64/Jetson 选择 aarch64 包。实际 CPU 型号、相机分辨率和帧率尚未提供，不能承诺 50 FPS。

| 类别 | 必需项 | 说明 |
|---|---|---|
| 上位机 | Linux 电脑、稳定电源、相机接口 | 新配机器可先按 4 核以上、8 GB 内存、预留 20 GB 磁盘规划；这是开发余量建议，不是已测最低配置 |
| 系统 | Ubuntu 22.04 + UTF-8 locale | 若已有整车系统，优先和队伍 ROS 版本一致；不要直接在其他系统执行本文安装命令 |
| 编译 | GCC/G++ 支持 C++17、CMake ≥3.18、Git、colcon、rosdep | Jammy 默认工具链满足代码语言要求；接口生成也需要 C 编译器 |
| ROS | Humble、rclcpp、ament、消息生成、tf2、serial_driver | package.xml 声明 ROS 依赖，rosdep 负责安装；大小符串口桥使用 serial_driver |
| 图像库 | OpenCV 开发包 | core/imgproc/highgui/videoio/dnn/calib3d；系统包 libopencv-dev |
| 推理 | ONNX Runtime **C/C++ CPU SDK** | 文中固定 1.24.4，对齐本地 SDK；Linux 兼容性仍需按后文检查。pip 包不能代替头文件和链接库 |
| 模型 | models/best.onnx | 已随 Git 仓库保存，不用重新训练或导出 |
| 相机 | 对应厂商 Linux SDK、ROS2 相机节点、USB/网口权限 | 本仓库不包含海康/迈德威视驱动，需另外准备 |
| 空间计算 | 相机内参/畸变、CameraInfo、真实 TF | 只有像素调试时可以暂不接 TF；空间输出必须明确坐标和标定 |
| PnP | 四个扇叶物理点与对应点序 | 测量完成前 enable_pnp=false；默认 7m 是示例，必须改为实测距离 |
| 调试 | rqt_image_view、rosbag2；桌面或远程查看电脑 | 无桌面运行关闭 show_debug_window，调试图可由另一台 ROS 电脑显示 |

当前代码使用 CPU 执行：ORT 没有追加 CUDA/TensorRT provider，OpenCV 后端也指定 CPU。仅部署此模块不要求 CUDA、cuDNN、TensorRT、PyTorch、Ultralytics 或 Conda；以后启用 GPU 需要修改后端并重新验证，不是装上 GPU 软件就自动加速。

Jetson 要先确认 JetPack 所带 Ubuntu 版本。Ubuntu 20.04 的设备不能直接混装本文 Humble/Jammy 包；保留厂商系统，在确认容器、系统升级或其他 ROS 适配方案后再部署。

## 2. 安装系统与 ROS 依赖

以下命令在**上位机 Bash**执行，不在 Windows PowerShell 执行。已有 Humble 的机器跳过 ROS 安装，仅补开发依赖。

先检查系统和架构：

```bash
cat /etc/os-release
uname -m
locale
```

首次安装 ROS 时，按官方 Humble Ubuntu 安装文档完成 UTF-8 locale、Universe、ros2-apt-source 软件源和系统更新。官方文档与源码镜像见文末；软件源步骤可能更新，因此这里不复制过期的 apt-key 命令。

软件源配置好后，调试机可使用 desktop 版：

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools \
  build-essential cmake git curl ca-certificates pkg-config \
  python3-colcon-common-extensions python3-rosdep libopencv-dev \
  ros-humble-rqt-image-view ros-humble-rosbag2
source /opt/ros/humble/setup.bash
```

无桌面上位机可将 desktop 替换为 ros-humble-ros-base；其余按需要保留。ROS 节点和 colcon 使用系统 Python；不要在 Conda 环境里构建首个版本。

```bash
# 只有首次配置 rosdep 才初始化；若提示已初始化，不要删除原有配置。
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

## 3. 准备 ONNX Runtime C++ SDK

本地使用的 SDK VERSION_NUMBER 为 1.24.4，因此这里固定此版作为起点，不宣称它是最新版本。已核对官方存在 Linux x64 和 aarch64 CPU 压缩包。Windows DLL 不能复制到 Linux 使用。

```bash
case "$(uname -m)" in
  x86_64) ORT_ARCH=x64 ;;
  aarch64) ORT_ARCH=aarch64 ;;
  *) echo "当前架构不在本说明范围内"; exit 1 ;;
esac
ORT_VERSION=1.24.4
ORT_ASSET="onnxruntime-linux-${ORT_ARCH}-${ORT_VERSION}"
mkdir -p "$HOME/opt"
curl --fail --location \
  "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_ASSET}.tgz" \
  --output "$HOME/opt/${ORT_ASSET}.tgz"
tar -xzf "$HOME/opt/${ORT_ASSET}.tgz" -C "$HOME/opt"
export ONNXRUNTIME_DIR="$HOME/opt/${ORT_ASSET}"
export LD_LIBRARY_PATH="$ONNXRUNTIME_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
test -f "$ONNXRUNTIME_DIR/include/onnxruntime_cxx_api.h"
test -f "$ONNXRUNTIME_DIR/lib/libonnxruntime.so"
ldd "$ONNXRUNTIME_DIR/lib/libonnxruntime.so"
```

`ldd` 不应出现 not found、GLIBC 或 GLIBCXX 版本缺失。如果当前发布包与目标机不兼容，先处理 SDK 构建/版本匹配，不要替换系统 libc。本文没有在目标机验证该二进制。

每个用于构建、运行或测试的终端都需要 source ROS 并设置上述 SDK 路径；环境变量只在当前终端生效。确认环境后可写入自己的启动脚本。仅 `pip install onnxruntime` 不足以让本项目 CMake 找到 C++ SDK。

## 4. 获取项目并构建

首次部署建议单独使用 `~/rm_buff_ws`，确认后再集成到队伍 `~/rm_ws`，避免与旧同名包混淆。同一工作空间里只能有一份 rm_buff_tracker。

```bash
mkdir -p "$HOME/rm_buff_ws/src"
cd "$HOME/rm_buff_ws/src"
git clone --branch main https://github.com/thekfjie/RM_BUFF_V2.git rm_buff_tracker
cd "$HOME/rm_buff_ws"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-select rm_buff_tracker \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DONNXRUNTIME_DIR="$ONNXRUNTIME_DIR"
source install/setup.bash
```

如果仓库已存在，先保存本地修改再 `git pull --ff-only`，不要再次 clone 到同一路径。若以前用不同工具链/SDK 构建过，用新的 workspace 或 build/install 目录重新构建；不要复制 Windows 的 build/install。

编译日志应出现：

```text
ONNX Runtime found at ...
ROS 2 environment found, building buff_node
tf2 found, building independent BUFF spatial channel
```

若显示 `ROS 2 not found, building standalone only`，即使命令返回成功也不代表 ROS 部署完成。确认 source 环境和 rosdep 依赖，再构建。

```bash
ros2 pkg prefix rm_buff_tracker
ros2 pkg executables rm_buff_tracker
ros2 interface show rm_buff_tracker/msg/BuffObservation
ros2 interface show rm_buff_tracker/msg/BuffTargetState
sha256sum src/rm_buff_tracker/models/best.onnx
ctest --test-dir build/rm_buff_tracker --output-on-failure
```

预期能找到 `buff_node`、`buff_detector_node`、`buff_tracker_node`、`buff_serial_bridge`；后者缺失应核对构建日志是否显示 `serial_driver not found` 和 rosdep 安装结果。消息里应有 `camera_aim_position` / `prediction_horizon`。模型 SHA-256 为：

```text
2eb7bc53384650ef3be242ad6bb0f768e60b543f31c2d77ed94d9bd6b02bc7dc
```

CTest 应运行 camera_geometry_test、angle_predictor_test、tracker_regression_test。这里给的是**待执行命令**；本轮没有运行这些测试。集成到已有系统后，所有订阅新消息的自定义包也要重新构建，不只重建发布节点。

## 5. 相机、标定和 TF

选择实际相机对应的厂商 SDK 和 ROS2 驱动，确认 CPU 架构匹配。SDK/USB 权限问题先在厂商软件中解决，再进入 ROS。

本地已有参考驱动包名为 hik_camera 或 mindvision_camera，但它们不是此 GitHub 仓库的一部分。采用队伍已经能稳定出图的驱动即可，不必重新换相机链路。

需要相机发布：

```text
/camera/image_raw   sensor_msgs/msg/Image
/camera/camera_info sensor_msgs/msg/CameraInfo
```

名称可以不同，通过 launch 参数指定。检查：

```bash
ros2 topic list -t
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --field header --once
ros2 topic echo /camera/camera_info --once
```

图像优先 bgr8/rgb8；CameraInfo 的 K/D、宽高、frame_id 要与原始图像一致。时间戳必须是正确 ROS 采集时间，不能为零、硬件开机计数或另一台未对时电脑的时间。默认处理后的帧年龄超过 0.25 秒就失效，不要先放宽阈值掩盖排队。

实机 `use_sim_time=false`。rosbag 回放要所有相关节点一致设置 use_sim_time=true，并播放 /clock；不能用旧采集时间配当前系统时间。

做空间输出时，需由队伍提供真实 TF 链，例如：

```text
odom -> gimbal_link -> camera_link -> camera_optical_frame
```

运动部分由云台姿态反馈提供，静态部分由实测外参/URDF 提供。不要用全零变换冒充标定。验证命令：

```bash
ros2 run tf2_ros tf2_echo odom camera_optical_frame
```

还没有 TF 时，可在当天的空间配置中暂设 `enable_tf: false` 做相机相对输出实验；此时 `tf_ready=false`，输出角度不是绝对云台命令。无 CameraInfo 时先使用下面的单节点像素链路。

## 6. 两条链路分阶段启动

所有终端先 source `/opt/ros/humble/setup.bash` 和当前 workspace 的 `install/setup.bash`，并设置 ONNX Runtime 库路径。两条链路首次分别运行，避免同时推理挤占上位机资源。

### A. 先做像素调试

```bash
cd "$HOME/rm_buff_ws"
cp -n src/rm_buff_tracker/config/lab/buff_node_lab.yaml \
  src/rm_buff_tracker/config/match/buff_pixel_local.yaml
```

修改此文件的 color、mode、HSV 和模型路径；节点键保持 buff_tracker_node。然后：

```bash
ros2 launch rm_buff_tracker buff_node.launch.py \
  params_file:="$HOME/rm_buff_ws/src/rm_buff_tracker/config/match/buff_pixel_local.yaml" \
  image_topic:=/camera/image_raw
```

### B. 再做空间通道

```bash
cd "$HOME/rm_buff_ws"
cp -n src/rm_buff_tracker/config/lab/buff_spatial_channel.yaml \
  src/rm_buff_tracker/config/match/buff_spatial_local.yaml
```

修改下表参数；保留两个节点键 buff_detector_node 和 buff_spatial_tracker_node。

| 参数 | 上机如何填写 |
|---|---|
| color | 要识别的目标色 red/blue，不是默认取己方色 |
| mode | small 或 big；修改后重启节点 |
| onnx_model_path | models/best.onnx，或确认后的绝对路径 |
| HSV | 蓝色默认 90–130；红色可从 0–15 起调，但实际应按曝光测量，单改 color 不会切 HSV |
| freq | 实际处理频率的近似值；有时间戳的大符不靠它假设固定帧间隔 |
| delta_t | 基础额外提前量；先用示例 0.2 观察，不当作最终实测参数 |
| target_distance | 实测米数；PnP 关闭时输出是此固定距离近似 |
| enable_pnp | 初始 false；四个物理点、点序、标定确认后再开 |
| enable_compensation | 初始 false；实测弹速/延迟后再开，当前不是完整弹道求解 |
| yolo_refresh_interval | 空间默认 1，每帧复核；先测耗时再决定是否改变 |
| show_debug_window | 无桌面 false；用 debug_image 远程看图 |
| prediction_lead_time | 保持 0，避免 Tracker 重复预测 |

```bash
ros2 launch rm_buff_tracker buff_spatial_channel.launch.py \
  params_file:="$HOME/rm_buff_ws/src/rm_buff_tracker/config/match/buff_spatial_local.yaml" \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  target_frame:=odom
```

launch 中的 image_topic、camera_info_topic、target_frame 会覆盖对应 YAML 项，所以以启动参数为准。外部 params_file 不需要重新编译；修改后重启节点，当前不支持算法参数热更新。

## 7. 看什么才算跑起来

像素链路：

```bash
ros2 topic echo /buff_tracker_node/target --once
ros2 topic hz /buff_tracker_node/debug_image
```

空间链路：

```bash
ros2 topic echo /buff/detector/observation --once
ros2 topic echo /buff/tracker/target --once
ros2 topic hz /buff/tracker/target
ros2 run rqt_image_view rqt_image_view
```

先看目标框和方向，再看 tracking、prediction_ready、source、tf_ready、prediction_horizon。关闭 PnP 时 `PIXEL_RAY`/pnp_ready=false 是预期；TF 未接上时不能把输出当绝对角。大符需要足够样本与时间跨度，启动几帧不 ready 不等于程序损坏。

相机完全断流后应出现失效状态，不能持续沿用最后一个目标。TEMP_LOST 只是短时估计，prediction_ready=false；tracking=true 本身不是发射许可。

建议保存原始图像、CameraInfo、/tf、/tf_static 和 /buff 两个状态话题的 rosbag，以及当日 YAML、Git 提交号、相机曝光和模型散列。用采集时间对应未来观测评估预测，不能只看圆点是否平滑。

## 8. 常见部署问题

| 现象 | 优先检查 |
|---|---|
| 找不到 rm_buff_tracker | source 是否正确、ros2 pkg prefix 是否指向旧 workspace、是否只构建 standalone |
| 找不到 libonnxruntime.so | ONNXRUNTIME_DIR、LD_LIBRARY_PATH、SDK 的 CPU 架构及 ldd 输出 |
| 构建报缺头文件 | rosdep 是否成功，是否将 pip 包误当 C++ SDK |
| 模型解析失败 | 是否走了未验证的 OpenCV DNN 回退；建议使用 ORT 并核对模型散列 |
| tracking 一直 false | 图像/CameraInfo frame 和分辨率、时间戳年龄、目标色、模型标签、曝光 |
| YOLO 锁住又反复丢失 | HSV 范围与目标颜色、轮廓形态，先用录像查原因 |
| tf_ready=false | TF 链是否完整、方向正确、采集时刻是否有变换 |
| 大符总在重新拟合 | 丢帧间隔、时间倒序、目标切换、噪声导致超速拒绝 |
| 运行越来越慢 | 是否开了两条推理链、相机是否排队、CPU 耗时、调试图传输开销 |

## 9. 与电控的边界

已增加对 [步兵固件的大小符串口接口](BUFF_INFANTRY_SERIAL.md)；本仓库部署完成只代表视觉与桥的源码齐备，不代表已烧录或验证云台。桥默认 `enable_output: false`，但启动时仍独占串口并发布反馈 TF；切换装甲板/大小符必须停止旧串口节点。正式开输出前需要两边同步版本、真实相机标定/云台外参、PnP 实测物理点、实测延迟及弹速，并在无弹丸状态下验证方向与超时。大小符弹道、发射许可和命中效果尚未验证，不因串口联通而自动成立。

## 参考

- ROS Humble Ubuntu 安装：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- 官方文档源：https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst
- ONNX Runtime C/C++ 安装：https://onnxruntime.ai/docs/install/
- 固定 SDK 版本：https://github.com/microsoft/onnxruntime/releases/tag/v1.24.4

上述来源于 2026-09-22 核对。实际队伍如果使用不同系统、ROS 版本或 Jetson 型号，应先确认对应环境，不把本说明当成已验证兼容所有机器。
