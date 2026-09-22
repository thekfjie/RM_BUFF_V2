# 大小符视觉接口与集成约定

更新：2026-09-22。大小符视觉预测与串口角度适配方案见 [步兵下位机联通接口](BUFF_INFANTRY_SERIAL.md)；大小符弹道与发射归属尚待实测约定。当前修改尚未编译或运行验证。

## 模块位置和职责

ROS2 包名为 `rm_buff_tracker`，放在上位机工作空间 `~/rm_ws/src/rm_buff_tracker`，与装甲板自瞄并列。

```text
相机 image_raw + CameraInfo + 采集时刻 TF
  -> buff_detector_node：选靶、相位估计、一次未来位置预测
  -> BuffObservation
  -> buff_tracker_node：坐标转换、观测速度、超时状态
  -> BuffTargetState
  -> buff_serial_bridge（装甲板/大小符两种输入，一个串口拥有者，默认禁用控制输出）
  -> 步兵下位机 0xA5 装甲板 / 0xA6 大小符接口
```

视觉负责识别、目标关联、相位/转速与有效性。系统集成负责相机/TF/时钟、模式仲裁和协议。电控负责反馈、执行和通信超时。预测与弹道计算的归属待双方确认：每个环节只执行一次。

已有 `rm_serial_driver` 订阅的是装甲板 `auto_aim_interfaces/Target`。不能把大小符消息 remap 过去，也不能把像素、扇叶相位或已经预测的点冒充装甲板旋转状态；统一串口桥分别订阅两个话题，以 `active_mode=armor|buff` 运行时选择一条路径。部署统一桥时只需停止旧驱动一次，之后切模式不必释放串口。

## 入口选择

| 入口 | 用途 | 行为与限制 |
|---|---|---|
| `buff_node` | 兼容像素调试 | YOLO 初始化/丢失重锁、HSV 几何跟踪。HSV 不证明当前激活状态；输出像素，不是控制角 |
| `buff_spatial_channel.launch.py` | 空间接口实验 | 默认每帧 YOLO 复核可选类别并给出关键点，Tracker 处理 TF 和状态 |

空间 YAML 的 `yolo_refresh_interval` 改为 1。这是为了逐帧核对目标状态，不代表已测得目标机能达到 50 FPS。必须测实际耗时；若提高间隔，间隔内 HSV 帧不会重新确认激活状态，不能默认具备击打有效性。

颜色参数表示**目标颜色**。本地模型元数据为 `{0: RR, 1: RW, 2: BR, 3: BW}`，关键点形状 `[5,2]`；按本仓库标签约定只选择 0/2，禁止向 1/3 命中类别回退。模型元数据只证明类别名称，灯效与目标状态对应仍需用实拍确认。不得套用 Hello World 模型的类别编号或九关键点顺序。

## 时间和坐标

- 图像和 TF 必须使用一致的 ROS 时间；`header.stamp` 是图像采集时间，单位按 ROS 标准。零时间、未来时间、超过最大年龄的图像判无效。
- 单位统一为米、秒、弧度；速度分别为 m/s、rad/s。
- 相机坐标采用 optical：x 向右、y 向下、z 向前；相机相对 yaw 向右为正、pitch 向上为正。
- 配置的 `target_frame` 必须采用 x 前/y 左/z 上的约定。TF 成功后，位置和角度都属于该 frame；yaw 向左为正。TF 失败时保留相机坐标且 `tf_ready=false`。
- 对相机在右方的目标，optical 相对 yaw 为正，转换到同原点 FLU 后 yaw 为负。这两个量不能混用。
- 输出角度是从消息坐标系原点指向目标的视线角。还没有处理枪口偏置和重力弹道，不能直接称为最终电机命令。

## BuffObservation（Detector → Tracker）

| 字段 | 定义 |
|---|---|
| `camera_position` | 采集时刻的观测扇叶中心，不能再解释为未来点 |
| `camera_aim_position` | 采集时刻加 `prediction_horizon` 的未来目标点 |
| `yaw/pitch` | 当前观测的相机相对角 |
| `camera_pose` | 当帧 PnP 成功时的测量位姿；其他时候只填射线位置和单位姿态，不应据此认为已测得姿态 |
| `phase/raw_phase` | 连续转子相位/所选扇叶原始图像角；小符 phase 含 KF 校正，换叶不应作为转子转动 |
| `phase_velocity` | 转子相位速度，不是云台 yaw 速度 |
| `source` | `PIXEL_RAY` 固定距离近似、`PNP` 当帧度量深度、`PNP_HELD` 有效期内保持、`NONE` 无效 |
| `depth_age` | 度量深度距当前图像的时间；固定距离为 -1 |
| `pnp_ready` | 当帧或 TTL 内的度量深度可用，不等于当帧测到了完整姿态 |
| `tracking/prediction_ready` | 当前观测有效/相位预测有效；均不代表发射许可 |

开启 PnP 时必须给出**恰好四个**扇叶平面物理点，顺序对应图像关键点 0、1、3、4，单位米、原点取打击板中心；第 2 个关键点是 R 中心，不混入此四点。不得直接采用对方九关键点模型的点序。

PnP 失败时最多保持 `pnp_depth_max_age` 秒；超过 TTL 输出失效，绝不偷偷切回固定 7m。关闭 PnP 才使用明确配置的 `target_distance`。标定分辨率、frame_id、畸变模型必须匹配；畸变模型当前支持 plumb_bob/rational_polynomial，不支持鱼眼。

## BuffTargetState（Tracker → 集成层）

| 字段 | 定义 |
|---|---|
| `header` | 观测采集时间、实际输出坐标系 |
| `position/velocity` | 当前观测位置/连续同一目标的差分速度，换叶或坐标/深度类型改变时速度重置 |
| `predicted_position` | Detector 已完成预测的未来点；Tracker 不再追加一次线性预测 |
| `prediction_horizon` | 未来点相对于 header.stamp 的时域 |
| `yaw/pitch` | 当前观测在实际输出坐标系中的视线角 |
| `predicted_yaw/predicted_pitch` | 从未来点重新求得的视线角 |
| `tf_ready` | 是否转换到了配置的 target_frame；false 时只能按相机相对状态消费 |
| `tracker_state` | `DETECTING` 新轨迹或换叶、`TRACKING` 连续轨迹、`TEMP_LOST` 短时估计、`LOST` 已失效 |

`TEMP_LOST` 可以带 `tracking=true` 供显示或后续重新关联，但 `prediction_ready=false`、`pnp_ready=false`，不是有效预测命令。完全断流由定时器触发失效；发布 LOST 时清除旧速度状态。消费者也必须自行检查收包时间与有效期，不能依赖上游一定能发最后一包。

`prediction_lead_time` 必须为 0；若需要改变提前量，在 Detector 改完整预测时域。

单节点 `BuffTarget` 新增 `prediction_horizon`、`phase_correction`。其 x/y 始终为像素。裸 `PointStamped prediction` 只作可视化兼容输出，没有足够的控制有效性信息。

## 延迟和预测归属

实时 ROS 链路的时域为：

```text
delta_t + 当前帧年龄
    +（启用 enable_compensation 时：距离/弹速 + 通信延迟 + 云台延迟 + 额外延迟）
```

空间通道用当前可用 PnP 深度更新飞行时间，否则使用明确配置的固定距离。`delta_t` 是额外提前量，不应再包含已经单列的延迟。离线入口没有真实发布时刻，只使用配置的预测时域。

小符采用固定转速的标量相位 KF；大符采用正弦速度解析积分。`enable_compensation` 只负责时域补偿，**没有**重力/空气阻力/枪口偏置弹道解算。队伍若选择下位机预测，应重新约定发送原始相位模型或观测状态的接口；目前消息没有完整 a/ω/φ/b 参数，因此不能宣称已支持下位机完整大符预测。

## 尚需实车确认

当前相位与圆周预测仍在图像平面进行。PnP 深度并不等于完整三维转子平面预测；斜视、车体移动和云台运动误差仍需验证。当前没有最新赛季完整灯效状态机、发射授权或电控协议。

本次修改消息定义后，所有消费者必须重新生成 ROS 接口并重新构建。历史 `install/` 可执行文件不能代表当前源码；不要把旧消息二进制与新节点混用。参数当前按启动时配置使用，修改后重启，不支持在线切换比赛模式。

## 后续验证顺序（本次未执行）

1. 目标 ROS 环境编译，确认三个节点及消息生成成功。
2. 运行三个核心测试：相位/拟合、几何、多个候选的跟踪回归。
3. 录像覆盖红蓝、双向、跨 ±π、换叶、已命中类别、双目标、暂失/断流、时间倒退。
4. 固定机位确认预测时域；记录采集/发布时间及检测、PnP、拟合耗时。
5. 验证 TF 正负号、PnP 与保持/过期状态；再接入团队确认后的适配器。

验收依据是带时间戳的未来位置误差和状态正确性，而不只是预测点平滑。云台响应和命中效果需要双方同步记录命令与反馈后判断。
