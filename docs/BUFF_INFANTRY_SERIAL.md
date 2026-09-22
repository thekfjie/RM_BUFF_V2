# 大小符与步兵下位机联通（静态接口）

参考固件：[2026-RoboMaster-Infantry](https://github.com/An-OrangeMan/2026-RoboMaster-Infantry)，本地比对版本 `ccfa82c`。本仓库新增 `buff_serial_bridge`，配套固件修改以 [可应用补丁](../integration/infantry/0001-Add-guarded-BUFF-aim-command-to-infantry-controller.patch) 随仓库保存，对应本地固件分支提交 `2f21374`。两边必须使用同一版协议；只更新上位机不能直接在旧固件上启用输出。本次仅静态修改，未编译、通信测试、烧录或上机。

## 谁做什么

- 视觉 detector 预测一次未来扇叶点；tracker 将它转换到 `odom`，输出 `/buff/tracker/target`。
- `buff_serial_bridge` 独占 115200/8N1 串口，接收固件的 `0x5A` 姿态反馈（28 字节、CRC16），发布 `odom -> gimbal_link`，把未来点相对于云台轴原点转成绝对角，通过 `0xA6` 帧发给固件。
- 固件的 `0xA6` 分支只使用已算出的绝对 yaw/pitch，不做装甲板外推或重力解算；遥控器开关仍决定是否接管。原 `0xA5` 装甲板协议继续独立使用。同一串口不能同时运行装甲板 `rm_serial_driver` 与大小符桥。两种模式切换时停止旧节点、释放端口，再启动新节点。
- 不发送开火位，不新增自动发射。命令角不是命中保证：当前尚无大小符重力、枪口偏置、弹速实测与发射许可闭环；`prediction_ready` 仅表示目标预测可用。

## 字节协议

采用小端 IEEE-754 单精度、无结构体填充。CRC16 初始化 `0xffff`、反射多项式 `0x8408`，校验前 46 字节，末尾低字节先发送。与固件 `CRC8_CRC16.c` 算法一致。所有保留位和保留 float 必须置零。

| 偏移 | 大小 | 内容 |
|---|---:|---|
| 0 | 1 | `0xA6` |
| 1 | 1 | `valid`: 0 禁用、1 瞄准；不是开火许可 |
| 2 | 4 | 绝对 INS yaw，rad，正方向对应 `odom` 的 +y |
| 6 | 4 | 绝对 INS pitch，rad：目标仰角取负（现有 `0x5A` 反馈 pitch = -INS pitch） |
| 10 | 4 | 视觉已预测时域，相对图像采集时间，秒；固件只作范围校验 |
| 14 | 32 | 8 个预留 float，全部为 0 |
| 46 | 2 | CRC16，低字节在前 |

有效 `0xA6` 帧的角必须为有限值，yaw 在 `(-π,π)`，pitch 在 `(-π/2,π/2)`，时域在 `[0,0.8]` 秒。固件只在 CRC 正确、`valid=1`、距最后有效收帧小于 160 ms 且遥控器允许自瞄时采用命令。现有 `gimbal_task.c` 最终以遥控器两个模式拨杆都置于上位作为接管条件；鼠标右键判断随后会被拨杆判断覆盖，不能只按右键测试。`valid=0` 和断流会撤销目标有效状态；云台仍受原限幅约束（yaw π/8、pitch π/4）。固件收到完整的第 48 字节立刻校验 CRC，帧体中的 `0xA5`、`0xA6` 作为普通数据处理。

## 安装与启动顺序

准备 Ubuntu 22.04 / ROS 2 Humble、相机及正确 CameraInfo、相机与云台外参、`serial_driver` ROS 包、串口读写权限；ONNX Runtime 和编译环境见 [部署文档](DEPLOYMENT_UBUNTU22_HUMBLE.md)。空间算法仍需实测目标距离或 PnP 的四个物理点；安全起见桥默认 `require_pnp: true` 且 `enable_output: false`，未完成 PnP 时不会发有效命令。修改现场配置前先确认枪口和旋转中心相对 `gimbal_link` 的实测位置。当前固件反馈只给姿态、TF 平移固定为零，所以桥默认云台轴就是 `odom` 原点；若不一致，必须完善平移 TF，不能硬套角度。

```bash
source /opt/ros/humble/setup.bash
source ~/rm_buff_ws/install/setup.bash
ros2 pkg executables rm_buff_tracker    # 应看到 buff_serial_bridge
ros2 launch rm_buff_tracker buff_serial_bridge.launch.py
```

上面是串口读反馈、输出禁用的静态准备命令，不是本次已执行记录。启动前关闭 `rm_serial_driver`，检查 `device_name` 是否为实际稳定设备路径并确认权限。另一个终端先启动相机、静态外参和 `buff_spatial_channel.launch.py`，确保 `odom -> gimbal_link -> camera_optical_frame` 的时间戳、光轴和角方向都符合实测。桥只消费 `header.frame_id=odom` 且连续 `TRACKING`、`tf_ready`、`prediction_ready`、`camera_info_ready` 的新鲜点；默认还要求 `pnp_ready`。采集帧年龄不得超过 120 ms、反馈年龄不超过 120 ms，预测时域必须覆盖当前帧年龄且不超过 800 ms，否则发送 `valid=0`。

准备启用时，在自己的 YAML 副本中显式设置 `enable_output: true`；不要在未核对新固件及方向时修改仓库默认值。先在**无弹丸、受限云台行程**下确认 yaw/pitch 方向、反馈 TF、跨 ±π、超时撤销与遥控器切换，再测延迟和弹速，校正预测时域。任何旧固件、旧消息或旧桥二进制与新协议混用都不能当作已联通。

下位机仓库的两个 Keil 工程分别引用各自的 `application/`，队友需要确认实际烧录的是哪一套；本次修改同步了这两套源码，未修改编译产物。下位机远端仓库不在本机可推送权限内，需由固件仓库维护者审阅集成后再烧录。在固件仓库原始 `ccfa82c` 及干净工作区中应用：

```bash
cd ~/2026-RoboMaster-Infantry
git apply --check ~/rm_buff_ws/src/rm_buff_tracker/integration/infantry/0001-Add-guarded-BUFF-aim-command-to-infantry-controller.patch
git am ~/rm_buff_ws/src/rm_buff_tracker/integration/infantry/0001-Add-guarded-BUFF-aim-command-to-infantry-controller.patch
```

补丁保留原固件 GBK 编码与 CRLF 换行；若上游文件已有修改，先由固件维护者解决冲突，不要强制覆盖。
