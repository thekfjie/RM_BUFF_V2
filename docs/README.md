# RM_BUFF_V2

一个基于 C++/OpenCV 的 **RoboMaster 能量机关（大符 / 小符）离线复刻与调参仓库**。

这个项目的目标不是延续旧的失败 C++ 语义，而是尽量贴近原版 Python 项目
`RM_Buff_Tracker_GUT-main` 中 `predict_example_main.py` / `set_parameter.py` 的行为，先把**离线复刻、调参、追踪、预测**这条链跑通。

---

## 当前状态

### 已完成

- 以 `predict_example_main.py` 为基线的 C++ 入口
- 每个 example 使用自己的 `parameter.yaml`
- `start` 帧后手动选择 `R` 与扇叶 ROI
- `tracker.update -> angleObserver -> predictor -> draw -> imshow`
- 保留原版 **fail-fast** 语义
- 小符路径已验证：
  - `8_dark_blue_small`
  - `9_dark_red_small`
- 大符路径已补齐主流程对齐（含 `bigPredictor`）
- C++ 版调参面板（`--tune`）
- 结束后角度文本输出 + 角度曲线窗口

### 还没做成“比赛可直接上场版”

目前这套更像：

- 离线复刻工具
- 调参工具
- 算法验证工具

还**不是**完整比赛上场版本，因为还缺：

- 自动初始化（不再手圈 `R` / 扇叶）
- 丢失后的自动恢复 / 重锁
- 实时相机接入与工程化状态机
- 与云台 / 发射控制链路联动

---

## 目录说明

```text
RM_BUFF_V2.1/
├─ src/                  # 核心 C++ 代码
├─ CMakeLists.txt        # CMake 构建入口
├─ CMakePresets.json     # VS2022 预设
├─ .clangd               # 本地 LSP / compile flags
├─ README.md             # 本说明
├─ 指令说明.txt           # 常用命令小抄
└─ AI_HANDOFF_RESEARCH.md
```

---

## 依赖与前提

### 1. OpenCV

当前默认构建环境是 Windows + VS2022 + OpenCV。

运行前请先把 OpenCV DLL 放进 PATH：

```bat
set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH%
```

### 2. Python 原版项目 / 示例数据

默认情况下，这个仓库会去读取原版 Python 项目的 example 配置和视频。

默认假设原版项目位于：

```text
../RM_Buff_Tracker_GUT-main
```

也就是例如：

```text
E:/RM/rm_vision/RM_BUFF_V2.1
E:/RM/rm_vision/RM_Buff_Tracker_GUT-main
```

如果你的 Python 原版项目不在这个位置，可以用：

```bat
--python-root <path>
```

---

## 构建

最省事的方式：直接双击项目根目录里的 `构建Release.bat`。

另外，`小符蓝.bat`、`小符红.bat`、`大符蓝.bat`、`大符红.bat`、`调参模式.bat` 在发现 exe 不存在时，也会先自动调用构建脚本。

这些脚本已经直接绑定到 `E:\RM\rm_vision\examples` 里的对应 `parameter.yaml`，不再需要手动 `--prompt-path` 选文件。

```bat
cd /d E:\RM\rm_vision\RM_BUFF_V2.1
cmake --preset vs2022-release
cmake --build build/vs2022-release --config Release
```

可执行文件路径：

```text
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe
```

---

## 最常用运行方式

### 1) 默认运行（= `8_dark_blue_small`）

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe
```

默认等价于：

- parameter: `examples/example_for_prediction/8_dark_blue_small/parameter.yaml`
- color: `blue`
- mode: `small`
- freq: `50`
- deltaT: `0.2`

### 2) 交互输入路径

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe --prompt-path
```

会提示输入：

- `parameter.yaml` 路径，或
- 视频路径（若同目录有 `parameter.yaml` 会自动识别）

### 3) 直接传 `parameter.yaml`

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe ^
  "E:\RM\rm_vision\RM_Buff_Tracker_GUT-main\examples\example_for_prediction\9_dark_red_small\parameter.yaml" ^
  --color red ^
  --mode small
```

### 4) 直接传视频路径

如果视频同目录下存在 `parameter.yaml` / `parameter.yml`，程序会自动推断并使用：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe ^
  --video "E:\RM\rm_vision\RM_Buff_Tracker_GUT-main\examples\example_for_prediction\8_dark_blue_small\dark_blue_small.MP4"
```

---

## 调参模式（C++ 版 `set_parameter`）

### 启动方式

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe --tune
```

启动后会在 CMD 中提示：

- `Tuner video path (press ENTER to use default)`
- `Initial preview speed (0.25~8.0, default 1.0)`

也可以显式指定：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe ^
  --tune ^
  "E:\RM\rm_vision\RM_Buff_Tracker_GUT-main\examples\example_for_prediction\10_dark_red_small_near\parameter.yaml"
```

### 调参流程

1. 在 CMD 中输入视频路径（或回车使用默认）
2. 输入预览速度
3. 视频自动预览
4. `SPACE` 冻结当前帧进入调参
5. 选择：
   - `roi`：中心 `R`
   - `roi2`：待击打扇叶
6. 调整滑条：
   - `LH`
   - `LS`
   - `LV`
   - `UH`
   - `US`
   - `UV`
   - `kernel`
   - `outside`
   - `inside`
7. `q` 保存回当前 `parameter.yaml`
8. `Esc` 取消不保存

### 额外说明

- `Tracking / frame / mask / res` 窗口都支持自由缩放
- 也支持直接跳到指定帧调参：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe ^
  --tune ^
  --tune-frame 245 ^
  "E:\RM\rm_vision\RM_Buff_Tracker_GUT-main\examples\example_for_prediction\8_dark_blue_small\parameter.yaml"
```

---

## 大符 / 小符最简命令小抄

如果你不想自己复制命令，直接双击项目根目录里的这些文件即可：

- `小符蓝.bat`
- `小符红.bat`
- `大符蓝.bat`
- `大符红.bat`
- `调参模式.bat`

这些脚本会自动设置 OpenCV `PATH`；如果还没编译出 exe，会先自动构建；并直接指向 `E:\RM\rm_vision\examples` 下的对应样例。

- `小符蓝.bat` -> `E:\RM\rm_vision\examples\example_for_prediction\8_dark_blue_small\parameter.yaml`
- `小符红.bat` -> `E:\RM\rm_vision\examples\example_for_prediction\9_dark_red_small\parameter.yaml`
- `大符蓝.bat` -> `E:\RM\rm_vision\examples\example_for_prediction\6_dark_blue_big\parameter.yaml`
- `大符红.bat` -> `E:\RM\rm_vision\examples\example_for_prediction\7_dark_red_big\parameter.yaml`
- `调参模式.bat` -> `E:\RM\rm_vision\examples\example_for_prediction\10_dark_red_small_near\parameter.yaml`

### 小符蓝

```bat
set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH% && "E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe" --python-root "E:\RM\rm_vision" --parameter "E:\RM\rm_vision\examples\example_for_prediction\8_dark_blue_small\parameter.yaml" --mode small --color blue
```

### 小符红

```bat
set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH% && "E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe" --python-root "E:\RM\rm_vision" --parameter "E:\RM\rm_vision\examples\example_for_prediction\9_dark_red_small\parameter.yaml" --mode small --color red
```

### 大符蓝

```bat
set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH% && "E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe" --python-root "E:\RM\rm_vision" --parameter "E:\RM\rm_vision\examples\example_for_prediction\6_dark_blue_big\parameter.yaml" --mode big --color blue
```

### 大符红

```bat
set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH% && "E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe" --python-root "E:\RM\rm_vision" --parameter "E:\RM\rm_vision\examples\example_for_prediction\7_dark_red_big\parameter.yaml" --mode big --color red
```

### 调参模式

```bat
set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH% && "E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe" --python-root "E:\RM\rm_vision" --tune --parameter "E:\RM\rm_vision\examples\example_for_prediction\10_dark_red_small_near\parameter.yaml"
```

---

## 额外参数

- `--python-root <path>`：Python 原版项目根目录
- `--prompt-path`：普通运行时交互输入场景路径
- `[parameter.yaml|video]`：位置参数，直接传入场景路径
- `--video <path>`：显式指定视频路径，并尝试自动推断同目录 `parameter.yaml`
- `--parameter <path>`：显式指定 `parameter.yaml`
- `--tune`：进入调参面板模式
- `--tune-frame <int>`：直接跳到指定帧调参
- `--color blue|red`
- `--mode small|big`
- `--freq <int>`
- `--deltaT <float>`
- `--imshow 0|1`
- `--r-box x,y,w,h`：跳过 `selectROI("roi", ...)`
- `--fan-box x,y,w,h`：跳过 `selectROI("roi2", ...)`

---

## 当前输出

普通运行结束后会：

1. 写出 angle 文本文件（例如 `8_dark_blue_small.txt`）
2. 在普通显示模式下弹出角度曲线窗口

如果使用：

```bat
--imshow 0
```

则会跳过窗口显示与角度图，便于自动化验证。

---

## 备注

- 这个仓库当前重点仍是**离线复刻与调参**，不是最终比赛上场工程
- 若后续继续往比赛版推进，下一优先级通常是：
  1. 自动初始化
  2. 自动恢复/重锁
  3. 实时相机输入
  4. 云台/发射联动
