# AI Handoff Research

## 1. 这份文档的用途
这份文档是给新对话里的 AI 的完整交接材料，目标是让它不需要重新翻旧对话，就能理解当前项目背景、原版 Python 算法、当前 C++ 偏差、已经踩过的坑、编译运行方式，以及下一步最合理的工作路线。

当前结论非常明确：
- 现在的 C++ 程序已经不是原版 Python 的 1:1 复刻。
- 它混入了很多额外语义（统一参数、自动恢复、宽松接受、复杂 UI/日志/播放器逻辑）。
- 所以现在最合理的路线不是继续在现有 C++ 上局部打补丁，而是回到原版 Python，先做一版行为尽量一致的 Python 复刻版，确认算法和行为基线，再考虑增强或重写 C++。

## 2. 项目路径
### 原版 Python（算法真源）
- E:/RM/rm_vision/RM_Buff_Tracker_GUT-main

### 当前 C++ 项目（问题版 / 参考版）
- E:/RM/rm_vision/rm_rune_offline_test

### 新交接目录
- E:/RM/rm_vision/RM_Buff_Tracker_GUT_cpp

## 3. 原版 Python 以什么为准
必须以这个入口为准，不要以 main.py 为准：
- E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/predict_example_main.py

关键源码：
- 检测/跟踪：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/utils/buffTracker.py
- 预测：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/utils/angleProcessor.py
- 数据平滑：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/utils/dataProcessor.py
- 参数加载：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/utils/parameterUtils.py
- README：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/readme.md
- 预测说明：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/predict.md

## 4. 原版 Python 的真实行为
### 4.1 fail-fast，不是 fail-open
原版每帧会做：tracker.update(frame, True)。一旦失败，就直接报错退出，不做自动恢复。

### 4.2 每个视频用自己的 parameter.yaml
例如：E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/examples/example_for_prediction/8_dark_blue_small/parameter.yaml
里面包含 HSV、MayBeTarget 三个阈值、outsideRate、insideRate、kernel、video relative path、start。
当前 C++ 最大偏差之一：不管播哪个视频，都统一读 E:/RM/rm_vision/rm_rune_offline_test/current_tuning_params.yaml。

### 4.3 原版主检测链
1. HSV 二值化
2. __GetAlternateBoxs 找中心 R 候选
3. 用 CIoU 从候选里挑出最像上一帧 R_Box 的那个
4. 没找到就直接失败
5. 以 R_Box 为圆心画甜甜圈遮罩
6. 在遮罩后的 mask 上找亮扇叶
7. 用上一帧扇叶位置平移后做 IoU 关联
8. 更新状态机（target / unlighted / shot）
9. angleObserver 连续化角度
10. smallPredictor / bigPredictor 预测

### 4.4 原版核心前提
R 不能丢。这不是建议，而是算法成立的前提。

## 5. 当前 C++ 为什么又卡又不准
### 5.1 不准的根因
- 统一参数文件，破坏了 example 逐个视频调好的前提。
- R 接受策略被改得过宽：kMinAcceptedRScore=-0.70、r_hold_frames=10、候选数<=1时直接接受。
- fan-empty 时仍可能返回 success，失败语义从 fail-fast 被改成 fail-open。
- 状态机 / target id / reset 逻辑已经偏离原版。
- 为了看起来稳加入的自动恢复，反而把错误锁定放大。

### 5.2 卡顿的根因
检测本身通常只要几毫秒，卡主要来自：预览模式、暂停恢复、倍速和 frame skipping、每帧日志 flush、多窗口 imshow/waitKey、频繁 clone。
也就是说，Python 的流畅很大程度上来自它是一个很薄的 demo loop，而 C++ 当前程序已经是播放器/调试器/检测器的混合体。

## 6. 当前 C++ 的编译 / 运行方式
项目一直是纯命令行，不依赖 VSCode。

### 6.1 配置与编译
项目有 E:/RM/rm_vision/rm_rune_offline_test/CMakePresets.json
主要 preset：vs2022-release、vs2022-debug
命令：
cd /d E:/RM/rm_vision/rm_rune_offline_test
cmake --preset vs2022-release
cmake --build --preset vs2022-release

### 6.2 产物位置
- build/vs2022-release/Release/rune_detector_test.exe
- build/vs2022-release/Release/rune_tuner.exe

### 6.3 运行前 OpenCV DLL
先在 cmd 里执行：
set PATH=D:/develop/opencv_windows/opencv/build/x64/vc15/bin;<YOUR_EXISTING_PATH>
然后运行：
E:/RM/rm_vision/rm_rune_offline_test/build/vs2022-release/Release/rune_detector_test.exe
### 6.4 编译遇到过的问题
- 有过一次 mspdbcore.dll 相关异常，但后来用 preset 流程正常编译通过。
- 结论：以 cmake --preset / cmake --build --preset 为准，不要自己乱拼 VS 命令。

## 7. 已经踩过的坑
- 参数文件曾误落到 C:/Users/Kfjie，后来固定回项目根 current_tuning_params.yaml。
- 主程序和调参窗口曾混在一起，后来拆成 rune_detector_test / rune_tuner。
- 空图 imshow 崩溃，修过。
- 暂停逻辑修歪过一次，后来改成冻结最后一张带叠加的画面。
- 日志曾一直覆盖，后来改成 logs 目录、时间戳+视频名、最多保留 5 份。
- 把甜甜圈遮罩前移到找 R 候选之前，曾把默认视频搞坏。

## 8. 当前最重要的诊断结论
### 8.1 默认视频和 bright 视频暴露的是同一类问题
- 有时候 r_candidates == 0，但程序仍继续成功。
- 有时候 r_candidates == 1 且分数很差，也被直接接受。
- 结果：R 会锁到旋转扇叶中心、地面亮块/杂物，蓝框/target 乱飘。

### 8.2 bright 视频后半段
- 视频：E:/RM/rm_vision/examples/1_8mm_red_bright/8mm_red_bright.mp4
- 问题：R 会锁到旋转装甲板/扇叶中心。
- 不是预测点先飞，而是 R_Box 本身先错。

### 8.3 默认视频也会被错误修复搞坏
- 视频：E:/RM/rm_vision/examples/5_old_buff_red_dark/old_buff_red_dark.avi
- 曾经某轮修复直接破坏了默认视频稳定性。
- 说明不能盲目增强鲁棒性，必须先保证对原版 example 的行为一致。

## 9. 建议新 AI 的工作路线
### 第一阶段：只做原版行为复刻
- 不要先做漂亮 UI
- 不要先做自动恢复
- 不要先做 unified tuning
- 不要先做复杂播放器
- 直接复刻 predict_example_main.py 的入口行为、buffTracker.py 的检测/状态机、angleProcessor.py 的预测。

### 第二阶段：只加最少量增强
1. 可关日志
2. 暂停冻结当前叠加帧
3. 支持切换不同 example 参数
4. 再考虑轻量播放器功能

### 第三阶段：明确不要先做的事
- 不要先统一成一份全局参数文件
- 不要先加自动 reset / auto recovery
- 不要先加“候选少就直接接受”的宽松逻辑
- 不要先把失败语义改成 success
- 不要先优化 UI 胜过算法正确性。

## 10. 建议先验证的视频
### 原版 Python 推荐先跑
- E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/examples/example_for_prediction/8_dark_blue_small/dark_blue_small.MP4
- E:/RM/rm_vision/RM_Buff_Tracker_GUT-main/examples/example_for_prediction/9_dark_red_small/dark_red_small.MP4

### C++ 对照常用视频
- 默认：E:/RM/rm_vision/examples/5_old_buff_red_dark/old_buff_red_dark.avi
- 问题明显：E:/RM/rm_vision/examples/1_8mm_red_bright/8mm_red_bright.mp4

## 11. 可直接贴给新 AI 的起手说明
先不要继续在 C++ 上打补丁。
我要一版尽量贴近 RM_Buff_Tracker_GUT-main 原版行为的 Python 复刻版。
以 predict_example_main.py 为标准入口，不要以 main.py 为准。
重点先保留原版的 fail-fast 语义、每个 example 独立 parameter.yaml、R 不能丢这个前提。
不要先做 unified tuning、不要先做自动恢复、不要先做复杂 UI。
做一个最薄的 runner：读视频、到 start 选 ROI、update、predict、draw、imshow。

## 12. 核心一句话
不要把当前 C++ 程序当原版算法本体。它已经混进了太多额外语义。新 AI 应该回到原版 Python，先复刻行为，再谈增强。

## 13. 已实锤踩过的坑（这些优先级高于主观推断）
这一节只记录已经真的犯过、并且确认过的问题。
这些内容比上面的阶段性判断更稳，因为它们不是“推测会出错”，而是“已经出过错”。

### 13.1 参数文件路径跟着 cwd 乱跑
- 之前把 `current_tuning_params.yaml` 写成了相对路径，结果程序从哪里启动，就保存到哪里。
- 实际出现过误保存到：`C:/Users/Kfjie/current_tuning_params.yaml`
- 后来已改成：固定保存到项目根目录。
- 结论：任何配置文件路径都不要默认依赖当前工作目录。

### 13.2 误在用户目录根下留下垃圾文件
- 除了 `current_tuning_params.yaml`，还误留下过 `C:/Users/Kfjie/current_tuning_params.txt`
- 这类文件已经清掉。
- 结论：不要把调参/缓存/导出文件随手丢到用户根目录；必须明确落点。

### 13.3 主程序和调参程序混在一起会把问题越调越乱
- 之前把 runtime 和 tuning 混到一个程序里。
- 结果是：主程序既负责检测预测，又负责看 `mask/res` 和滑条调参，运行视图与调参视图语义混淆。
- 后来才拆成：
  - `rune_detector_test.exe`：运行程序
  - `rune_tuner.exe`：调参程序
- 结论：runtime 和 tuner 必须分离，尤其不要让 runtime 的显示结果冒充 raw tuning truth。

### 13.4 OpenCV 会因为空图 `imshow` 直接闪退
- 已发生过：reset 后本轮继续往下走，`debug_frame` 为空，喂给 `imshow` 后直接断言崩溃。
- 报错形式类似：`size.width>0 && size.height>0`
- 结论：所有 `imshow` 前都应该明确 guard 图像是否为空。

### 13.5 重播 / reset 时如果不清状态，会出现第二轮越跑越乱
- 已发生过：视频回到第 0 帧时，只重置了 `cap`，没有重置 tracker / observer / predictor / history。
- 表现是：第一轮还能跑，第二轮状态继承上一轮，越来越乱，甚至完全不再锁定。
- 结论：只要存在 replay / reset，就必须明确哪些状态要清、哪些状态要保留。

### 13.6 reset 后计时器会制造假的高 FPS 观感
- 已发生过：reset 后 `RunFPS` 出现 200/300+ 的夸张数字。
- 这不是算法真的变快，而是计时器重置后第一帧样本异常。
- 结论：性能指标必须防止 reset 后第一帧污染统计。

### 13.7 `waitKey(1)` 会让“看起来很快”误导判断
- 已发生过：预览阶段用近似 `waitKey(1)`，结果视频看起来天然快于原视频。
- 后来才确认：这不是算法快，而是显示节奏逻辑不对。
- 结论：判断“算法是否卡顿”之前，先确认显示/播放器节奏是否正确。

### 13.8 OpenCV DLL 不是编译问题，而是运行时 PATH 问题
- 已发生过：`opencv_world412.dll` 缺失，双击 exe 无法运行。
- 实际解决方式是给 cmd 设置：
  - `set PATH=D:\develop\opencv_windows\opencv\build\x64\vc15\bin;%PATH%`
- 结论：遇到 DLL 缺失时，先查 PATH 和运行库，不要误判成代码坏了。

### 13.9 Windows 头文件会污染 `small` 宏
- 已发生过：包含 `windows.h` 后，`MoveMode::small` 被宏污染，编译直接炸。
- 后来通过 `#undef small` 处理。
- 结论：Windows 平台下，`windows.h` 相关宏污染必须小心。

### 13.10 手机录屏/截图里的“看起来卡”“点重叠”不一定是算法错
- 已发生过：`current prediction` / `previous prediction` 很接近时，看起来像一团红绿字重叠，容易误判成预测严重错误。
- 也发生过：录屏里看起来“卡”，但实际 `Detect ms` 很低，是显示层节奏问题。
- 结论：不要只凭录屏观感下结论，必须结合状态字、检测耗时、框是否真的漂移来判断。

### 13.11 看视频内容时，不能假设 AI 能直接读 mp4 本体
- 已发生过：直接读取 `mp4` 会失败，后面是通过 `ffmpeg` 抽帧后逐帧分析解决的。
- 结论：如果新 AI 要分析视频，应优先用 `ffprobe/ffmpeg` 提取关键帧，不要假设自己能直接“看视频”。

### 13.12 不要随手清理 `C:/Users/Kfjie` 顶层目录
- 已确认：那个目录下绝大多数是系统目录、系统链接或软件配置，不是“乱文件”。
- 真正误落的只有我们自己的参数文件。
- 结论：遇到用户目录顶层“看起来很乱”，只能精确清理已知误文件，不能泛删。

