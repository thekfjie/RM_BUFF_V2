# Run Configs

`standalone_*.yaml` 是离线入口 `predict_example_main` 的运行配置。

以后常改的东西，优先改这里，不用进 `cpp`：

- `pythonRoot`
  指向 `E:/RM/rm_vision` 这一层根目录
- `parameterPath`
  指向样例目录下的 `parameter.yaml`
- `videoPath`
  可选，填了就覆盖 `parameter.yaml` 里的视频路径
- `color`
  `red` 或 `blue`
- `mode`
  `small` 或 `big`
- `freq`
  算法频率
- `deltaT`
  预测提前量
- `imshow`
  `1` 显示窗口，`0` 不显示
- `detector`
  `hsv` 或 `yolo`
- `onnxModelPath`
  YOLO 模型路径
- `yoloRelockIntervalFrames`
  丢失后，YOLO 两次重锁尝试之间至少隔多少帧
- `yoloRelockAfterMisses`
  连续丢多少帧后才开始重锁
- `tune`
  `1` 表示调参模式
- `tuneFrame`
  固定调参帧；填 `-1` 表示启动后自己选
- `rBox` / `fanBox`
  手动指定初始化 ROI，格式是 `[x, y, w, h]`

推荐用法：

- 双击 `scripts/` 里的 bat
- 或直接运行：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe --config E:\RM\rm_vision\RM_BUFF_V2.1\config\standalone_yolo_small_blue.yaml
```

如果你要新建一套场景配置，直接复制：

- `run_config_template.yaml`

再改成你自己的文件名即可。
