# Config Layout

`config/` 现在按三层管理：

- `templates/`
  放空模板，不直接上场跑
- `lab/`
  放实验室验证过的基础预设
- `match/`
  放比赛当天复制出来再微调的副本

推荐工作流：

1. 先从 `templates/` 复制一份新配置
2. 在实验室调好后，存到 `lab/`
3. 比赛前从 `lab/` 复制到 `match/`
4. 比赛现场只改 `match/` 里的文件

## 各目录用途

### `templates/`

- `standalone_run_template.yaml`
  离线运行模板
- `buff_node_template.yaml`
  ROS2 参数模板
- `default_params_template.yaml`
  原始 `parameter.yaml` 模板

### `lab/`

这里放你平时已经验证过的基线配置，比如：

- `standalone_yolo_small_blue.yaml`
- `standalone_yolo_small_red.yaml`
- `standalone_yolo_big_blue.yaml`
- `standalone_yolo_big_red.yaml`
- `buff_node_lab.yaml`

### `match/`

这里不放固定答案，专门留给比赛当天复制和微调。

例如你可以这样复制：

```bat
copy E:\RM\rm_vision\RM_BUFF_V2.1\config\lab\standalone_yolo_small_blue.yaml E:\RM\rm_vision\RM_BUFF_V2.1\config\match\standalone_yolo_small_blue_match.yaml
```

## 现在一般改哪里

### 离线跑视频

优先改：

- `config/lab/standalone_*.yaml`
- 比赛时改 `config/match/*.yaml`

运行示例：

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\build\vs2022-release\Release\predict_example_main.exe --config E:\RM\rm_vision\RM_BUFF_V2.1\config\lab\standalone_yolo_small_blue.yaml
```

### ROS2 上机

优先改：

- `config/lab/buff_node_lab.yaml`
- 比赛时复制到 `config/match/` 再改

launch 默认参数文件已经指向：

- `config/lab/buff_node_lab.yaml`

## 常改字段

以后你通常只需要改这些：

- `parameterPath`
- `videoPath`
- `color`
- `mode`
- `freq`
- `deltaT`
- `big_fit_*`（大符时间戳拟合、ω 搜索、内点和断流阈值）
- `detector`
- `onnxModelPath`
- `yoloRelockIntervalFrames`
- `yoloRelockAfterMisses`
- `tune`
- `rBox`
- `fanBox`

这些字段在各 YAML 里都已经带中文注释。
