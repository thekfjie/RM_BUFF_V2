# YOLO Test Tools

This folder contains pure-YOLO debug tools for checking whether an ONNX model
is actually detecting the buff target on both full videos and single images.

## Build target

`yolo_video_test`
`yolo_image_test`
`camera_geometry_test`
`angle_predictor_test`

The executable is produced at:

`build/vs2022-release/Release/yolo_video_test.exe`
`build/vs2022-release/Release/yolo_image_test.exe`

`camera_geometry_test` uses synthetic projections to cover pixel rays and
IPPE candidate continuity. `angle_predictor_test` feeds an irregular timestamp
sine-motion sequence and checks model fitting, analytic integration and reset
after an observation gap.

## Single Image

Use `run_yolo_image_test.bat` to inspect one image in detail:

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\tests\run_yolo_image_test.bat ^
  --image E:\RM\buff_dataset\images\train\004b0247-558.jpg ^
  --show
```

If a matching label file exists, the tool auto-loads it and writes:

- `*__image_compare.jpg`: left ground truth, right prediction
- `*__image_compare__pred.jpg`: prediction only
- `*__image_compare.txt`: per-object class, bbox, 5 keypoints, derived `R box`

## Quick Start

Use one of the preset batch files in this folder to export a full annotated
video directly:

- `run_small_blue_full.bat`
- `run_small_red_full.bat`
- `run_big_blue_full.bat`
- `run_big_red_full.bat`

They all call `run_yolo_video_test.bat`, which uses:

- model: `models/best.onnx`
- exporter: `build/vs2022-release/Release/yolo_video_test.exe`
- output dir: `tests/output/`

The preset scripts now also pass `--show`, so double-clicking them opens a
live preview window while the export runs. Press `Esc` or `Q` to stop early.

## Generic Example

```bat
E:\RM\rm_vision\RM_BUFF_V2.1\tests\run_yolo_video_test.bat ^
  --parameter E:\RM\rm_vision\examples\example_for_prediction\8_dark_blue_small\parameter.yaml ^
  --output E:\RM\rm_vision\RM_BUFF_V2.1\tests\output\small_blue_full.avi ^
  --show
```

If you do not pass `--max-frames`, the whole source video is exported.
Use `--wait-ms 1` to control preview playback speed.

## Outputs

Files are written to `tests/output/`:

- `*.avi`: full annotated video with boxes, class labels, and keypoints
- `*.csv`: per-frame detections
- `*.jpg`: first annotated frame preview
- `*__image_compare.jpg`: single-image GT/prediction comparison
- `*__image_compare__pred.jpg`: single-image prediction overlay
- `*__image_compare.txt`: single-image detailed dump
