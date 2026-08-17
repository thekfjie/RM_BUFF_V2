# No-ROS Simulator Validation Path

This workflow avoids ROS 2 entirely.

## What it does

1. `at_vision_simulator_noros` renders the simulator in real time.
2. An offscreen capture camera follows the simulator main camera.
3. Captured frames are saved as JPEG files under a new `captures/session_xxxx/` directory.
4. `RM_BUFF_V2.1` standalone reads that image sequence directly and runs the normal YOLO + tracker + prediction pipeline.

## Simulator side

Use:

- `<simulator-noros-root>\run_noros_capture.bat`

Optional argument:

- first argument = capture fps

Example:

```bat
<simulator-noros-root>\run_noros_capture.bat 30
```

Each run creates a new session directory like:

```text
<simulator-noros-root>\captures\session_0001
```

Inside that folder you will get:

- `frame_000001.jpg`
- `frame_000002.jpg`
- ...
- `capture_info.txt`
- `ffmpeg_make_video.bat`

## BUFF_V2.1 side

Use:

- `<RM_BUFF_V2.1-root>\scripts\sim_sequence_small_red.bat`

Arguments:

- first argument = the session directory
- second argument = fps hint for the standalone predictor, defaults to `30`

Example:

```bat
<RM_BUFF_V2.1-root>\scripts\sim_sequence_small_red.bat ^
  <simulator-noros-root>\captures\session_0001 ^
  30
```

## Notes

- The simulator no longer depends on ROS 2 in this local fork.
- The simulator still needs Rust/Cargo to build and run.
- The standalone predictor now accepts an image-sequence directory anywhere it previously accepted a video path.
- Tune mode is still video-only for now.
- If you change the simulator capture fps, pass the same fps to `sim_sequence_small_red.bat`.
