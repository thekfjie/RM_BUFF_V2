# at_vision_simulator <-> RM_BUFF_V2.1

This note documents the current recommended Win11 bring-up path:

`at_vision_simulator` publishes `/image_raw`

`rm_buff_tracker/buff_node` subscribes to that image stream and publishes:

- `~/prediction`
- `~/target`
- `~/debug_state`
- `~/debug_image`

## Files added for this workflow

- `config/lab/buff_node_simulator.yaml`
- `scripts/ros2_sim_0_env.bat`
- `scripts/ros2_sim_1_build_rm_buff_tracker.bat`
- `scripts/ros2_sim_2_run_buff_node.bat`
- `scripts/ros2_sim_3_run_at_vision_simulator.bat`
- `scripts/ros2_sim_4_watch_topics.bat`
- `scripts/ros2_sim_all.bat`

## First-time setup

1. Edit `scripts/ros2_sim_0_env.bat`.
2. Check `ROS2_SETUP`, `RM_WS`, `OpenCV_DIR`, and `AT_VISION_SIM_DIR`.
3. If you want ONNX Runtime, also check `ONNXRUNTIME_DIR` and `ONNXRUNTIME_LIB_DIR`.
4. Open `config/lab/buff_node_simulator.yaml` and set `color` and `mode` to match the target you want to test.

## Bring-up order

1. Run `scripts/ros2_sim_1_build_rm_buff_tracker.bat`.
2. Run `scripts/ros2_sim_2_run_buff_node.bat`.
3. Run `scripts/ros2_sim_3_run_at_vision_simulator.bat`.
4. Run `scripts/ros2_sim_4_watch_topics.bat`.

If you want the helper to open all three runtime windows for you, run:

`scripts/ros2_sim_all.bat`

## Current scope

This is still a one-way integration.

It validates:

- simulator image publishing
- YOLO-assisted initialization and relock
- HSV/F_BuffTracker handoff stability
- prediction/debug topic output

It does not yet provide closed-loop gimbal control back into the simulator.

The current simulator codebase publishes `/tf`, `/camera_info`, `/image_raw`, `/gimbal_pose`, `/odom_pose`, and `/camera_pose`, but there is no ROS-side gimbal control subscriber wired into the simulator yet.

Also note that `/image_raw` follows the simulator main camera. Keep the simulator in Robot view during bring-up unless you intentionally want a different test view.
