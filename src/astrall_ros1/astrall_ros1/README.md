# astrall_ros1

ROS Noetic wrapper for the Astrall SDK. It also contains configuration templates
for the front and rear RoboSense AIRY lidars.

## Build

The SDK only provides Linux shared libraries, so build this package on Ubuntu.

```bash
mkdir -p ~/catkin_ws/src
ln -s /path/to/astrall_sdk_1_0_5/astrall_ros1 ~/catkin_ws/src/astrall_ros1
cd ~/catkin_ws
catkin_make -DASTRALL_SDK_ROOT=/path/to/astrall_sdk_1_0_5
source devel/setup.bash
```

## Run

Install FFmpeg. The camera bridge uses the same raw H.264 decoding path that was
verified with `ffplay` and publishes standard ROS images.

```bash
sudo apt update
sudo apt install ffmpeg
```

```bash
roslaunch astrall_ros1 bringup.launch
```

Run keyboard control in a second terminal after sourcing the same workspace:

```bash
rosrun astrall_ros1 astrall_keyboard_teleop_node
```

The keyboard node only publishes `/cmd_vel`; it does not initialize the Astrall
SDK or request control authority. Keep `astrall_driver_node` as the only running
SDK client, and do not run `sdk_demo` at the same time. Press `2` to stand,
wait until `/astrall/sport_status` is `0xB102` (`45314`), then press `4` to
enter MOVE mode. Controls are `w/s` for forward/backward, `a/d` for lateral
motion, `q/e` for yaw, space or `x` to stop, and `3` to lie down. Optional
private parameters are `linear_speed` (default `0.25`),
`angular_speed` (default `0.5`), `publish_rate` (default `20.0`), and
`key_timeout` (default `0.6`).

To run the robot driver, both lidars, LIO-SAM, and RViz together, use:

```bash
roslaunch astrall_ros1 slam_bringup.launch
```

Use the combined launch only when the robot is already upright and stationary.
If it starts prone, first launch `bringup.launch`, stand the robot and enter
MOVE mode, then start `lio_sam/run.launch`. Initializing LIO-SAM during the
stand-up transition can destabilize IMU preintegration.

Then run `astrall_keyboard_teleop_node` in a separate terminal. The combined
launch disables LIO-SAM's duplicate robot-state TF publisher and unused NavSat
nodes. LIO-SAM consumes `/front/lidar_points` and `/imu/data`; the rear lidar
remains available on `/rear/lidar_points` but is not fused by this LIO-SAM
configuration.

### FAST-LIO with the front AIRY

FAST-LIO is configured for the front 96-beam AIRY and the Astrall base IMU.
The AIRY driver publishes the required `ring` and per-point `time` fields, so
no point-cloud conversion node is needed. Start the robot upright and keep it
stationary while the IMU initializes, then run:

```bash
roslaunch astrall_ros1 fast_lio_bringup.launch
```

This launch starts the Astrall driver, both AIRY lidars, FAST-LIO, and RViz.
FAST-LIO consumes `/front/lidar_points` and `/imu/data`; the rear cloud remains
available but is not fused. Its world/body frames are `map` and `base_link`.
The main outputs are `/Odometry`, `/cloud_registered`, and `/path`.

Before moving the robot, verify the input layout and rates:

```bash
rostopic echo -n1 /front/lidar_points/fields
rostopic hz /front/lidar_points
rostopic hz /imu/data
```

The cloud must contain `x`, `y`, `z`, `intensity`, `ring`, and `time`; the
expected nominal rates are 10 Hz and 250 Hz. If mapping bends or duplicates
walls during motion, measure LiDAR-to-IMU time offset instead of enabling
FAST-LIO software synchronization blindly, then set
`common/time_offset_lidar_to_imu` in `fast_lio/config/airy96.yaml`.

Enable both AIRY lidars through the rslidar_sdk multi-lidar configuration:

```bash
roslaunch astrall_ros1 bringup.launch enable_lidars:=true
```

The camera sends a raw H.264 stream to UDP port 6000. The
`astrall_h264_camera_node` process runs FFmpeg, scales decoded frames to 640x480
RGB, and publishes `/camera/image_raw`. Stop `ffplay` before launching because
only one process should consume the UDP stream. Output dimensions and frame rate
can be changed with `camera_width`, `camera_height`, and `camera_fps` launch
arguments.

## Interfaces

Subscribed topic:

```text
/cmd_vel                    geometry_msgs/Twist
```

Published topics:

```text
/imu/data                   sensor_msgs/Imu
/camera/image_raw           sensor_msgs/Image
/astrall/sport_data         std_msgs/Float32MultiArray
/astrall/sdk_status         std_msgs/UInt16
/astrall/system_status      std_msgs/UInt32
/astrall/error_code         std_msgs/UInt32
/astrall/warn_code          std_msgs/UInt32
/astrall/sport_status       std_msgs/UInt16
/battery_state              sensor_msgs/BatteryState
```

Services:

```text
/astrall/set_sdk_auth       std_srvs/SetBool
/astrall/set_move_mode      std_srvs/SetBool
/astrall/stand              std_srvs/Trigger
/astrall/down               std_srvs/Trigger
/astrall/set_light          std_srvs/SetBool
```

The Astrall SDK camera subscription remains enabled because it requests the
network stream, but `camera_sdk_callback_enabled` defaults to `false` so the SDK
does not also consume the H.264 payload. The decoded ROS image is published by
`astrall_h264_camera_node` on `/camera/image_raw`.

The RoboSense node reads `config/rslidar.yaml` through its `config_path`
parameter. It publishes `/front/lidar_points`, `/rear/lidar_points`,
`/front/lidar_imu`, and `/rear/lidar_imu`.
