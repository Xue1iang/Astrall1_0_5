# Astrall1_0_5

Astrall1_0_5 is a ROS Noetic and C++ SDK workspace for the Astrall quadruped robot platform.
The repository provides the Astrall SDK shared libraries, a standalone C++ SDK demo, and a ROS1 integration package for robot state publishing, motion command control, camera streaming, LiDAR access, and LiDAR-Inertial SLAM experiments.

本仓库主要用于 Astrall 机器人在 Ubuntu / ROS Noetic 环境下的二次开发，包括 SDK 调用测试、ROS 话题桥接、键盘遥控、相机图像发布、RoboSense LiDAR 接入，以及 FAST-LIO / LIO-SAM 等建图功能集成。

---

## Repository Structure

```bash
Astrall1_0_5/
├── demo/                  # Standalone C++ SDK demo
│   ├── CMakeLists.txt
│   ├── lib/               # Demo SDK library and headers
│   └── src/               # Demo source code
│
├── sdk/                   # Astrall SDK files
│   ├── include/           # SDK header files
│   ├── arm64/             # ARM64 shared libraries
│   └── x86-64/            # x86-64 shared libraries
│
└── src/                   # ROS catkin workspace source folder
    ├── astrall_ros1/      # ROS Noetic wrapper for Astrall SDK
    ├── FAST_LIO/          # FAST-LIO package/configuration
    ├── gscam/             # Camera related ROS package
    └── rslidar_sdk/       # RoboSense LiDAR SDK package
```

---

## Main Features

* Astrall SDK initialization and status monitoring
* SDK control authority request and release
* Robot sport mode switching
* Velocity control through SDK or ROS `/cmd_vel`
* IMU, sport status, system status, warning/error code publishing
* Battery state publishing
* RGB camera stream decoding and ROS image publishing
* Front and rear RoboSense LiDAR support
* FAST-LIO / LIO-SAM based LiDAR-Inertial SLAM bringup
* Keyboard teleoperation support

---

## Tested Environment

Recommended environment:

```bash
Ubuntu 20.04
ROS Noetic
CMake >= 3.10
C++17
```

The SDK provides Linux shared libraries for both `x86-64` and `arm64`, so it can be used on a normal Ubuntu PC or an onboard computer such as NVIDIA Jetson, depending on the robot-side deployment environment.

---

## Dependencies

Install ROS Noetic first, then install common build and runtime dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  ffmpeg \
  ros-noetic-desktop-full \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-robot-state-publisher \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions
```

After installing ROS, source the ROS environment:

```bash
source /opt/ros/noetic/setup.bash
```

You may also add it to `~/.bashrc`:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Clone the Repository

```bash
git clone https://github.com/Xue1iang/Astrall1_0_5.git
cd Astrall1_0_5
```

---

## Build the Standalone SDK Demo

The standalone demo is located in `demo/`.

```bash
cd demo
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

Run the demo:

```bash
./sdk_demo
```

Run the demo with camera subscription enabled:

```bash
./sdk_demo --camera
```

or:

```bash
./sdk_demo -c
```

The demo program includes examples for:

* SDK initialization
* heartbeat
* device information query
* system status query
* battery status query
* IMU data subscription
* sport data subscription
* camera RGB data subscription
* sport mode switching
* SDK control authority request
* velocity command control
* light control

---

## SDK Demo Keyboard Control

After running `sdk_demo`, the following keyboard commands are supported by the demo program:

| Key | Function                        |
| --- | ------------------------------- |
| `1` | Switch to damping mode          |
| `2` | Switch to fixed stand mode      |
| `3` | Switch to fixed down mode       |
| `4` | Switch to move mode             |
| `5` | Auto charging mode              |
| `6` | Exit charging pile              |
| `9` | Request SDK control authority   |
| `0` | Stop motion command             |
| `w` | Increase forward velocity       |
| `s` | Increase backward velocity      |
| `a` | Increase lateral-left velocity  |
| `d` | Increase lateral-right velocity |
| `q` | Increase yaw velocity           |
| `e` | Decrease yaw velocity           |
| `n` | Turn on light                   |
| `o` | Turn off light                  |

Typical control sequence:

```text
1. Run sdk_demo
2. Press 9 to request SDK authority
3. Press 2 to stand
4. Press 4 to enter MOVE mode
5. Use w/s/a/d/q/e to control motion
6. Press 0 to stop
```

> Safety note: test the robot in an open area and keep the physical remote controller or emergency stop available.

---

## Build the ROS Noetic Workspace

The repository already contains a `src/` folder, so the repository root can be used as a catkin workspace.

```bash
cd Astrall1_0_5
catkin_make -DASTRALL_SDK_ROOT=$PWD
source devel/setup.bash
```

If the build succeeds, add the workspace setup script to `~/.bashrc`:

```bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Run Astrall ROS Driver

Start the basic Astrall ROS bringup:

```bash
roslaunch astrall_ros1 bringup.launch
```

This starts the Astrall ROS driver and publishes robot state information.

Common published topics include:

```bash
/imu/data
/camera/image_raw
/astrall/sport_data
/astrall/sdk_status
/astrall/system_status
/astrall/error_code
/astrall/warn_code
/astrall/sport_status
/battery_state
```

Common subscribed topic:

```bash
/cmd_vel
```

Common services:

```bash
/astrall/set_sdk_auth
/astrall/set_move_mode
/astrall/stand
/astrall/down
/astrall/set_light
```

Check published topics:

```bash
rostopic list
```

Check IMU output:

```bash
rostopic echo /imu/data
```

Check sport status:

```bash
rostopic echo /astrall/sport_status
```

---

## Keyboard Teleoperation in ROS

Run the driver first:

```bash
roslaunch astrall_ros1 bringup.launch
```

Open another terminal:

```bash
source devel/setup.bash
rosrun astrall_ros1 astrall_keyboard_teleop_node
```

The keyboard teleoperation node publishes `/cmd_vel`.
It does not initialize the Astrall SDK by itself, so `astrall_driver_node` should remain running.

Typical sequence:

```text
1. Launch bringup.launch
2. Request SDK authority
3. Switch robot to stand mode
4. Switch robot to MOVE mode
5. Run keyboard teleop
```

Keyboard controls:

| Key            | Function             |
| -------------- | -------------------- |
| `w` / `s`      | Forward / backward   |
| `a` / `d`      | Lateral left / right |
| `q` / `e`      | Yaw left / right     |
| `space` or `x` | Stop                 |
| `2`            | Stand                |
| `3`            | Lie down             |

---

## Camera Stream

Install FFmpeg:

```bash
sudo apt update
sudo apt install -y ffmpeg
```

The Astrall camera stream is decoded and published as:

```bash
/camera/image_raw
```

View the image with:

```bash
rqt_image_view /camera/image_raw
```

or:

```bash
rosrun image_view image_view image:=/camera/image_raw
```

The camera stream uses UDP port `6000`.
If you are using `ffplay` to test the UDP stream manually, stop `ffplay` before launching the ROS camera node, because only one process should consume the stream at the same time.

---

## Enable RoboSense LiDAR

To launch the Astrall driver with LiDAR enabled:

```bash
roslaunch astrall_ros1 bringup.launch enable_lidars:=true
```

Expected LiDAR topics:

```bash
/front/lidar_points
/rear/lidar_points
/front/lidar_imu
/rear/lidar_imu
```

Check LiDAR point cloud rate:

```bash
rostopic hz /front/lidar_points
rostopic hz /rear/lidar_points
```

Check point cloud fields:

```bash
rostopic echo -n1 /front/lidar_points/fields
```

---

## Run FAST-LIO

FAST-LIO is configured for the front RoboSense AIRY LiDAR and the Astrall base IMU.

Start the robot upright and keep it stationary during IMU initialization, then run:

```bash
roslaunch astrall_ros1 fast_lio_bringup.launch
```

Main FAST-LIO outputs:

```bash
/Odometry
/cloud_registered
/path
```

Input topics:

```bash
/front/lidar_points
/imu/data
```

The rear LiDAR point cloud remains available but is not fused by the default FAST-LIO configuration.

Before moving the robot, verify input data:

```bash
rostopic hz /front/lidar_points
rostopic hz /imu/data
rostopic echo -n1 /front/lidar_points/fields
```

The point cloud should contain fields such as:

```text
x, y, z, intensity, ring, time
```

---

## Run LIO-SAM Bringup

To run the robot driver, LiDAR, LIO-SAM and RViz together:

```bash
roslaunch astrall_ros1 slam_bringup.launch
```

Recommended procedure:

```text
1. Start the robot in a stable upright state.
2. Keep the robot stationary during initialization.
3. Launch slam_bringup.launch.
4. Start keyboard teleoperation in another terminal.
```

If the robot starts prone, first use `bringup.launch` to stand the robot and enter MOVE mode, then start the SLAM launch file. Initializing LiDAR-Inertial SLAM during the stand-up transition may affect IMU preintegration and mapping quality.

---

## ROS Multi-Machine Usage

If the robot onboard computer runs `roscore` and your laptop only subscribes to topics, configure the ROS network as follows.

On the robot computer:

```bash
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=ROBOT_IP
roscore
```

On your laptop:

```bash
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=LAPTOP_IP
rostopic list
```

Then subscribe to robot topics:

```bash
rostopic echo /imu/data
rostopic echo /battery_state
rostopic echo /astrall/sport_status
```

Make sure both computers can ping each other:

```bash
ping ROBOT_IP
ping LAPTOP_IP
```

---

## Useful Debug Commands

List ROS topics:

```bash
rostopic list
```

Check topic frequency:

```bash
rostopic hz /imu/data
rostopic hz /front/lidar_points
```

Echo sport status:

```bash
rostopic echo /astrall/sport_status
```

Echo battery state:

```bash
rostopic echo /battery_state
```

View TF tree:

```bash
rosrun tf view_frames
```

View camera:

```bash
rqt_image_view
```

Run RViz:

```bash
rviz
```

---

## Important Notes

1. Do not run `sdk_demo` and the ROS `astrall_driver_node` at the same time. Only one SDK client should control the robot at a time.

2. Request SDK control authority before sending movement commands.

3. Confirm the robot is in MOVE mode before publishing velocity commands.

4. Keep the robot stationary when initializing FAST-LIO or LIO-SAM.

5. If camera images are not published, check whether another process is already occupying UDP port `6000`.

6. If ROS topics can be listed but no data can be received on another computer, check `ROS_MASTER_URI`, `ROS_IP`, firewall settings, and network reachability.

7. Use the correct SDK shared library for your CPU architecture:

   * `sdk/x86-64/` for x86-64 PC
   * `sdk/arm64/` for ARM64 / Jetson platform

---

## License

No license file is currently provided in this repository.
Please confirm usage and redistribution permission before using this repository in commercial or public projects.

---

## Maintainer

Xue1iang

Repository:

```text
https://github.com/Xue1iang/Astrall1_0_5
```
