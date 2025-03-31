# ROS Workspace for SUAVE

## Initialize your workspace and environment
```
sudo apt install git -y
mkdir ~/Dev
cd ~/Dev
git clone https://github.com/Danielgolan0925/suavecpp-ros2_ws.git
cd suavecpp-ros2_ws/setup
source setup.bash
```

## Install PX4-Autopilot
```
cd ~/Dev
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

## To build
Dependencies:
- ROS Humble (Including devlopment tools)
- MAVSDK
- PCL

```
colcon build
source ./install/setup.bash
```

## To run
Launch PX4 SITL
```
cd PX4-Autopilot
make px4_sitl gz_x500
```
Launch suave_main
```
ros2 run suave_launch suave_main
```

## Running RTAB with IR
Start Realsense Camera Node
```
ros 
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
```
In a separate Terminal
Start RTABMaps
```
ros
ros2 param set /camera/camera depth_module.emitter_enabled 0
ros2 launch ~/Dev/suavecpp-ros2_ws/launch/suave_slam.py
```
