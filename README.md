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
As a note, if you are running it on the actual drone, make sure that it is in manual mode otherwise it will not be able to switch to offboard mode. 

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

## Updating FTDI Adaptor board
If you have issues with communication between the pixhawk and your companion computer, you may need to swap your FTDI board. Once you have switched it, run the command in your terminal below to see what the identifier for the new board is: 
```
ls /dev/serial/by-id/
```
That will list all the devices plugged in over usb by serial ID. Then update your bashrc with the command (Don't forget to update the new FTDI ID): 
```
echo 'export SUAVE_MAVLINK_SERIAL_ID=usb-FTDI_FT232R_USB_UART_<NEW_FTDI_IDENTIFIER>-if00-port0' >> ~/.bashrc
source ~/.bashrc
```
## Running the Masking Before Flight
If you want to verify the correct camera ID before running the masking for one of the drones, run the command below: 
```
ros
python3 ~/Dev/suavecpp-ros2_ws/src/suave_controls/suave_controls/masking_pid_publisher.py
```
After running this, if you see something similar to 
```
[main] suave_controls/masking_pid_publisher.py
Unknown device "/dev/video8": No such file or directory
This is the correct camera ID: 6
[ WARN:0@0.171] global cap_v4l.cpp:999 open VIDEOIO(V4L2:/dev/video6): can't open camera by index
[ WARN:0@0.252] global obsensor_stream_channel_v4l2.cpp:82 xioctl ioctl: fd=-1, req=-2140645888
[ WARN:0@0.252] global obsensor_stream_channel_v4l2.cpp:138 queryUvcDeviceInfoList ioctl error return: 9
[ WARN:0@0.253] global obsensor_stream_channel_v4l2.cpp:82 xioctl ioctl: fd=-1, req=-2140645888
[ WARN:0@0.253] global obsensor_stream_channel_v4l2.cpp:138 queryUvcDeviceInfoList ioctl error return: 9
[ERROR:0@0.253] global obsensor_uvc_stream_channel.cpp:158 getStreamChannelGroup Camera index out of range
Failed to open camera with ID 6
```
This might mean that the LattePanda may not have access to the side camera. Run the commnd below to give it access (Be sure to update the video ID with what you get in your error message)
```
sudo chmod 777 /dev/video6
```
If you want to see what the bounding boxes from what the LattePanda side camera sees, then ssh into the LattePanda with the command:
```
ssh -YC suave@<IP_Address>
```
And uncomment line 237 in masking_pid_publisher.py and rerun it. 
```
cv2.imshow('Live Stream with Bounding Box and Center Point', frame)
```
## m_drone->offboard().start() failed: Command Denied
This error can mean a few different things. 
1. The first trouble shooting for this is to make sure the drone is in manual mode when starting the script. If it is not, PX4 will not be able to switch to offboard mode.
2. Ensure that the realsense is running properly and that the realsense is setup correctly. It must have a USB 3 cable to check that look at the usb port on the cable and it is blue than it should be usb3, however the USB-c portion may not be. SSH -YC into the drone and run the command
3. If the realsense is running properly and you are still getting this error then run RTAB individually and ensure that it can open properly.
```
realsense-viewer
```
4. You may have a bad FTDI connection. Verify that the FTDI ID is the same as what the script is looking for. If it is then check the connection to the board itself. Then reboot the LattePanda and try it again. 
