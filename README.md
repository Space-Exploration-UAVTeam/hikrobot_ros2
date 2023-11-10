# hikrobot_ros2
ROS2 driver for hikrobot Aera scan cameras: https://www.hikrobotics.com/en/machinevision/visionproduct?typeId=78&id=145

## 1. Prerequisites
### 1.1 hikrobot MVS driver for Linux
https://www.hikrobotics.com/en/machinevision/service/download?module=0

### 1.2 ROS2
This package is tested under ROS Galactic.

## 2. Build 
Clone the repository to your catkin workspace (for example `~/ros2_ws/`):
```
cd ~/ros2_ws/src/
git clone https://github.com/Space-Exploration-UAVTeam/hikrobot_ros2.git
```
Then build the package with:
```
cd ~/ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
```

## 3. Run
### 1.1 Single camera, with stand alone exposure time calculating
```
ros2 launch hikrobot_ros2 MvCameraPub.launch
```
### 1.2 Other executables
to be...

## 4. Hardware triggering
to be...

## 5. Licence
The source code is released under BSD 3-Clause license.
