# Installation & Setup

These instructions assume **Ubuntu 22.04** with **ROS 2 Humble**.  
Adjust paths if your workspace layout is different.

---

## 1. Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide, then source it:

```bash
source /opt/ros/humble/setup.bash
```
## 2. Clone this Repository

```
git clone https://github.com/vipUHDT/MAVROS-Autonomous.git
```
## 3. Install Required Packages

This project depends on:
- Livox-SDK2
- livox_msgs
- livox_ros_driver2
- mavros
- pointcloud_to_laserscan

### 3.1 Install Binary Packages

```
sudo apt update
sudo apt install \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-pointcloud-to-laserscan
```

Install MAVROS geographic data
```
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

## 4. Install Dependencies with rosdep
```
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Build Livox LiDAR packages

Build Livox-SDK by following https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md

Build livox_ros_driver2
```
cd ~ros2_ws/src/livox_ros_driver2
./setup.sh humble
```

## 5. Build the Workspace with Colcon Build Tool
```
cd ~/ros2_ws
colcon build --symlink-install
```
Source the workspace
```
source install/setup.bash
```
Add to ~/.bashrc (Automatically sources ROS workspace)
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
## 6. Verify Installation
```
ros2 pkg list | grep -E 'livox|mavros|pointcloud_to_laserscan'
```
You should see
- livox_ros_driver2
- livox_msgs
- mavros
- pointcloud_to_laserscan
