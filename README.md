# Programming Assignment: ROS Gazebo Integration
***
**Subject:** ENPM 808X  
**Name:** Shantanu Parab  
**UID:** 119347539  
**Directory ID:** sparab@umd.edu
***

### System Requirements
ROS2 Humble Hawksbill
Follow the installation instructions given in the link  
[ROS 2 Humble Hawksbill Installation](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)  

Install Gazebo,Turlebot3 and Plugins  
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-plugins
```
Gazebo Variables setup
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
    prefix turtlebot3_gazebo \
    `/share/turtlebot3_gazebo/models/
```

### Instructions to Create workspace and build package
```bash
#Create a Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
#Clone Package
git clone https://github.com/shantanuparabumd/roomba.git
# cd if you're still in the ``src`` directory with the ``roomba`` clone
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
#Source your WS
. install/local_setup.bash
#Build Package
colcon build --packages-select roomba
```


### Instruction to run nodes
Open a new terminal to run publisher node
```bash
cd ~/ros2_ws
. install/local_setup.bash
ros2 run roomba roomba_algo
```

### Ros Bag Launch
Note: Make sure you delete the existing my_bag folder
```bash
cd ~/ros2_ws/src/roomba
# Run the bellow command to run along with the bag file
ros2 launch roomba system.py ros_bag:='True'
# Run the bellow command to run along without the bag file
ros2 launch roomba system.py
```
**Check the rosbag output**
```bash
#Run in One terminal
ros2 launch roomba gazebo_launch.py
#Run in Another terminal
cd ~/ros2_ws/src/roomba
ros2 bag play my_bag
```
Note: The rosbag is created in the directory where the launch file is executed from.  
The rosbag play command should be played in the same directory.

### CPP Check and CPP Lint
```bash
cd ~/ros2_ws/src/roomba
bash cppcheck.sh
bash cpplint.sh
```