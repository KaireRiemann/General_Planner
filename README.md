Task Level General-Purpose Planner for Robots
## Install Guide
Install mavros
```
sudo apt install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
cd /opt/ros/${ROS_DISTRO}/lib/mavros
sudo ./install_geographiclib_datasets.sh 
```