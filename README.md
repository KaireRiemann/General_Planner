Task Level General-Purpose Planner for Robots
## Install Guide
Install mavros
```
sudo apt install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
cd /opt/ros/${ROS_DISTRO}/lib/mavros
sudo ./install_geographiclib_datasets.sh 
```

## General State to State Planning FrameWork
<td align="center" width="38.5%"><img src="docs/images/General State to State.png" alt="Fig1" width="100%"></td>