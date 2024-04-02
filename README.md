# astra_camera_ros_dashing

## Requirements

- Ubuntu 18.04
- ROS2 Dashing

## Install

Download [OpenNI](https://structure.io/openni/) - choose the version for your CPU architecture. Extract the archive to some path. After that `cd` into your OpenNI-Linux-xxx-xxx downloaded folder and run the following commands:

```
sudo chmod +x install.sh
sudo ./install.sh
```

Now source the newly appeared OpenNIDevEnvironment file:
```
source OpenNIDevEnvironment
```

**Be aware that you will have to source it in every terminal which uses the nodes from the package!**

Then install the other dependencies:

```
pip3 install openni
sudo apt install opencv python3-colcon-common-extensions
```

Now clone the package into your `src` folder:

```
cd /path/to/workspace/src
git clone https://github.com/Yyote/astra_camera_ros_dashing.git
```

And then build and source your workspace.

```
colcon build
source install/setup.bash
```

## Run 

```
ros2 run astra_camera_ros_dashing astra_node
```
