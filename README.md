# These are packages for using MTF Module with ROS

## Installation Instructions

The following instructions support ROS Kinetic, on Ubuntu 16.04.

### Step 1 : Install the ROS distribution
- #### Install ROS Kinetic, on Ubuntu 16.04

### Step 2 : Install driver
- #### Create a catkin workspace
```bash
$mkdir –p ~/catkin_ws/src
$cd ~/catkin_ws/src/
Copy the driver source to the path(catkin_ws/src)
```

- #### driver build
```bash
$catkin_init_workspace
$cd ..
$catkin_make clean
$catkin_make -DCMAKE_BUILD_TYPE=Release
```

- #### Setting environment
$sudo mtfinstall.sh

## Usage Instructions

Connect the camera power and execute the following command

```bash
$roslaunch depth_mtf depth_camera.launch
```

Topics
- /depth/mtf/amplitude_raw : IR Image
- /depth/mtf/depth_raw : Depth Image
- /detph/mtf/points : Point Cloud Image

Operating Test
```bash
$rqt
/depth/mtf/amplitude_raw, /depth/mtf/depth_raw
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/67537195-4d1a0400-f6a8-11e9-8768-5fcf0e612862.png"/></p>

```bash
$rosrun rviz rviz
Fixed Frame : pcl
PointCloud2 : /depth/mtf/points
```
