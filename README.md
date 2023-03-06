# ros2_rs_pcl
ROS 2 sample of Realsense with PCL library

## Supported ROS 2 distributions
・Foxy (master)

・Humble (humble-devel is in development)

## Requirements
- Laptop PC
  - Ubuntu 20.04 Foxy
- Realsense D435/D435i

## Installation
```
sudo apt install ros-$ROS_DISTRO-pcl-*
```

Install the ROS2 wrapper for realsense

[ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)

and configure Realsense to work with ROS2

## Usage
### PCL filter
Open two shells.
In the first shell, run the ros2_intel_realsense node
```
ros2 launch realsense_examples rs_camera.launch.py
```

In the second shell, run the pcl_filter node:
```
ros2 run ros2_rs_pcl rs_pcl_filter
```

### PCL clustering
Open two shells.
In the first shell, run the ros2_intel_realsense node
```
ros2 launch realsense_examples rs_camera.launch.py
```

In the second shell, run the pcl_clustering node:
```
ros2 run ros2_rs_pcl rs_pcl_clustering
```

![rs clustering img](/img/pcl_cluster.png)

### PCL and visualization marker sample
Open two shells.
In the first shell, run the ros2_intel_realsense node
```
ros2 launch realsense_examples rs_camera.launch.py
```

In the second shell, run the pcl_clustering node:
```
ros2 run ros2_rs_pcl rs_pcl_marker
```

![rs clustering img](/img/pcl_and_marker.png)

## License
This repository is licensed under the MIT license, see LICENSE.
