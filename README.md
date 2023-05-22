# ros2_rs_pcl
ROS 2 sample of Realsense with PCL library

## Supported ROS 2 distributions

[![humble][humble-badge]][humble]
[![foxy][foxy-badge]][foxy]
[![ubuntu22][ubuntu22-badge]][ubuntu22]
[![ubuntu20][ubuntu20-badge]][ubuntu20]

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

[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[foxy-badge]: https://img.shields.io/badge/-FOXY-orange?style=flat-square&logo=ros
[foxy]: https://docs.ros.org/en/foxy/index.htmll
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
[ubuntu20-badge]: https://img.shields.io/badge/-UBUNTU%2020%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu20]: https://releases.ubuntu.com/focal/
