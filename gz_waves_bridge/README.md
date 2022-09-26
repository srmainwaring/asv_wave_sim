# Gazebo Waves Bridge

A ROS2 bridge for `gz-waves`.

## Install

```zsh
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17 -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib -DHIGHFIVE_USE_EIGEN=ON --packages-select gz_waves_bridge gz-waves1
```

## Run

```zsh
ros2 launch gz_waves_bridge ellipsoid_buoy.launch.py
```

## Nodes

### body_response_publisher

The body response publisher subscribes to an odometry message and
republishes the position and orientation using marine dof labels and
conventions. Mainly for plotting and visualisation.

#### Subscribed Topics

- `/odom` (`nav_msgs/Odometry`)


#### Published Topics

- `/surge` (`std_msgs/Float64`)
- `/sway` (`std_msgs/Float64`)
- `/heave` (`std_msgs/Float64`)
- `/roll` (`std_msgs/Float64`)
- `/pitch` (`std_msgs/Float64`)
- `/yaw` (`std_msgs/Float64`)
