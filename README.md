# ASV Wave Simulator

This package contains plugins that support the simulation of waves and surface vessels in Gazebo.  

## Installation

You will need a working installation of ROS and Gazebo in order to use this package.

The package was built and tested with:

- Gazebo version 9.4.1
- ROS Melodic Morenia

and was developed on a Mac Pro 1, 1 running OSX 10.11.6.

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
```

Create a catkin workspace:

```bash
mkdir asv_ws
cd asv_ws
mkdir src
catkin init
```

Clone the ASV Wave Simulator repository:

```bash
cd src
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/asv_wave_sim.git
```

Compile the packages:

```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

or with tests:

```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --catkin-make-args run_tests
```

## Usage

Manually run the tests:

```bash
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Algorithm_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Geometry_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Grid_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Physics_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Wavefield_TEST
```

Launch a Gazebo session with `roslaunch`:

```bash
roslaunch asv_wave_gazebo ocean_world.launch verbose:=true
```

Publish a wave parameters message:

```bash
./devel/lib/asv_wave_sim_gazebo_plugins/WaveMsgPublisher \
  --number 3 \
  --amplitude 1 \
  --period 7 \
  --direction 1 1 \
  --scale 2 \
  --angle 1 \
  --steepness 1
```

Publish a hydrodynamics parameters message:

```bash
./devel/lib/asv_wave_sim_gazebo_plugins/HydrodynamicsMsgPublisher \
  --model box \
  --damping_on true \
  --viscous_drag_on true \
  --pressure_drag_on false \
  --cDampL1 10 \
  --cDampL2 1 \
  --cDampR1 10 \
  --cDampR2 1
```

## License

ASV Wave Simulator is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ASV Wave Simulator is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.


This project makes use of other open source software, for full details see the
file [LICENSE_THIRDPARTY](LICENSE_THIRDPARTY).
