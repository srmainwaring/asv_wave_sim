# ASV Wave Simulator

This package contains plugins that support the simulation of waves and surface vessels in Gazebo.  

![Wave Simulation](https://github.com/srmainwaring/asv_wave_sim/wiki/images/ocean_waves_rs750.jpg)

## Dependencies

You will need a working installation of ROS and Gazebo in order to use this package.


## Ubuntu

- Ubuntu 18.04
- ROS Melodic Morenia
- Gazebo version 9.0.0

Install CGAL 4.13 libraries:

```bash
sudo apt-get install libcgal-dev
```

### macOS

- OSX 10.11.6
- ROS Melodic Morenia
- Gazebo version 9.6.0

Install CGAL 4.13 libraries:

```bash
brew install cgal
```

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
source /usr/local/share/gazebo-9/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p asv_ws/src
cd asv_ws
catkin init
```

Configure catkin:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Clone and build the package

Clone the `asv_wave_sim` repository:

```bash
cd src
git clone https://github.com/srmainwaring/asv_wave_sim.git
```

Compile the packages:

```bash
catkin build
```

or with tests:

```bash
catkin build --catkin-make-args run_tests
```

## Usage

The wiki has details about how to configure and use the plugins:

- [WavefieldPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/WavefieldPlugin)
- [WavefieldVisualPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/WavefieldVisualPlugin)
- [HydrodynamicsPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/HydrodynamicsPlugin)

## Tests

Manually run the tests:

```bash
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Algorithm_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Geometry_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Grid_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Physics_TEST
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Wavefield_TEST
```

## Examples

![Wave Simulation](https://github.com/srmainwaring/asv_wave_sim/wiki/images/ocean_waves_box_example.gif)

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

For more detail see the [Example](https://github.com/srmainwaring/asv_wave_sim/wiki/Example) page in the wiki.

## Build Status

### Develop Job Status

|    | Melodic |
|--- |--- |
| asv_wave_sim | [![Build Status](https://travis-ci.org/srmainwaring/asv_wave_sim.svg?branch=feature%2Ffft_waves)](https://travis-ci.org/srmainwaring/asv_wave_sim) |


### Release Job Status

|    | Melodic |
|--- |--- |
| asv_wave_sim | [![Build Status](https://travis-ci.org/srmainwaring/asv_wave_sim.svg?branch=master)](https://travis-ci.org/srmainwaring/asv_wave_sim) |

## License

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.

This project makes use of other open source software, for full details see the
file [LICENSE_THIRDPARTY](LICENSE_THIRDPARTY).

## Acknowledgments

- Jacques Kerner's two part blog describing boat physics for games: [Water interaction model for boats in video games](https://www.gamasutra.com/view/news/237528/Water_interaction_model_for_boats_in_video_games.php) and [Water interaction model for boats in video games: Part 2](https://www.gamasutra.com/view/news/263237/Water_interaction_model_for_boats_in_video_games_Part_2.php).
- The [CGAL](https://doc.cgal.org) libraries are used for the wave field and model meshes.
- The [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) package for the orginal vertex shaders used in the wave field visuals.
- The [VMRC](https://bitbucket.org/osrf/vmrc) package for textures and meshes used
in the wave field visuals.
