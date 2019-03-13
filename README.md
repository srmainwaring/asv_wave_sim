# Gazebo Wave Simulator


This package contains plugins that support the simulation and visualisation
of waves in Gazebo. It is a simplified version of the
[full wave simulator](https://github.com/srmainwaring/asv_wave_sim/):
it includes a trochoidal wave generator but excludes boat dynamics. It comprises
a visual plugin for the Gazebo client that synchronises the rendered scene with
the wave physics running in the server.
This package is licensed under [Apache 2.0](LICENSE).

![Wave Simulation](https://github.com/srmainwaring/asv_wave_sim/wiki/images/ocean_waves_rs750.jpg)

## Dependencies

You will need a working installation of ROS and Gazebo in order to use this package.


## Ubuntu

- Ubuntu 18.04
- ROS Melodic Morenia
- Gazebo version 9.0.0

### macOS

- OSX 10.11.6
- ROS Melodic Morenia
- Gazebo version 9.6.0

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
source /usr/local/share/gazebo-9/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p wave_ws/src
cd wave_ws
catkin init
```

Configure catkin:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Clone and build the package

Clone the `wave_sim` repository:

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

## Tests

Manually run the tests:

```bash
./devel/lib/asv_wave_sim_gazebo_plugins/UNIT_Wavefield_TEST
```

## Examples

![Wave Simulation](https://github.com/srmainwaring/asv_wave_sim/wiki/images/ocean_waves_box_example.gif)

Launch a Gazebo session with `roslaunch`:

```bash
roslaunch wave_gazebo ocean_world.launch verbose:=true
```

Publish a wave parameters message:

```bash
./devel/lib/wave_gazebo_plugins/WaveMsgPublisher \
  --number 3 \
  --amplitude 1 \
  --period 7 \
  --direction 1 1 \
  --scale 2 \
  --angle 1 \
  --steepness 1
```

For more detail see the [Example](https://github.com/srmainwaring/asv_wave_sim/wiki/Example) page in the wiki.

## License

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

This project makes use of other open source software, for full details see the
file [LICENSE_THIRDPARTY](LICENSE_THIRDPARTY).

## Acknowledgments

- Jacques Kerner's two part blog describing boat physics for games: [Water interaction model for boats in video games](https://www.gamasutra.com/view/news/237528/Water_interaction_model_for_boats_in_video_games.php) and [Water interaction model for boats in video games: Part 2](https://www.gamasutra.com/view/news/263237/Water_interaction_model_for_boats_in_video_games_Part_2.php).
- The [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) package for the orginal vertex shaders used in the wave field visuals.
- The [VRX](https://bitbucket.org/osrf/vrx) package for textures and meshes used in the wave field visuals.
