# Wave Sim

[![Ubuntu Bionic CI](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ubuntu-bionic-ci.yml/badge.svg)](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ubuntu-bionic-ci.yml)
[![Ubuntu Focal CI](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ubuntu-focal-ci.yml/badge.svg)](https://github.com/srmainwaring/asv_wave_sim/actions/workflows/ubuntu-focal-ci.yml)

This package contains plugins that support the simulation of waves and surface vessels in Gazebo.  

![Wave Simulation](https://github.com/srmainwaring/asv_wave_sim/wiki/images/ocean_waves_rs750_fft.jpg)

## Notes

This is a prototype branch containing an updated wave engine that uses FFTs to generate the wavefield physics and visuals.

There are changes in the way that the wave parameters need to be set, and it may not be possible to avoid breaking the existing interface used to specify trochoidal waves. This is still work in progress, and the current version has a fixed set of wave parameters.

The library has additional dependencies on two FFT libraries:

- [clMathLibraries/clFFT](https://github.com/clMathLibraries/clFFT)
- [fftw](http://www.fftw.org/)

Aside from adding the option to use a FFT generated wavefield, the major change is in the way that the visuals are generated. Previously the wave displacements for visuals were generated in the shader code, the visual plugin was used to update shader parameters for wave amplitudes and frequency. Now the entire mesh for the visual is dynamically updated in the the library then pushed into the rendering engine. This means there is no need to maintain various sized meshes in the media files, however it does require working around Gazebos requirement for static meshes and there is a custom Visual that implements this. The OpenCL FFT library allows this work to be offloaded to the GPU when configured.

## Dependencies

You will need a working installation of ROS and Gazebo in order to use this package.


## Ubuntu

- Ubuntu 20.04
- ROS Noetic
- Gazebo version 11.12.0

Install CGAL, FFTW3, TBB and clFFT:

```bash
sudo apt-get install libcgal-dev libfftw3-dev libtbb-dev libclfft-dev
```

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/noetic/setup.bash
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

Clone the `feature/fft-waves-v2` branch of the `asv_wave_sim` repository:

```bash
cd src
git clone https://github.com/srmainwaring/asv_wave_sim.git -b feature/fft-waves-v2
```

Compile the packages:

```bash
catkin build
```

or with tests:

```bash
catkin build --catkin-make-args run_tests
```

## WavefieldModelPlugin

A Gazebo model plugin to simulate water waves.

### Usage

Add the SDF for the plugin to the ```<model>``` element of your wave model.

```xml
<plugin name="wavefield" filename="libWavefieldModelPlugin.so">
  <static>false</static>
  <update_rate>30</update_rate>
  <size>1000 1000</size>
  <cell_count>50 50</cell_count>
  <wave>
    <number>3</number>
    <scale>1.5</scale>
    <angle>0.4</angle>
    <steepness>1.0</steepness>
    <amplitude>0.4</amplitude>
    <period>8.0</period>
    <direction>1 1</direction>
  </wave>
  <markers>
    <wave_patch>false</wave_patch>
    <wave_patch_size>4 4</wave_patch_size>
  </markers>
</plugin>
```

### Subscribed Topics

1. `~/request` (`gazebo::msgs::Request`)

2. `~/wave` (`gazebo::msgs::Param_V`)

### Published Topics

1. `~/reponse` (`gazebo::msgs::Response`)

2. `/marker` (`ignition::msgs::Marker`)

### Parameters

1. `<static>` (`bool`, default: `false`) \
  Create a static wave field if set to `true`.  

2. `<update_rate>` (`double`, default: `30.0`) \
  The rate in Hz at which the wavefield is updated.

3. `<size>` (`Vector2D`, default: `(1000 1000)`) \
  A two component vector for the size of the wave field in each direction.

4. `<cell_count>` (`int`, default: `(50 50)`) \
  A two component vector for the number of grid cells in each direction.

5. `<number>` (`int`, default: `1`) \
  The number of component waves.

6. `<scale>` (`double`, default: `2.0`) \
  The scale between the mean and largest / smallest component waves.

7. `<angle>` (`double`, default: `2*pi/10`) \
  The angle between the mean wave direction and the largest / smallest component waves.

8. `<steepness>` (`double`, default: `1.0`) \
  A parameter in [0, 1] controlling the wave steepness with 1 being steepest.

9. `<amplitude>` (`double`, default: `0.0`) \
  The amplitude of the mean wave in [m].

10. `<period>` (`double`, default: `1.0`) \
  The period of the mean wave in [s].

11. `<phase>` (`double`, default: `0.0`) \
  The phase of the mean wave.

12. `<direction>` (`Vector2D`, default: `(1 0)`) \
  A two component vector specifiying the direction of the mean wave.

13. `<wave_patch>` (`bool`, default: `false`) \
  Display a wave patch marker if set to `true`.

14. `<wave_patch_size>` (`Vector2D`, default: `(4 4)`) \
  A two component vector for the size of the wave marker (in units of the wave grid).


## WavefieldVisualPlugin

A Gazebo visual plugin to synchronise and control a vertex shader rendering Gerstner waves.

### Usage

Add the SDF for the plugin to the `<visual>` element of your wave model.

The SDF parameters specifying the wave are all optional, and normal
use will be overridden.

If this visual is loaded as part of a wave model that also contains
the plugin `WavefieldModelPlugin`, then it will receive a response
to its request for `~/wave_param` and set the wave parameters to be
consistent with the wave generator operating on the physics server.

```xml
<plugin name="wavefield_visual" filename="libWavefieldVisualPlugin.so">
  <static>false</static>
  <wave>
    <number>3</number>
    <scale>1.5</scale>
    <angle>0.4</angle>
    <steepness>1.0</steepness>
    <amplitude>0.4</amplitude>
    <period>8.0</period>
    <direction>1 1</direction>
  </wave>
</plugin>
```

### Subscribed Topics

1. `~/reponse` (`gazebo::msgs::Response`)

2. `~/wave` (`gazebo::msgs::Param_V`)

3. `~/world_stats` (`gazebo::msgs::WorldStatistics`)

4. `/marker` (`ignition::msgs::Marker`)

### Published Topics

1. `~/request` (`gazebo::msgs::Request`)

### Parameters

1. `<static>` (`bool`, default: `false`) \
  Display a static wave field if set to `true`.  

2. `<number>` (`int`, default: `1`) \
  The number of component waves.

3. `<scale>` (`double`, default: `2.0`) \
  The scale between the mean and largest / smallest component waves.

4. `<angle>` (`double`, default: `2*pi/10`) \
  The angle between the mean wave direction and the largest / smallest component waves.

5. `<steepness>` (`double`, default: `1.0`) \
  A parameter in [0, 1] controlling the wave steepness with 1 being steepest.

6. `<amplitude>` (`double`, default: `0.0`) \
  The amplitude of the mean wave in [m].

7. `<period>` (`double`, default: `1.0`) \
  The period of the mean wave in [s].

8. `<phase>` (`double`, default: `0.0`) \
  The phase of the mean wave.

9. `<direction>` (`Vector2D`, default: `(1 0)`) \
  A two component vector specifiying the direction of the mean wave.

 ## HydrodynamicsPlugin

 A Gazebo model plugin to manage buoyancy and hydrodynamic force
 calculations for a buoyant object.

## Usage
 
Add the SDF for the plugin to the `<model>` element of your model.
 
```xml
<plugin name="hydrodynamics" filename="libHydrodynamicsPlugin.so">
  <!-- Wave Model -->
  <wave_model>ocean_waves</wave_model>

  <!-- Hydrodynamics -->
  <damping_on>true</damping_on>
  <viscous_drag_on>true</viscous_drag_on>
  <pressure_drag_on>true</pressure_drag_on>

  <!-- Linear and Angular Damping -->
  <cDampL1>1.0E-6</cDampL1>
  <cDampL2>1.0E-6</cDampL2>
  <cDampR1>1.0E-6</cDampR1>
  <cDampR2>1.0E-6</cDampR2>

  <!-- 'Pressure' Drag -->
  <cPDrag1>1.0E+2</cPDrag1>
  <cPDrag2>1.0E+2</cPDrag2>
  <fPDrag>0.4</fPDrag>
  <cSDrag1>1.0E+2</cSDrag1>
  <cSDrag2>1.0E+2</cSDrag2>
  <fSDrag>0.4</fSDrag>
  <vRDrag>1.0</vRDrag>

  <!-- Markers -->
  <markers>
    <update_rate>30</update_rate>
    <water_patch>false</water_patch>
    <waterline>false</waterline>
    <underwater_surface>false</underwater_surface>
  </markers>
</plugin>
```

## Subscribed Topics

1. `~/hydrodynamics` (`gazebo::msgs::Param_V`)

## Published Topics

1. `/marker` (`ignition::msgs::Marker`)

## Parameters

1. `<wave_model>` (`string`, default: `""`) \
  Name of the wave model referencing the plugin `WavefieldModelPlugin`.

2. `<damping_on>` (`bool`, default: `true`) \
  Set to `false` to disable damping forces.

3. `<viscous_drag_on>` (`bool`, default: `true`) \
  Set to false to disable viscous drag forces.

4. `<pressure_drag_on>` (`bool`, default: `true`) \
  Set to `false` to disable pressure drag forces.

5. `<cDampL1>` (`double`, default: `1.0E-6`) \
  Linear damping coefficient for linear motion.

6. `<cDampL2>` (`double`, default: `1.0E-6`) \
  Quadratic damping coefficient for linear motion.

7. `<cDampR1>` (`double`, default: `1.0E-6`) \
  Linear damping coefficient for angular motion.

8. `<cDampR2>` (`double`, default: `1.0E-6`) \
  Quadratic damping coefficient for angular motion.

9. `<cPDrag1>` (`double`, default: `1.0E+2`) \
  Linear coecoefficientff for positive pressure drag.

10. `<cPDrag2>` (`double`, default: `1.0E+2`) \
  Quadratic coefficient for positive pressure drag.

11. `<fPDrag>` (`double`, default: `0.4`) \
  Exponential coefficient for positive pressure drag.

12. `<cSDrag1>` (`double`, default: `1.0E+2`) \
  Linear coecoefficientff for negative pressure drag.

13. `<cSDrag2>` (`double`, default: `1.0E+2`) \
  Quadratic coefficient for negative pressure drag.

14. `<fSDrag>` (`double`, default: `0.4`) \
  Exponential coefficient for negative pressure drag.

15. `<vRDrag>` (`double`, default: `1.0`) \
  Reference speed for pressure drag.

16. `<update_rate>` (`double`, default: `30.0`) \
  Update rate for publishing visual markers.

17. `<water_patch>` (`bool`, default: `false`) \
  Set true to display water patch visual markers.

18. `<waterline>` (`bool`, default: `false`) \
  Set true to display water line visual markers.

19. `<underwater_surface>` (`bool`, default: `false`) \
  Set true to display underwater surface visual markers.

## Examples

The wiki has details about how to configure and use the plugins:

- [WavefieldPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/WavefieldPlugin)
- [WavefieldVisualPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/WavefieldVisualPlugin)
- [HydrodynamicsPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/HydrodynamicsPlugin)

## Tests

The wiki has details about how to configure and use the plugins:

- [WavefieldPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/WavefieldPlugin)
- [WavefieldVisualPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/WavefieldVisualPlugin)
- [HydrodynamicsPlugin](https://github.com/srmainwaring/asv_wave_sim/wiki/HydrodynamicsPlugin)

## Tests

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
