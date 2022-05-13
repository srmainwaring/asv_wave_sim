# Gazebo Marine

This package contains plugins that support the simulation of waves and surface vessels in Gazebo.  

![Gazebo Marine](https://github.com/srmainwaring/asv_wave_sim/wiki/images/gz-marine-v4b.jpg)

## Notes

***!! Migrating from Ignition Gazebo to Gazebo Sim !!: [A new era for Gazebo](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356)***

The branch `gz-marine` represents a major reworking of the original wave simulation code developed for Gazebo9 and Gazebo11. 

A number of features available in the original version, such as updating parameters via messages, have not been migrated to Gazebo Sim. On the other hand there are new features from the `feature/fft_waves` development branch that have been included, such as Ocean tiling and different wave generation methods.

There are changes in the way that the wave parameters need to be set, and it may not be possible to avoid breaking the existing interface used to specify trochoidal waves.

The library has additional dependencies on two FFT libraries:

- [clMathLibraries/clFFT](https://github.com/clMathLibraries/clFFT)
- [fftw](http://www.fftw.org/)

These can be installed on linux with:

```bash
sudo apt-get update && apt-get install fftw clfft
```

And on macOS with:

```bash
brew fftw3 libclfft-dev libfftw3-dev
```

Aside from adding the option to use a FFT generated wavefield, the major change is in the way that the visuals are generated. Previously the wave displacements for visuals were generated in the shader code, the visual plugin was used to update shader parameters for wave amplitudes and frequency. Now the entire mesh for the visual is dynamically updated in the the library then pushed into the rendering engine. This means there is no need to maintain various sized meshes in the media files, however it does require working around Gazebos requirement for static meshes and there is a custom Visual that implements this. The OpenCL FFT library allows this work to be offloaded to the GPU when configured.

## Dependencies

You will need a working installation of Gazebo Garden in order to use this package. This will require a from source build, see the [Gazebo Garden documents](https://gazebosim.org/docs/garden) for details.

The dependency on ROS has been removed.

## Ubuntu (pending tests)

- Ubuntu 20.04
- Gazebo Garden

Install CGAL:

```bash
sudo apt-get install libcgal-dev
```

### macOS

- OSX 11.6.2
- Gazebo Garden

Install CGAL (5.3.1):

```bash
brew install cgal
```

## Installation

We suppose the Gazebo source has been cloned to a developer workspace `~/gz_ws/src`.

### Build Gazebo

On macOS you can build with the `RPATH` settings disabled. This allows you to run Gazebo from the install directory without having to disable SIP. From `~/gz_ws` run:

```bash
colcon build --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DCMAKE_MACOSX_RPATH=FALSE \
-DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib
```

Then source the installation:

```bash
source ./install/setup.zsh
```

### Clone and build the package

Clone the `asv_wave_sim` repository:

```bash
cd ~/gz_ws/src
git clone https://github.com/srmainwaring/asv_wave_sim.git -b gz-marine
```

Compile the package:

```bash
colcon build --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DCMAKE_MACOSX_RPATH=FALSE \
-DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib \
--packages-select ignition-marine1
```

Then re-source the workspace:

```bash
source ./install/setup.zsh
```

## Usage

### Set environment variables

```bash
# for future use - to support multiple Gazebo versions
export IGNITION_VERSION=garden

# not usually required as should default to localhost address
export IGN_IP=127.0.0.1

# ensure gazebo finds the config for this installation
export IGN_CONFIG_PATH=\
$HOME/gz_ws/install/share/ignition

# ensure the model and world files are found
export IGN_GAZEBO_RESOURCE_PATH=\
$HOME/gz_ws/src/asv_wave_sim/ignition-marine-models/models:\
$HOME/gz_ws/src/asv_wave_sim/ignition-marine-models/world_models:\
$HOME/gz_ws/src/asv_wave_sim/ignition-marine-models/worlds
```

## Examples

Launch a Gazebo session.

Server:

```bash
ign gazebo -v4 -s -r waves.sdf
```

Client:

```bash
ign gazebo -v4 -g
```

The session should include a wave field and the floating objects depicted in the image at head of this document.

## Changes

There are some changes to the plugin SDF schema for hydrodynamics and waves.   

### Waves model and visual plugins

- The `filename` and `name` attributes for the wave model and visal plugins have changed.
- The `<size>` element has been renamed to `<tile_size>` and moved into `<waves>`
- The `<cell_count>` element has been moved into `<waves>`
- Add new element `<algorithm>` to specify the wave generation algorithm. Valid options are: `sinusoid`, `trochoid` and `fft`.
- Add new element `<wind_velocity>` for use with the `fft` algorithm.
- Add new element `<wind_speed>` for use with the `fft` algorithm.
- Add new element `<wind_angle_deg>` for use with the `fft` algorithm.

```xml
<plugin
    filename="ignition-marine1-waves-model-system"
    name="ignition::gazebo::systems::WavesModel">
    <static>0</static>
    <update_rate>30</update_rate>
    <wave>
      <!-- Algorithm: sinusoid, trochoid, fft  -->
      <algorithm>fft</algorithm>

      <!-- Cell count must be a power of 2 for fft waves -->
      <tile_size>256</tile_size>
      <cell_count>128</cell_count>
      
      <!-- `fft` waves parameters -->
      <wind_speed>5.0</wind_speed>
      <wind_angle_deg>45</wind_angle_deg>
      <steepness>1</steepness>

      <!-- `trochoid` waves parameters -->
      <number>3</number>
      <scale>1.5</scale>
      <angle>0.4</angle>
      <amplitude>0.4</amplitude>
      <period>8.0</period>
      <phase>0.0</phase>
      <steepness>1.0</steepness>
      <direction>1 0</direction>
    </wave>
</plugin>
```

### Hydrodynamics plugin

- The `filename` and `name` attributes for the hydrodynamics plugin have changed.
- The hydrodynamics parameters are now scoped in an additional `<hydrodynamics>` element.
- The `<wave_model>` element is not currently used.
- The `<markers>` element is not currently used (not implemented).

```xml
<plugin
  filename="ignition-marine1-hydrodynamics-system"
  name="ignition::gazebo::systems::Hydrodynamics">

   <!-- Hydrodynamics -->
   <hydrodynamics>
    <damping_on>1</damping_on>
    <viscous_drag_on>1</viscous_drag_on>
    <pressure_drag_on>1</pressure_drag_on>

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
  </hydrodynamics>
</plugin>
```

## Tests

```bash
# build with tests
$ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib -DBUILD_TESTING=ON --packages-select ignition-marine1

# run tests
colcon test --merge-install 

# check results
colcon test-results --all --verbose 
```

Testing within a project build directory

```bash
$ cd ~/gz_ws/src/asv_wave_sim/gz-marine
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
$ make && make test
```

## Build Status

### Develop Job Status

|    | Melodic |
|--- |--- |
| asv_wave_sim | [![Build Status](https://travis-ci.com/srmainwaring/asv_wave_sim.svg?branch=feature%2Ffft_waves)](https://travis-ci.com/srmainwaring/asv_wave_sim) |


### Release Job Status

|    | Melodic |
|--- |--- |
| asv_wave_sim | [![Build Status](https://travis-ci.com/srmainwaring/asv_wave_sim.svg?branch=master)](https://travis-ci.com/srmainwaring/asv_wave_sim) |


## License

This is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the [GNU General Public License](LICENSE) for more details.

This project makes use of other open source software, for full details see the file [LICENSE_THIRDPARTY](LICENSE_THIRDPARTY).

## Acknowledgments

- Jacques Kerner's two part blog describing boat physics for games: [Water interaction model for boats in video games](https://www.gamasutra.com/view/news/237528/Water_interaction_model_for_boats_in_video_games.php) and [Water interaction model for boats in video games: Part 2](https://www.gamasutra.com/view/news/263237/Water_interaction_model_for_boats_in_video_games_Part_2.php).
- The [CGAL](https://doc.cgal.org) libraries are used for the wave field and model meshes.
- The [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) package for the orginal vertex shaders used in the wave field visuals.
- The [VMRC](https://bitbucket.org/osrf/vmrc) package for textures and meshes used in the wave field visuals.
- Jerry Tessendorf's paper on	[Simulating Ocean Water](https://people.cs.clemson.edu/~jtessen/reports/papers_files/coursenotes2004.pdf)
- Curtis Mobley's web book [Ocean Optics](https://www.oceanopticsbook.info/) in particular the section on [Modeling Sea Surfaces](https://www.oceanopticsbook.info/view/surfaces/level-2/modeling-sea-surfaces) and [example IDL code](https://www.oceanopticsbook.info/packages/iws_l2h/conversion/files/IDL-SurfaceGenerationCode.zip)  