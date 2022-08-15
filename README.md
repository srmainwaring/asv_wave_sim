# Gazebo Waves

This package contains plugins that support the simulation of waves and surface vessels in [Gazebo](https://gazebosim.org/home).

![Gazebo Waves](https://github.com/srmainwaring/asv_wave_sim/wiki/images/gz-waves-v5b.jpg)

The latest version represents a major reworking of the wave simulation code originally developed for Gazebo9 and Gazebo11. It complies with the naming conventions used in the community note [*"A new era for Gazebo"*](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356) and targets [Gazebo Garden](https://gazebosim.org/docs/garden) or later.

There are new features including FFT wave generation methods, ocean tiling, and support for the ogre2 render engine. There are some changes in the way that the wave parameters need to be set, but as far possible we have attempted to retain compatibility with the Gazebo9 version. Further details are described below.

## Previous version

The previous version can be obtained by either checking out the tag `v0.1.2` or the branch [`gazebo9`](https://github.com/srmainwaring/asv_wave_sim/tree/gazebo9).

## Dependencies

- Install [Gazebo Garden](https://gazebosim.org/docs/garden) which may need to be built from source.
- The simulation uses the [CGAL](https://www.cgal.org/) library for mesh manipulation and [FFTW](http://www.fftw.org/) to compute Fourier transforms. Both libraries are licensed GPLv3.
- The dependency on ROS has been removed.

### macOS Big Sur Version 11.6.2

```bash
brew install cgal fftw
```

### Ubuntu 22.04

```bash
sudo apt-get install libcgal-dev libfftw3-dev
```

If running on an Ubuntu virtual machine you may need to use software rendering if the hypervisor does not support hardware acceleration for OpenGL 4.2+. Install `mesa-utils` to enable llvmpipe:

```bash
sudo apt-get install mesa-utils
```

To use the llvmpipe software renderer, prefix Gazebo commands with the `LIBGL_ALWAYS_SOFTWARE` environment variable:

```bash
LIBGL_ALWAYS_SOFTWARE=1 gz sim waves.sdf
```

## Installation

We suppose the Gazebo source has been cloned to a developer workspace `~/gz_ws/src`.

### Build Gazebo

On macOS you can run Gazebo from the install directory without having to disable SIP by disabling the `RPATH` option in the build:

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
git clone https://github.com/srmainwaring/asv_wave_sim.git
```

Compile the package:

```bash
colcon build --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DCMAKE_MACOSX_RPATH=FALSE \
-DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib \
--packages-select gz-waves1
```

Then re-source the workspace:

```bash
source ./install/setup.zsh
```

### Build the GUI plugin (optional) 

There is an optional GUI plugin that controls the wave parameters.

```bash
cd ~/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control 
mkdir build && cd build
cmake .. && make
```

## Usage

### Set environment variables

```bash
# for future use - to support multiple Gazebo versions
export GZ_VERSION=garden

# not usually required as should default to localhost address
export GZ_IP=127.0.0.1

# ensure gazebo finds the config for this installation
export GZ_CONFIG_PATH=\
$HOME/gz_ws/install/share/gz

# ensure the model and world files are found
export GZ_SIM_RESOURCE_PATH=\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/worlds

# ensure the gui plugin is found
export GZ_GUI_PLUGIN_PATH=\
$HOME/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build
```

## Examples

Launch a Gazebo session.

Server:

```bash
gz sim -v4 -s -r waves.sdf
```

Client:

```bash
gz sim -v4 -g
```

The session should include a wave field and some floating objects.

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
    filename="gz-waves1-waves-model-system"
    name="gz::sim::systems::WavesModel">
    <static>0</static>
    <update_rate>30</update_rate>
    <wave>
      <!-- Algorithm: sinusoid, trochoid, fft  -->
      <algorithm>fft</algorithm>

      <!-- Cell count must be a power of 2 for fft waves -->
      <tile_size>256</tile_size>
      <cell_count>128</cell_count>
      
      <!-- Either: `fft` waves parameters -->
      <wind_speed>5.0</wind_speed>
      <wind_angle_deg>45</wind_angle_deg>
      <steepness>1</steepness>

      <!-- Or: `trochoid` waves parameters -->
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

The waves visual plugin has the same algorithm elements as the model plugin and extra elements to control the shading algorithm. Two approaches are available:

  - `DYNAMIC_GEOMETRY` uses PBS shaders and is suitable for small areas.
  - `DYNAMIC_TEXTURE` uses a custom shader and is suitable for tiled areas.

```xml
<plugin
    filename="gz-waves1-waves-visual-system"
    name="gz::sim::systems::WavesVisual">
  <static>0</static>

  <!-- set the mesh deformation method  -->
  <mesh_deformation_method>DYNAMIC_GEOMETRY</mesh_deformation_method>

  <!-- number of additional tiles along each axis -->
  <tiles_x>-1 1</tiles_x>
  <tiles_y>-1 1</tiles_y>
  <wave>
    <!-- `fft` wave parameters -->
    <algorithm>fft</algorithm>
    <tile_size>100</tile_size>
    <cell_count>256</cell_count>
    <wind_speed>5</wind_speed>
    <wind_angle_deg>135</wind_angle_deg>
    <steepness>2</steepness>
  </wave>

  <!--
    Shader parameters only apply when using DYNAMIC_TEXTURE
  -->

  <!-- shader program -->
  <shader language="glsl">
    <vertex>materials/waves_vs.glsl</vertex>
    <fragment>materials/waves_fs.glsl</fragment>
  </shader>
  <shader language="metal">
    <vertex>materials/waves_vs.metal</vertex>
    <fragment>materials/waves_fs.metal</fragment>
  </shader>

  <!-- vertex shader params -->
  <param>
    <shader>vertex</shader>
    <name>world_matrix</name>
  </param>
  <param>
    <shader>vertex</shader>
    <name>worldviewproj_matrix</name>
  </param>
  <param>
    <shader>vertex</shader>
    <name>camera_position</name>
  </param>
  <param>
    <shader>vertex</shader>
    <name>rescale</name>
    <value>0.5</value>
    <type>float</type>
  </param>
  <param>
    <shader>vertex</shader>
    <name>bumpScale</name>
    <value>64 64</value>
    <type>float_array</type>
  </param>
  <param>
    <shader>vertex</shader>
    <name>bumpSpeed</name>
    <value>0.01 0.01</value>
    <type>float_array</type>
  </param>
  <param>
    <shader>vertex</shader>
    <name>t</name>
    <value>TIME</value>
  </param>

  <!-- pixel shader params -->
  <param>
    <shader>fragment</shader>
    <name>deepColor</name>
    <value>0.0 0.05 0.2 1.0</value>
    <type>float_array</type>
  </param>
  <param>
    <shader>fragment</shader>
    <name>shallowColor</name>
    <value>0.0 0.1 0.3 1.0</value>
    <type>float_array</type>
  </param>
  <param>
    <shader>fragment</shader>
    <name>fresnelPower</name>
    <value>5.0</value>
    <type>float</type>
  </param>
  <param>
    <shader>fragment</shader>
    <name>hdrMultiplier</name>
    <value>0.4</value>
    <type>float</type>
  </param>
  <param>
    <shader>fragment</shader>
    <name>bumpMap</name>
    <value>materials/wave_normals.dds</value>
    <type>texture</type>
    <arg>0</arg>
  </param>
  <param>
    <shader>fragment</shader>
    <name>cubeMap</name>
    <value>materials/skybox_lowres.dds</value>
    <type>texture_cube</type>
    <arg>1</arg>
  </param>

</plugin>
```


### Hydrodynamics plugin

- The `filename` and `name` attributes for the hydrodynamics plugin have changed.
- The hydrodynamics parameters are now scoped in an additional `<hydrodynamics>` element.
- The buoyancy and hydrodynamics forces can be applied to specific entities
in a model using the `<enable>` element. The parameter should be a fully
scoped model entity (model, link or collision name).
- The `<wave_model>` element is not used.

```xml
<plugin
  filename="gz-waves1-hydrodynamics-system"
  name="gz::sim::systems::Hydrodynamics">

  <!-- Apply hydrodynamics to the entire model (default) -->
  <enable>model_name</enable>

  <!-- Or apply hydrodynamics to named links -->
  <enable>model_name::link1</enable>
  <enable>model_name::link2</enable>

  <!-- Or apply hydrodynamics to named collisions -->
  <enable>model_name::link1::collision1</enable>
  <enable>model_name::link1::collision2</enable>

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

  <!-- Control visibility of markers -->
  <markers>
    <update_rate>10</update_rate>
    <water_patch>1</water_patch>
    <waterline>1</waterline>
    <underwater_surface>1</underwater_surface>
  </markers>
</plugin>
```

## Tests

```bash
# build with tests
$ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib -DBUILD_TESTING=ON --packages-select gz-waves1

# run tests
colcon test --merge-install 

# check results
colcon test-result --all --verbose 
```

Testing within a project build directory

```bash
$ cd ~/gz_ws/src/asv_wave_sim/gz-waves
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