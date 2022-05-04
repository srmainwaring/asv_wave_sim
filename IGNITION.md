
# Ignition Migration

Notes concerning the migration to Ignition

System:
- macOS Big Sur 11.6.2
- Xcode: 13.2.1

## Branches

| feature | base | description |
| --- | --- | --- |
| fft_waves | master| Legacy Gazebo11 version. Includes FFT wave generator, tiling, dynamic reconfigure (ROS). Shader uses InverseTransposeModel matrix to map TBNs |
| gazebo11 | master | legacy Gazebo11 version. Includes early version of FFT wave generator |
| havyard-842 | feature/ign-garden-wip | Add ship model (Havyard 842 tug) |
| ign-garden-dyn-geom | master | Replicate Ogre dynamic geometry sample in Ignition |
| ign-garden-wip | master | Main port to Ignition |
| ign-garden-wip | ign-garden-wip-shaders | Experiments with custom shaders |

## Legacy TBB version (Gazebo11)

The legacy version of plugin and Gazebo depend on an old version of TBB. Set the following environment
variables before running the build.

```bash
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/tbb@2020_u3
export CPATH=${CPATH}:/usr/local/opt/tbb@2020_u3/include
export LIBRARY_PATH=${LIBRARY_PATH}:/usr/local/opt/tbb@2020_u3/lib
```

## Colcon build

If you do not want to disable SIP on macOS, you can provide additional CMake arguments to
control the `RPATH` settings:

```
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib
```

## Ignition environment variables

Supposing that the colcon workspace directory containing the `src` folder is `$HOME/wave_sim_ws`,
then the following environment variables should be set to enable Gazebo to locate the plugins
and models:

```bash
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:\
$HOME/wave_sim_ws/src/asv_wave_sim/asv_wave_sim_gazebo/models:\
$HOME/wave_sim_ws/src/asv_wave_sim/asv_wave_sim_gazebo/world_models:\
$HOME/wave_sim_ws/src/asv_wave_sim/asv_wave_sim_gazebo/worlds

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}:\
$HOME/wave_sim_ws/src/asv_wave_sim/ign-marine/build/lib
```

## CGAL library usage

The CGAL library is used for the following calculations:

`Algorithm`
- CGAL not used

`Convert`
- types: Point3, Vector2, Vector3
- functions: ignition::math <=> CGAL

`Geometry`
- types: Triangle, Point3, Vector2, Vector3, Direction3, Mesh, AABBTree
- functions:  


## Ogre2 HLMS Guides

The list below contains links to articles in the Ogre 2+ forum that discuss the Hlms. 

- [[2.1] Hlms - Create basic vertex shader](https://forums.ogre3d.org/viewtopic.php?f=2&t=85410&p=524471#p524412)

- [Matching Hlms PBS struct in GLSL/HLSL to C++](https://forums.ogre3d.org/viewtopic.php?f=25&t=84066)

- [[Solved][2.1] Trying to create a new HLMS](https://forums.ogre3d.org/viewtopic.php?f=25&t=83763&p=519279#p519340)

- [Trying to create a new HLMS](http://www.ogre3d.org/forums/viewtopic.php?f=25&t=83763)

- [Tutorial on how to pass additional textures to HLMS PBS](https://forums.ogre3d.org/viewtopic.php?f=25&t=84539)
    - how to modify an Hlms to pass additional textures, and located them in the Hlms shader templates

- [How Does HLMS which Shader Template](https://forums.ogre3d.org/viewtopic.php?f=25&t=84510)
      - describes how Hlms finds the shader templates to compile for a particular Hlms implementation 

- [Easier way to communicate with hlms shader?]

- [Matching Hlms PBS struct in GLSL/HLSL to C++](https://forums.ogre3d.org/viewtopic.php?f=25&t=84066)
    - describes how the `struct Material` in Hlms shader code relates to the HlmsDatablocks

- [Performance Questions]


- [[SOLVED][2.1] How to Fog](https://forums.ogre3d.org/viewtopic.php?f=25&t=82878)
    - discussion and example customising Pbs to add fog.

- [[2.1] Easier way to communicate with hlms shader?](https://forums.ogre3d.org/viewtopic.php?f=25&t=83081#p518819)
    - similar to the above (for fog), but using a HlmsListener

Additional Notes

- Each material (const packed) buffer is 64kb 