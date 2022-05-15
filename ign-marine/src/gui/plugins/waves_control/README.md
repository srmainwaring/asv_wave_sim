# GUI system plugin

This example shows how to create a GUI system plugin.

Ignition Gazebo supports any kind of Ignition GUI plugin
(`ignition::gui::Plugin`). Gazebo GUI plugins are a special type of Ignition
GUI plugin which also have access to entity and component updates coming from
the server.

See `WavesControl.hh` for more information.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd examples/plugin/waves_control
mkdir build
cd build
cmake ..
make
~~~

This will generate the `WavesControl` library under `build`.

## Run

Add the library to the path:

~~~
cd src/asv_wave_sim/ign-marine/src/gui/plugin/waves_control
export IGN_GUI_PLUGIN_PATH=`pwd`/build
~~~

Then run a world, for example:

    ign gazebo -v4 waves.sdf

From the GUI plugin menu on the top-right, choose "Waves Control".

You'll see your plugin, displaying the world name `waves`.
