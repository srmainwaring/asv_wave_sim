# Waves Control GUI system plugin

This is a GUI system plugin for controlling the waves environment. It is based on the Gazebo Sim example `gz-sim/examples/plugin/gui_system_plugin` which demonstrates how these plugins can access entity and component updates coming from the server.

See `WavesControl.hh` for more information.

## Build

From the root of the `asv_wave_sim` repository, do the following to build the example:

```bash
$ cd gz-waves/src/gui/plugin/waves_control
$ mkdir build
$ cd build
$ cmake ..
$ make
```

This will generate the `WavesControl` library under `build`.

## Run

Add the library to the `GZ_GUI_PLUGIN_PATH`:

```bash
$ cd gz-waves/src/gui/plugin/waves_control
$ export GZ_GUI_PLUGIN_PATH=$(pwd)/build
```

Then run a world, for example:

```
$ gz sim -v4 waves.sdf
```

From the GUI plugin menu on the top-right, choose "Waves Control".
