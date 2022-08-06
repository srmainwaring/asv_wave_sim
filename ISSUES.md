# Issues

System: macOS Monterey Version 12.4, MacBook Pro (16 inch, 2021), Apple M1 Max

## Plugin loading order

If the server is run before the client we see a crash:

```bash
# server
gz sim -v4 -s -r waves.sdf

# client
% gz sim -v4 -g
[Msg] Gazebo Sim GUI    v7.0.0~pre1
...
[GUI] [Dbg] [GuiRunner.cc:362] Loaded system [gz::sim::systems::WavesVisual] for entity [6] in GUI
[GUI] [Wrn] [Application.cc:811] [QT] file::/Gazebo/GazeboDrawer.qml:147:3: QML Dialog: Binding loop detected for property "implicitHeight"
[GUI] [Wrn] [Application.cc:811] [QT] file::/WorldStats/WorldStats.qml:53:3: QML RowLayout: Binding loop detected for property "x"
[GUI] [Wrn] [Application.cc:811] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:52:3: QML RenderWindowOverlay: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[GUI] [Wrn] [Application.cc:811] [QT] file::/EntityContextMenuPlugin/EntityContextMenuPlugin.qml:67:3: QML EntityContextMenu: Detected anchors on an item that is managed by a layout. This is undefined behavior; use Layout.alignment instead.
[GUI] [Dbg] [MinimalScene.cc:641] Creating gz-renderering interface for Metal
[GUI] [Dbg] [MinimalScene.cc:794] Creating render thread interface for Metal
[GUI] [Msg] Loading plugin [gz-rendering-ogre2]
[GUI] [Dbg] [MinimalScene.cc:583] Create scene [scene]
[GUI] [Err] [BaseStorage.hh:645] Cannot add item created by another render-engine
[GUI] [Err] [BaseStorage.hh:645] Cannot add item created by another render-engine
Stack trace (most recent call last):
#6    Object "QtCore", at 0x10862b34b, in QObject::event(QEvent*) + 595
#5    Object "libMinimalScene.dylib", at 0x1235e6c33, in gz::gui::plugins::RenderWindowItem::Ready() + 231
#4    Object "libMinimalScene.dylib", at 0x1235e60ff, in gz::gui::plugins::RenderThread::Initialize() + 47
#3    Object "libMinimalScene.dylib", at 0x1235e4b37, in gz::gui::plugins::IgnRenderer::Initialize() + 1735
#2    Object "libsystem_platform.dylib", at 0x1a46f74a3, in _sigtramp + 55
#1    Object "libgz-tools2-backward.dylib", at 0x101093583, in backward::SignalHandling::sig_handler(int, __siginfo*, void*) + 19
#0    Object "libgz-tools2-backward.dylib", at 0x1010935db, in backward::SignalHandling::handleSignal(int, __siginfo*, void*) + 59
zsh: segmentation fault  gz sim -v4 -g
```

This is because on the M1 mac the plugin `gz::sim::systems::WavesVisual` is being loaded before `MinimalScene`.

Reversing the order offers a temporary fix, but note that this does not work when in debug mode.


## Shader params attempting to load glsl shaders on Metal graphics API

The issue seems to arise in `void ShaderParamPrivate::OnUpdate()` where the `glsl` shaders are set if they appear in the SDF regardless whether the `OpenGL` graphics API is used or not. This will cause compilation errors on macOS M1.


Proposed fix in `gz-sim` e153b651132221efa7f497dde8a34bbb7a3765cb

## Use Ogre inverse_transpose_world_matrix

The custom wave shaders calculate `inverse_transpose_world_matrix` (ie. the normal matrix) in the vertex shader. Use the Ogre supplied automatic variable instead.



## Plugin load order - TestVisual plugin experiments

**Objective:** create a minimal visual system plugin and attempt to replicate the `Cannot add item created by another render-engine' error observed in the waves visual plugin.
