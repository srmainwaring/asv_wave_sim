#!/bin/bash

# Configure build dependencies
# https://gazebosim.org/docs/garden/install_osx_src#install-dependencies

# dartsim@6.10.0
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/dartsim@6.10.0
export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/usr/local/opt/dartsim@6.10.0/lib:/usr/local/opt/octomap/local
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/dartsim@6.10.0/lib/pkgconfig
# qt5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/qt@5

# Python scripts installed with: `pip3 install --user <package>`
export PATH=$PATH:$HOME/Library/Python/3.11/bin:$HOME/Library/Python/3.12/bin

# Use desktop_notification- to suppress error:
#   ERROR:colcon.colcon_notification.desktop_notification.terminal_notifier:
#   Could not find the colcon-terminal-notifier.app in the install prefix '/Users/runner/Library/Python/3.12'
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17 -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib --event-handlers desktop_notification-
