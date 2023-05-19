#!/bin/bash

# Ship - Quadplane Landing Example
#
# 

# Kill all SITL binaries when exiting
trap "killall -9 arduplane & killall -9 ardurover" SIGINT SIGTERM EXIT

# edit the location of these directories if different
ARDUPILOT_ROOT="$HOME/Code/ardupilot/ardupilot"
SITL_MODELS_DIR="$HOME/Code/ardupilot/simulation/SITL_Models"
GZ_WAVES_MODELS_DIR="$HOME/Code/robotics/gz_waves_ws/src/asv_wave_sim/gz-waves-models"

# assume we start the script from the root directory
ROOTDIR=$ARDUPILOT_ROOT
PLANE=$ROOTDIR/build/sitl/bin/arduplane
ROVER=$ROOTDIR/build/sitl/bin/ardurover

# drones will be located here
# Swansea Bay
HOMELAT=51.525571
HOMELONG=-3.954478
HOMEALT=0.0

# Default: CUAV
# HOMELAT=-35.363262
# HOMELONG=149.165237
# HOMEALT=584.0

#--------------------------------------------------------------------
# ArduPilot SITL

# build binary if not present
[ -x "$PLANE" -a -x "$ROVER" ] || {
    ./waf configure --board sitl
    ./waf plane rover
}

#--------------------------------------------------------------------
# Ship

mkdir -p sitl/ship

SHIP_DEFAULTS="$GZ_WAVES_MODELS_DIR/config/havyard.param"

# additional parameter file for the ship unit
cat <<EOF > sitl/ship/leader.param
SYSID_THISMAV 17
AUTO_OPTIONS 7
EK3_SRC1_POSZ 3
EOF

(cd sitl/ship && $ROVER -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --slave 0 --instance 1 --sysid 17 --defaults $SHIP_DEFAULTS,leader.param) &

#--------------------------------------------------------------------
# Quadplane

mkdir -p sitl/quadplane/scripts

cp $ARDUPILOT_ROOT/libraries/AP_Scripting/examples/plane_ship_landing.lua sitl/quadplane/scripts

QUADPLANE_DEFAULTS="$SITL_MODELS_DIR/Ignition/config/alti_transition_quad.param"

# additional parameter file for the quadplane
cat <<EOF > sitl/quadplane/follower.param
SYSID_THISMAV 1
AUTO_OPTIONS 7
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_Y 0
FOLL_OFS_Z 10
FOLL_OFS_TYPE 1
FOLL_SYSID 17
FOLL_DIST_MAX 1000
FOLL_YAW_BEHAVE 2
FOLL_ALT_TYPE 1
FS_LONG_ACTN 0
Q_RTL_MODE 0
Q_OPTIONS 32
RTL_AUTOLAND 0
SCR_ENABLE 1
SCR_HEAP_SIZE 100000
SCR_DEBUG_OPTS 0
SHIP_ENABLE 1
EOF

(cd sitl/quadplane && $PLANE -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --slave 0 --instance 0 --sysid 1 --defaults $QUADPLANE_DEFAULTS,follower.param) &

wait

