#!/bin/bash

# Launch MAVProxy for multiple vehicles using TCP
# 
# Usage
# 
#   $ ./mavproxy-multi-vehicle.sh <N = number of vehicles>
#
# Will start a MAVProxy session where vehicles may be accessed using
# 
#   MAV> vehicle 1
#   ...
#   MAV> vehicle N
#
# The map and console are also loaded
#  

# set the number of vehicles, default is 1
NVEHICLES=1
if [ "$#" -gt "0" ]; then
  NVEHICLES=$1
fi

# make the MAVProxy argument list, ports for each vehicle are offset by 10
MAVPROXY_ARGS=""
for i in $(seq $NVEHICLES); do
  IDX=$(expr $i '-' 1)
  PORT_OFFSET=$(expr ${IDX} '*' 10)
  OUT1_PORT=$(expr ${PORT_OFFSET} '+' 14550)
  OUT2_PORT=$(expr ${PORT_OFFSET} '+' 14551)
  MASTER_PORT=$(expr ${PORT_OFFSET} '+' 5760)
  SITL_PORT=$(expr ${PORT_OFFSET} '+' 5501)

  MAVPROXY_ARGS="${MAVPROXY_ARGS} --out 127.0.0.1:${OUT1_PORT}" 
  MAVPROXY_ARGS="${MAVPROXY_ARGS} --out 127.0.0.1:${OUT2_PORT}" 
  MAVPROXY_ARGS="${MAVPROXY_ARGS} --master tcp:127.0.0.1:${MASTER_PORT}" 
  MAVPROXY_ARGS="${MAVPROXY_ARGS} --sitl 127.0.0.1:${SITL_PORT}" 

  # echo "i: ${i}, PORT_OFFSET: ${PORT_OFFSET}, OUT1_PORT: ${OUT1_PORT}"
done

# add map and console
MAVPROXY_ARGS="${MAVPROXY_ARGS} --map --console"
# echo ${MAVPROXY_ARGS}

# start MAVProxy
mavproxy.py ${MAVPROXY_ARGS}
