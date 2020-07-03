// Copyright (C) 2020  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "wave_sim_config/server.h"
#include "wave_sim_config/WaveSimConfig.h"

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <cstdint>

namespace wave_sim_config
{
} // wave_sim_config

void callback(wave_sim_config::WaveSimConfig &config, uint32_t level)
{
  ROS_INFO("wind_angle: %f", config.wind_angle);
  ROS_INFO("wind_speed: %f", config.wind_speed);
  ROS_INFO("tile_size: %f", config.tile_size);
  ROS_INFO("tile_resolution: %d", config.tile_resolution);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wave_sim_config_server");

  dynamic_reconfigure::Server<wave_sim_config::WaveSimConfig> server;
  dynamic_reconfigure::Server<wave_sim_config::WaveSimConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);

  server.setCallback(f);

  ROS_INFO("Start spinning");
  ros::spin();

  return 0;
}
