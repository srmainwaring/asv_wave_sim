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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include <cstdint>

using namespace gazebo;

namespace wave_sim_config
{
  class WaveSimConfigServer
  {
  public:
    virtual ~WaveSimConfigServer()
    {
      wave_pub_.reset();
      transport_node_.reset();
      transport::fini();
    }
    
    WaveSimConfigServer()
    {
      // Gazebo transport
      transport::init();
      transport::run();
      transport_node_.reset(new transport::Node());
      transport_node_->Init();

      // Gazebo publisher
      std::string topic("~/wave");
      wave_pub_ = transport_node_->Advertise<gazebo::msgs::Param_V>(topic);

      // ROS dynamic_reconfigure server
      dynamic_reconfigure::Server<wave_sim_config::WaveSimConfig>::CallbackType f
        = boost::bind(&WaveSimConfigServer::configCallback, this, _1, _2);
      server_.setCallback(f);
    }

    void configCallback(wave_sim_config::WaveSimConfig &config, uint32_t level)
    {
      ROS_INFO("wind_angle: %f", config.wind_angle);
      ROS_INFO("wind_speed: %f", config.wind_speed);
      ROS_INFO("tile_size: %f", config.tile_size);
      ROS_INFO("tile_resolution: %d", config.tile_resolution);

      publishGazeboMsg(config);
    }

    void update()
    {
      // ROS_INFO("update...");
    }

    void publishGazeboMsg(const wave_sim_config::WaveSimConfig &config)
    {
      try 
      {
        gazebo::msgs::Param_V wave_msg;

        {
          auto nextParam = wave_msg.add_param();
          nextParam->set_name("wind_angle");
          nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
          nextParam->mutable_value()->set_int_value(config.wind_angle);
        }
        {
          auto nextParam = wave_msg.add_param();
          nextParam->set_name("wind_speed");
          nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
          nextParam->mutable_value()->set_int_value(config.wind_speed);
        }
        {
          auto nextParam = wave_msg.add_param();
          nextParam->set_name("tile_size");
          nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
          nextParam->mutable_value()->set_int_value(config.tile_size);
        }
        {
          auto nextParam = wave_msg.add_param();
          nextParam->set_name("tile_resolution");
          nextParam->mutable_value()->set_type(msgs::Any::INT32);
          nextParam->mutable_value()->set_int_value(config.tile_resolution);
        }

        // Don't block forever...
        wave_pub_->WaitForConnection(common::Time(1, 0));

        // Publish message (block while message is written)
        wave_pub_->Publish(wave_msg, true);

        ROS_INFO_STREAM("Publishing on topic [" << wave_pub_->GetTopic() << "]");
        ROS_INFO_STREAM(wave_msg.DebugString());
      }
      catch(const gazebo::common::Exception &e)
      {
        ROS_WARN_STREAM("" << e.GetErrorStr());
      }
      catch(const std::exception &e)
      {
        ROS_WARN_STREAM("" << e.what());
      }
      catch(...)
      {
        ROS_WARN("Unknown error");
      }
    }

  private:
    // Gazebo
    transport::NodePtr transport_node_;
    transport::PublisherPtr wave_pub_;

    // ROS
    dynamic_reconfigure::Server<wave_sim_config::WaveSimConfig> server_;
  };

} // wave_sim_config

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wave_sim_config_server");

  wave_sim_config::WaveSimConfigServer server;

  ros::Rate rate(10);
  while (ros::ok())
  {

    server.update();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
