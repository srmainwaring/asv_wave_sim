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
      // Parameters for trochoid waves
      ROS_INFO("number: %d", config.number);
      ROS_INFO("steepness: %f", config.steepness);
      ROS_INFO("scale: %f", config.scale);
      ROS_INFO("angle: %f", config.angle);
      ROS_INFO("period: %f", config.period);
      ROS_INFO("amplitude: %f", config.amplitude);
      ROS_INFO("direction.x: %f", config.groups.direction.x);
      ROS_INFO("direction.y: %f", config.groups.direction.y);

      // Parameters for FFT waves
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

        // Parameters for trochoid waves
        {
          auto param = wave_msg.add_param();
          param->set_name("number");
          param->mutable_value()->set_type(msgs::Any::INT32);
          param->mutable_value()->set_int_value(config.number);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("scale");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.scale);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("steepness");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.steepness);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("angle");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.angle);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("period");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.period);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("amplitude");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.amplitude);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("direction");
          param->mutable_value()->set_type(msgs::Any::VECTOR3D);
          param->mutable_value()->mutable_vector3d_value()->set_x(config.groups.direction.x);
          param->mutable_value()->mutable_vector3d_value()->set_y(config.groups.direction.y);
          param->mutable_value()->mutable_vector3d_value()->set_z(0);
        }

        // Parameters for FFT waves
        {
          auto param = wave_msg.add_param();
          param->set_name("wind_angle");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wind_angle);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("wind_speed");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wind_speed);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("tile_size");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.tile_size);
        }
        {
          auto param = wave_msg.add_param();
          param->set_name("tile_resolution");
          param->mutable_value()->set_type(msgs::Any::INT32);
          param->mutable_value()->set_int_value(config.tile_resolution);
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
