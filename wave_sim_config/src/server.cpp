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
#include "wave_sim_config/WaveConfig.h"
#include "wave_sim_config/WaveSinusoidConfig.h"
#include "wave_sim_config/WaveTileConfig.h"
#include "wave_sim_config/WaveWindConfig.h"

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include <cstdint>
#include <memory>

using namespace gazebo;

namespace wave_sim_config
{
  class WaveSimConfigServer
  {
  public:
    virtual ~WaveSimConfigServer()
    {
      wave_wind_pub_.reset();
      wave_tile_pub_.reset();
      wave_sinusoid_pub_.reset();
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

      // ROS dynamic_reconfigure servers
      {
        wave_pub_ = transport_node_->Advertise<gazebo::msgs::Param_V>("~/wave");

        ros::NodeHandle node_handle("wave_config_server");
        wave_server_.reset(new dynamic_reconfigure::Server<wave_sim_config::WaveConfig>(node_handle));
        dynamic_reconfigure::Server<wave_sim_config::WaveConfig>::CallbackType f
          = boost::bind(&WaveSimConfigServer::waveCallback, this, _1, _2);
        wave_server_->setCallback(f);
      }
      {
        wave_sinusoid_pub_ = transport_node_->Advertise<gazebo::msgs::Param_V>("~/wave/sinusoid");

        ros::NodeHandle node_handle("wave_sinusoid_config_server");
        wave_sinusoid_server_.reset(new dynamic_reconfigure::Server<wave_sim_config::WaveSinusoidConfig>(node_handle));
        dynamic_reconfigure::Server<wave_sim_config::WaveSinusoidConfig>::CallbackType f
          = boost::bind(&WaveSimConfigServer::waveSinusoidCallback, this, _1, _2);
        wave_sinusoid_server_->setCallback(f);
      }
      {
        wave_tile_pub_ = transport_node_->Advertise<gazebo::msgs::Param_V>("~/wave/tile");

        ros::NodeHandle node_handle("wave_tile_config_server");
        wave_tile_server_.reset(new dynamic_reconfigure::Server<wave_sim_config::WaveTileConfig>(node_handle));
        dynamic_reconfigure::Server<wave_sim_config::WaveTileConfig>::CallbackType f
          = boost::bind(&WaveSimConfigServer::waveTileCallback, this, _1, _2);
        wave_tile_server_->setCallback(f);
      }
      {
        wave_wind_pub_ = transport_node_->Advertise<gazebo::msgs::Param_V>("~/wave/wind");

        ros::NodeHandle node_handle("wave_wind_config_server");
        wave_wind_server_.reset(new dynamic_reconfigure::Server<wave_sim_config::WaveWindConfig>(node_handle));
        dynamic_reconfigure::Server<wave_sim_config::WaveWindConfig>::CallbackType f
          = boost::bind(&WaveSimConfigServer::waveWindCallback, this, _1, _2);
        wave_wind_server_->setCallback(f);
      }
    }

  private:
    void waveCallback(wave_sim_config::WaveConfig &config, uint32_t level)
    {
      ROS_INFO("number: %d", config.number);
      ROS_INFO("steepness: %f", config.steepness);
      ROS_INFO("scale: %f", config.scale);
      ROS_INFO("angle: %f", config.angle);
      ROS_INFO("period: %f", config.period);
      ROS_INFO("amplitude: %f", config.amplitude);
      ROS_INFO("direction.x: %f", config.groups.direction.x);
      ROS_INFO("direction.y: %f", config.groups.direction.y);

      publishWaveMsg(config);
    }

    void waveSinusoidCallback(wave_sim_config::WaveSinusoidConfig &config, uint32_t level)
    {
      ROS_INFO("wave_angle: %f", config.wave_angle);
      ROS_INFO("wave_period: %f", config.wave_period);
      ROS_INFO("wave_amplitude: %f", config.wave_amplitude);

      publishWaveSinusoidMsg(config);
    }

    void waveTileCallback(wave_sim_config::WaveTileConfig &config, uint32_t level)
    {
      ROS_INFO("tile_size: %f", config.tile_size);
      ROS_INFO("tile_resolution: %d", config.tile_resolution);      

      publishWaveTileMsg(config);
    }

    void waveWindCallback(wave_sim_config::WaveWindConfig &config, uint32_t level)
    {
      ROS_INFO("wind_angle: %f", config.wind_angle);
      ROS_INFO("wind_speed: %f", config.wind_speed);

      publishWaveWindMsg(config);
    }

    void publishWaveMsg(const wave_sim_config::WaveConfig &config)
    {
      try 
      {
        gazebo::msgs::Param_V msg;

        {
          auto param = msg.add_param();
          param->set_name("number");
          param->mutable_value()->set_type(msgs::Any::INT32);
          param->mutable_value()->set_int_value(config.number);
        }
        {
          auto param = msg.add_param();
          param->set_name("scale");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.scale);
        }
        {
          auto param = msg.add_param();
          param->set_name("steepness");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.steepness);
        }
        {
          auto param = msg.add_param();
          param->set_name("angle");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.angle);
        }
        {
          auto param = msg.add_param();
          param->set_name("period");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.period);
        }
        {
          auto param = msg.add_param();
          param->set_name("amplitude");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.amplitude);
        }
        {
          auto param = msg.add_param();
          param->set_name("direction");
          param->mutable_value()->set_type(msgs::Any::VECTOR3D);
          param->mutable_value()->mutable_vector3d_value()->set_x(config.groups.direction.x);
          param->mutable_value()->mutable_vector3d_value()->set_y(config.groups.direction.y);
          param->mutable_value()->mutable_vector3d_value()->set_z(0);
        }

        // Don't block forever...
        wave_pub_->WaitForConnection(common::Time(1, 0));

        // Publish message (block while message is written)
        wave_pub_->Publish(msg, true);

        ROS_INFO_STREAM("Publishing on topic [" << wave_pub_->GetTopic() << "]");
        ROS_INFO_STREAM(msg.DebugString());
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

    void publishWaveSinusoidMsg(const wave_sim_config::WaveSinusoidConfig &config)
    {
      try 
      {
        gazebo::msgs::Param_V msg;

        {
          auto param = msg.add_param();
          param->set_name("wave_angle");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wave_angle);
        }
        {
          auto param = msg.add_param();
          param->set_name("wave_period");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wave_period);
        }
        {
          auto param = msg.add_param();
          param->set_name("wave_amplitude");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wave_amplitude);
        }

        // Don't block forever...
        wave_sinusoid_pub_->WaitForConnection(common::Time(1, 0));

        // Publish message (block while message is written)
        wave_sinusoid_pub_->Publish(msg, true);

        ROS_INFO_STREAM("Publishing on topic [" << wave_sinusoid_pub_->GetTopic() << "]");
        ROS_INFO_STREAM(msg.DebugString());
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

    void publishWaveTileMsg(const wave_sim_config::WaveTileConfig &config)
    {
      try 
      {
        gazebo::msgs::Param_V msg;

        {
          auto param = msg.add_param();
          param->set_name("tile_size");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.tile_size);
        }
        {
          auto param = msg.add_param();
          param->set_name("tile_resolution");
          param->mutable_value()->set_type(msgs::Any::INT32);
          param->mutable_value()->set_int_value(config.tile_resolution);
        }

        // Don't block forever...
        wave_tile_pub_->WaitForConnection(common::Time(1, 0));

        // Publish message (block while message is written)
        wave_tile_pub_->Publish(msg, true);

        ROS_INFO_STREAM("Publishing on topic [" << wave_tile_pub_->GetTopic() << "]");
        ROS_INFO_STREAM(msg.DebugString());
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

    void publishWaveWindMsg(const wave_sim_config::WaveWindConfig &config)
    {
      try 
      {
        gazebo::msgs::Param_V msg;

        {
          auto param = msg.add_param();
          param->set_name("wind_angle");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wind_angle);
        }
        {
          auto param = msg.add_param();
          param->set_name("wind_speed");
          param->mutable_value()->set_type(msgs::Any::DOUBLE);
          param->mutable_value()->set_double_value(config.wind_speed);
        }

        // Don't block forever...
        wave_wind_pub_->WaitForConnection(common::Time(1, 0));

        // Publish message (block while message is written)
        wave_wind_pub_->Publish(msg, true);

        ROS_INFO_STREAM("Publishing on topic [" << wave_wind_pub_->GetTopic() << "]");
        ROS_INFO_STREAM(msg.DebugString());
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

    // Gazebo
    transport::NodePtr transport_node_;
    transport::PublisherPtr wave_pub_;
    transport::PublisherPtr wave_sinusoid_pub_;
    transport::PublisherPtr wave_tile_pub_;
    transport::PublisherPtr wave_wind_pub_;

    // ROS
    std::unique_ptr<dynamic_reconfigure::Server<wave_sim_config::WaveConfig>> wave_server_;
    std::unique_ptr<dynamic_reconfigure::Server<wave_sim_config::WaveSinusoidConfig>> wave_sinusoid_server_;
    std::unique_ptr<dynamic_reconfigure::Server<wave_sim_config::WaveTileConfig>> wave_tile_server_;
    std::unique_ptr<dynamic_reconfigure::Server<wave_sim_config::WaveWindConfig>> wave_wind_server_;
  };

} // wave_sim_config

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wave_sim_config_server");

  wave_sim_config::WaveSimConfigServer server;
  ros::spin();

  // ros::Rate rate(10);
  // while (ros::ok())
  // {
  //   server.update();
  //   ros::spinOnce();
  //   rate.sleep();
  // }

  return 0;
}
