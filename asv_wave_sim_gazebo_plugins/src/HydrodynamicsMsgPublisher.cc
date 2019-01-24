// Copyright (C) 2019  Rhys Mainwaring
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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/lexical_cast/try_lexical_convert.hpp>
#include <boost/program_options.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

using namespace gazebo;
namespace po = boost::program_options;

///////////////////////////////////////////////////////////////////////////////
/// Stack Overflow
/// Accepting negative doubles with boost::program_options
/// Answer: Aleksey Vitebskiy 2016
/// <https://stackoverflow.com/questions/4107087/accepting-negative-doubles-with-boostprogram-options>
///
std::vector<po::option> ignore_numbers(std::vector<std::string>& args)
{
  std::vector<po::option> result;
  int pos = 0;
  while(!args.empty())
  {
    const auto& arg = args[0];
    double num;
    if (boost::conversion::try_lexical_convert(arg, num))
    {
      result.push_back(po::option());
      po::option& opt = result.back();

      opt.position_key = pos++;
      opt.value.push_back(arg);
      opt.original_tokens.push_back(arg);

      args.erase(args.begin());
    }
    else
    {
      break;
    }
  }

  return result;
}

///////////////////////////////////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  try 
  {
    // Copyright notice.
    std::cout
      << "ASV Wave Simulator: hydrodynamics parameter publisher.\n"
      << "Copyright (C) 2019  Rhys Mainwaring.\n"
      << "Released under the GNU General Public License.\n\n";

    // Program options
    po::options_description options("Publish hydrodynamics parameters to gztopic \"~/<model>/hydrodynamics\"");

    options.add_options()
      ("help,h", 
        "Dispay this help screen.")
      ("model", po::value<std::string>(),
        "The name of the model to receive the parameters.")
      ("damping_on", po::value<bool>(),
        "Set to false to disable damping forces.")
      ("viscous_drag_on", po::value<bool>(),
        "Set to false to disable viscous drag forces.")
      ("pressure_drag_on", po::value<bool>(),
        "Set to false to disable pressure drag forces.")
      ("cDampL1", po::value<double>(),
        "Linear damping coeff for linear motion.")
      ("cDampL2", po::value<double>(),
        "Quadratic damping coeff for linear motion.")
      ("cDampR1", po::value<double>(),
        "Linear damping coeff for angular motion.")
      ("cDampR2", po::value<double>(),
        "Quadratic damping coeff for angular motion.")
      ("cPDrag1", po::value<double>(),
        "Linear coeff for positive pressure drag.")
      ("cPDrag2", po::value<double>(),
        "Quadratic coeff for positive pressure drag.")
      ("fPDrag", po::value<double>(),
        "Exponent coeff for positive pressure drag.")
      ("cSDrag1", po::value<double>(),
        "Linear coeff for negative pressure drag.")
      ("cSDrag2", po::value<double>(),
        "Quadratic coeff for negative pressure drag.")
      ("fPDrag", po::value<double>(),
        "Exponent coeff for negative pressure drag.")
      ("vRDrag", po::value<double>(),
        "Reference speed for pressure drag.");

    po::variables_map vm;
    // po::store(po::parse_command_line(_argc, _argv, options), vm);
    po::store(po::command_line_parser(_argc, _argv)
      .extra_style_parser(&ignore_numbers)
      .options(options)
      .run(), vm);

    po::notify(vm);

    if (vm.count("help") || vm.empty())
    {
      std::cout << options << std::endl;
      return 0;
    }

    std::string modelName;
    if (vm.count("model"))
    {
      modelName = vm["model"].as<std::string>();
    }

    // Transport
    transport::init();
    transport::run();
    transport::NodePtr node(new transport::Node());
    node->Init();

    std::string topic("~");
    if (!modelName.empty())
    {
      topic.append("/").append(modelName);
    }
    topic.append("/hydrodynamics");
    transport::PublisherPtr hydroPub =
      node->Advertise<gazebo::msgs::Param_V>(topic);

    // Message
    gazebo::msgs::Param_V hydroMsg;
    if (vm.count("damping_on"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("damping_on");
      nextParam->mutable_value()->set_type(msgs::Any::BOOLEAN);
      nextParam->mutable_value()->set_bool_value(vm["damping_on"].as<bool>());
    }
    if (vm.count("viscous_drag_on"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("viscous_drag_on");
      nextParam->mutable_value()->set_type(msgs::Any::BOOLEAN);
      nextParam->mutable_value()->set_bool_value(vm["viscous_drag_on"].as<bool>());
    }
    if (vm.count("pressure_drag_on"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("pressure_drag_on");
      nextParam->mutable_value()->set_type(msgs::Any::BOOLEAN);
      nextParam->mutable_value()->set_bool_value(vm["pressure_drag_on"].as<bool>());
    }
    if (vm.count("cDampL1"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cDampL1");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cDampL1"].as<double>());
    }
    if (vm.count("cDampL2"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cDampL2");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cDampL2"].as<double>());
    }
    if (vm.count("cDampR1"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cDampR1");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cDampR1"].as<double>());
    }
    if (vm.count("cDampR2"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cDampR2");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cDampR2"].as<double>());
    }
    if (vm.count("cPDrag1"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cPDrag1");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cPDrag1"].as<double>());
    }
    if (vm.count("cPDrag2"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cPDrag2");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cPDrag2"].as<double>());
    }
    if (vm.count("fPDrag"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("fPDrag");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["fPDrag"].as<double>());
    }
    if (vm.count("cSDrag1"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cSDrag1");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cSDrag1"].as<double>());
    }
    if (vm.count("cSDrag2"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("cSDrag2");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["cSDrag2"].as<double>());
    }
    if (vm.count("fSDrag"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("fSDrag");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["fSDrag"].as<double>());
    }
    if (vm.count("vRDrag"))
    {
      auto nextParam = hydroMsg.add_param();
      nextParam->set_name("vRDrag");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["vRDrag"].as<double>());
    }
    
    // Don't block forever...
    hydroPub->WaitForConnection(common::Time(1, 0));

    // Publish message (block while message is written)
    hydroPub->Publish(hydroMsg, true);

    std::cout << "Publishing on topic [" << hydroPub->GetTopic() << "]" << std::endl;
    std::cout << hydroMsg.DebugString() << std::endl;

    // Tear down
    hydroPub.reset();
    transport::fini();    
  }
  catch(const gazebo::common::Exception &_e)
  {
    std::cout << _e.GetErrorStr() << std::endl;
    transport::fini();
    return -1;
  }
  catch(const std::exception &_e)
  {
    std::cout << _e.what() << std::endl;
    transport::fini();
    return -1;
  }
  catch(...)
  {
    std::cout << "Unknown Error" << std::endl;
    transport::fini();
    return -1;
  }

  return 0;
}
