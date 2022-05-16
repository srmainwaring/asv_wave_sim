// Copyright (C) 2022  Rhys Mainwaring
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

#include "WavesControl.hh"

#include <ignition/msgs/any.pb.h>
#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/param_v.pb.h>

#include <ignition/transport/Node.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/gui/GuiEvents.hh>

#include <mutex>
#include <string>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_MARINE_VERSION_NAMESPACE
{
  /// \brief Private data class for WavesControl
  class WavesControlPrivate
  {
    /// \brief Publish a wave parameters message to /world/<world>/waves
    public: void PublishWaveParams();

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publish to topic /world/<world>/waves
    public: transport::Node::Publisher pub;

    /// \brief Current state of the water patch checkbox
    public: bool waterPatchCheckboxState{false};

    /// \brief Previous state of the water patch checkbox
    public: bool waterPatchCheckboxPrevState{false};

    /// \brief Current state of the waterline checkbox
    public: bool waterlineCheckboxState{false};

    /// \brief Previous state of the waterline checkbox
    public: bool waterlineCheckboxPrevState{false};

    /// \brief Current state of the submerged triangle checkbox
    public: bool submergedTriangleCheckboxState{false};

    /// \brief Previous state of the submerged triangle checkbox
    public: bool submergedTriangleCheckboxPrevState{false};

    /// \brief Wind speed
    public: double windSpeed{5.0};

    /// \brief Wind angle
    public: double windAngle{0.0};

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: windSpeed and windAngle
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Name of the world
    public: std::string worldName;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
void WavesControlPrivate::PublishWaveParams()
{
  // parameters
  ignition::msgs::Param msg;

  // wind speed
  {
    ignition::msgs::Any value;
    value.set_type(ignition::msgs::Any::DOUBLE);
    value.set_double_value(this->windSpeed);
    (*msg.mutable_params())["wind_speed"] = value;
  }
  // wind angle
  {
    ignition::msgs::Any value;
    value.set_type(ignition::msgs::Any::DOUBLE);
    value.set_double_value(this->windAngle);
    (*msg.mutable_params())["wind_angle"] = value;
  }

  // publish message
  this->pub.Publish(msg);
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
WavesControl::WavesControl()
  : GuiSystem(), dataPtr(new WavesControlPrivate)
{
}

/////////////////////////////////////////////////
WavesControl::~WavesControl() = default;

/////////////////////////////////////////////////
void WavesControl::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
  {
    this->title = "Waves Control";
  }
}

//////////////////////////////////////////////////
void WavesControl::Update(const ignition::gazebo::UpdateInfo & /*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    // Get the name of the world
    if (this->dataPtr->worldName.empty())
    {
      _ecm.Each<components::World, components::Name>(
        [&](const Entity &,
            const components::World *,
            const components::Name *_name) -> bool
        {
          // Assume there's only one world
          this->dataPtr->worldName = _name->Data();
          return false;
        });
    }

    // Initialise the publisher
    std::string topic("/world/" + this->dataPtr->worldName + "/waves");
    this->dataPtr->pub = this->dataPtr->node.Advertise<ignition::msgs::Param>(topic);
    if (!this->dataPtr->pub)
    {
      ignerr << "Error advertising topic [" << topic << "]\n";
    }

    this->dataPtr->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

    // water patch markers
    if (this->dataPtr->waterPatchCheckboxPrevState &&
        !this->dataPtr->waterPatchCheckboxState)
    {
      ignmsg << "Removing water patch markers...\n";
    }

    this->dataPtr->waterPatchCheckboxPrevState =
        this->dataPtr->waterPatchCheckboxState;
    
    // waterline markers
    if (this->dataPtr->waterlineCheckboxPrevState &&
        !this->dataPtr->waterlineCheckboxState)
    {
      ignmsg << "Removing waterline markers...\n";
    }

    this->dataPtr->waterlineCheckboxPrevState =
        this->dataPtr->waterlineCheckboxState;

    // submerged triangle markers
    if (this->dataPtr->submergedTriangleCheckboxPrevState &&
        !this->dataPtr->submergedTriangleCheckboxState)
    {
      ignmsg << "Removing submerged triangle markers...\n";
    }

    this->dataPtr->submergedTriangleCheckboxPrevState =
        this->dataPtr->submergedTriangleCheckboxState;
  }
}

//////////////////////////////////////////////////
void WavesControl::OnShowWaterPatchMarkers(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->waterPatchCheckboxState = _checked;
}

//////////////////////////////////////////////////
void WavesControl::OnShowWaterlineMarkers(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->waterlineCheckboxState = _checked;
}

//////////////////////////////////////////////////
void WavesControl::OnShowSubmergedTriangleMarkers(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->submergedTriangleCheckboxState = _checked;
}

//////////////////////////////////////////////////
void WavesControl::UpdateWindSpeed(double _windSpeed)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windSpeed = _windSpeed;

  ignmsg << "Wind Speed: " << _windSpeed << "\n";

  this->dataPtr->PublishWaveParams();
}

//////////////////////////////////////////////////
void WavesControl::UpdateWindAngle(double _windAngle)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windAngle = _windAngle;

  ignmsg << "Wind Angle: " << _windAngle << "\n";

  this->dataPtr->PublishWaveParams();
}

//////////////////////////////////////////////////
// Register this plugin
IGNITION_ADD_PLUGIN(WavesControl,
                    ignition::gui::Plugin)
