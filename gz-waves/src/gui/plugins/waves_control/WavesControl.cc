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

#include <gz/msgs/any.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>

#include <gz/transport/Node.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>

#include <mutex>
#include <string>

namespace gz
{
namespace sim
{
inline namespace GZ_WAVES_VERSION_NAMESPACE
{
  /// \brief Private data class for WavesControl
  class WavesControlPrivate
  {
    /// \brief Publish a wave parameters message to /world/<world>/waves
    public: void PublishWaveParams();

    /// \brief Publish a wave markers message to /world/<world>/waves/markers
    public: void PublishWaveMarkers();

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publish to topic /world/<world>/waves
    public: transport::Node::Publisher pubWaveParams;

    /// \brief Publish to topic /world/<world>/waves/markers
    public: transport::Node::Publisher pubWaveMarkers;

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
    public: double windAngle{135.0};

    /// \brief Wave steepness
    public: double steepness{2.0};

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

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
void WavesControlPrivate::PublishWaveParams()
{
  // parameters
  gz::msgs::Param msg;

  // wind speed
  {
    gz::msgs::Any value;
    value.set_type(gz::msgs::Any::DOUBLE);
    value.set_double_value(this->windSpeed);
    (*msg.mutable_params())["wind_speed"] = value;
  }
  // wind angle
  {
    gz::msgs::Any value;
    value.set_type(gz::msgs::Any::DOUBLE);
    value.set_double_value(this->windAngle);
    (*msg.mutable_params())["wind_angle"] = value;
  }
  // steepness
  {
    gz::msgs::Any value;
    value.set_type(gz::msgs::Any::DOUBLE);
    value.set_double_value(this->steepness);
    (*msg.mutable_params())["steepness"] = value;
  }

  // publish message
  this->pubWaveParams.Publish(msg);
}

//////////////////////////////////////////////////
void WavesControlPrivate::PublishWaveMarkers()
{
  // parameters
  gz::msgs::Param msg;

  // water patch markers
  {
    gz::msgs::Any value;
    value.set_type(gz::msgs::Any::BOOLEAN);
    value.set_bool_value(this->waterPatchCheckboxState);
    (*msg.mutable_params())["water_patch"] = value;
  }
  // waterline markers
  {
    gz::msgs::Any value;
    value.set_type(gz::msgs::Any::BOOLEAN);
    value.set_bool_value(this->waterlineCheckboxState);
    (*msg.mutable_params())["waterline"] = value;
  }
  // underwater surface markers
  {
    gz::msgs::Any value;
    value.set_type(gz::msgs::Any::BOOLEAN);
    value.set_bool_value(this->submergedTriangleCheckboxState);
    (*msg.mutable_params())["underwater_surface"] = value;
  }

  // publish message
  this->pubWaveMarkers.Publish(msg);
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
void WavesControl::Update(const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm)
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

    // Initialise the publishers
    {
      std::string topic("/world/" + this->dataPtr->worldName + "/waves");
      this->dataPtr->pubWaveParams = this->dataPtr->node.Advertise<gz::msgs::Param>(topic);
      if (!this->dataPtr->pubWaveParams)
      {
        gzerr << "Error advertising topic [" << topic << "]\n";
      }
    }

    {
      std::string topic("/world/" + this->dataPtr->worldName + "/waves/markers");
      this->dataPtr->pubWaveMarkers = this->dataPtr->node.Advertise<gz::msgs::Param>(topic);
      if (!this->dataPtr->pubWaveMarkers)
      {
        gzerr << "Error advertising topic [" << topic << "]\n";
      }
    }

    this->dataPtr->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

    // water patch markers
    if (this->dataPtr->waterPatchCheckboxPrevState &&
        !this->dataPtr->waterPatchCheckboxState)
    {
      gzmsg << "Removing water patch markers...\n";
    }

    this->dataPtr->waterPatchCheckboxPrevState =
        this->dataPtr->waterPatchCheckboxState;
    
    // waterline markers
    if (this->dataPtr->waterlineCheckboxPrevState &&
        !this->dataPtr->waterlineCheckboxState)
    {
      gzmsg << "Removing waterline markers...\n";
    }

    this->dataPtr->waterlineCheckboxPrevState =
        this->dataPtr->waterlineCheckboxState;

    // submerged triangle markers
    if (this->dataPtr->submergedTriangleCheckboxPrevState &&
        !this->dataPtr->submergedTriangleCheckboxState)
    {
      gzmsg << "Removing submerged triangle markers...\n";
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
  this->dataPtr->PublishWaveMarkers();
}

//////////////////////////////////////////////////
void WavesControl::OnShowWaterlineMarkers(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->waterlineCheckboxState = _checked;
  this->dataPtr->PublishWaveMarkers();
}

//////////////////////////////////////////////////
void WavesControl::OnShowSubmergedTriangleMarkers(bool _checked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->submergedTriangleCheckboxState = _checked;
  this->dataPtr->PublishWaveMarkers();
}

//////////////////////////////////////////////////
void WavesControl::UpdateWindSpeed(double _windSpeed)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windSpeed = _windSpeed;

  gzmsg << "Wind Speed: " << _windSpeed << "\n";

  this->dataPtr->PublishWaveParams();
}

//////////////////////////////////////////////////
void WavesControl::UpdateWindAngle(double _windAngle)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windAngle = _windAngle;

  gzmsg << "Wind Angle: " << _windAngle << "\n";

  this->dataPtr->PublishWaveParams();
}

//////////////////////////////////////////////////
void WavesControl::UpdateSteepness(double _steepness)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->steepness = _steepness;

  gzmsg << "Steepness: " << _steepness << "\n";

  this->dataPtr->PublishWaveParams();
}

//////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(WavesControl,
              gz::gui::Plugin)
