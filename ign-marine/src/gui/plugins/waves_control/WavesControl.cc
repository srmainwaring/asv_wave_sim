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

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/gui/GuiEvents.hh>


namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_MARINE_VERSION_NAMESPACE
{
  /// \brief Private data class for WavesControl
  class WavesControlPrivate
  {
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
  };
}
}
}

using namespace ignition;
using namespace gazebo;

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

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

    // water patch markers
    if (this->dataPtr->waterPatchCheckboxPrevState &&
        !this->dataPtr->waterPatchCheckboxState)
    {
      // Remove the markers
      // this->dataPtr->positionMarkerMsg.set_action(
      //   ignition::msgs::Marker::DELETE_ALL);

      ignmsg << "Removing water patch markers...\n";
      // this->dataPtr->node.Request(
      //   "/marker", this->dataPtr->positionMarkerMsg);

      // Change action in case checkbox is checked again
      // this->dataPtr->positionMarkerMsg.set_action(
      //   ignition::msgs::Marker::ADD_MODIFY);
    }

    this->dataPtr->waterPatchCheckboxPrevState =
        this->dataPtr->waterPatchCheckboxState;
    
    // if (!this->dataPtr->waterPatchCheckboxState)
    //   return;

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
}

//////////////////////////////////////////////////
void WavesControl::UpdateWindAngle(double _windAngle)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->windAngle = _windAngle;

  ignmsg << "Wind Angle: " << _windAngle << "\n";
}

//////////////////////////////////////////////////
// Register this plugin
IGNITION_ADD_PLUGIN(WavesControl,
                    ignition::gui::Plugin)
