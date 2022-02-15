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

#include "Hydrodynamics.hh"

#include "ignition/marine/Wavefield.hh"
#include "ignition/marine/components/Wavefield.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Element.hh>

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::HydrodynamicsPrivate
{
  /// \brief Destructor
  public: ~HydrodynamicsPrivate();

  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  /// \param[in] _sdf Pointer to sdf::Element that contains configuration
  /// parameters for the system.
  public: void Load(EntityComponentManager &_ecm);

  /// \brief Calculate and update the hydrodynamics for the link.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateHydrodynamics(const UpdateInfo &_info,
                    EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdf;

  /// \brief Initialization flag
  public: bool initialized{false};

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief The wavefield entity for this system
  public: Entity wavefieldEntity{kNullEntity};

  /// \brief The wavefield.
  public: marine::WavefieldPtr wavefield;
};

/////////////////////////////////////////////////
Hydrodynamics::Hydrodynamics() : System(),
    dataPtr(std::make_unique<HydrodynamicsPrivate>())
{
}

/////////////////////////////////////////////////
Hydrodynamics::~Hydrodynamics()
{
}

/////////////////////////////////////////////////
void Hydrodynamics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  IGN_PROFILE("Hydrodynamics::Configure");

  ignmsg << "Hydrodynamics: configuring\n";

  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "The Hydrodynamics system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdf = _sdf->Clone();
}

//////////////////////////////////////////////////
void Hydrodynamics::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Hydrodynamics::PreUpdate");

  /// \todo(anyone) support reset / rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm);
    this->dataPtr->initialized = true;
  }

  if (_info.paused)
    return;

  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->UpdateHydrodynamics(_info, _ecm);
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
HydrodynamicsPrivate::~HydrodynamicsPrivate()
{
};

/////////////////////////////////////////////////
void HydrodynamicsPrivate::Load(EntityComponentManager &_ecm)
{
  // Create a new entity and register a wavefield component with it.
  this->wavefieldEntity = _ecm.EntityByComponents(components::Name("WAVEFIELD"));
  // this->wavefieldEntity = _ecm.EntityByComponents(marine::components::Wavefield());
  if (this->wavefieldEntity == kNullEntity)  
  {
    ignwarn << "No wavefield found, no hydrodynamics forces will be calculated\n";
    return;
  }

  auto comp = _ecm.Component<marine::components::Wavefield>(this->wavefieldEntity);
  if (comp)
  {
    this->wavefield = comp->Data();
  }

  if (!this->wavefield)
  {
    ignwarn << "Invalid wavefield, no hydrodynamics forces will be calculated\n";
    return;
  }

  this->validConfig = true;
}

/////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateHydrodynamics(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  double simTime = std::chrono::duration<double>(_info.simTime).count();

  // get the wave height at the origin
  marine::Point3 point(0.0, 0.0, 0.0);
  double waveHeight{0.0};
  this->wavefield->Height(point, waveHeight);

  ignmsg << "[" << simTime << "] : " << waveHeight << "\n";  
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(Hydrodynamics,
                    ignition::gazebo::System,
                    Hydrodynamics::ISystemConfigure,
                    Hydrodynamics::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Hydrodynamics,
  "ignition::gazebo::systems::Hydrodynamics")
