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

#include "WavesModel.hh"

#include "ignition/marine/OceanTile.hh"
#include "ignition/marine/Utilities.hh"
#include "ignition/marine/Wavefield.hh"
#include "ignition/marine/WaveParameters.hh"

#include "ignition/marine/components/Wavefield.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Name.hh>

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

/// \brief Modelled on the Wind and LiftDrags systems.
/// Applies at the model level rather than the world 
class ignition::gazebo::systems::WavesModelPrivate
{
  /// \brief Destructor
  public: ~WavesModelPrivate();

  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  /// \param[in] _sdf Pointer to sdf::Element that contains configuration
  /// parameters for the system.
  public: void Load(EntityComponentManager &_ecm);

  /// \brief Calculate and update the waves component.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateWaves(const UpdateInfo &_info,
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

  ////////// BEGIN FROM WavefieldEntity

  /// \brief The size of the wavefield
  public: math::Vector2d size;

  /// \brief The number of grid cells in the wavefield
  public: math::Vector2i cellCount;

  /// \brief The wave parameters.
  public: marine::WaveParametersPtr waveParams;

  /// \brief The wavefield.
  public: marine::WavefieldPtr wavefield;

  ////////// END FROM WavefieldEntity

  /// \brief Update rate [Hz].
  public: double updateRate{30.0};

  /// \brief Previous update time.
  public: double lastUpdateTime{0};
};

/////////////////////////////////////////////////
WavesModel::WavesModel() : System(),
    dataPtr(std::make_unique<WavesModelPrivate>())
{
}

/////////////////////////////////////////////////
WavesModel::~WavesModel()
{
}

/////////////////////////////////////////////////
void WavesModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  IGN_PROFILE("WavesModel::Configure");

  ignmsg << "WavesModel: configuring\n";

  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "The WavesModel system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdf = _sdf->Clone();
}

//////////////////////////////////////////////////
void WavesModel::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  IGN_PROFILE("WavesModel::PreUpdate");

  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // this->dataPtr->currentSimTime = _info.simTime;

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
    this->dataPtr->UpdateWaves(_info, _ecm);
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
WavesModelPrivate::~WavesModelPrivate()
{
};

/////////////////////////////////////////////////
void WavesModelPrivate::Load(EntityComponentManager &_ecm)
{
  // Wavefield Parameters
  this->size = marine::Utilities::SdfParamVector2d(
      *this->sdf, "size", math::Vector2d(256.0, 256.0));
  
  this->cellCount = marine::Utilities::SdfParamVector2i(
      *this->sdf, "cell_count", math::Vector2i(128, 128));

  // Wave Parameters
  this->waveParams.reset(new marine::WaveParameters());
  if (this->sdf->HasElement("wave"))
  {
    sdf::ElementPtr sdfWave = this->sdf->GetElement("wave");
    this->waveParams->SetFromSDF(*sdfWave);
  }

  // Wavefield  
  std::string meshName = "WAVEFIELD";

  // double simTime = this->GetWorld()->SimTime().Double();

  this->wavefield.reset(new marine::WavefieldOceanTile(meshName));
  this->wavefield->SetParameters(this->waveParams);
  // this->wavefield->Update(simTime);

  // Create a new entity and register a wavefield component with it.
  this->wavefieldEntity = _ecm.CreateEntity();

  _ecm.CreateComponent(this->wavefieldEntity,
      components::Name(meshName));

  auto comp = _ecm.CreateComponent(this->wavefieldEntity,
      marine::components::Wavefield());
  comp->Data() = this->wavefield;

  ignmsg << "WavesModel: created wavefield in entity ["
      << this->wavefieldEntity << "]\n";

  // fetch the wavefield back to check...
  auto wfComp = _ecm.Component<marine::components::Wavefield>(this->wavefieldEntity);
  if (!wfComp)
  {
    ignwarn << "WavesModel: could not find wavefield in entity ["
        << this->wavefieldEntity << "]\n";
  }
  else
  {
    ignmsg << "WavesModel: found wavefield with params" <<  std::endl;
    this->waveParams->DebugPrint();
  }

  this->validConfig = true;
}

/////////////////////////////////////////////////
void WavesModelPrivate::UpdateWaves(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  /// \todo check if model is static
  {  
    /// \todo improve wave model update performance
    // Throttle update [30 FPS by default]
    auto updatePeriod = 1.0/this->updateRate;
    double simTime = std::chrono::duration<double>(_info.simTime).count();
    if ((simTime - this->lastUpdateTime) > updatePeriod)
    {
      // ignmsg << "[" << simTime << "] updating wave model\n";
      this->wavefield->Update(simTime);
      this->lastUpdateTime = simTime;
    }
  }
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(WavesModel,
                    ignition::gazebo::System,
                    WavesModel::ISystemConfigure,
                    WavesModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WavesModel,
  "ignition::gazebo::systems::WavesModel")
