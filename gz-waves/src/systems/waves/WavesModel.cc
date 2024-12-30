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

#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>

#include "gz/waves/Utilities.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"
#include "gz/waves/components/Wavefield.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

/// \brief Modelled on the Wind and LiftDrags systems.
/// Applies at the model level rather than the world
class WavesModelPrivate
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

  /// \brief Set the wavefield to be static [false].
  public: bool isStatic{false};

  /// \brief Update rate [Hz].
  public: double updateRate{30.0};

  /// \brief The wave parameters.
  public: waves::WaveParametersPtr waveParams;

  /// \brief The wavefield.
  public: waves::WavefieldPtr wavefield;

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
    EventManager &/*_eventMgr*/)
{
  GZ_PROFILE("WavesModel::Configure");

  gzmsg << "WavesModel: configuring\n";

  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The WavesModel system should be attached to a model entity. "
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
  GZ_PROFILE("WavesModel::PreUpdate");

  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // this->dataPtr->currentSimTime = _info.simTime;

  /// \todo(anyone) support reset / rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
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
  // World name
  std::string worldName;
  _ecm.Each<components::World, components::Name>(
    [&](const Entity &,
        const components::World *,
        const components::Name *_name) -> bool
    {
      // Assume there's only one world
      worldName = _name->Data();
      return false;
    });

  // Update parameters
  this->isStatic = waves::Utilities::SdfParamBool(
      *this->sdf,  "static", this->isStatic);

  this->updateRate = waves::Utilities::SdfParamDouble(
      *this->sdf,  "update_rate", this->updateRate);

  // Wave parameters
  this->waveParams.reset(new waves::WaveParameters());
  if (this->sdf->HasElement("wave"))
  {
    auto sdfWave = this->sdf->GetElement("wave");
    this->waveParams->SetFromSDF(*sdfWave);
  }

  // Wavefield
  std::string entityName = "wavefield";

  this->wavefield.reset(new waves::Wavefield(worldName));
  this->wavefield->SetParameters(this->waveParams);

  // Create a new entity and register a wavefield component with it.
  this->wavefieldEntity = _ecm.CreateEntity();

  _ecm.CreateComponent(this->wavefieldEntity,
      components::Name(entityName));

  auto comp = _ecm.CreateComponent(this->wavefieldEntity,
      waves::components::Wavefield());
  comp->Data() = this->wavefield;

  gzmsg << "WavesModel: created wavefield in entity ["
      << this->wavefieldEntity << "]\n";

  // fetch the wavefield back to check...
  auto wfComp = _ecm.Component<waves::components::Wavefield>(
      this->wavefieldEntity);
  if (!wfComp)
  {
    gzwarn << "WavesModel: could not find wavefield in entity ["
        << this->wavefieldEntity << "]\n";
  }
  else
  {
    gzmsg << "WavesModel: found wavefield with params" <<  std::endl;
    this->waveParams->DebugPrint();
  }

  this->validConfig = true;
}

/////////////////////////////////////////////////
void WavesModelPrivate::UpdateWaves(const UpdateInfo &_info,
    EntityComponentManager &/*_ecm*/)
{
  if (!this->isStatic)
  {
    // Throttle update [30 FPS by default]
    auto updatePeriod = 1.0/this->updateRate;
    double simTime = std::chrono::duration<double>(_info.simTime).count();
    if ((simTime - this->lastUpdateTime) > updatePeriod)
    {
      this->wavefield->Update(simTime);
      this->lastUpdateTime = simTime;
    }
  }
}

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(gz::sim::systems::WavesModel,
              gz::sim::System,
              gz::sim::systems::WavesModel::ISystemConfigure,
              gz::sim::systems::WavesModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::WavesModel,
                   "gz::sim::systems::WavesModel")
