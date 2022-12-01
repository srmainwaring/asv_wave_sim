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

#include "WavesClient.hh"

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/transport.hh>

#include <sdf/Element.hh>

#include "gz/waves/Wavefield.hh"
#include "gz/waves/components/Wavefield.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::WavesClient::Impl
{
public:
  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  void Init(EntityComponentManager &_ecm);

  /// \brief Update the physics.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  void Update(const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Initialize the wavefield.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  bool InitWavefield(EntityComponentManager &_ecm);

  /// \brief Model interface
  sim::Model model{kNullEntity};

  /// \brief Initialization flag
  bool initialized{false};

  /// \brief Set during Load to true if the configuration for the system
  ///        is valid and the post-update can run
  bool validConfig{false};

  /// \brief The wavefield entity for this system
  Entity wavefieldEntity{kNullEntity};

  /// \brief The wavefield.
  waves::WavefieldWeakPtr wavefield;

   /// \brief Name of the world
  std::string worldName;

  /// \brief Previous update time (s).
  double prevTime;

  /// \brief Mutex to protect wave marker updates.
  std::recursive_mutex mutex;

  /// \brief Transport node for wave marker messages
  transport::Node node;
};

//////////////////////////////////////////////////
void WavesClient::Impl::Init(EntityComponentManager &_ecm)
{
  if(!this->InitWavefield(_ecm))
    return;

  this->validConfig = true;
}

/////////////////////////////////////////////////
bool WavesClient::Impl::InitWavefield(EntityComponentManager &_ecm)
{
  /// \todo - remove hardcoded name
  // Retrieve the wavefield entity using the Name component
  std::string entityName = "wavefield";
  this->wavefieldEntity = _ecm.EntityByComponents(components::Name(entityName));
  // this->wavefieldEntity =
  //    _ecm.EntityByComponents(waves::components::Wavefield());
  if (this->wavefieldEntity == kNullEntity)  
  {
    gzwarn << "No wavefield found.\n";
    return false;
  }

  auto comp = _ecm.Component<waves::components::Wavefield>(
      this->wavefieldEntity);
  if (comp)
  {
    this->wavefield = comp->Data();
  }

  if (!this->wavefield.lock())
  {
    gzwarn << "Invalid wavefield.\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void WavesClient::Impl::Update(const UpdateInfo &_info,
    EntityComponentManager &/*_ecm*/)
{
  // Get the wave height at the origin
  double simTime = std::chrono::duration<double>(_info.simTime).count();
  cgal::Point3 point(0.0, 0.0, 0.0);
  double waveHeight{0.0};
  this->wavefield.lock()->Height(point, waveHeight);

  gzmsg << "[" << simTime << "] : " << waveHeight << "\n";
}


//////////////////////////////////////////////////
/////////////////////////////////////////////////
WavesClient::~WavesClient() = default;

/////////////////////////////////////////////////
WavesClient::WavesClient() : System(),
    impl(std::make_unique<WavesClient::Impl>())
{
}

/////////////////////////////////////////////////
void WavesClient::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  GZ_PROFILE("WavesClient::Configure");

  gzmsg << "WavesClient: configuring\n";

  // Get the name of the world
  if (this->impl->worldName.empty())
  {
    _ecm.Each<components::World, components::Name>(
      [&](const Entity &,
          const components::World *,
          const components::Name *_name) -> bool
      {
        // Assume there's only one world
        this->impl->worldName = _name->Data();
        return false;
      });
  }

  // Capture the model entity
  this->impl->model = Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "The WavesClient system must be attached to a model entity. "
        << "Failed to initialize." << "\n";
    return;
  }
}

//////////////////////////////////////////////////
void WavesClient::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("WavesClient::PreUpdate");

  /// \todo(srmainwaring) support reset / rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << "\n";
  }

  if (!this->impl->initialized)
  {
    // We call Init here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->impl->Init(_ecm);
    this->impl->initialized = true;
  }

  if (_info.paused)
    return;

  if (this->impl->initialized && this->impl->validConfig)
  {
    this->impl->Update(_info, _ecm);
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(WavesClient,
              gz::sim::System,
              WavesClient::ISystemConfigure,
              WavesClient::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WavesClient,
                    "gz::sim::systems::WavesClient")
