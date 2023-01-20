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

#include <Eigen/Dense>

#include <chrono>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

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
  void Init(EntityComponentManager &ecm);

  /// \brief Update the physics.
  void Update(const UpdateInfo &info, EntityComponentManager &ecm);

  /// \brief Initialize the wavefield.
  bool InitWavefield(EntityComponentManager &ecm);

  /// \brief Model interface.
  sim::Model model_{kNullEntity};

  /// \brief Initialization flag.
  bool initialized_{false};

  /// \brief Set to true during Init if the system configuration
  ///        is valid and the updates can run.
  bool valid_config_{false};

  /// \brief The wavefield entity for this system.
  Entity wavefield_entity_{kNullEntity};

  /// \brief The wavefield.
  waves::WavefieldConstWeakPtr wavefield_;
};

//////////////////////////////////////////////////
void WavesClient::Impl::Init(EntityComponentManager &ecm)
{
  if(!InitWavefield(ecm))
    return;

  valid_config_ = true;
}

/////////////////////////////////////////////////
bool WavesClient::Impl::InitWavefield(EntityComponentManager &ecm)
{
  /// \todo - remove hardcoded name
  // Retrieve the wavefield entity using the Name component
  std::string entityName = "wavefield";
  wavefield_entity_ = ecm.EntityByComponents(components::Name(entityName));
  // this->wavefield_entity_ =
  //    ecm.EntityByComponents(waves::components::Wavefield());
  if (wavefield_entity_ == kNullEntity)  
  {
    gzwarn << "No wavefield found.\n";
    return false;
  }

  auto comp = ecm.Component<waves::components::Wavefield>(wavefield_entity_);
  if (comp)
  {
    wavefield_ = comp->Data();
  }

  if (!wavefield_.lock())
  {
    gzwarn << "Invalid wavefield.\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void WavesClient::Impl::Update(const UpdateInfo &info,
    EntityComponentManager &/*ecm*/)
{
  // wave height at the origin
  double simTime = std::chrono::duration<double>(info.simTime).count();
  Eigen::Vector3d point(0.0, 0.0, 0.0);
  double waveHeight{0.0};
  wavefield_.lock()->Height(point, waveHeight);

  gzmsg << "[" << simTime << "] : " << waveHeight << "\n";
}


//////////////////////////////////////////////////
/////////////////////////////////////////////////
WavesClient::~WavesClient() = default;

/////////////////////////////////////////////////
WavesClient::WavesClient() : System(),
    impl_(std::make_unique<WavesClient::Impl>())
{
}

/////////////////////////////////////////////////
void WavesClient::Configure(
    const Entity &entity,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &ecm,
    EventManager &/*_eventMgr*/)
{
  GZ_PROFILE("WavesClient::Configure");

  // Capture the model entity
  impl_->model_ = Model(entity);
  if (!impl_->model_.Valid(ecm))
  {
    gzerr << "The WavesClient system must be attached to a model entity.\n";
    return;
  }
}

//////////////////////////////////////////////////
void WavesClient::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
  GZ_PROFILE("WavesClient::PreUpdate");

  /// \todo(srmainwaring) support reset / rewind
  if (info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(info.dt).count()
        << "s]. System may not work properly." << "\n";
  }

  if (!impl_->initialized_)
  {
    // We call Init here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    impl_->Init(ecm);
    impl_->initialized_ = true;
  }

  if (info.paused)
    return;

  if (impl_->initialized_ && impl_->valid_config_)
    impl_->Update(info, ecm);
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(
    WavesClient,
    gz::sim::System,
    WavesClient::ISystemConfigure,
    WavesClient::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    WavesClient,
    "gz::sim::systems::WavesClient")
