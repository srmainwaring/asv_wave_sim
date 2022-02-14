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

#include "OceanTile.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
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

class ignition::gazebo::systems::WavesModelPrivate
{
  /// \brief Path to the model
  public: std::string modelPath;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection {nullptr};

  /// \brief Entity id of the visual
  public: Entity entity = kNullEntity;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;

  /////////////////
  // OceanTile

  // using standard (static) common::Mesh
  public: rendering::OceanTilePtr oceanTile;

  /// \brief Used in DynamicMesh example
  public: common::MeshPtr         oceanTileMesh;

  /// \brief Destructor
  public: ~WavesModelPrivate();

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();
};

/////////////////////////////////////////////////
WavesModel::WavesModel()
    : System(), dataPtr(std::make_unique<WavesModelPrivate>())
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

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  // capture entity 
  // this->dataPtr->entity = _entity;
  // auto nameComp = _ecm.Component<components::Name>(_entity);
  // this->dataPtr->visualName = nameComp->Data();

}

//////////////////////////////////////////////////
void WavesModel::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &)
{
  IGN_PROFILE("WavesModel::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
}

//////////////////////////////////////////////////
WavesModelPrivate::~WavesModelPrivate()
{
};

void WavesModelPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(WavesModel,
                    ignition::gazebo::System,
                    WavesModel::ISystemConfigure,
                    WavesModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WavesModel,
  "ignition::gazebo::systems::WavesModel")
