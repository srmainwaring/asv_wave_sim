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

// #include "../../../include/asv_wave_sim_gazebo_plugins/WaveParameters.hh"
// #include "../../../include/asv_wave_sim_gazebo_plugins/Wavefield.hh"

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

// using namespace asv;

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Modelled on the Wind system, but applied at the model level rather than the world 
class ignition::gazebo::systems::WavesModelPrivate
{
  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  /// \param[in] _sdf Pointer to sdf::Element that contains configuration
  /// parameters for the system.
  public: void Load(EntityComponentManager &_ecm,
                    const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Calculate and update the waves component.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateWaves(const UpdateInfo &_info,
                           EntityComponentManager &_ecm);


  /// \brief Model entity to which this system is attached
  public: Entity modelEntity = kNullEntity;

  /// \brief Wavefield entity on which this system operates (one per model)
  public: Entity wavefieldEntity = kNullEntity;

  ////////// FROM WavefieldEntity

 /// \brief The size of the wavefield
  public: math::Vector2d size;

  /// \brief The number of grid cells in the wavefield
  public: math::Vector2i cellCount;

  /// \brief The wave parameters.
  // public: std::shared_ptr<WaveParameters> waveParams;

  /// \brief The wavefield.
  // public: std::shared_ptr<Wavefield> wavefield;



  /// \brief Path to the model
  public: std::string modelPath;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection {nullptr};


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
//////////////////////////////////////////////////
WavesModelPrivate::~WavesModelPrivate()
{
};

/////////////////////////////////////////////////
void WavesModelPrivate::Load(EntityComponentManager &_ecm,
    const std::shared_ptr<const sdf::Element> &_sdf)
{
#if 0
  Base::Load(_sdf);

  // Wavefield Parameters
  this->data->size      = Utilities::SdfParamVector2(*_sdf, "size",       Vector2(1000, 1000));
  this->data->cellCount = Utilities::SdfParamVector2(*_sdf, "cell_count", Vector2(50, 50));

  // Wave Parameters
  this->data->waveParams.reset(new WaveParameters());
  if (_sdf->HasElement("wave"))
  {
    sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
    this->data->waveParams->SetFromSDF(*sdfWave);
  }

  // @DEBUG_INFO
  // ignmsg << "WavefieldEntity..." <<  std::endl;
  // this->data->waveParams->DebugPrint();
#endif
}

/////////////////////////////////////////////////
void WavesModelPrivate::UpdateWaves(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
#if 0
  // Update the mesh
  double simTime = this->GetWorld()->SimTime().Double();
  this->data->wavefield->Update(simTime);
#endif
}

/////////////////////////////////////////////////
void WavesModelPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // initialise on first pass...
#if 0  
    // Wavefield  
    std::string meshName = "_WAVEFIELD";
    std::string meshPath = "";

    double simTime = this->GetWorld()->SimTime().Double();

// @TODO SWITCH WAVE SIMULATION TYPE
#if 0
    this->data->wavefield.reset(new WavefieldGerstner(
      meshName,
      { this->data->size[0], this->data->size[1] },
      { static_cast<size_t>(this->data->cellCount[0]), static_cast<size_t>(this->data->cellCount[1]) }
    ));
#else
    this->data->wavefield.reset(new WavefieldOceanTile(meshName));

    this->data->wavefield->SetParameters(this->data->waveParams);
    this->data->wavefield->Update(simTime);
#endif
#endif
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(WavesModel,
                    ignition::gazebo::System,
                    WavesModel::ISystemConfigure,
                    WavesModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WavesModel,
  "ignition::gazebo::systems::WavesModel")
