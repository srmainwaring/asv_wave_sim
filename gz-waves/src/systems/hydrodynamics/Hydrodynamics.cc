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

#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <utility>
#include <vector>
#include <string>

#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport.hh>

#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>

#include "gz/waves/Convert.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/MeshTools.hh"
#include "gz/waves/Physics.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/Utilities.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WavefieldSampler.hh"

#include "gz/waves/components/Wavefield.hh"

#include "Collision.hh"

namespace gz
{
namespace sim
{
namespace systems
{
//////////////////////////////////////////////////
// Utilties

//////////////////////////////////////////////////
/// \brief Transform a meshe's vertex points in the world frame
///        according to a Pose.
///
/// \param[in] _pose    A pose defining a translation and rotation
///                     in the world frame.
/// \param[in] _source  The original mesh to transform.
/// \param[out] _target The transformed mesh.
void ApplyPose(
  const gz::math::Pose3d& _pose,
  const cgal::Mesh& _source,
  cgal::Mesh& _target)
{
  for (
    auto&& it = std::make_pair(std::begin(_source.vertices()),
        std::begin(_target.vertices()));
    it.first != std::end(_source.vertices()) &&
        it.second != std::end(_target.vertices());
    ++it.first, ++it.second)
  {
    const auto& v0 = *it.first;
    const auto& v1 = *it.second;
    const cgal::Point3& p0 = _source.point(v0);

    // Affine transformation
    gz::math::Vector3d gzP0(p0.x(), p0.y(), p0.z());
    gz::math::Vector3d gzP1 = _pose.Rot().RotateVector(gzP0) + _pose.Pos();

    cgal::Point3& p1 = _target.point(v1);
    p1 = cgal::Point3(gzP1.X(), gzP1.Y(), gzP1.Z());
  }
}

//////////////////////////////////////////////////
// NOT REQUIRED: component additions are carried out
//               in Link::EnableVelocityChecks()

#if 0
//////////////////////////////////////////////////
void AddWorldPose(const Entity &_entity, EntityComponentManager &_ecm)
{
  if (!_ecm.Component<components::WorldPose>(_entity))
  {
    _ecm.CreateComponent(_entity, components::WorldPose());
  }
}

//////////////////////////////////////////////////
void AddInertial(const Entity &_entity, EntityComponentManager &_ecm)
{
  if (!_ecm.Component<components::Inertial>(_entity))
  {
    _ecm.CreateComponent(_entity, components::Inertial());
  }
}

//////////////////////////////////////////////////
void AddCollision(const Entity &_entity, EntityComponentManager &_ecm)
{
  if (!_ecm.Component<components::Collision>(_entity))
  {
    _ecm.CreateComponent(_entity, components::Collision());
  }
}

//////////////////////////////////////////////////
void AddWorldLinearVelocity(const Entity &_entity,
    EntityComponentManager &_ecm)
{
  if (!_ecm.Component<components::WorldLinearVelocity>(_entity))
  {
    _ecm.CreateComponent(_entity, components::WorldLinearVelocity());
  }
}

//////////////////////////////////////////////////
void AddWorldAngularVelocity(const Entity &_entity,
    EntityComponentManager &_ecm)
{
  if (!_ecm.Component<components::WorldAngularVelocity>(_entity))
  {
    _ecm.CreateComponent(_entity, components::WorldAngularVelocity());
  }
}

//////////////////////////////////////////////////
void AddAxisAlignedBox(const Entity &_entity,
    EntityComponentManager &_ecm)
{
  if (!_ecm.Component<components::AxisAlignedBox>(_entity))
  {
    _ecm.CreateComponent(_entity, components::AxisAlignedBox());
  }
}
#endif

//////////////////////////////////////////////////
math::AxisAlignedBox CreateAxisAlignedBox(cgal::MeshPtr _mesh)
{
  if (std::begin(_mesh->vertices()) == std::end(_mesh->vertices()))
    return math::AxisAlignedBox();

  auto v0 = std::begin(_mesh->vertices());
  auto& p0 = _mesh->point(*v0);

  double min_x = p0.x();
  double min_y = p0.y();
  double min_z = p0.z();
  double max_x = min_x;
  double max_y = min_y;
  double max_z = min_z;

  for (const auto& vertex : _mesh->vertices())
  {
    auto& point = _mesh->point(vertex);
    min_x = std::min(point.x(), min_x);
    min_y = std::min(point.y(), min_y);
    min_z = std::min(point.z(), min_z);
    max_x = std::max(point.x(), max_x);
    max_y = std::max(point.y(), max_y);
    max_z = std::max(point.z(), max_z);
  }

  math::Vector3d vec1(min_x, min_y, min_z);
  math::Vector3d vec2(max_x, max_y, max_z);
  return math::AxisAlignedBox(vec1, vec2);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////

/// \internal
/// \brief A class to hold data required for hydrodynamics calculations
/// for each link in a model.
class HydrodynamicsLinkData
{
  /// \brief Destructor.
  public: virtual ~HydrodynamicsLinkData()
  {
    for (auto& ptr : this->hydrodynamics)
      ptr.reset();
    for (auto& ptr : this->initLinkMeshes)
      ptr.reset();
    this->wavefieldSampler.reset();
  }

  /// \brief A Link entity.
  // public: sim::Entity link;
  public: sim::Link link{kNullEntity};

  /// \brief The wavefield sampler for this link.
  public: waves::WavefieldSamplerPtr wavefieldSampler;

  /// \brief The initial meshes for this link.
  public: std::vector<cgal::MeshPtr> initLinkMeshes;

  /// \brief The transformed meshes for this link.
  public: std::vector<cgal::MeshPtr> linkMeshes;

  /// \brief The collision entities for this link.
  public: std::vector<Entity> linkCollisions;

  /// \brief Objects to compute the hydrodynamics forces for each link mesh.
  public: std::vector<waves::HydrodynamicsPtr> hydrodynamics;

  /// \brief Marker messages for the water patch.
  public: msgs::Marker waterPatchMsg;

  /// \brief Marker messages for the waterline.
  public: std::vector<msgs::Marker> waterlineMsgs;

  /// \brief Marker messages for the underwater portion of the mesh.
  public: std::vector<msgs::Marker> underwaterSurfaceMsgs;
};

typedef std::shared_ptr<HydrodynamicsLinkData> HydrodynamicsLinkDataPtr;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
class HydrodynamicsPrivate
{
  /// \brief Destructor
  public: ~HydrodynamicsPrivate();

  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void Init(EntityComponentManager &_ecm);

  /// \brief Update the physics and markers.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void Update(const UpdateInfo &_info,
                      EntityComponentManager &_ecm);

  /// \brief Initialize the wavefield.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  private: bool InitWavefield(EntityComponentManager &_ecm);

  /// \brief Initialize the physics.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  private: bool InitPhysics(EntityComponentManager &_ecm);

  /// \brief Calculate and update the physics for the link.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  private: void UpdatePhysics(const UpdateInfo &_info,
                              EntityComponentManager &_ecm);

  /// \brief Check if an entity is enabled or not.
  /// \param[in] _entity Target entity
  /// \param[in] _ecm Entity component manager
  /// \return True if hydrodynamics should be applied.
  public: bool IsEnabled(Entity _entity,
      const EntityComponentManager &_ecm) const;

  /// \brief Iterate over the links in a model, and create a CGAL SurfaceMesh
  /// for each collison in each link.
  ///
  /// \param[in]  _model    The model being processed.
  /// \param[out] _links    A vector holding a copy of pointers to
  ///                       the model's links.
  /// \param[out] _meshes   A vector of vectors containing a surface mesh
  ///                       for each collision in a link.
  /// \param[out] _collisionElements  A vector of vectors containing the
  ///                       collision entities in a link.
  public: void CreateCollisionMeshes(
    EntityComponentManager &_ecm,
    sim::Model _model,
    std::vector<sim::Entity>& _links,
    std::vector<std::vector<cgal::MeshPtr>>& _meshes,
    std::vector<std::vector<Entity>>& _collisions);

  /// \brief Model interface
  public: sim::Model model{kNullEntity};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdf;

  /// \brief General initialisation
  public: bool initialised{false};

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief The wavefield entity for this system
  public: Entity wavefieldEntity{kNullEntity};

  /// \brief The wavefield.
  public: waves::WavefieldConstWeakPtr wavefield;

  ////////// BEGIN HYDRODYNAMICS PLUGIN

  /// \brief Hydrodynamics parameters for the entire model.
  public: waves::HydrodynamicsParametersPtr hydroParams;

  /// \brief Hydrodynamic physics for each Link.
  public: std::vector<HydrodynamicsLinkDataPtr> hydroData;

  /// \brief The wave model name. This is used to retrieve a pointer
  ///        to the wave field.
  // public: std::string waveModelName;

  /// \brief Scoped names of links that hydrodynamics should apply to.
  /// If empty, apply to all links.
  public: std::unordered_set<std::string> enabled;

  public: bool InitMarkers(EntityComponentManager &_ecm);
  public: void InitWaterPatchMarkers(EntityComponentManager &_ecm);
  public: void InitWaterlineMarkers(EntityComponentManager &_ecm);
  public: void InitUnderwaterSurfaceMarkers(EntityComponentManager &_ecm);

  public: void UpdateMarkers(const UpdateInfo &_info,
                             EntityComponentManager &_ecm);
  public: void UpdateWaterPatchMarkers();
  public: void UpdateWaterlineMarkers();
  public: void UpdateUnderwaterSurfaceMarkers();

  public: void DeleteWaterPatchMarkers();
  public: void DeleteWaterlineMarkers();
  public: void DeleteUnderwaterSurfaceMarkers();

  /// \brief Callback for topic "/world/<world>/waves/markers".
  ///
  /// \param[in] _msg Wave parameters message.
  public: void OnWaveMarkersMsg(const gz::msgs::Param &_msg);

  /// \brief Name of the world
  public: std::string worldName;

  /// \brief Water patch markers are initialised
  public: bool initializedWaterPatch{false};

  /// \brief Waterline markers are initialised
  public: bool initializedWaterline{false};

  /// \brief Underwater surface markers are initialised
  public: bool initializedUnderwaterSurface{false};

  /// \brief Show the water patch markers.
  public: bool showWaterPatch {false};

  /// \brief Mark water patch markers for deletion.
  public: bool shouldDeleteWaterPatch {false};

  /// \brief Show the waterline markers.
  public: bool showWaterline {false};

  /// \brief Mark waterline markers for deletion.
  public: bool shouldDeleteWaterline {false};

  /// \brief Show the underwater surface.
  public: bool showUnderwaterSurface {false};

  /// \brief Mark underwater surface for deletion.
  public: bool shouldDeleteUnderwaterSurface {false};

  /// \brief The update rate for visual markers (Hz).
  public: double updateRate {30.0};

  /// \brief Previous update time (s).
  public: double prevTime;

  /// \brief Mutex to protect wave marker updates.
  public: std::recursive_mutex mutex;

  /// \brief Transport node for wave marker messages
  public: transport::Node node;

  ////////// END HYDRODYNAMICS PLUGIN

};

//////////////////////////////////////////////////
Hydrodynamics::Hydrodynamics() : System(),
    dataPtr(std::make_unique<HydrodynamicsPrivate>())
{
}

//////////////////////////////////////////////////
Hydrodynamics::~Hydrodynamics()
{
}

//////////////////////////////////////////////////
void Hydrodynamics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  GZ_PROFILE("Hydrodynamics::Configure");

  gzmsg << "Hydrodynamics: configuring\n";

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

  // Capture the model entity
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The Hydrodynamics system should be attached to a model entity. "
           << "Failed to initialise." << "\n";
    return;
  }
  this->dataPtr->sdf = _sdf->Clone();

  // Subscribe to wave marker updates
  std::string topic("/world/" + this->dataPtr->worldName + "/waves/markers");
  this->dataPtr->node.Subscribe(
      topic, &HydrodynamicsPrivate::OnWaveMarkersMsg, this->dataPtr.get());

  // Empty sdf element used as a placeholder for missing elements
  auto sdfEmpty = std::make_shared<sdf::Element>();

  // Scoped names of entities (links, collisions) to apply hydrodynamics
  if (this->dataPtr->sdf->HasElement("enable"))
  {
    for (auto enableElem = this->dataPtr->sdf->FindElement("enable");
        enableElem != nullptr;
        enableElem = enableElem->GetNextElement("enable"))
    {
      this->dataPtr->enabled.insert(enableElem->Get<std::string>());
      gzmsg << "Hydrodynamics enable: "
          << enableElem->Get<std::string>() << "\n";
    }
  }

  // Hydrodynamics parameters
  this->dataPtr->hydroParams.reset(new waves::HydrodynamicsParameters());

  auto sdfHydro = sdfEmpty;
  if (this->dataPtr->sdf->HasElement("hydrodynamics"))
  {
    sdfHydro = _sdf->GetElementImpl("hydrodynamics");
  }
  this->dataPtr->hydroParams->SetFromSDF(*sdfHydro);

  // Markers
  if (_sdf->HasElement("markers"))
  {
    sdf::ElementPtr sdfMarkers = _sdf->GetElementImpl("markers");
    this->dataPtr->updateRate =
        waves::Utilities::SdfParamDouble(*sdfMarkers, "update_rate", 30.0);
    this->dataPtr->showWaterPatch =
        waves::Utilities::SdfParamBool(*sdfMarkers, "water_patch", false);
    this->dataPtr->showWaterline =
        waves::Utilities::SdfParamBool(*sdfMarkers, "waterline", false);
    this->dataPtr->showUnderwaterSurface =
        waves::Utilities::SdfParamBool(
              *sdfMarkers, "underwater_surface", false);
  }
}

///////////////////////////////////////////////////
void Hydrodynamics::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("Hydrodynamics::PreUpdate");

  /// \todo(anyone) support reset / rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << "\n";
  }

  if (!this->dataPtr->initialised)
  {
    // We call Init here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Init(_ecm);
    this->dataPtr->initialised = true;
  }

  if (_info.paused)
    return;

  if (this->dataPtr->initialised && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
HydrodynamicsPrivate::~HydrodynamicsPrivate()
{
  if (this->initializedWaterPatch)
    this->DeleteWaterPatchMarkers();

  if (this->initializedWaterline)
    this->DeleteWaterlineMarkers();

  if (this->initializedUnderwaterSurface)
    this->DeleteUnderwaterSurfaceMarkers();
};

//////////////////////////////////////////////////
void HydrodynamicsPrivate::Init(EntityComponentManager &_ecm)
{
  if (!this->InitWavefield(_ecm))
    return;

  if (!this->InitPhysics(_ecm))
    return;

  if (!this->InitMarkers(_ecm))
    return;

  this->validConfig = true;
}

//////////////////////////////////////////////////
bool HydrodynamicsPrivate::InitWavefield(EntityComponentManager &_ecm)
{
  /// \todo - remove hardcoded name
  // Retrieve the wavefield entity using the Name component
  std::string entityName = "wavefield";
  this->wavefieldEntity =
      _ecm.EntityByComponents(components::Name(entityName));
  // this->wavefieldEntity =
  //    _ecm.EntityByComponents(waves::components::Wavefield());
  if (this->wavefieldEntity == kNullEntity)
  {
    gzwarn << "No wavefield found, no hydrodynamic forces will be calculated\n";
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
    gzwarn << "Invalid wavefield, no hydrodynamic forces will be calculated\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
/// \todo add checks for a valid wavefield and lock the weak_ptr
bool HydrodynamicsPrivate::InitPhysics(EntityComponentManager &_ecm)
{
  gzmsg << "Hydrodynamics: initialise physics\n";

  std::string modelName(this->model.Name(_ecm));

  // Populate link meshes
  std::vector<sim::Entity> links;
  std::vector<std::vector<cgal::MeshPtr>> meshes;
  std::vector<std::vector<Entity>> collisions;
  this->CreateCollisionMeshes(_ecm, this->model, links, meshes, collisions);
  gzmsg << "Hydrodynamics: links:  " << links.size()
      << ", meshes: " << meshes.size()
      << ", collisions: " << collisions.size() << "\n";

  for (size_t i = 0; i < links.size(); ++i)
  {
    // Create storage
    waves::Index meshCount = meshes[i].size();
    std::shared_ptr<HydrodynamicsLinkData> hd(new HydrodynamicsLinkData);
    this->hydroData.push_back(hd);
    hd->initLinkMeshes.resize(meshCount);
    hd->linkMeshes.resize(meshCount);
    hd->linkCollisions.resize(meshCount);
    hd->hydrodynamics.resize(meshCount);
    hd->waterlineMsgs.resize(meshCount);
    hd->underwaterSurfaceMsgs.resize(meshCount);

    // Wavefield and Link
    hd->link = sim::Link(links[i]);
    hd->link.EnableVelocityChecks(_ecm);

    std::string modelScopedLinkName =
        removeParentScope(
            scopedName(hd->link.Entity(), _ecm, "::", false), "::");

    gzmsg << "Hydrodynamics: initialising link ["
        << modelScopedLinkName << "]\n";
    gzmsg << "Hydrodynamics: link has ["
        << meshCount << "] collision meshes\n";

    /// \note bug on initialisation
    /// Issue 1:
    /// The link may contain more than one collision, and at present the
    /// water patch is centred on the origin of the link. This could
    /// cause issues if the patch was just sufficient to contain the
    /// collision AABB and the collision was offset from the link.
    ///
    /// Issue 2:
    /// The hd->link.WorldInertialPose does not appear to be updated when
    /// this function is called, so on the first call (initialisation) the
    /// value returned is the origin. The utility function worldPose is
    /// correct, and its usage for the linkPose, linkCoGPose and collisionPose
    /// ensure that they are correct.
    ///
    /// See also:
    /// HydrodynamicsPrivate::UpdatePhysics it appears that the component
    /// data for the link is updated one time-step behind.
    ///

    /// \todo check that the link has valid pose components
    // The link pose is required for the water patch, the CoM pose for dynamics.
    // gz::math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();
    gz::math::Pose3d linkPose = worldPose(hd->link.Entity(), _ecm);
    gzmsg << "Hydrodynamics: link world pose: " << linkPose << "\n";

    /// \todo subtle difference here - inertial pose includes
    /// any rotation of the inertial matrix where CoG pose does not.
    // gz::math::Pose3d linkCoMPose = hd->link->WorldCoGPose();
    // gz::math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();
    auto inertial = _ecm.Component<components::Inertial>(hd->link.Entity());
    gz::math::Pose3d linkCoMPose = linkPose * inertial->Data().Pose();
    gzmsg << "Hydrodynamics: link world CoM pose: " << linkCoMPose << "\n";

    // RigidBody - the pose of the CoM is required for the dynamics.
    cgal::Vector3 linVelocity = waves::ToVector3(
        hd->link.WorldLinearVelocity(_ecm).value());
    cgal::Vector3 angVelocity = waves::ToVector3(
        hd->link.WorldAngularVelocity(_ecm).value());
    /// \todo WorldCoGPose is currently not available
    // cgal::Vector3 linVelocityCoM = waves::ToVector3(
    //     hd->link.WorldCoGLinearVelocity(_ecm).value());
    // cgal::Vector3 linVelocityCoM = linVelocity;

    // First pass - store collisions and create bounding box
    auto bbox = math::AxisAlignedBox();
    for (waves::Index j=0; j < meshCount; ++j)
    {
      // Mesh (SurfaceMesh copy performs a deep copy of all properties)
      std::shared_ptr<cgal::Mesh> initLinkMesh = meshes[i][j];
      std::shared_ptr<cgal::Mesh> linkMesh =
          std::make_shared<cgal::Mesh>(*initLinkMesh);
      GZ_ASSERT(linkMesh != nullptr,
          "Invalid Mesh returned from CreateCollisionMeshes");

      auto linkCollision = collisions[i][j];

      // Mesh
      hd->initLinkMeshes[j] = initLinkMesh;
      hd->linkMeshes[j] = linkMesh;
      hd->linkCollisions[j] = linkCollision;

      // Update link mesh
      auto collisionPose = worldPose(hd->linkCollisions[j], _ecm);
      ApplyPose(collisionPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);

      // DEBUG_INFO
      // gzmsg << "Hydrodynamics: collision pose\n";
      // gzmsg << collisionPose << "\n";

      // Axis aligned box
      bbox += CreateAxisAlignedBox(hd->linkMeshes[j]);
    }
    gzmsg << "Hydrodynamics: link bounding box:"
          << " size: " << bbox.Size()
          << " center: " << bbox.Center() << "\n";

    // Compute the size of the water patch for this link. Scale the patch
    // so that it's bounding sphere contains the bbox
    // (i.e. the scale factor >= sqrt(3)).
    double patchSize = 1.0;
    patchSize = std::max(bbox.XLength(), patchSize);
    patchSize = std::max(bbox.YLength(), patchSize);
    patchSize = std::max(bbox.ZLength(), patchSize);
    patchSize *= 2.0;

    gzmsg << "Hydrodynamics: set water patch size: " << patchSize << "\n";
    std::shared_ptr<waves::Grid> initWaterPatch(
        new waves::Grid({patchSize, patchSize}, {4, 4}));

    // WavefieldSampler - this is updated by the pose of the link (not the CoM).
    /// \todo add checks that the wavefield weak_ptr is valid
    hd->wavefieldSampler.reset(new waves::WavefieldSampler(
        this->wavefield.lock(), initWaterPatch));
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();

    // Second pass - create hydrodynamics
    for (waves::Index j=0; j < meshCount; ++j)
    {
      // Initialise hydrodynamics for each collision.
      hd->hydrodynamics[j].reset(
        new waves::Hydrodynamics(
          this->hydroParams,
          hd->linkMeshes[j],
          hd->wavefieldSampler));
      hd->hydrodynamics[j]->Update(
        hd->wavefieldSampler, linkCoMPose, linVelocity, angVelocity);
    }
  }

  gzmsg << "Hydrodynamics: done initialise physics\n";
  return true;
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->UpdatePhysics(_info, _ecm);
  this->UpdateMarkers(_info, _ecm);
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdatePhysics(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  ////////// BEGIN TESTING
  #if 0
  // Get the wave height at the origin
  double simTime = std::chrono::duration<double>(_info.simTime).count();
  Eigen::Vector3d point(0.0, 0.0, 0.0);
  double waveHeight{0.0};
  this->wavefield.lock()->Height(point, waveHeight);
  gzmsg << "[" << simTime << "] : " << waveHeight << "\n";
  #endif
  ////////// END TESTING

  /// \todo add checks for a valid wavefield and lock the waek_ptr

  for (auto& hd : this->hydroData)
  {
    // The link pose is required for the water patch, the CoM pose for dynamics.
    // gz::math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();
    gz::math::Pose3d linkPose = worldPose(hd->link.Entity(), _ecm);
    // DEBUG_INFO
    // gzmsg << "Hydrodynamics: link world pose\n";
    // gzmsg << linkPose << "\n";

    /// \todo WorldCoGPose is currently not available
    // gz::math::Pose3d linkCoMPose = hd->link.WorldCoGPose(_ecm).value();
    // gz::math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();
    auto inertial = _ecm.Component<components::Inertial>(hd->link.Entity());
    gz::math::Pose3d linkCoMPose = linkPose * inertial->Data().Pose();;
    // DEBUG_INFO
    // gzmsg << "Hydrodynamics: link world CoM pose\n";
    // gzmsg << linkCoMPose << "\n";

    // Update water patch
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();
    // auto waterPatch = hd->wavefieldSampler->GetWaterPatch();

    // RigidBody - the pose of the CoM is required for the dynamics.
    /// \todo check the components are available and valid
    cgal::Vector3 linVelocity = waves::ToVector3(
        hd->link.WorldLinearVelocity(_ecm).value());
    cgal::Vector3 angVelocity = waves::ToVector3(
        hd->link.WorldAngularVelocity(_ecm).value());
    /// \todo WorldCoGLinearVel is currently not available
    // cgal::Vector3 linVelocityCoM = waves::ToVector3(
    //     hd->link.WorldCoGLinearVel(_ecm).value());
    // cgal::Vector3 linVelocityCoM = linVelocity;

    // Meshes
    // waves::Index nSubTri = 0;
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      // Update link mesh
      auto linkCollision = hd->linkCollisions[j];
      auto collisionPose = worldPose(linkCollision, _ecm);
      ApplyPose(collisionPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);

      // DEBUG_INFO
      // gzmsg << "Hydrodynamics: collision pose\n";
      // gzmsg << collisionPose << "\n";

      // Update hydrodynamics
      hd->hydrodynamics[j]->Update(
        hd->wavefieldSampler, linkCoMPose, linVelocity, angVelocity);

      // Apply forces to the Link
      auto force = waves::ToGz(hd->hydrodynamics[j]->Force());
      if (force.IsFinite())
      {
        hd->link.AddWorldForce(_ecm, force);
      }

      // Apply torques to the link
      auto torque = waves::ToGz(hd->hydrodynamics[j]->Torque());
      if (torque.IsFinite())
      {
        hd->link.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, torque);
      }

      // Info for Markers
      // nSubTri += hd->hydrodynamics[j]->GetSubmergedTriangles().size();

      // DEBUG_INFO
      // gzmsg << "Link:         " << hd->link->GetName() << "\n";
      // gzmsg << "Position:     " << linkPose.Pos() << "\n";
      // gzmsg << "Rotation:     " << linkPose.Rot().Euler() << "\n";
      // gzmsg << "SubTriCount:  " << nSubTri << "\n";
      // gzmsg << "Force:        " << force << "\n";
      // gzmsg << "Torque:       " << torque << "\n";
    }
  }
}


//////////////////////////////////////////////////
/// \note copied from gz-sim/src/systems/buoyancy/Buoyancy.cc
bool HydrodynamicsPrivate::IsEnabled(Entity _entity,
    const EntityComponentManager &_ecm) const
{
  // If there's nothing enabled, all entities are enabled
  if (this->enabled.empty())
  {
    // gzdbg << "All collisions for ["
    //     << this->model.Name(_ecm)
    //     << "] are enabled\n";
    return true;
  }

  auto entity = _entity;
  while (entity != kNullEntity)
  {
    // Fully scoped name
    auto name = scopedName(entity, _ecm, "::", false);
    // gzdbg << "Scoped name: " << name << "\n";

    // Remove world name
    name = removeParentScope(name, "::");
    // gzdbg << "Remove parent scoped name: " << name << "\n";

    if (this->enabled.find(name) != this->enabled.end())
    {
      // gzdbg << "Collision for ["
      //     << name << "] is enabled\n";
      return true;
    }

    // Check parent
    auto parentComp = _ecm.Component<components::ParentEntity>(entity);

    if (nullptr == parentComp)
      return false;

    entity = parentComp->Data();
  }

  return false;
}


//////////////////////////////////////////////////
void HydrodynamicsPrivate::CreateCollisionMeshes(
  EntityComponentManager &_ecm,
  sim::Model _model,
  std::vector<sim::Entity>& _links,
  std::vector<std::vector<cgal::MeshPtr>>& _meshes,
  std::vector<std::vector<Entity>>& _collisions)
{
  // There will be more than one mesh per link if the link contains
  // mutiple collisions. Links with no collisions enabled are skipped.

  // Model
  std::string modelName(_model.Name(_ecm));

  // Links
  for (auto& linkEntity : _model.Links(_ecm))
  {
    GZ_ASSERT(linkEntity != kNullEntity, "Link must be valid");
    sim::Link link(linkEntity);
    std::string linkName(link.Name(_ecm).value());
    std::string modelScopedLinkName =
        removeParentScope(
            scopedName(linkEntity, _ecm, "::", false), "::");


    /// check link has valid name
    GZ_ASSERT(link.Name(_ecm).has_value(), "Link must be have valid name");
    std::vector<std::shared_ptr<cgal::Mesh>> linkMeshes;
    std::vector<Entity> linkCollisions;

    gzmsg << "Hydrodynamics: checking collision meshes for link ["
        << modelScopedLinkName << "]\n";

    // Collisions
    for (auto& collisionEntity : link.Collisions(_ecm))
    {
      GZ_ASSERT(collisionEntity != kNullEntity, "Collision must be valid");

      sim::Collision collision(collisionEntity);
      std::string collisionName(collision.Name(_ecm).value());
      std::string modelScopedCollisionName =
          removeParentScope(
              scopedName(collisionEntity, _ecm, "::", false), "::");

      // check this collision has hydrodynamics enabled
      if (!this->IsEnabled(collisionEntity, _ecm))
      {
        gzmsg << "Hydrodynamics: skipping collision ["
            << modelScopedCollisionName << "]\n";
        continue;
      }
      gzmsg << "Hydrodynamics: including collision ["
          << modelScopedCollisionName << "]\n";

      // get the collision element
      const components::CollisionElement *coll =
        _ecm.Component<components::CollisionElement>(collisionEntity);

      if (!coll)
      {
        gzerr << "Invalid collision pointer. This shouldn't happen\n";
        continue;
      }

      // double volume = 0;
      switch (coll->Data().Geom()->Type())
      {
        case sdf::GeometryType::BOX:
        {
          // Get shape from the collision component
          auto& box = coll->Data().Geom()->BoxShape()->Shape();

          // Create the gazebo mesh
          std::string meshName = std::string(modelName)
              .append(".").append(linkName)
              .append(".").append(collisionName)
              .append(".box");
          gz::common::MeshManager::Instance()->CreateBox(
              meshName,
              box.Size(),
              gz::math::Vector2d(1, 1));
          GZ_ASSERT(gz::common::MeshManager::Instance()->HasMesh(meshName),
              "Failed to create Mesh for Box");

          // Create the CGAL surface mesh
          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
          waves::MeshTools::MakeSurfaceMesh(
              *gz::common::MeshManager::Instance()->
                  MeshByName(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);
          linkCollisions.push_back(collisionEntity);

          gzmsg << "Type:       " << "BOX" << "\n";
          gzmsg << "Size:       " << box.Size() << "\n";
          gzmsg << "MeshName:   " << meshName << "\n";
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << "\n";
          break;
        }
        case sdf::GeometryType::SPHERE:
        {
          // Get shape from the collision component
          auto& sphere = coll->Data().Geom()->SphereShape()->Shape();

          // Create the gazebo mesh
          std::string meshName = std::string(modelName)
              .append(".").append(linkName)
              .append(".").append(collisionName)
              .append(".sphere");
          gz::common::MeshManager::Instance()->CreateSphere(
              meshName,
              sphere.Radius(),        // radius
              8,                      // rings
              8);                     // segments
          GZ_ASSERT(gz::common::MeshManager::Instance()->HasMesh(meshName),
              "Failed to create Mesh for Sphere");

          // Create the CGAL surface mesh
          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
          waves::MeshTools::MakeSurfaceMesh(
              *gz::common::MeshManager::Instance()->
                  MeshByName(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);
          linkCollisions.push_back(collisionEntity);

          gzmsg << "Type:       " << "SPHERE" << "\n";
          gzmsg << "Radius:     " << sphere.Radius() << "\n";
          gzmsg << "MeshName:   " << meshName << "\n";
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << "\n";
          break;
        }
        case sdf::GeometryType::CYLINDER:
        {
          auto& cylinder = coll->Data().Geom()->CylinderShape()->Shape();

          // Create the gazebo mesh
          std::string meshName = std::string(modelName)
              .append(".").append(linkName)
              .append(".").append(collisionName)
              .append(".cylinder");
          gz::common::MeshManager::Instance()->CreateCylinder(
              meshName,
              cylinder.Radius(),      // radius
              cylinder.Length(),      // length,
              1,                      // rings
              32);                    // segments
          GZ_ASSERT(gz::common::MeshManager::Instance()->HasMesh(meshName),
              "Failed to create Mesh for Cylinder");

          // Create the CGAL surface mesh
          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
          waves::MeshTools::MakeSurfaceMesh(
              *gz::common::MeshManager::Instance()->
                  MeshByName(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);
          linkCollisions.push_back(collisionEntity);

          gzmsg << "Type:       " << "CYLINDER" << "\n";
          gzmsg << "Radius:     " << cylinder.Radius() << "\n";
          gzmsg << "Length:     " << cylinder.Length() << "\n";
          gzmsg << "MeshName:   " << meshName << "\n";
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << "\n";
          break;
        }
        case sdf::GeometryType::PLANE:
        {
          // Ignore plane shapes.
          break;
        }
        case sdf::GeometryType::MESH:
        {
          // auto& meshShape = coll->Data().Geom()->MeshShape();
          std::string uri = coll->Data().Geom()->MeshShape()->Uri();
          std::string filePath = coll->Data().Geom()->MeshShape()->FilePath();

          std::string file = asFullPath(uri, filePath);
          // if (gz::common::MeshManager::Instance()->IsValidFilename(file))
          // {
          //   const gz::common::Mesh *mesh =
          //     gz::common::MeshManager::Instance()->Load(file);
          //   if (mesh)
          //     volume = mesh->Volume();
          //   else
          //     gzerr << "Unable to load mesh[" << file << "]\n";
          // }
          // else
          // {
          //   gzerr << "Invalid mesh filename[" << file << "]\n";
          // }

          // Mesh
          if (!gz::common::MeshManager::Instance()->IsValidFilename(file))
          {
            gzerr << "Mesh: " << file << " was not loaded"<< "\n";
            return;
          }

          // Create the CGAL surface mesh
          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
          waves::MeshTools::MakeSurfaceMesh(
              *gz::common::MeshManager::Instance()->Load(file), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);
          linkCollisions.push_back(collisionEntity);

          gzmsg << "Type:       " << "MESH" << "\n";
          gzmsg << "Uri:        " << uri << "\n";
          gzmsg << "FilePath:   " << filePath << "\n";
          gzmsg << "MeshFile:   " << file << "\n";
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << "\n";
          break;
        }
        default:
        {
          gzerr << "Unsupported collision geometry["
            << static_cast<int>(coll->Data().Geom()->Type()) << "]\n";
          break;
        }
      }
    }

    // Add link and meshes if it contains enabled collisions
    if (!linkMeshes.empty())
    {
      _links.push_back(linkEntity);
      _meshes.push_back(linkMeshes);
      _collisions.push_back(linkCollisions);
    }
  }
}

//////////////////////////////////////////////////
bool HydrodynamicsPrivate::InitMarkers(
    EntityComponentManager &_ecm)
{
  // initialise each marker type
  if (!this->initializedWaterPatch && this->showWaterPatch)
    this->InitWaterPatchMarkers(_ecm);

  if (!this->initializedWaterline && this->showWaterline)
    this->InitWaterlineMarkers(_ecm);

  if (!this->initializedUnderwaterSurface && this->showUnderwaterSurface)
    this->InitUnderwaterSurfaceMarkers(_ecm);

  return true;
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::InitWaterPatchMarkers(
    EntityComponentManager &_ecm)
{
  std::string modelName(this->model.Name(_ecm));
  waves::Index markerId = 1;
  for (auto&& hd : this->hydroData)
  {
    hd->waterPatchMsg.set_ns(modelName + "/water_patch");
    hd->waterPatchMsg.set_id(markerId++);
    hd->waterPatchMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
    hd->waterPatchMsg.set_type(gz::msgs::Marker::TRIANGLE_LIST);
    hd->waterPatchMsg.set_visibility(gz::msgs::Marker::GUI);

    // Set material properties
    gz::msgs::Set(
      hd->waterPatchMsg.mutable_material()->mutable_ambient(),
      gz::math::Color(0, 0, 1, 0.7));
    gz::msgs::Set(
      hd->waterPatchMsg.mutable_material()->mutable_diffuse(),
      gz::math::Color(0, 0, 1, 0.7));
  }
  this->initializedWaterPatch = true;
}

///////////////////////////////////////////////////
void HydrodynamicsPrivate::InitWaterlineMarkers(
    EntityComponentManager &_ecm)
{
  std::string modelName(this->model.Name(_ecm));
  waves::Index markerId = 1;
  for (auto&& hd : this->hydroData)
  {
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      hd->waterlineMsgs[j].set_ns(modelName + "/waterline");
      hd->waterlineMsgs[j].set_id(markerId++);
      hd->waterlineMsgs[j].set_action(gz::msgs::Marker::ADD_MODIFY);
      hd->waterlineMsgs[j].set_type(gz::msgs::Marker::LINE_LIST);
      hd->waterlineMsgs[j].set_visibility(gz::msgs::Marker::GUI);

      // Set material properties
      gz::msgs::Set(
        hd->waterlineMsgs[j].mutable_material()->mutable_ambient(),
        gz::math::Color(0, 0, 0, 1));
      gz::msgs::Set(
        hd->waterlineMsgs[j].mutable_material()->mutable_diffuse(),
        gz::math::Color(0, 0, 0, 1));
    }
  }
  this->initializedWaterline = true;
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::InitUnderwaterSurfaceMarkers(
    EntityComponentManager &_ecm)
{
  std::string modelName(this->model.Name(_ecm));
  waves::Index markerId = 1;
  for (auto&& hd : this->hydroData)
  {
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      hd->underwaterSurfaceMsgs[j].set_ns(modelName + "/underwater_surface");
      hd->underwaterSurfaceMsgs[j].set_id(markerId++);
      hd->underwaterSurfaceMsgs[j].set_action(gz::msgs::Marker::ADD_MODIFY);
      hd->underwaterSurfaceMsgs[j].set_type(gz::msgs::Marker::TRIANGLE_LIST);
      hd->underwaterSurfaceMsgs[j].set_visibility(gz::msgs::Marker::GUI);

      // Set material properties
      gz::msgs::Set(
        hd->underwaterSurfaceMsgs[j].mutable_material()->mutable_ambient(),
        gz::math::Color(0, 1, 0, 0.7));
      gz::msgs::Set(
        hd->underwaterSurfaceMsgs[j].mutable_material()->mutable_diffuse(),
        gz::math::Color(0, 1, 0, 0.7));
    }
  }
  this->initializedUnderwaterSurface = true;
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateMarkers(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // Throttle update [30 FPS by default]
  double updatePeriod = 1.0/this->updateRate;
  double currentTime = std::chrono::duration<double>(_info.simTime).count();
  if ((currentTime - this->prevTime) < updatePeriod)
  {
    return;
  }
  this->prevTime = currentTime;

  if (this->showWaterPatch)
  {
    if (!this->initializedWaterPatch)
      this->InitWaterPatchMarkers(_ecm);

    this->UpdateWaterPatchMarkers();
  }
  else
  {
    if (this->shouldDeleteWaterPatch)
    {
      this->DeleteWaterPatchMarkers();
      this->shouldDeleteWaterPatch = false;
    }
  }

  if (this->showWaterline)
  {
    if (!this->initializedWaterline)
      this->InitWaterlineMarkers(_ecm);

    this->UpdateWaterlineMarkers();
  }
  else
  {
    if (this->shouldDeleteWaterline)
    {
      this->DeleteWaterlineMarkers();
      this->shouldDeleteWaterline = false;
    }
  }

  if (this->showUnderwaterSurface)
  {
    if (!this->initializedUnderwaterSurface)
      this->InitUnderwaterSurfaceMarkers(_ecm);

    this->UpdateUnderwaterSurfaceMarkers();
  }
  else
  {
    if (this->shouldDeleteUnderwaterSurface)
    {
      this->DeleteUnderwaterSurfaceMarkers();
      this->shouldDeleteUnderwaterSurface = false;
    }
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateWaterPatchMarkers()
{
  for (auto&& hd : this->hydroData)
  {
    auto& grid = *hd->wavefieldSampler->GetWaterPatch();

    // clear and update
    hd->waterPatchMsg.mutable_point()->Clear();
    hd->waterPatchMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
    for (waves::Index ix=0; ix < grid.GetCellCount()[0]; ++ix)
    {
      for (waves::Index iy=0; iy < grid.GetCellCount()[1]; ++iy)
      {
        for (waves::Index k=0; k < 2; ++k)
        {
          cgal::Triangle tri = grid.GetTriangle(ix, iy, k);
          gz::msgs::Set(hd->waterPatchMsg.add_point(), waves::ToGz(tri[0]));
          gz::msgs::Set(hd->waterPatchMsg.add_point(), waves::ToGz(tri[1]));
          gz::msgs::Set(hd->waterPatchMsg.add_point(), waves::ToGz(tri[2]));
        }
      }
    }
    this->node.Request("/marker", hd->waterPatchMsg);
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateWaterlineMarkers()
{
  for (auto&& hd : this->hydroData)
  {
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      hd->waterlineMsgs[j].mutable_point()->Clear();
      hd->waterlineMsgs[j].set_action(gz::msgs::Marker::ADD_MODIFY);
      if (hd->hydrodynamics[j]->GetWaterline().empty())
      {
        /// \todo workaround. The previous marker is not cleared
        ///       if a cleared point list is published.
        gz::msgs::Set(hd->waterlineMsgs[j]
            .add_point(), gz::math::Vector3d::Zero);
        gz::msgs::Set(hd->waterlineMsgs[j]
            .add_point(), gz::math::Vector3d::Zero);
      }
      for (auto&& line : hd->hydrodynamics[j]->GetWaterline())
      {
        gz::msgs::Set(hd->waterlineMsgs[j]
            .add_point(), waves::ToGz(line.point(0)));
        gz::msgs::Set(hd->waterlineMsgs[j]
            .add_point(), waves::ToGz(line.point(1)));
      }
      this->node.Request("/marker", hd->waterlineMsgs[j]);
    }
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateUnderwaterSurfaceMarkers()
{
  for (auto&& hd : this->hydroData)
  {
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
      hd->underwaterSurfaceMsgs[j].set_action(gz::msgs::Marker::ADD_MODIFY);
      if (hd->hydrodynamics[j]->GetSubmergedTriangles().empty())
      {
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j]
            .add_point(), gz::math::Vector3d::Zero);
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j]
            .add_point(), gz::math::Vector3d::Zero);
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j]
            .add_point(), gz::math::Vector3d::Zero);
      }
      for (auto&& tri : hd->hydrodynamics[j]->GetSubmergedTriangles())
      {
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j]
            .add_point(), waves::ToGz(tri[0]));
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j]
            .add_point(), waves::ToGz(tri[1]));
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j]
            .add_point(), waves::ToGz(tri[2]));
      }
      this->node.Request("/marker", hd->underwaterSurfaceMsgs[j]);
    }
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::DeleteWaterPatchMarkers()
{
  for (auto&& hd : this->hydroData)
  {
    hd->waterPatchMsg.mutable_point()->Clear();
    hd->waterPatchMsg.set_action(gz::msgs::Marker::DELETE_MARKER);
    this->node.Request("/marker", hd->waterPatchMsg);
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::DeleteWaterlineMarkers()
{
  for (auto&& hd : this->hydroData)
  {
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      hd->waterlineMsgs[j].mutable_point()->Clear();
      hd->waterlineMsgs[j].set_action(gz::msgs::Marker::DELETE_MARKER);
      this->node.Request("/marker", hd->waterlineMsgs[j]);
    }
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::DeleteUnderwaterSurfaceMarkers()
{
  for (auto&& hd : this->hydroData)
  {
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
      hd->underwaterSurfaceMsgs[j].set_action(gz::msgs::Marker::DELETE_MARKER);
      this->node.Request("/marker", hd->underwaterSurfaceMsgs[j]);
    }
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::OnWaveMarkersMsg(const gz::msgs::Param &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->mutex);

  // extract parameters
  {
    auto it = _msg.params().find("water_patch");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is bool
      auto param = it->second;
      // auto type = param.type();
      auto value = param.bool_value();
      if (this->showWaterPatch && !value)
      {
        this->shouldDeleteWaterPatch = true;
      }
      this->showWaterPatch = value;
    }
  }
  {
    auto it = _msg.params().find("waterline");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is bool
      auto param = it->second;
      // auto type = param.type();
      auto value = param.bool_value();
      if (this->showWaterline && !value)
      {
        this->shouldDeleteWaterline = true;
      }
      this->showWaterline = value;
    }
  }
  {
    auto it = _msg.params().find("underwater_surface");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is bool
      auto param = it->second;
      // auto type = param.type();
      auto value = param.bool_value();
      if (this->showUnderwaterSurface && !value)
      {
        this->shouldDeleteUnderwaterSurface = true;
      }
      this->showUnderwaterSurface = value;
    }
  }
}

}  // namespace systems
}  // namespace sim
}  // namespace gz

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(gz::sim::systems::Hydrodynamics,
              gz::sim::System,
              gz::sim::systems::Hydrodynamics::ISystemConfigure,
              gz::sim::systems::Hydrodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::Hydrodynamics,
                    "gz::sim::systems::Hydrodynamics")
