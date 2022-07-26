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

#include "gz/waves/Convert.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/MeshTools.hh"
#include "gz/waves/Physics.hh"
#include "gz/waves/Utilities.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WavefieldSampler.hh"

#include "gz/waves/components/Wavefield.hh"

#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport.hh>

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

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

namespace gz
{
namespace sim
{
  /////////////////////////////////////////////////
  // Collision (similar to sim::Link and sim::Model interfaces)
 
  class CollisionPrivate
  {
    /// \brief Id of link entity.
    public: Entity id{kNullEntity};
  };

  class Collision
  {
    /// \brief Destructor
    public: ~Collision() = default;

    /// \brief Constructor
    /// \param[in] _entity Collision entity
    public: explicit Collision(sim::Entity _entity = kNullEntity)
      : dataPtr(std::make_unique<CollisionPrivate>())
    {
      this->dataPtr->id = _entity;
    }

    /// \brief Copy constructor
    /// \param[in] _collision Collision to copy.
    public: Collision(const Collision &_collision)
      : dataPtr(std::make_unique<CollisionPrivate>(*_collision.dataPtr))
    {
    }

    /// \brief Move constructor
    /// \param[in] _collision Collision to move.
    public: Collision(Collision &&_collision) noexcept = default;

    /// \brief Move assignment operator.
    /// \param[in] _collision Collision component to move.
    /// \return Reference to this.
    public: Collision &operator=(Collision &&_collision) noexcept = default;

    /// \brief Copy assignment operator.
    /// \param[in] _collision Collision to copy.
    /// \return Reference to this.
    public: Collision &operator=(const Collision &_collision)
    {
      *this->dataPtr = (*_collision.dataPtr);
      return *this;
    }

    /// \brief Get the entity which this Collision is related to.
    /// \return Collision entity.
    public: sim::Entity Entity() const
    {
      return this->dataPtr->id;
    }

    /// \brief Check whether this link correctly refers to an entity that
    /// has a components::Collision.
    /// \param[in] _ecm Entity-component manager.
    /// \return True if it's a valid link in the manager.
    public: bool Valid(const EntityComponentManager &_ecm) const
    {
      return nullptr != _ecm.Component<components::Collision>(
          this->dataPtr->id);
    }

    /// \brief Get the link's unscoped name.
    /// \param[in] _ecm Entity-component manager.
    /// \return Collision's name or nullopt if the entity does not have a
    /// components::Name component
    public: std::optional<std::string> Name(
        const EntityComponentManager &_ecm) const
    {
      return _ecm.ComponentData<components::Name>(this->dataPtr->id);
    }

    /// \brief Get the parent link
    /// \param[in] _ecm Entity-component manager.
    /// \return Parent Model or nullopt if the entity does not have a
    /// components::ParentEntity component.
    public: std::optional<Model> ParentLink(
        const EntityComponentManager &_ecm) const
    {
      auto parent = _ecm.Component<components::ParentEntity>(this->dataPtr->id);

      if (!parent)
        return std::nullopt;

      return std::optional<Model>(parent->Data());
    }

    /// \brief Pointer to private data.
    private: std::unique_ptr<CollisionPrivate> dataPtr;
  };

namespace systems
{
  /////////////////////////////////////////////////
  // Utilties
  
  /// \brief Iterate over the links in a model, and create a CGAL SurfaceMesh
  /// for each collison in each link.
  ///
  /// \param[in]  _model    The model being processed. 
  /// \param[out] _links    A vector holding a copy of pointers to the the model's links. 
  /// \param[out] _meshes   A vector of vectors containing a surface mesh for each collision in a link.
  /// \param[out] _collisionElements  A vector of vectors containing the collision entities in a link.
  void CreateCollisionMeshes(
    EntityComponentManager &_ecm,
    sim::Model _model,
    std::vector<sim::Entity>& _links,
    std::vector<std::vector<cgal::MeshPtr>>& _meshes,
    std::vector<std::vector<Entity>>& _collisions)
  {
    // There will be more than one mesh per link if the link contains mutiple collisions.

    // Model
    std::string modelName(_model.Name(_ecm));

    // Links
    for (auto& linkEntity : _model.Links(_ecm))
    {
      GZ_ASSERT(linkEntity != kNullEntity, "Link must be valid");
      _links.push_back(linkEntity);
      sim::Link link(linkEntity);

      /// \todo check link has valid name component
      std::string linkName(link.Name(_ecm).value());
      std::vector<std::shared_ptr<cgal::Mesh>> linkMeshes;
      std::vector<Entity> linkCollisions;

      gzmsg << "Hydrodynamics: create collision mesh for link ["
          << linkName << "]\n";
      
      // Collisions
      for (auto& collisionEntity : link.Collisions(_ecm))
      {
        GZ_ASSERT(collisionEntity != kNullEntity, "Collision must be valid");
  
        sim::Collision collision(collisionEntity);
        std::string collisionName(collision.Name(_ecm).value());
        gzmsg << "Hydrodynamics: collision name [" << collisionName << "]\n";

        const components::CollisionElement *coll =
          _ecm.Component<components::CollisionElement>(collisionEntity);

        if (!coll)
        {
          gzerr << "Invalid collision pointer. This shouldn't happen\n";
          continue;
        }

        double volume = 0;
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
                *gz::common::MeshManager::Instance()->MeshByName(meshName), *mesh);
            GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);
            linkCollisions.push_back(collisionEntity);

            gzmsg << "Type:       " << "BOX" << std::endl;
            gzmsg << "Size:       " << box.Size() << std::endl;
            gzmsg << "MeshName:   " << meshName << std::endl;            
            gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
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
                *gz::common::MeshManager::Instance()->MeshByName(meshName), *mesh);
            GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);
            linkCollisions.push_back(collisionEntity);

            gzmsg << "Type:       " << "SPHERE" << std::endl;
            gzmsg << "Radius:     " << sphere.Radius() << std::endl;
            gzmsg << "MeshName:   " << meshName << std::endl;            
            gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
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
                *gz::common::MeshManager::Instance()->MeshByName(meshName), *mesh);
            GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);
            linkCollisions.push_back(collisionEntity);

            gzmsg << "Type:       " << "CYLINDER" << std::endl;
            gzmsg << "Radius:     " << cylinder.Radius() << std::endl;
            gzmsg << "Length:     " << cylinder.Length() << std::endl;
            gzmsg << "MeshName:   " << meshName << std::endl;            
            gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
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
              gzerr << "Mesh: " << file << " was not loaded"<< std::endl;
              return;
            } 

            // Create the CGAL surface mesh
            std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
            waves::MeshTools::MakeSurfaceMesh(
                *gz::common::MeshManager::Instance()->Load(file), *mesh);
            GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);
            linkCollisions.push_back(collisionEntity);

            gzmsg << "Type:       " << "MESH" << std::endl;
            gzmsg << "Uri:        " << uri << std::endl;
            gzmsg << "FilePath:   " << filePath << std::endl;
            gzmsg << "MeshFile:   " << file << std::endl;
            gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
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

      // Add meshes for this link
      _meshes.push_back(linkMeshes);
      _collisions.push_back(linkCollisions);
    }
  } 

  /////////////////////////////////////////////////
  /// \brief Transform a meshes vertex points in the world frame according to a Pose.
  ///
  /// \param[in] _pose    A pose defining a translation and rotation in the world frame.
  /// \param[in] _source  The original mesh to transform.
  /// \param[out] _target The transformed mesh.
  void ApplyPose(
    const gz::math::Pose3d& _pose,
    const cgal::Mesh& _source,
    cgal::Mesh& _target)
  {
    for (
      auto&& it = std::make_pair(std::begin(_source.vertices()), std::begin(_target.vertices()));
      it.first != std::end(_source.vertices()) && it.second != std::end(_target.vertices());
      ++it.first, ++it.second)
    {
      auto& v0 = *it.first;
      auto& v1 = *it.second;
      const cgal::Point3& p0 = _source.point(v0);

      // Affine transformation
      gz::math::Vector3d gzP0(p0.x(), p0.y(), p0.z());
      gz::math::Vector3d gzP1 = _pose.Rot().RotateVector(gzP0) + _pose.Pos();

      cgal::Point3& p1 = _target.point(v1);
      p1 = cgal::Point3(gzP1.X(), gzP1.Y(), gzP1.Z());
    }
  }

  /////////////////////////////////////////////////
  // NOT REQUIRED: component additions are carried out
  //               in Link::EnableVelocityChecks()

#if 0
  /////////////////////////////////////////////////
  void AddWorldPose(const Entity &_entity, EntityComponentManager &_ecm)
  {
    if (!_ecm.Component<components::WorldPose>(_entity))
    {
      _ecm.CreateComponent(_entity, components::WorldPose());
    }
  }

  /////////////////////////////////////////////////
  void AddInertial(const Entity &_entity, EntityComponentManager &_ecm)
  {
    if (!_ecm.Component<components::Inertial>(_entity))
    {
      _ecm.CreateComponent(_entity, components::Inertial());
    }
  }

  /////////////////////////////////////////////////
  void AddCollision(const Entity &_entity, EntityComponentManager &_ecm)
  {
    if (!_ecm.Component<components::Collision>(_entity))
    {
      _ecm.CreateComponent(_entity, components::Collision());
    }
  }

  /////////////////////////////////////////////////
  void AddWorldLinearVelocity(const Entity &_entity, EntityComponentManager &_ecm)
  {
    if (!_ecm.Component<components::WorldLinearVelocity>(_entity))
    {
      _ecm.CreateComponent(_entity, components::WorldLinearVelocity());
    }
  }

  /////////////////////////////////////////////////
  void AddWorldAngularVelocity(const Entity &_entity, EntityComponentManager &_ecm)
  {
    if (!_ecm.Component<components::WorldAngularVelocity>(_entity))
    {
      _ecm.CreateComponent(_entity, components::WorldAngularVelocity());
    }
  }
#endif

  /////////////////////////////////////////////////
  // HydrodynamicsLinkData

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
}
}
}

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::HydrodynamicsPrivate
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

  /// \brief Model interface
  public: sim::Model model{kNullEntity};

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
  public: waves::WavefieldWeakPtr wavefield;

  ////////// BEGIN HYDRODYNAMICS PLUGIN

  /// \brief Hydrodynamics parameters for the entire model.
  public: waves::HydrodynamicsParametersPtr hydroParams;

  /// \brief Hydrodynamic physics for each Link.
  public: std::vector<HydrodynamicsLinkDataPtr> hydroData;

  /// \brief The wave model name. This is used to retrieve a pointer to the wave field.
  // public: std::string waveModelName;

  public: bool InitMarkers(EntityComponentManager &_ecm);
  public: void InitWaterPatchMarkers(EntityComponentManager &_ecm);
  public: void InitWaterlineMarkers(EntityComponentManager &_ecm);
  public: void InitUnderwaterSurfaceMarkers(EntityComponentManager &_ecm);

  public: void UpdateMarkers(const UpdateInfo &_info,
                        EntityComponentManager &_ecm);
  public: void UpdateWaterPatchMarkers(const UpdateInfo &_info,
                        EntityComponentManager &_ecm);
  public: void UpdateWaterlineMarkers(const UpdateInfo &_info,
                        EntityComponentManager &_ecm);
  public: void UpdateUnderwaterSurfaceMarkers(const UpdateInfo &_info,
                        EntityComponentManager &_ecm);

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

  /// \brief Show the waterline markers.
  public: bool showWaterline {false};

  /// \brief Show the underwater surface.
  public: bool showUnderwaterSurface {false};

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
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdf = _sdf->Clone();

  // Subscribe to wave marker updates
  std::string topic("/world/" + this->dataPtr->worldName + "/waves/markers");
  this->dataPtr->node.Subscribe(
      topic, &HydrodynamicsPrivate::OnWaveMarkersMsg, this->dataPtr.get());

  // Empty sdf element used as a placeholder for missing elements 
  auto sdfEmpty = std::make_shared<sdf::Element>();

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
    this->dataPtr->updateRate            = waves::Utilities::SdfParamDouble(*sdfMarkers, "update_rate",         30.0);
    this->dataPtr->showWaterPatch        = waves::Utilities::SdfParamBool(*sdfMarkers,   "water_patch",         false);
    this->dataPtr->showWaterline         = waves::Utilities::SdfParamBool(*sdfMarkers,   "waterline",           false);
    this->dataPtr->showUnderwaterSurface = waves::Utilities::SdfParamBool(*sdfMarkers,   "underwater_surface",  false);
  }
}

//////////////////////////////////////////////////
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
        << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Init here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Init(_ecm);
    this->dataPtr->initialized = true;
  }

  if (_info.paused)
    return;

  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
HydrodynamicsPrivate::~HydrodynamicsPrivate()
{
};

/////////////////////////////////////////////////
void HydrodynamicsPrivate::Init(EntityComponentManager &_ecm)
{
  if(!this->InitWavefield(_ecm))
    return;

  if(!this->InitPhysics(_ecm))
    return;

  if(!this->InitMarkers(_ecm))
    return;

  this->validConfig = true;
}

/////////////////////////////////////////////////
bool HydrodynamicsPrivate::InitWavefield(EntityComponentManager &_ecm)
{
  /// \todo - remove hardcoded name
  // Retrieve the wavefield entity using the Name component
  std::string entityName = "wavefield";
  this->wavefieldEntity = _ecm.EntityByComponents(components::Name(entityName));
  // this->wavefieldEntity = _ecm.EntityByComponents(waves::components::Wavefield());
  if (this->wavefieldEntity == kNullEntity)  
  {
    gzwarn << "No wavefield found, no hydrodynamic forces will be calculated\n";
    return false;
  }

  auto comp = _ecm.Component<waves::components::Wavefield>(this->wavefieldEntity);
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

/////////////////////////////////////////////////
bool HydrodynamicsPrivate::InitPhysics(EntityComponentManager &_ecm)
{
  gzmsg << "Hydrodynamics: initialise physics\n";

  /// \todo add checks for a valid wavefield and lock the waek_ptr

  std::string modelName(this->model.Name(_ecm));

  // Populate link meshes
  std::vector<sim::Entity> links;
  std::vector<std::vector<cgal::MeshPtr>> meshes;
  std::vector<std::vector<Entity>> collisions;
  CreateCollisionMeshes(_ecm, this->model, links, meshes, collisions);
  gzmsg << "Hydrodynamics: links:  " << links.size() << std::endl;
  gzmsg << "Hydrodynamics: meshes: " << meshes.size() << std::endl;

  for (size_t i=0; i<links.size(); ++i)
  {
    // Create storage
    size_t meshCount = meshes[i].size();
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

    gzmsg << "Hydrodynamics: initialising link ["
        << hd->link.Name(_ecm).value() << "]\n";
    gzmsg << "Hydrodynamics: link has ["
        << meshCount << "] collision meshes\n";

    /// \todo check that the link has valid pose components

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
    /// value retured is the origin. The utility function worldPose is
    /// correct, and its usage for the linkPose, linkCoGPose and collisionPose
    /// ensure that they are correct.
    ///
    /// See also: 
    /// HydrodynamicsPrivate::UpdatePhysics it appears that the component
    /// data for the link is updated one time-step behind.
    ///

    // The link pose is required for the water patch, the CoM pose for dynamics.
    // gz::math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();
    gz::math::Pose3d linkPose = worldPose(hd->link.Entity(), _ecm);
    gzmsg << "Hydrodynamics: link world pose\n";
    gzmsg << linkPose << "\n";

    /// \todo subtle difference here - inertial pose includes
    /// any rotation of the inertial matrix where CoG pose does not.
    // gz::math::Pose3d linkCoMPose = hd->link->WorldCoGPose();
    // gz::math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();    
    auto inertial = _ecm.Component<components::Inertial>(hd->link.Entity());
    gz::math::Pose3d linkCoMPose = linkPose * inertial->Data().Pose();
    gzmsg << "Hydrodynamics: link world CoM pose\n";
    gzmsg << linkCoMPose << "\n";

    // Water patch grid
    /// \todo fix hardcoded patch size. CollisionBoundingBox is not currently available 
    // auto boundingBox = hd->link->CollisionBoundingBox();
    // double patchSize = 2.2 * boundingBox.Size().Length();
    double patchSize = 20.0;
    gzmsg << "Hydrodynamics: set water patch size: "
        << patchSize << std::endl;
    std::shared_ptr<waves::Grid> initWaterPatch(
        new waves::Grid({patchSize, patchSize}, { 4, 4 }));

    // WavefieldSampler - this is updated by the pose of the link (not the CoM).
    /// \todo add checks that the wavefield weak_ptr is valid
    hd->wavefieldSampler.reset(new waves::WavefieldSampler(
        this->wavefield.lock(), initWaterPatch));
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();

    // RigidBody - the pose of the CoM is required for the dynamics. 
    cgal::Vector3 linVelocity = waves::ToVector3(
        hd->link.WorldLinearVelocity(_ecm).value());
    cgal::Vector3 angVelocity = waves::ToVector3(
        hd->link.WorldAngularVelocity(_ecm).value());
    /// \todo WorldCoGPose is currently not available
    // cgal::Vector3 linVelocityCoM = waves::ToVector3(
    //     hd->link.WorldCoGLinearVelocity(_ecm).value());
    cgal::Vector3 linVelocityCoM = linVelocity;

    for (size_t j=0; j<meshCount; ++j)
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
      auto collisionPose = worldPose(linkCollision, _ecm);
      ApplyPose(collisionPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);
      
      // DEBUG_INFO
      // gzmsg << "Hydrodynamics: collision pose\n";
      // gzmsg << collisionPose << "\n";

      // Initialise Hydrodynamics
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

/////////////////////////////////////////////////
void HydrodynamicsPrivate::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->UpdatePhysics(_info, _ecm);
  this->UpdateMarkers(_info, _ecm);
}

/////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdatePhysics(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  ////////// BEGIN TESTING

  // Get the wave height at the origin
  double simTime = std::chrono::duration<double>(_info.simTime).count();
  cgal::Point3 point(0.0, 0.0, 0.0);
  double waveHeight{0.0};
  this->wavefield.lock()->Height(point, waveHeight);

  // gzmsg << "[" << simTime << "] : " << waveHeight << "\n";  

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
    cgal::Vector3 linVelocityCoM = linVelocity;

    // Meshes
    size_t nSubTri = 0;
    for (size_t j=0; j<hd->linkMeshes.size(); ++j)
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
      nSubTri += hd->hydrodynamics[j]->GetSubmergedTriangles().size();

      // DEBUG_INFO
      // gzmsg << "Link:         " << hd->link->GetName() << std::endl;
      // gzmsg << "Position:     " << linkPose.Pos() << std::endl;
      // gzmsg << "Rotation:     " << linkPose.Rot().Euler() << std::endl;
      // gzmsg << "SubTriCount:  " << nSubTri << std::endl;
      // gzmsg << "Force:        " << force << std::endl;
      // gzmsg << "Torque:       " << torque << std::endl;
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
  // marker lifetime
  double updatePeriodNsec = static_cast<uint32_t>(
    1.0E9/this->updateRate);

  std::string modelName(this->model.Name(_ecm));
  int markerId = 0;
  for (auto&& hd : this->hydroData)
  {
    hd->waterPatchMsg.set_ns(modelName + "/water_patch");
    hd->waterPatchMsg.set_id(markerId++);
    hd->waterPatchMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
    hd->waterPatchMsg.set_type(gz::msgs::Marker::TRIANGLE_LIST);
    hd->waterPatchMsg.set_visibility(gz::msgs::Marker::GUI);

    // set lifetime
    hd->waterPatchMsg.mutable_lifetime()->set_sec(0);
    hd->waterPatchMsg.mutable_lifetime()->set_nsec(updatePeriodNsec);

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

//////////////////////////////////////////////////
void HydrodynamicsPrivate::InitWaterlineMarkers(
    EntityComponentManager &_ecm)
{
  // marker lifetime
  double updatePeriodNsec = static_cast<uint32_t>(
    1.0E9/this->updateRate);

  std::string modelName(this->model.Name(_ecm));
  int markerId = 0;
  for (auto&& hd : this->hydroData)
  {
    for (size_t j=0; j<hd->linkMeshes.size(); ++j)
    {
      hd->waterlineMsgs[j].set_ns(modelName + "/waterline");
      hd->waterlineMsgs[j].set_id(markerId++);
      hd->waterlineMsgs[j].set_action(gz::msgs::Marker::ADD_MODIFY);
      hd->waterlineMsgs[j].set_type(gz::msgs::Marker::LINE_LIST);
      hd->waterlineMsgs[j].set_visibility(gz::msgs::Marker::GUI);

      // set lifetime
      hd->waterlineMsgs[j].mutable_lifetime()->set_sec(0);
      hd->waterlineMsgs[j].mutable_lifetime()->set_nsec(updatePeriodNsec);

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
  // marker lifetime
  double updatePeriodNsec = static_cast<uint32_t>(
    1.0E9/this->updateRate);

  std::string modelName(this->model.Name(_ecm));
  int markerId = 0;
  for (auto&& hd : this->hydroData)
  {
    for (size_t j=0; j<hd->linkMeshes.size(); ++j)
    {
      hd->underwaterSurfaceMsgs[j].set_ns(modelName + "/underwater_surface");
      hd->underwaterSurfaceMsgs[j].set_id(markerId++);
      hd->underwaterSurfaceMsgs[j].set_action(gz::msgs::Marker::ADD_MODIFY);
      hd->underwaterSurfaceMsgs[j].set_type(gz::msgs::Marker::TRIANGLE_LIST);
      hd->underwaterSurfaceMsgs[j].set_visibility(gz::msgs::Marker::GUI);

      // set lifetime
      hd->underwaterSurfaceMsgs[j].mutable_lifetime()->set_sec(0);
      hd->underwaterSurfaceMsgs[j].mutable_lifetime()->set_nsec(updatePeriodNsec);

      // Set material properties
      gz::msgs::Set(
        hd->underwaterSurfaceMsgs[j].mutable_material()->mutable_ambient(),
        gz::math::Color(0, 0, 1, 0.7));
      gz::msgs::Set(
        hd->underwaterSurfaceMsgs[j].mutable_material()->mutable_diffuse(),
        gz::math::Color(0, 0, 1, 0.7));
    }
  }
  this->initializedUnderwaterSurface = true;
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateMarkers(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  std::string topicName("/marker");

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
    if(!this->initializedWaterPatch)
      this->InitWaterPatchMarkers(_ecm);

    this->UpdateWaterPatchMarkers(_info, _ecm);
  }

  if (this->showWaterline)
  {
    if(!this->initializedWaterline)
      this->InitWaterlineMarkers(_ecm);

    this->UpdateWaterlineMarkers(_info, _ecm);
  }

  if (this->showUnderwaterSurface)
  {
    if(!this->initializedUnderwaterSurface)
      this->InitUnderwaterSurfaceMarkers(_ecm);

    this->UpdateUnderwaterSurfaceMarkers(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateWaterPatchMarkers(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  std::string topicName("/marker");

  for (auto&& hd : this->hydroData)
  {
    auto& grid = *hd->wavefieldSampler->GetWaterPatch();

    // clear and update
    hd->waterPatchMsg.mutable_point()->Clear();
    for (size_t ix=0; ix<grid.GetCellCount()[0]; ++ix)
    {
      for (size_t iy=0; iy<grid.GetCellCount()[1]; ++iy)
      {
        for (size_t k=0; k<2; ++k)
        {
          cgal::Triangle tri = grid.GetTriangle(ix, iy, k);
          gz::msgs::Set(hd->waterPatchMsg.add_point(), waves::ToGz(tri[0]));
          gz::msgs::Set(hd->waterPatchMsg.add_point(), waves::ToGz(tri[1]));
          gz::msgs::Set(hd->waterPatchMsg.add_point(), waves::ToGz(tri[2]));
        }
      }
    }
    this->node.Request(topicName, hd->waterPatchMsg);
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateWaterlineMarkers(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  std::string topicName("/marker");

  for (auto&& hd : this->hydroData)
  {
    for (size_t j=0; j<hd->linkMeshes.size(); ++j)
    {
      hd->waterlineMsgs[j].mutable_point()->Clear();
      if (hd->hydrodynamics[j]->GetWaterline().empty())
      {
        /// \todo workaround. The previous marker is not cleared if a cleared point list is published.
        gz::msgs::Set(hd->waterlineMsgs[j].add_point(), gz::math::Vector3d::Zero);
        gz::msgs::Set(hd->waterlineMsgs[j].add_point(), gz::math::Vector3d::Zero);
      }
      for (auto&& line : hd->hydrodynamics[j]->GetWaterline())
      {
        gz::msgs::Set(hd->waterlineMsgs[j].add_point(), waves::ToGz(line.point(0)));
        gz::msgs::Set(hd->waterlineMsgs[j].add_point(), waves::ToGz(line.point(1)));
      }
      this->node.Request(topicName, hd->waterlineMsgs[j]);
    }
  }
}

//////////////////////////////////////////////////
void HydrodynamicsPrivate::UpdateUnderwaterSurfaceMarkers(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  std::string topicName("/marker");

  for (auto&& hd : this->hydroData)
  {
    for (size_t j=0; j<hd->linkMeshes.size(); ++j)
    {
      hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
      if (hd->hydrodynamics[j]->GetSubmergedTriangles().empty())
      {
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), gz::math::Vector3d::Zero);
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), gz::math::Vector3d::Zero);
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), gz::math::Vector3d::Zero);
      }
      for (auto&& tri : hd->hydrodynamics[j]->GetSubmergedTriangles())
      {
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), waves::ToGz(tri[0]));
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), waves::ToGz(tri[1]));
        gz::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), waves::ToGz(tri[2]));
      }
      this->node.Request(topicName, hd->underwaterSurfaceMsgs[j]);
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
      auto type = param.type();
      auto value = param.bool_value();
      this->showWaterPatch = value;
    }
  }
  {
    auto it = _msg.params().find("waterline");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is bool
      auto param = it->second;
      auto type = param.type();
      auto value = param.bool_value();
      this->showWaterline = value;
    }
  }
  {
    auto it = _msg.params().find("underwater_surface");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is bool
      auto param = it->second;
      auto type = param.type();
      auto value = param.bool_value();
      this->showUnderwaterSurface = value;
    }
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(Hydrodynamics,
              gz::sim::System,
              Hydrodynamics::ISystemConfigure,
              Hydrodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Hydrodynamics,
                    "gz::sim::systems::Hydrodynamics")
