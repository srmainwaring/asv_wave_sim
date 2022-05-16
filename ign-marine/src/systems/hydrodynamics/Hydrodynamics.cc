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

#include "ignition/marine/Convert.hh"
#include "ignition/marine/Grid.hh"
#include "ignition/marine/MeshTools.hh"
#include "ignition/marine/Physics.hh"
#include "ignition/marine/Utilities.hh"
#include "ignition/marine/Wavefield.hh"
#include "ignition/marine/WavefieldSampler.hh"

#include "ignition/marine/components/Wavefield.hh"

#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Element.hh>

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

namespace ignition
{
namespace gazebo
{
  /////////////////////////////////////////////////
  // Collision (similar to gazebo::Link and gazebo::Model interfaces)
 
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
    public: explicit Collision(gazebo::Entity _entity = kNullEntity)
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
    public: gazebo::Entity Entity() const
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
  void CreateCollisionMeshes(
    EntityComponentManager &_ecm,
    gazebo::Model _model,
    std::vector<gazebo::Entity>& _links,
    std::vector<std::vector<cgal::MeshPtr>>& _meshes)
  {
    // There will be more than one mesh per link if the link contains mutiple collisions.

    // Model
    std::string modelName(_model.Name(_ecm));

    // Links
    for (auto& linkEntity : _model.Links(_ecm))
    {
      IGN_ASSERT(linkEntity != kNullEntity, "Link must be valid");
      _links.push_back(linkEntity);
      gazebo::Link link(linkEntity);

      /// \todo check link has valid name component
      std::string linkName(link.Name(_ecm).value());
      std::vector<std::shared_ptr<cgal::Mesh>> linkMeshes;

      ignmsg << "Hydrodynamics: create collision mesh for link ["
          << linkName << "]\n";
      
      // Collisions
      for (auto& collisionEntity : link.Collisions(_ecm))
      {
        IGN_ASSERT(collisionEntity != kNullEntity, "Collision must be valid");
  
        gazebo::Collision collision(collisionEntity); 
        std::string collisionName(collision.Name(_ecm).value());
        ignmsg << "Hydrodynamics: collision name [" << collisionName << "]\n";

        const components::CollisionElement *coll =
          _ecm.Component<components::CollisionElement>(collisionEntity);

        if (!coll)
        {
          ignerr << "Invalid collision pointer. This shouldn't happen\n";
          continue;
        }

        double volume = 0;
        switch (coll->Data().Geom()->Type())
        {
          case sdf::GeometryType::BOX:
          {
            // Get shape from the collision component
            auto& box = coll->Data().Geom()->BoxShape()->Shape();

            // Create the ignition mesh
            std::string meshName = std::string(modelName)
                .append(".").append(linkName)
                .append(".").append(collisionName)
                .append(".box");
            common::MeshManager::Instance()->CreateBox(
                meshName,
                box.Size(),
                math::Vector2d(1, 1));          
            IGN_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
                "Failed to create Mesh for Box");

            // Create the CGAL surface mesh
            std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
            marine::MeshTools::MakeSurfaceMesh(
                *common::MeshManager::Instance()->MeshByName(meshName), *mesh);
            IGN_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);

            ignmsg << "Type:       " << "BOX" << std::endl;
            ignmsg << "Size:       " << box.Size() << std::endl;
            ignmsg << "MeshName:   " << meshName << std::endl;            
            ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
            break;
          }
          case sdf::GeometryType::SPHERE:
          {
            // Get shape from the collision component
            auto& sphere = coll->Data().Geom()->SphereShape()->Shape();

            // Create the ignition mesh
            std::string meshName = std::string(modelName)
                .append(".").append(linkName)
                .append(".").append(collisionName)
                .append(".sphere");
            common::MeshManager::Instance()->CreateSphere(
                meshName,
                sphere.Radius(),        // radius
                8,                      // rings
                8);                     // segments
            IGN_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
                "Failed to create Mesh for Sphere");

            // Create the CGAL surface mesh
            std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
            marine::MeshTools::MakeSurfaceMesh(
                *common::MeshManager::Instance()->MeshByName(meshName), *mesh);
            IGN_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);

            ignmsg << "Type:       " << "SPHERE" << std::endl;
            ignmsg << "Radius:     " << sphere.Radius() << std::endl;
            ignmsg << "MeshName:   " << meshName << std::endl;            
            ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
            break;
          }
          case sdf::GeometryType::CYLINDER:
          {
            auto& cylinder = coll->Data().Geom()->CylinderShape()->Shape();

            // Create the ignition mesh
            std::string meshName = std::string(modelName)
                .append(".").append(linkName)
                .append(".").append(collisionName)
                .append(".cylinder");
            common::MeshManager::Instance()->CreateCylinder(
                meshName,
                cylinder.Radius(),      // radius
                cylinder.Length(),      // length,
                1,                      // rings
                32);                    // segments
            IGN_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
                "Failed to create Mesh for Cylinder");

            // Create the CGAL surface mesh
            std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
            marine::MeshTools::MakeSurfaceMesh(
                *common::MeshManager::Instance()->MeshByName(meshName), *mesh);
            IGN_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);

            ignmsg << "Type:       " << "CYLINDER" << std::endl;
            ignmsg << "Radius:     " << cylinder.Radius() << std::endl;
            ignmsg << "Length:     " << cylinder.Length() << std::endl;
            ignmsg << "MeshName:   " << meshName << std::endl;            
            ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
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
            // if (common::MeshManager::Instance()->IsValidFilename(file))
            // {
            //   const common::Mesh *mesh =
            //     common::MeshManager::Instance()->Load(file);
            //   if (mesh)
            //     volume = mesh->Volume();
            //   else
            //     ignerr << "Unable to load mesh[" << file << "]\n";
            // }
            // else
            // {
            //   ignerr << "Invalid mesh filename[" << file << "]\n";
            // }

            // Mesh
            if (!common::MeshManager::Instance()->IsValidFilename(file))
            {
              ignerr << "Mesh: " << file << " was not loaded"<< std::endl;
              return;
            } 

            // Create the CGAL surface mesh
            std::shared_ptr<cgal::Mesh> mesh = std::make_shared<cgal::Mesh>();
            marine::MeshTools::MakeSurfaceMesh(
                *common::MeshManager::Instance()->Load(file), *mesh);
            IGN_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
            linkMeshes.push_back(mesh);

            ignmsg << "Type:       " << "MESH" << std::endl;
            ignmsg << "Uri:        " << uri << std::endl;
            ignmsg << "FilePath:   " << filePath << std::endl;
            ignmsg << "MeshFile:   " << file << std::endl;
            ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
            break;
          }
          default:
          {
            ignerr << "Unsupported collision geometry["
              << static_cast<int>(coll->Data().Geom()->Type()) << "]\n";
            break;
          }
        }
      }

      // Add meshes for this link
      _meshes.push_back(linkMeshes);
    }
  } 

  /////////////////////////////////////////////////
  /// \brief Transform a meshes vertex points in the world frame according to a Pose.
  ///
  /// \param[in] _pose    A pose defining a translation and rotation in the world frame.
  /// \param[in] _source  The original mesh to transform.
  /// \param[out] _target The transformed mesh.
  void ApplyPose(
    const math::Pose3d& _pose,
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
      math::Vector3d ignP0(p0.x(), p0.y(), p0.z());
      math::Vector3d ignP1 = _pose.Rot().RotateVector(ignP0) + _pose.Pos();

      cgal::Point3& p1 = _target.point(v1);
      p1 = cgal::Point3(ignP1.X(), ignP1.Y(), ignP1.Z());
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
    // public: gazebo::Entity link;
    public: gazebo::Link link{kNullEntity};

    /// \brief The wavefield sampler for this link.
    public: marine::WavefieldSamplerPtr wavefieldSampler;
    
    /// \brief The initial meshes for this link.
    public: std::vector<cgal::MeshPtr> initLinkMeshes;

    /// \brief The transformed meshes for this link.
    public: std::vector<cgal::MeshPtr> linkMeshes;

    /// \brief Objects to compute the hydrodynamics forces for each link mesh.
    public: std::vector<marine::HydrodynamicsPtr> hydrodynamics;

    /// \brief Marker messages for the water patch.
    // public: msgs::Marker waterPatchMsg;

    /// \brief Marker messages for the waterline.
    // public: std::vector<msgs::Marker> waterlineMsgs;

    /// \brief Marker messages for the underwater portion of the mesh.
    // public: std::vector<msgs::Marker> underwaterSurfaceMsgs;
  };

  typedef std::shared_ptr<HydrodynamicsLinkData> HydrodynamicsLinkDataPtr;
}
}
}

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::HydrodynamicsPrivate
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
  public: gazebo::Model model{kNullEntity};

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
  public: marine::WavefieldWeakPtr wavefield;

  ////////// BEGIN HYDRODYNAMICS PLUGIN

  /// \brief Hydrodynamics parameters for the entire model.
  public: marine::HydrodynamicsParametersPtr hydroParams;

  /// \brief Hydrodynamic physics for each Link.
  public: std::vector<HydrodynamicsLinkDataPtr> hydroData;

  // /// \brief The wave model name. This is used to retrieve a pointer to the wave field.
  // public: std::string waveModelName;

  // /// \brief Show the water patch markers.
  // public: bool showWaterPatch;

  // /// \brief Show the waterline markers.
  // public: bool showWaterline;

  // /// \brief Show the underwater surface.
  // public: bool showUnderwaterSurface;

  // /// \brief The update rate for visual markers.
  // public: double updateRate;

  // /// \brief Previous update time.
  // public: common::Time prevTime;

  // /// \brief Connection to the World Update events.
  // public: event::ConnectionPtr updateConnection;

  // /// \brief Ignition transport node for igntopic "/marker".
  // public: transport::Node ignNode;

  // /// \brief Gazebo transport node.
  // public: transport::NodePtr gzNode;

  // /// \brief Subscribe to gztopic "~/hydrodynamics".
  // public: transport::SubscriberPtr hydroSub;

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
  IGN_PROFILE("Hydrodynamics::Configure");

  ignmsg << "Hydrodynamics: configuring\n";

  // Capture the model entity
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "The Hydrodynamics system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdf = _sdf->Clone();

  // // Transport
  // this->data->gzNode = transport::NodePtr(new transport::Node());
  // this->data->gzNode->Init(this->data->world->Name() + "/" + this->data->model->GetName());

  // // Subscribers
  // this->data->hydroSub = this->data->gzNode->Subscribe(
  //   "~/hydrodynamics", &HydrodynamicsPlugin::OnHydrodynamicsMsg, this);

  // // Bind the update callback to the world update event 
  // this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
  //   std::bind(&HydrodynamicsPlugin::OnUpdate, this));

  // Moved to HydrodynamicsPrivate::Load (deferred until other entities available)
  // // Wave Model
  // this->data->waveModelName = Utilities::SdfParamString(*_sdf, "wave_model", "");

  // Empty sdf element used as a placeholder for missing elements 
  auto sdfEmpty = std::make_shared<sdf::Element>();

  // Hydrodynamics parameters
  this->dataPtr->hydroParams.reset(new marine::HydrodynamicsParameters());

  auto sdfHydro = sdfEmpty;
  if (this->dataPtr->sdf->HasElement("hydrodynamics"))
  {
    sdfHydro = _sdf->GetElementImpl("hydrodynamics");
  }
  this->dataPtr->hydroParams->SetFromSDF(*sdfHydro);


  // // Markers
  // if (_sdf->HasElement("markers"))
  // {
  //   sdf::ElementPtr sdfMarkers = _sdf->GetElement("markers");
  //   this->data->updateRate            = Utilities::SdfParamDouble(*sdfMarkers, "update_rate",         30.0);
  //   this->data->showWaterPatch        = Utilities::SdfParamBool(*sdfMarkers,   "water_patch",         false);
  //   this->data->showWaterline         = Utilities::SdfParamBool(*sdfMarkers,   "waterline",           false);
  //   this->data->showUnderwaterSurface = Utilities::SdfParamBool(*sdfMarkers,   "underwater_surface",  false);
  // }

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

  // if(!this->InitMarkers(_ecm))
  //   return;

  this->validConfig = true;
}

/////////////////////////////////////////////////
bool HydrodynamicsPrivate::InitWavefield(EntityComponentManager &_ecm)
{
  /// \todo - remove hardcoded name
  // Retrieve the wavefield entity using the Name component
  std::string entityName = "wavefield";
  this->wavefieldEntity = _ecm.EntityByComponents(components::Name(entityName));
  // this->wavefieldEntity = _ecm.EntityByComponents(marine::components::Wavefield());
  if (this->wavefieldEntity == kNullEntity)  
  {
    ignwarn << "No wavefield found, no hydrodynamic forces will be calculated\n";
    return false;
  }

  auto comp = _ecm.Component<marine::components::Wavefield>(this->wavefieldEntity);
  if (comp)
  {
    this->wavefield = comp->Data();
  }

  if (!this->wavefield.lock())
  {
    ignwarn << "Invalid wavefield, no hydrodynamic forces will be calculated\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool HydrodynamicsPrivate::InitPhysics(EntityComponentManager &_ecm)
{
  ignmsg << "Hydrodynamics: initialise physics\n";

  /// \todo add checks for a valid wavefield and lock the waek_ptr

  std::string modelName(this->model.Name(_ecm));

  // Populate link meshes
  std::vector<gazebo::Entity> links;
  std::vector<std::vector<cgal::MeshPtr>> meshes;
  CreateCollisionMeshes(_ecm, this->model, links, meshes);
  ignmsg << "Hydrodynamics: links:  " << links.size() << std::endl;
  ignmsg << "Hydrodynamics: meshes: " << meshes.size() << std::endl;

  for (size_t i=0; i<links.size(); ++i)
  {
    // Create storage
    size_t meshCount = meshes[i].size();
    std::shared_ptr<HydrodynamicsLinkData> hd(new HydrodynamicsLinkData);
    this->hydroData.push_back(hd);
    hd->initLinkMeshes.resize(meshCount);
    hd->linkMeshes.resize(meshCount);
    hd->hydrodynamics.resize(meshCount);
    // hd->waterlineMsgs.resize(meshCount);
    // hd->underwaterSurfaceMsgs.resize(meshCount);

    // Wavefield and Link
    hd->link = gazebo::Link(links[i]);
    hd->link.EnableVelocityChecks(_ecm);

    ignmsg << "Hydrodynamics: initialising link ["
        << hd->link.Name(_ecm).value() << "]\n";
    ignmsg << "Hydrodynamics: link has ["
        << meshCount << "] collision meshes\n";

    /// \todo check that the link has valid pose components

    ignmsg << "Hydrodynamics: getting link world pose\n";
    // The link pose is required for the water patch, the CoM pose for dynamics.
    math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();

    ignmsg << "Hydrodynamics: getting link world CoM pose\n";
    /// \todo subtle difference here - inertial pose includes
    /// any rotation of the inertial matrix where CoG pose does not.
    // math::Pose3d linkCoMPose = hd->link->WorldCoGPose();
    math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();

    // Water patch grid
    /// \todo fix hardcoded patch size. CollisionBoundingBox is not currently available 
    // auto boundingBox = hd->link->CollisionBoundingBox();
    // double patchSize = 2.2 * boundingBox.Size().Length();
    double patchSize = 20.0;
    ignmsg << "Hydrodynamics: set water patch size: "
        << patchSize << std::endl;
    std::shared_ptr<marine::Grid> initWaterPatch(
        new marine::Grid({patchSize, patchSize}, { 4, 4 }));

    // WavefieldSampler - this is updated by the pose of the link (not the CoM).
    /// \todo add checks that the wavefield weak_ptr is valid
    hd->wavefieldSampler.reset(new marine::WavefieldSampler(
        this->wavefield.lock(), initWaterPatch));
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();

    // RigidBody - the pose of the CoM is required for the dynamics. 
    cgal::Vector3 linVelocity = marine::ToVector3(
        hd->link.WorldLinearVelocity(_ecm).value());
    cgal::Vector3 angVelocity = marine::ToVector3(
        hd->link.WorldAngularVelocity(_ecm).value());
    /// \todo WorldCoGPose is currently not available
    // cgal::Vector3 linVelocityCoM = marine::ToVector3(
    //     hd->link.WorldCoGLinearVelocity(_ecm).value());
    cgal::Vector3 linVelocityCoM = linVelocity;

    for (size_t j=0; j<meshCount; ++j)
    {
      // Mesh (SurfaceMesh copy performs a deep copy of all properties)
      std::shared_ptr<cgal::Mesh> initLinkMesh = meshes[i][j];
      std::shared_ptr<cgal::Mesh> linkMesh =
          std::make_shared<cgal::Mesh>(*initLinkMesh);
      IGN_ASSERT(linkMesh != nullptr, "Invalid Mesh returned from CopyMesh");

      // Mesh
      hd->initLinkMeshes[j] = initLinkMesh;
      hd->linkMeshes[j] = linkMesh;

      // Update link mesh
      ApplyPose(linkPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);

      // Initialise Hydrodynamics
      hd->hydrodynamics[j].reset(
        new marine::Hydrodynamics(
          this->hydroParams,
          hd->linkMeshes[j],
          hd->wavefieldSampler));
      hd->hydrodynamics[j]->Update(
        hd->wavefieldSampler, linkCoMPose, linVelocity, angVelocity);
    }
  }

  ignmsg << "Hydrodynamics: done initialise physics\n";
  return true;
}

/////////////////////////////////////////////////
void HydrodynamicsPrivate::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->UpdatePhysics(_info, _ecm);
  // this->UpdateMarkers(_info, _ecm);
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

  // ignmsg << "[" << simTime << "] : " << waveHeight << "\n";  

  ////////// END TESTING

  /// \todo add checks for a valid wavefield and lock the waek_ptr

  for (auto& hd : this->hydroData)
  {
    // The link pose is required for the water patch, the CoM pose for dynamics.
    math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();
    /// \todo WorldCoGPose is currently not available
    // math::Pose3d linkCoMPose = hd->link.WorldCoGPose(_ecm).value();
    math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();

    // Update water patch
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();
    // auto waterPatch = hd->wavefieldSampler->GetWaterPatch();

    // RigidBody - the pose of the CoM is required for the dynamics. 
    /// \todo check the components are available and valid
    cgal::Vector3 linVelocity = marine::ToVector3(
        hd->link.WorldLinearVelocity(_ecm).value());
    cgal::Vector3 angVelocity = marine::ToVector3(
        hd->link.WorldAngularVelocity(_ecm).value());
    /// \todo WorldCoGLinearVel is currently not available
    // cgal::Vector3 linVelocityCoM = marine::ToVector3(
    //     hd->link.WorldCoGLinearVel(_ecm).value());
    cgal::Vector3 linVelocityCoM = linVelocity;

    // Meshes
    size_t nSubTri = 0;
    for (size_t j=0; j<hd->linkMeshes.size(); ++j)
    {
      // Update link mesh
      ApplyPose(linkPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);
      
      // Update hydrodynamics
      hd->hydrodynamics[j]->Update(
        hd->wavefieldSampler, linkCoMPose, linVelocity, angVelocity);

      // Apply forces to the Link
      auto force = marine::ToIgn(hd->hydrodynamics[j]->Force());
      if (force.IsFinite()) 
      {
        hd->link.AddWorldForce(_ecm, force);
      }

      // Apply torques to the link
      auto torque = marine::ToIgn(hd->hydrodynamics[j]->Torque());
      if (torque.IsFinite()) 
      {
        hd->link.AddWorldWrench(_ecm, math::Vector3d::Zero, torque);
      }

      // Info for Markers
      nSubTri += hd->hydrodynamics[j]->GetSubmergedTriangles().size();

      // DEBUG_INFO
      // ignmsg << "Link:         " << hd->link->GetName() << std::endl;
      // ignmsg << "Position:     " << linkPose.Pos() << std::endl;
      // ignmsg << "Rotation:     " << linkPose.Rot().Euler() << std::endl;
      // ignmsg << "SubTriCount:  " << nSubTri << std::endl;
      // ignmsg << "Force:        " << force << std::endl;
      // ignmsg << "Torque:       " << torque << std::endl;
    }
  }
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(Hydrodynamics,
                    ignition::gazebo::System,
                    Hydrodynamics::ISystemConfigure,
                    Hydrodynamics::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Hydrodynamics,
  "ignition::gazebo::systems::Hydrodynamics")
