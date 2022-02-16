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

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Name.hh>

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

#if 0
    // Links
    for (auto&& link : _model->GetLinks())
    {
      GZ_ASSERT(link != nullptr, "Link must be valid");
      _links.push_back(link);
      std::string linkName(link->GetName());
      std::vector<std::shared_ptr<cgal::Mesh>> linkMeshes;
      
      // Collisions
      for (auto&& collision : link->GetCollisions())
      {
        GZ_ASSERT(collision != nullptr, "Collision must be valid");
        std::string collisionName(collision->GetName());
  
        // Shape
        physics::ShapePtr shape = collision->GetShape();
        GZ_ASSERT(shape != nullptr, "Shape must be valid");
        ignmsg << "Shape:      " << shape->TypeStr() << std::endl;
        ignmsg << "Scale:      " << shape->Scale() << std::endl;
        ignmsg << "Type:       " << std::hex << shape->GetType() << std::dec << std::endl;

        if (shape->HasType(physics::Base::EntityType::BOX_SHAPE))
        {
          // BoxShape
          ignmsg << "Type:       " << "BOX_SHAPE" << std::endl;
          physics::BoxShapePtr box = boost::dynamic_pointer_cast<physics::BoxShape>(shape);
          GZ_ASSERT(box != nullptr, "Failed to cast Shape to BoxShape");
          ignmsg << "Size:       " << box->Size() << std::endl;

          // Mesh
          std::string meshName = std::string(modelName)
            .append(".").append(linkName)
            .append(".").append(collisionName)
            .append(".box");
          common::MeshManager::Instance()->CreateBox(
            meshName,
            box->Size(),
            math::Vector2d(1, 1));          
          GZ_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
            "Failed to create Mesh for BoxShape");

          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);
          
          // ignmsg << "Mesh:       " << mesh->GetName() << std::endl;
          ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }
      
        if (shape->HasType(physics::Base::EntityType::CYLINDER_SHAPE))
        {
          // CylinderShape
          ignmsg << "Type:       " << "CYLINDER_SHAPE" << std::endl;
          physics::CylinderShapePtr cylinder = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
          GZ_ASSERT(cylinder != nullptr, "Failed to cast Shape to CylinderShape");
          ignmsg << "Radius:     " << cylinder->GetRadius() << std::endl;
          ignmsg << "Length:     " << cylinder->GetLength() << std::endl;

          // Mesh
          std::string meshName = std::string(modelName)
            .append("::").append(linkName)
            .append("::").append(collisionName)
            .append("::cylinder");
          common::MeshManager::Instance()->CreateCylinder(
            meshName,
            cylinder->GetRadius(),  // radius
            cylinder->GetLength(),  // length,
            1,                      // rings
            32);                    // segments
          GZ_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
            "Failed to create Mesh for Cylinder");

          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);

          ignmsg << "Mesh:       " << meshName << std::endl;
          ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }      

        if (shape->HasType(physics::Base::EntityType::SPHERE_SHAPE))
        {
          // SphereShape
          ignmsg << "Type:       " << "SPHERE_SHAPE" << std::endl;
          physics::SphereShapePtr sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
          GZ_ASSERT(sphere != nullptr, "Failed to cast Shape to SphereShape");
          ignmsg << "Radius:     " << sphere->GetRadius() << std::endl;

          // Mesh
          std::string meshName = std::string(modelName)
            .append(".").append(linkName)
            .append(".").append(collisionName)
            .append(".cylinder");
          common::MeshManager::Instance()->CreateSphere(
            meshName,
            sphere->GetRadius(),    // radius
            8,                      // rings
            8);                     // segments
          GZ_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
            "Failed to create Mesh for Cylinder");

          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);

          ignmsg << "Mesh:       " << meshName << std::endl;
          ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }

        if (shape->HasType(physics::Base::EntityType::MESH_SHAPE))
        {
          // MeshShape
          ignmsg << "Type:       " << "MESH_SHAPE" << std::endl;
          physics::MeshShapePtr meshShape = boost::dynamic_pointer_cast<physics::MeshShape>(shape);
          GZ_ASSERT(meshShape != nullptr, "Failed to cast Shape to MeshShape");

          std::string meshUri = meshShape->GetMeshURI();
          std::string meshStr = common::find_file(meshUri);
          ignmsg << "MeshURI:    " << meshUri << std::endl;
          ignmsg << "MeshStr:    " << meshStr << std::endl;

          // Mesh
          if (!common::MeshManager::Instance()->HasMesh(meshStr))
          {
            ignerr << "Mesh: " << meshStr << " was not loaded"<< std::endl;
            return;
          } 

          std::shared_ptr<cgal::Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshStr), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);

          ignmsg << "Mesh:       " << meshStr << std::endl;
          ignmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }

      }

      // Add meshes for this link
      _meshes.push_back(linkMeshes);
    }
#endif
  } 

#if 0
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
#endif

  // NOT REQUIRED: we are using the ECM instead.
  //
  // /// \brief Retrive a pointer to a wavefield from the Wavefield plugin.
  // ///
  // /// \param _world           A pointer to the world containing the wave field.
  // /// \param _waveModelName   The name of the wavefield model containing the wave field. 
  // /// \return A valid wavefield if found and nullptr if not.
  // std::shared_ptr<const Wavefield> GetWavefield(
  //   physics::WorldPtr _world,
  //   const std::string& _waveModelName)
  // {
  //   GZ_ASSERT(_world != nullptr, "World is null");
  // 
  //   physics::ModelPtr wavefieldModel = _world->ModelByName(_waveModelName);    
  //   if(wavefieldModel == nullptr)
  //   {
  //     ignerr << "No Wavefield Model found with name '" << _waveModelName << "'." << std::endl;
  //     return nullptr;
  //   }
  //
  //   std::string wavefieldEntityName(WavefieldEntity::MakeName(_waveModelName));
  // 
  //   physics::BasePtr base = wavefieldModel->GetChild(wavefieldEntityName);
  //   boost::shared_ptr<WavefieldEntity> wavefieldEntity 
  //     = boost::dynamic_pointer_cast<WavefieldEntity>(base);
  //   if (wavefieldEntity == nullptr)
  //   {
  //     ignerr << "Wavefield Entity is null: " << wavefieldEntityName << std::endl;
  //     return nullptr;
  //   }    
  //   GZ_ASSERT(wavefieldEntity->GetWavefield() != nullptr, "Wavefield is null.");
  // 
  //   return wavefieldEntity->GetWavefield();
  // }

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
      for (auto&& ptr : this->hydrodynamics)
        ptr.reset();
      for (auto&& ptr : this->initLinkMeshes)
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

  // Hydrodynamics parameters
  this->dataPtr->hydroParams.reset(new marine::HydrodynamicsParameters());
  this->dataPtr->hydroParams->SetFromSDF(*this->dataPtr->sdf);

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
  /// \todo(srmainwaring): remove hardcoded name
  // Retrieve the wavefield entiry using the Name component
  this->wavefieldEntity = _ecm.EntityByComponents(components::Name("WAVEFIELD"));
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

  // NOT REQUIRED: we are using the ECM instead.
  //
  // // Wavefield
  // this->data->wavefield = GetWavefield(
  //   this->data->world, this->data->waveModelName);
  // if (this->data->wavefield == nullptr) 
  // {
  //   ignerr << "Wavefield is NULL" << std::endl;
  //   return;
  // }

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

    /// \todo(srmainwaring) - check that the link has valid pose
    /// information attached...

    // The link pose is required for the water patch, the CoM pose for dynamics.
    math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();

    /// \todo(srmainwaring) subtle difference here - inertial pose includes
    /// any rotation of the inertial matrix where CoG pose does not.
    // math::Pose3d linkCoMPose = hd->link->WorldCoGPose();
    math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();

#if 0
    // Water patch grid
    auto boundingBox = hd->link->CollisionBoundingBox();
    double patchSize = 2.2 * boundingBox.Size().Length();
    ignmsg << "Water patch size: " << patchSize << std::endl;
    std::shared_ptr<Grid> initWaterPatch(new Grid({patchSize, patchSize}, { 4, 4 }));

    // WavefieldSampler - this is updated by the pose of the link (not the CoM).
    hd->wavefieldSampler.reset(new WavefieldSampler(
      this->wavefield, initWaterPatch));
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();

    // RigidBody - the pose of the CoM is required for the dynamics. 
    cgal::Vector3 linVelocity = ToVector3(hd->link->WorldLinearVel());
    cgal::Vector3 angVelocity = ToVector3(hd->link->WorldAngularVel());
    cgal::Vector3 linVelocityCoM = ToVector3(hd->link->WorldCoGLinearVel());

    for (size_t j=0; j<meshCount; ++j)
    {
      // Mesh (SurfaceMesh copy performs a deep copy of all properties)
      std::shared_ptr<cgal::Mesh> initLinkMesh = meshes[i][j];
      std::shared_ptr<cgal::Mesh> linkMesh = std::make_shared<Mesh>(*initLinkMesh);
      GZ_ASSERT(linkMesh != nullptr, "Invalid Mesh returned from CopyMesh");

      // Mesh
      hd->initLinkMeshes[j] = initLinkMesh;
      hd->linkMeshes[j] = linkMesh;

      // Update link mesh
      ApplyPose(linkPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);

      // Initialise Hydrodynamics
      hd->hydrodynamics[j].reset(
        new Hydrodynamics(
          this->data->hydroParams,
          hd->linkMeshes[j],
          hd->wavefieldSampler));
      hd->hydrodynamics[j]->Update(
        hd->wavefieldSampler, linkCoMPose, linVelocity, angVelocity);
    }
#endif
  }

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
  double simTime = std::chrono::duration<double>(_info.simTime).count();

  // Get the wave height at the origin
  cgal::Point3 point(0.0, 0.0, 0.0);
  double waveHeight{0.0};
  this->wavefield.lock()->Height(point, waveHeight);

  // ignmsg << "[" << simTime << "] : " << waveHeight << "\n";  
  ////////// END TESTING

  // NOT REQUIRED: we are using the ECM instead.
  // 
  // // Update wavefield 
  // this->data->wavefield = GetWavefield(
  //   this->data->world, this->data->waveModelName);
  // if (this->data->wavefield == nullptr) 
  // {
  //   ignerr << "Wavefield is NULL" << std::endl;
  //   return;
  // }

  for (auto&& hd : this->hydroData)
  {
    // GZ_ASSERT(hd->link != nullptr, "Link is NULL");

    // The link pose is required for the water patch, the CoM pose for dynamics.
    math::Pose3d linkPose = hd->link.WorldPose(_ecm).value();
    // math::Pose3d linkCoMPose = hd->link.WorldCoGPose(_ecm).value();
    math::Pose3d linkCoMPose = hd->link.WorldInertialPose(_ecm).value();

    // Update water patch
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();
    // auto waterPatch = hd->wavefieldSampler->GetWaterPatch();

    // RigidBody - the pose of the CoM is required for the dynamics. 
    // cgal::Vector3 linVelocity = ToVector3(hd->link.WorldLinearVel(_ecm).value());
    // cgal::Vector3 angVelocity = ToVector3(hd->link.WorldAngularVel(_ecm).value());
    /// \todo - currently not available
    //cgal::Vector3 linVelocityCoM = ToVector3(hd->link.WorldCoGLinearVel(_ecm).value());
    // cgal::Vector3 linVelocityCoM = linVelocity;

#if 0
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
      auto force = ToIgn(hd->hydrodynamics[j]->Force());
      if (force.IsFinite()) 
      {
        hd->link->AddForce(force);
      }

      // Apply torques to the link
      auto torque = ToIgn(hd->hydrodynamics[j]->Torque());
      if (torque.IsFinite()) 
      {
        hd->link->AddTorque(torque);
      }

      // Info for Markers
      nSubTri += hd->hydrodynamics[j]->GetSubmergedTriangles().size();


      // @DEBUG_INFO
      // ignmsg << "Link:         " << hd->link->GetName() << std::endl;
      // ignmsg << "Position:     " << linkPose.Pos() << std::endl;
      // ignmsg << "Rotation:     " << linkPose.Rot().Euler() << std::endl;
      // ignmsg << "SubTriCount:  " << nSubTri << std::endl;
      // ignmsg << "Force:        " << force << std::endl;
      // ignmsg << "Torque:       " << torque << std::endl;
    }
#endif
  }
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(Hydrodynamics,
                    ignition::gazebo::System,
                    Hydrodynamics::ISystemConfigure,
                    Hydrodynamics::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Hydrodynamics,
  "ignition::gazebo::systems::Hydrodynamics")
