// Copyright (C) 2019  Rhys Mainwaring
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

#include "asv_wave_sim_gazebo_plugins/HydrodynamicsPlugin.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/MeshTools.hh"
#include "asv_wave_sim_gazebo_plugins/Physics.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldSampler.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/Shape.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Triangle3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport.hh>

#include <iostream>
#include <iomanip>
#include <string>
#include <exception>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_MODEL_PLUGIN(HydrodynamicsPlugin)

///////////////////////////////////////////////////////////////////////////////
// Utilties
  
  /// \brief Iterate over the links in a model, and create a CGAL SurfaceMesh
  /// for each collison in each link.
  ///
  /// \param[in]  _model    The model being processed. 
  /// \param[out] _links    A vector holding a copy of pointers to the the model's links. 
  /// \param[out] _meshes   A vector of vectors containing a surface mesh for each collision in a link.
  void CreateCollisionMeshes(
    physics::ModelPtr _model,
    std::vector<physics::LinkPtr>& _links,
    std::vector<std::vector<std::shared_ptr<Mesh>>>& _meshes)
  {
    // There will be more than one mesh per link if the link contains mutiple collisions.

    // Model
    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    std::string modelName(_model->GetName());

    // Links
    for (auto&& link : _model->GetLinks())
    {
      GZ_ASSERT(link != nullptr, "Link must be valid");
      _links.push_back(link);
      std::string linkName(link->GetName());
      std::vector<std::shared_ptr<Mesh>> linkMeshes;
      
      // Collisions
      for (auto&& collision : link->GetCollisions())
      {
        GZ_ASSERT(collision != nullptr, "Collision must be valid");
        std::string collisionName(collision->GetName());
  
        // Shape
        physics::ShapePtr shape = collision->GetShape();
        GZ_ASSERT(shape != nullptr, "Shape must be valid");
        gzmsg << "Shape:      " << shape->TypeStr() << std::endl;
        gzmsg << "Scale:      " << shape->Scale() << std::endl;
        gzmsg << "Type:       " << std::hex << shape->GetType() << std::dec << std::endl;

        if (shape->HasType(physics::Base::EntityType::BOX_SHAPE))
        {
          // BoxShape
          gzmsg << "Type:       " << "BOX_SHAPE" << std::endl;
          physics::BoxShapePtr box = boost::dynamic_pointer_cast<physics::BoxShape>(shape);
          GZ_ASSERT(box != nullptr, "Failed to cast Shape to BoxShape");
          gzmsg << "Size:       " << box->Size() << std::endl;

          // Mesh
          std::string meshName = std::string(modelName)
            .append(".").append(linkName)
            .append(".").append(collisionName)
            .append(".box");
          common::MeshManager::Instance()->CreateBox(
            meshName,
            box->Size(),
            ignition::math::Vector2d(1, 1));          
          GZ_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
            "Failed to create Mesh for BoxShape");

          std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);
          
          // gzmsg << "Mesh:       " << mesh->GetName() << std::endl;
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }
      
        if (shape->HasType(physics::Base::EntityType::CYLINDER_SHAPE))
        {
          // CylinderShape
          gzmsg << "Type:       " << "CYLINDER_SHAPE" << std::endl;
          physics::CylinderShapePtr cylinder = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
          GZ_ASSERT(cylinder != nullptr, "Failed to cast Shape to CylinderShape");
          gzmsg << "Radius:     " << cylinder->GetRadius() << std::endl;
          gzmsg << "Length:     " << cylinder->GetLength() << std::endl;

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

          std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);

          gzmsg << "Mesh:       " << meshName << std::endl;
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }      

        if (shape->HasType(physics::Base::EntityType::SPHERE_SHAPE))
        {
          // SphereShape
          gzmsg << "Type:       " << "SPHERE_SHAPE" << std::endl;
          physics::SphereShapePtr sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
          GZ_ASSERT(sphere != nullptr, "Failed to cast Shape to SphereShape");
          gzmsg << "Radius:     " << sphere->GetRadius() << std::endl;

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

          std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);

          gzmsg << "Mesh:       " << meshName << std::endl;
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }

        if (shape->HasType(physics::Base::EntityType::MESH_SHAPE))
        {
          // MeshShape
          gzmsg << "Type:       " << "MESH_SHAPE" << std::endl;
          physics::MeshShapePtr meshShape = boost::dynamic_pointer_cast<physics::MeshShape>(shape);
          GZ_ASSERT(meshShape != nullptr, "Failed to cast Shape to MeshShape");

          std::string meshUri = meshShape->GetMeshURI();
          std::string meshStr = common::find_file(meshUri);
          gzmsg << "MeshURI:    " << meshUri << std::endl;
          gzmsg << "MeshStr:    " << meshStr << std::endl;

          // Mesh
          if (!common::MeshManager::Instance()->HasMesh(meshStr))
          {
            gzerr << "Mesh: " << meshStr << " was not loaded"<< std::endl;
            return;
          } 

          std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
          MeshTools::MakeSurfaceMesh(
            *common::MeshManager::Instance()->GetMesh(meshStr), *mesh);
          GZ_ASSERT(mesh != nullptr, "Invalid Suface Mesh");
          linkMeshes.push_back(mesh);

          gzmsg << "Mesh:       " << meshStr << std::endl;
          gzmsg << "Vertex:     " << mesh->number_of_vertices() << std::endl;
        }

      }

      // Add meshes for this link
      _meshes.push_back(linkMeshes);
    }
    
  } 

  /// \brief Transform a meshes vertex points in the world frame according to a Pose.
  ///
  /// \param[in] _pose    A pose defining a translation and rotation in the world frame.
  /// \param[in] _source  The original mesh to transform.
  /// \param[out] _target The transformed mesh.
  void ApplyPose(
    const ignition::math::Pose3d& _pose,
    const Mesh& _source,
    Mesh& _target)
  {
    for (
      auto&& it = std::make_pair(std::begin(_source.vertices()), std::begin(_target.vertices()));
      it.first != std::end(_source.vertices()) && it.second != std::end(_target.vertices());
      ++it.first, ++it.second)
    {
      auto& v0 = *it.first;
      auto& v1 = *it.second;
      const Point3& p0 = _source.point(v0);

      // Affine transformation
      ignition::math::Vector3d ignP0(p0.x(), p0.y(), p0.z());
      ignition::math::Vector3d ignP1 = _pose.Rot().RotateVector(ignP0) + _pose.Pos();

      Point3& p1 = _target.point(v1);
      p1 = Point3(ignP1.X(), ignP1.Y(), ignP1.Z());
    }
  }

  /// \brief Retrive a pointer to a wavefield from the Wavefield plugin.
  ///
  /// \param _world           A pointer to the world containing the wave field.
  /// \param _waveModelName   The name of the wavefield model containing the wave field. 
  /// \return A valid wavefield if found and nullptr if not.
  std::shared_ptr<const Wavefield> GetWavefield(
    physics::WorldPtr _world,
    const std::string& _waveModelName)
  {
    GZ_ASSERT(_world != nullptr, "World is null");

    physics::ModelPtr wavefieldModel = _world->ModelByName(_waveModelName);    
    if(wavefieldModel == nullptr)
    {
      gzerr << "No Wavefield Model found with name '" << _waveModelName << "'." << std::endl;
      return nullptr;
    }

    std::string wavefieldEntityName(WavefieldEntity::MakeName(_waveModelName));

    physics::BasePtr base = wavefieldModel->GetChild(wavefieldEntityName);
    boost::shared_ptr<WavefieldEntity> wavefieldEntity 
      = boost::dynamic_pointer_cast<WavefieldEntity>(base);
    if (wavefieldEntity == nullptr)
    {
      gzerr << "Wavefield Entity is null: " << wavefieldEntityName << std::endl;
      return nullptr;
    }    
    GZ_ASSERT(wavefieldEntity->GetWavefield() != nullptr, "Wavefield is null.");

    return wavefieldEntity->GetWavefield();
  }

///////////////////////////////////////////////////////////////////////////////
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

    /// \brief A Link pointer.
    public: physics::LinkPtr link;

    /// \brief The wavefield sampler for this link.
    public: std::shared_ptr<WavefieldSampler> wavefieldSampler;
    
    /// \brief The initial meshes for this link.
    public: std::vector<std::shared_ptr<Mesh>> initLinkMeshes;

    /// \brief The transformed meshes for this link.
    public: std::vector<std::shared_ptr<Mesh>> linkMeshes;

    /// \brief Objects to compute the hydrodynamics forces for each link mesh.
    public: std::vector<std::shared_ptr<Hydrodynamics>> hydrodynamics;

    /// \brief Marker messages for the water patch.
    public: ignition::msgs::Marker waterPatchMsg;

    /// \brief Marker messages for the waterline.
    public: std::vector<ignition::msgs::Marker> waterlineMsgs;

    /// \brief Marker messages for the underwater portion of the mesh.
    public: std::vector<ignition::msgs::Marker> underwaterSurfaceMsgs;
  };

///////////////////////////////////////////////////////////////////////////////
// HydrodynamicsPluginPrivate

  /// \internal
  /// \brief A class to manage the private data required by the HydrodynamicsPlugin.
  class HydrodynamicsPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief Model pointer.
    public: physics::ModelPtr model;

    /// \brief Pointer to the World wavefield.
    public: std::shared_ptr<const Wavefield> wavefield;

    /// \brief Hydrodynamics parameters for the entire model.
    public: std::shared_ptr<HydrodynamicsParameters> hydroParams;

    /// \brief Hydrodynamic physics for each Link.
    public: std::vector<std::shared_ptr<HydrodynamicsLinkData>> hydroData;

    /// \brief The wave model name. This is used to retrieve a pointer to the wave field.
    public: std::string waveModelName;

    /// \brief Show the water patch markers.
    public: bool showWaterPatch;

    /// \brief Show the waterline markers.
    public: bool showWaterline;

    /// \brief Show the underwater surface.
    public: bool showUnderwaterSurface;

    /// \brief The update rate for visual markers.
    public: double updateRate;

    /// \brief Previous update time.
    public: common::Time prevTime;

    /// \brief Connection to the World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Ignition transport node for igntopic "/marker".
    public: ignition::transport::Node ignNode;

    /// \brief Gazebo transport node.
    public: transport::NodePtr gzNode;

    /// \brief Subscribe to gztopic "~/hydrodynamics".
    public: transport::SubscriberPtr hydroSub;
  };

///////////////////////////////////////////////////////////////////////////////
// HydrodynamicsPlugin

  HydrodynamicsPlugin::~HydrodynamicsPlugin()
  {
    // Clean up.
    this->Fini();
    for (auto&& ptr : this->data->hydroData)
      ptr.reset();
    this->data->hydroParams.reset();
    this->data->wavefield.reset();

    // Reset connections and transport.
    this->data->updateConnection.reset();
    this->data->hydroSub.reset();
    this->data->gzNode->Fini();
  }

  HydrodynamicsPlugin::HydrodynamicsPlugin() : 
    ModelPlugin(),
    data(new HydrodynamicsPluginPrivate())
  {
  }

  void HydrodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // @DEBUG_INFO
    // gzmsg << "Load HydrodynamicsPlugin" << std::endl;

    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf   != nullptr, "Invalid parameter _sdf");

    // Capture the Model & World pointers
    this->data->model = _model;
    this->data->world = _model->GetWorld();
    GZ_ASSERT(this->data->world != nullptr, "Model has invalid World");

    // Transport
    this->data->gzNode = transport::NodePtr(new transport::Node());
    this->data->gzNode->Init(this->data->world->Name() + "/" + this->data->model->GetName());

    // Subscribers
    this->data->hydroSub = this->data->gzNode->Subscribe(
      "~/hydrodynamics", &HydrodynamicsPlugin::OnHydrodynamicsMsg, this);

    // Bind the update callback to the world update event 
    this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HydrodynamicsPlugin::OnUpdate, this));

    // Wave Model
    this->data->waveModelName = Utilities::SdfParamString(*_sdf, "wave_model", "");

    // Hydrodynamics parameters
    this->data->hydroParams.reset(new HydrodynamicsParameters());
    this->data->hydroParams->SetFromSDF(*_sdf);
    
    // Markers
    if (_sdf->HasElement("markers"))
    {
      sdf::ElementPtr sdfMarkers = _sdf->GetElement("markers");
      this->data->updateRate            = Utilities::SdfParamDouble(*sdfMarkers, "update_rate",         30.0);
      this->data->showWaterPatch        = Utilities::SdfParamBool(*sdfMarkers,   "water_patch",         false);
      this->data->showWaterline         = Utilities::SdfParamBool(*sdfMarkers,   "waterline",           false);
      this->data->showUnderwaterSurface = Utilities::SdfParamBool(*sdfMarkers,   "underwater_surface",  false);
    }
  }

  void HydrodynamicsPlugin::OnUpdate()
  {
    this->UpdatePhysics();
    this->UpdateMarkers();
  }
  
  void HydrodynamicsPlugin::UpdatePhysics()
  {
    // Update wavefield 
    this->data->wavefield = GetWavefield(
      this->data->world, this->data->waveModelName);
    if (this->data->wavefield == nullptr) 
    {
      gzerr << "Wavefield is NULL" << std::endl;
      return;
    }

    for (auto&& hd : this->data->hydroData)
    {
      GZ_ASSERT(hd->link != nullptr, "Link is NULL");

      // The link pose is required for the water patch, the CoM pose for dynamics.
      ignition::math::Pose3d linkPose = hd->link->WorldPose();
      ignition::math::Pose3d linkCoMPose = hd->link->WorldCoGPose();

      // Update water patch
      hd->wavefieldSampler->ApplyPose(linkPose);
      hd->wavefieldSampler->UpdatePatch();
      // auto waterPatch = hd->wavefieldSampler->GetWaterPatch();

      // RigidBody - the pose of the CoM is required for the dynamics. 
      Vector3 linVelocity = ToVector3(hd->link->WorldLinearVel());
      Vector3 angVelocity = ToVector3(hd->link->WorldAngularVel());
      Vector3 linVelocityCoM = ToVector3(hd->link->WorldCoGLinearVel());

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
        // gzmsg << "Link:         " << hd->link->GetName() << std::endl;
        // gzmsg << "Position:     " << linkPose.Pos() << std::endl;
        // gzmsg << "Rotation:     " << linkPose.Rot().Euler() << std::endl;
        // gzmsg << "SubTriCount:  " << nSubTri << std::endl;
        // gzmsg << "Force:        " << force << std::endl;
        // gzmsg << "Torque:       " << torque << std::endl;
      }
    }
  }
  
  void HydrodynamicsPlugin::UpdateMarkers()
  {
    std::string topicName("/marker");

    // Throttle update [30 FPS by default]
    auto updatePeriod = 1.0/this->data->updateRate;
    auto currentTime = this->data->world->SimTime();
    if ((currentTime - this->data->prevTime).Double() < updatePeriod)
    {
      return;
    }
    this->data->prevTime = currentTime; 

    if (this->data->showWaterPatch)      
      this->UpdateWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->UpdateWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->UpdateUnderwaterSurfaceMarkers();
  }

  void HydrodynamicsPlugin::UpdateWaterPatchMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      auto& grid = *hd->wavefieldSampler->GetWaterPatch();

      hd->waterPatchMsg.mutable_point()->Clear();
      for (size_t ix=0; ix<grid.GetCellCount()[0]; ++ix)
      {
        for (size_t iy=0; iy<grid.GetCellCount()[1]; ++iy)
        {
          for (size_t k=0; k<2; ++k)
          {
            Triangle tri = grid.GetTriangle(ix, iy, k);
            ignition::msgs::Set(hd->waterPatchMsg.add_point(), ToIgn(tri[0]));
            ignition::msgs::Set(hd->waterPatchMsg.add_point(), ToIgn(tri[1]));
            ignition::msgs::Set(hd->waterPatchMsg.add_point(), ToIgn(tri[2]));
          }
        }
      }
      this->data->ignNode.Request(topicName, hd->waterPatchMsg);
    }
  }

  void HydrodynamicsPlugin::UpdateWaterlineMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].mutable_point()->Clear();
        if (hd->hydrodynamics[j]->GetWaterline().empty())
        {
          // @TODO workaround. The previous marker is not cleared if a cleared point list is published.
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        }
        for (auto&& line : hd->hydrodynamics[j]->GetWaterline())
        {
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ToIgn(line.point(0)));
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ToIgn(line.point(1)));
        }
        this->data->ignNode.Request(topicName, hd->waterlineMsgs[j]);
      }
    }
  }

  void HydrodynamicsPlugin::UpdateUnderwaterSurfaceMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
        if (hd->hydrodynamics[j]->GetSubmergedTriangles().empty())
        {
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        }
        for (auto&& tri : hd->hydrodynamics[j]->GetSubmergedTriangles())
        {
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ToIgn(tri[0]));
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ToIgn(tri[1]));
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ToIgn(tri[2]));
        }
        this->data->ignNode.Request(topicName, hd->underwaterSurfaceMsgs[j]);
      }
    }
  }

  void HydrodynamicsPlugin::Init()
  {
    // @DEBUG_INFO
    // gzmsg << "Init HydrodynamicsPlugin" << std::endl;
    this->HydrodynamicsPlugin::InitPhysics();
    this->HydrodynamicsPlugin::InitMarkers();
  }

  void HydrodynamicsPlugin::InitPhysics()
  {
    // Wavefield
    this->data->wavefield = GetWavefield(
      this->data->world, this->data->waveModelName);
    if (this->data->wavefield == nullptr) 
    {
      gzerr << "Wavefield is NULL" << std::endl;
      return;
    }

    std::string modelName(this->data->model->GetName());

    // Populate link meshes
    std::vector<physics::LinkPtr> links;
    std::vector<std::vector<std::shared_ptr<Mesh>>> meshes;
    CreateCollisionMeshes(this->data->model, links, meshes);
    gzmsg << "links:  " << links.size() << std::endl;
    gzmsg << "meshes: " << meshes.size() << std::endl;

    for (size_t i=0; i<links.size(); ++i)
    {
      // Create storage
      size_t meshCount = meshes[i].size();
      std::shared_ptr<HydrodynamicsLinkData> hd(new HydrodynamicsLinkData);
      this->data->hydroData.push_back(hd);
      hd->initLinkMeshes.resize(meshCount);
      hd->linkMeshes.resize(meshCount);
      hd->hydrodynamics.resize(meshCount);
      hd->waterlineMsgs.resize(meshCount);
      hd->underwaterSurfaceMsgs.resize(meshCount);

      // Wavefield and Link
      hd->link = links[i];

      // The link pose is required for the water patch, the CoM pose for dynamics.
      ignition::math::Pose3d linkPose = hd->link->WorldPose();
      ignition::math::Pose3d linkCoMPose = hd->link->WorldCoGPose();

      // Water patch grid
      auto boundingBox = hd->link->CollisionBoundingBox();
      double patchSize = 2.2 * boundingBox.Size().Length();
      gzmsg << "Water patch size: " << patchSize << std::endl;
      std::shared_ptr<Grid> initWaterPatch(new Grid({patchSize, patchSize}, { 4, 4 }));

      // WavefieldSampler - this is updated by the pose of the link (not the CoM).
      hd->wavefieldSampler.reset(new WavefieldSampler(
        this->data->wavefield, initWaterPatch));
      hd->wavefieldSampler->ApplyPose(linkPose);
      hd->wavefieldSampler->UpdatePatch();

      // RigidBody - the pose of the CoM is required for the dynamics. 
      Vector3 linVelocity = ToVector3(hd->link->WorldLinearVel());
      Vector3 angVelocity = ToVector3(hd->link->WorldAngularVel());
      Vector3 linVelocityCoM = ToVector3(hd->link->WorldCoGLinearVel());

      for (size_t j=0; j<meshCount; ++j)
      {
        // Mesh (SurfaceMesh copy performs a deep copy of all properties)
        std::shared_ptr<Mesh> initLinkMesh = meshes[i][j];
        std::shared_ptr<Mesh> linkMesh = std::make_shared<Mesh>(*initLinkMesh);
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
    }
  }

  void HydrodynamicsPlugin::InitMarkers()
  {
    if (this->data->showWaterPatch)      
      this->InitWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->InitWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->InitUnderwaterSurfaceMarkers();    
  }

  void HydrodynamicsPlugin::InitWaterPatchMarkers()
  {
    std::string modelName(this->data->model->GetName());
    int markerId = 0;
    for (auto&& hd : this->data->hydroData)
    {
      hd->waterPatchMsg.set_ns(modelName + "::water_patch");
      hd->waterPatchMsg.set_id(markerId++);
      hd->waterPatchMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      hd->waterPatchMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
      std::string waveMat("Gazebo/BlueTransparent");
      ignition::msgs::Material *waveMatMsg = hd->waterPatchMsg.mutable_material();
      GZ_ASSERT(waveMatMsg != nullptr, "Invalid Material pointer from waterPatchMsg");
      waveMatMsg->mutable_script()->set_name(waveMat);
    }
  }

  void HydrodynamicsPlugin::InitWaterlineMarkers()
  {
    std::string modelName(this->data->model->GetName());
    int markerId = 0;
    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].set_ns(modelName + "::waterline");
        hd->waterlineMsgs[j].set_id(markerId++);
        hd->waterlineMsgs[j].set_action(ignition::msgs::Marker::ADD_MODIFY);
        hd->waterlineMsgs[j].set_type(ignition::msgs::Marker::LINE_LIST);
        std::string lineMat("Gazebo/Black");
        ignition::msgs::Material *lineMatMsg = hd->waterlineMsgs[j].mutable_material();
        GZ_ASSERT(lineMatMsg != nullptr, "Invalid Material pointer from waterlineMsgs");
        lineMatMsg->mutable_script()->set_name(lineMat);
      }
    }
  }

  void HydrodynamicsPlugin::InitUnderwaterSurfaceMarkers()
  {
    std::string modelName(this->data->model->GetName());
    int markerId = 0;
    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].set_ns(modelName + "::submerged_triangles");
        hd->underwaterSurfaceMsgs[j].set_id(markerId++);
        hd->underwaterSurfaceMsgs[j].set_action(ignition::msgs::Marker::ADD_MODIFY);
        hd->underwaterSurfaceMsgs[j].set_type(ignition::msgs::Marker::TRIANGLE_LIST);
        std::string triMat("Gazebo/Blue");
        ignition::msgs::Material *triMatMsg = hd->underwaterSurfaceMsgs[j].mutable_material();
        GZ_ASSERT(triMatMsg != nullptr, "Invalid Material pointer from underwaterSurfaceMsgs");
        triMatMsg->mutable_script()->set_name(triMat);      
      }
    }
  }

  void HydrodynamicsPlugin::Reset()
  {
    // Reset time
    this->data->prevTime = this->data->world->SimTime(); 

    this->ResetPhysics();
    this->ResetMarkers();
  }

  void HydrodynamicsPlugin::ResetPhysics()
  {
    // no-op
  }

  void HydrodynamicsPlugin::ResetMarkers()
  {
    // Reset markers
    if (this->data->showWaterPatch)
      this->ResetWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->ResetWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->ResetUnderwaterSurfaceMarkers();
  }

  void HydrodynamicsPlugin::ResetWaterPatchMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      hd->waterPatchMsg.mutable_point()->Clear();
      ignition::msgs::Set(hd->waterPatchMsg.add_point(), ignition::math::Vector3d::Zero);
      ignition::msgs::Set(hd->waterPatchMsg.add_point(), ignition::math::Vector3d::Zero);
      ignition::msgs::Set(hd->waterPatchMsg.add_point(), ignition::math::Vector3d::Zero);
      this->data->ignNode.Request(topicName, hd->waterPatchMsg);
    } 
  }

  void HydrodynamicsPlugin::ResetWaterlineMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].mutable_point()->Clear();
        ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        this->data->ignNode.Request(topicName, hd->waterlineMsgs[j]);
      }
    } 
  }

  void HydrodynamicsPlugin::ResetUnderwaterSurfaceMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
        ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        this->data->ignNode.Request(topicName, hd->underwaterSurfaceMsgs[j]);
      }
    } 
  }

  void HydrodynamicsPlugin::Fini()
  {
    this->FiniPhysics();
    this->FiniMarkers();
  }

  void HydrodynamicsPlugin::FiniPhysics()
  {
    // no-op
  }

  void HydrodynamicsPlugin::FiniMarkers()
  {
    if (this->data->showWaterPatch)      
      this->FiniWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->FiniWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->FiniUnderwaterSurfaceMarkers();
  }

  void HydrodynamicsPlugin::FiniWaterPatchMarkers()
  {    
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      hd->waterPatchMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
      this->data->ignNode.Request(topicName, hd->waterPatchMsg);
    }    
  }

  void HydrodynamicsPlugin::FiniWaterlineMarkers()
  {    
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].set_action(ignition::msgs::Marker::DELETE_MARKER);
        this->data->ignNode.Request(topicName, hd->waterlineMsgs[j]);
      }
    }    
  }

  void HydrodynamicsPlugin::FiniUnderwaterSurfaceMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].set_action(ignition::msgs::Marker::DELETE_MARKER);
        this->data->ignNode.Request(topicName, hd->underwaterSurfaceMsgs[j]);
      }
    }    
  }

  void HydrodynamicsPlugin::OnHydrodynamicsMsg(ConstParam_VPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Hydrodynamics message must not be null");

    // Update hydrodynamics params
    auto& hydroParams = *this->data->hydroParams;
    hydroParams.SetFromMsg(*_msg);

    // @DEBUG_INFO
    gzmsg << "Hydrodynamics Model received message on topic [" 
      << this->data->hydroSub->GetTopic() << "]" << std::endl;
    hydroParams.DebugPrint();
  }

} // namespace gazebo
