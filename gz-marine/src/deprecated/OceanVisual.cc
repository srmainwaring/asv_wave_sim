#include "gz/marine/OceanVisual.hh"
#include "gz/marine/OceanTile.hh"
#include "gz/marine/Utilities.hh"

#include "gz/marine/Gazebo.hh"

#include <gz/common.hh>
#include <gz/common/MeshManager.hh>
// #include <gz/rendering/ogre_gazebo.h>
#include <gz/rendering.hh>
// #include <gz/rendering/RenderTypes.hh>
// #include <gz/rendering/Visual.hh>

#include <cmath>
#include <memory>
#include <thread>
#include <vector>

namespace ignition
{
namespace marine
{

///////////////////////////////////////////////////////////////////////////////
// OceanVisualPrivate

  class OceanVisualPrivate
  {
    /// \brief The visuals to attach    
    public: std::vector<rendering::VisualPtr> aboveOceanVisuals;
    public: std::vector<rendering::VisualPtr> belowOceanVisuals;

    /// \brief The visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief Generated mesh name (duplicated in tile->mesh...)
    public: std::string aboveOceanMeshName;
    public: std::string belowOceanMeshName;

    /// \brief World stats.
    public: double simTime, realTime, pauseTime;
    public: bool paused;

    /// \brief Ocean tile synchronises the mesh with the GPU
    public: std::unique_ptr<OceanTile> oceanTile;

    /// \brief Prevent Loading visual twice...
    public: bool isInitialised = false;

    /// \brief Mutex
    public: std::recursive_mutex mutex;

    /// \brief Event based connections.
    public: event::ConnectionPtr connection;

    /// \brief Node used to establish communication with gzserver.
    public: transport::NodePtr node;

    /// \brief Subscribe to gztopic "~/wave/wind".
    public: transport::SubscriberPtr waveWindSub;

    /// \brief Subscribe to gztopic "~/world_stats".
    public: transport::SubscriberPtr statsSub;
  };

///////////////////////////////////////////////////////////////////////////////
// OceanVisual

  OceanVisual::~OceanVisual()
  {
    this->data->connection.reset();
    this->data->statsSub.reset();
    this->data->waveWindSub.reset();
    this->data->node.reset();
  }

  OceanVisual::OceanVisual(
    const std::string &_name,
    rendering::VisualPtr _parent) :
    rendering::Visual(_name, _parent),
    data(new OceanVisualPrivate)
  {
    ignmsg << "Constructing OceanVisual..." << std::endl;

    rendering::Visual::SetType(VT_VISUAL);
  }

  void OceanVisual::Load(sdf::ElementPtr _sdf)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    ignmsg << "Loading OceanVisual..." << std::endl;
 
    GZ_ASSERT(_sdf != nullptr, "SDF Element is NULL");

    // Capture the sdf
    this->data->sdf = _sdf;

    this->Load();

    ignmsg << "Done loading OceanVisual." << std::endl;
  }

  void OceanVisual::Load()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    if (!this->data->isInitialised)
    {
      rendering::Visual::Load();

      // Transport
      this->data->node = transport::NodePtr(new transport::Node());
      this->data->node->Init();

      // Publishers

      // Subscribers
      this->data->waveWindSub = this->data->node->Subscribe(
        "~/wave/wind", &OceanVisual::OnWaveWindMsg, this);

      this->data->statsSub = this->data->node->Subscribe(
        "~/world_stats", &OceanVisual::OnStatsMsg, this);

      // Bind the update method to ConnectPreRender events
      this->data->connection = event::Events::ConnectPreRender(
          std::bind(&OceanVisual::OnUpdate, this));

      // @TODO: pass name to OceanTile where it is created independently but must match.
      this->data->aboveOceanMeshName = "AboveOceanTileMesh";
      this->data->belowOceanMeshName = "BelowOceanTileMesh";

      // @TODO Synchronise visual with physics...
      const int N = 128;
      const double L = 256.0;
      const double u = 5.0;

      this->data->oceanTile.reset(new OceanTile(N, L));
      this->data->oceanTile->SetWindVelocity(u, 0.0);
      this->data->oceanTile->Create();
      this->data->oceanTile->Update(0.0);

      // Water tiles -nX, -nX + 1, ...,0, 1, ..., nX, etc.
      const int nX = 3;
      const int nY = 3;
      for (int iy=-nY; iy<=nY; ++iy)
      {
        for (int ix=-nX; ix<=nX; ++ix)
        {
          ignition::math::Vector3d position(
            this->Position().X() + ix * L,
            this->Position().Y() + iy * L,
            this->Position().Z()
          );

          // Mesh Visual: Above
          {
            std::string visualName  = this->Name()
              + "_ABOVE_OCEAN_" + std::to_string(ix) + "_" + std::to_string(iy);
            rendering::VisualPtr visual(new rendering::Visual(visualName, shared_from_this()));
            visual->Load();
            gazebo::rendering::AttachMesh(*visual, this->data->aboveOceanMeshName);
            visual->SetPosition(position);
            visual->SetType(rendering::Visual::VT_VISUAL);

            // Set the material from the parent visual
            auto materialName = this->GetMaterialName();
            // std::string materialName("Gazebo/Green");
            visual->SetMaterial(materialName);

            this->data->aboveOceanVisuals.push_back(visual);
          }

          // Mesh Visual: Below
          {
            std::string visualName  = this->Name()
              + "_BELOW_OCEAN" + std::to_string(ix) + "_" + std::to_string(iy);
            rendering::VisualPtr visual(new rendering::Visual(visualName, shared_from_this()));
            visual->Load();
            gazebo::rendering::AttachMesh(*visual, this->data->belowOceanMeshName);
            visual->SetPosition(position);
            visual->SetType(rendering::Visual::VT_VISUAL);

            // Set the material from the parent visual
            auto materialName = this->GetMaterialName();
            // std::string materialName("Gazebo/Orange");
            visual->SetMaterial(materialName);

            this->data->belowOceanVisuals.push_back(visual);
          }
        }
      }

#if DEBUG
      for (auto visual : this->data->aboveOceanVisuals)
      {
          gzmsg << "AboveOceanVisual..." << std::endl;
          gzmsg << "Name: "                 << visual->Name() << std::endl;
          gzmsg << "Id: "                   << visual->GetId() << std::endl;
          gzmsg << "MaterialName: "         << visual->GetMaterialName() << std::endl;
          gzmsg << "MeshName: "             << visual->GetMeshName() << std::endl;
          gzmsg << "ShaderType: "           << visual->GetShaderType() << std::endl;
          gzmsg << "AttachedObjectCount: "  << visual->GetAttachedObjectCount() << std::endl;
      }
      
      for (auto visual : this->data->belowOceanVisuals)
      {
          gzmsg << "BelowOceanVisual..." << std::endl;
          gzmsg << "Name: "                 << visual->Name() << std::endl;
          gzmsg << "Id: "                   << visual->GetId() << std::endl;
          gzmsg << "MaterialName: "         << visual->GetMaterialName() << std::endl;
          gzmsg << "MeshName: "             << visual->GetMeshName() << std::endl;
          gzmsg << "ShaderType: "           << visual->GetShaderType() << std::endl;
          gzmsg << "AttachedObjectCount: "  << visual->GetAttachedObjectCount() << std::endl;
      }
 #endif
      // // Mesh Visual: Above
      // {
      //   this->data->aboveOceanVisual.reset(new rendering::Visual(aboveOceanVisualName, shared_from_this()));
      //   this->data->aboveOceanVisual->Load();
      //   ignition::rendering::AttachMesh(*this->data->aboveOceanVisual, this->data->aboveOceanMeshName);
      //   this->data->aboveOceanVisual->SetPosition(this->Position());
      //   this->data->aboveOceanVisual->SetType(rendering::Visual::VT_VISUAL);
      // }
      // // Mesh Visual: Below
      // {
      //   this->data->belowOceanVisual.reset(new rendering::Visual(belowOceanVisualName, shared_from_this()));
      //   this->data->belowOceanVisual->Load();
      //   ignition::rendering::AttachMesh(*this->data->belowOceanVisual, this->data->belowOceanMeshName);
      //   this->data->belowOceanVisual->SetPosition(this->Position());
      //   this->data->belowOceanVisual->SetType(rendering::Visual::VT_VISUAL);

      //   // Set the material from the parent visual
      //   auto materialName = this->GetMaterialName();
      //   // std::string materialName("Gazebo/Orange");
      //   this->data->belowOceanVisual->SetMaterial(materialName);
      // }

      this->SetVisibilityFlags(GZ_VISIBILITY_ALL);    
      this->data->isInitialised = true;
    }

  }

  /// Update the vertex buffer directly:
  /// http://wiki.ogre3d.org/RetrieveVertexData
  /// https://forums.ogre3d.org/viewtopic.php?t=68347
  /// https://forums.ogre3d.org/viewtopic.php?t=53882
  ///
  /// Notes: 
  /// 1. We must use a custom InsertMesh method as the vertex buffer needs to be created with
  ///   HardwareBuffer::Usage = Ogre::HardwareBuffer::HBU_DYNAMIC
  ///   useShadowBuffer = true
  ///
  /// vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
  ///           vertexDecl->getVertexSize(0),
  ///           vertexData->vertexCount,
  ///           Ogre::HardwareBuffer::HBU_DYNAMIC,
  ///           true);
  ///
  void OceanVisual::OnUpdate()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // ignmsg << "Updating OceanVisual..." << std::endl;

    if (this->data->paused)
      return;

    double time = this->data->simTime;
    // double time = std::fmod(this->data->simTime, _cycleTime);
    // ignmsg << "Time: " << time << std::endl;

    this->data->oceanTile->Update(time);

    // ignmsg << "Done updating OceanVisual." << std::endl;
  }

  void OceanVisual::OnWaveWindMsg(ConstParam_VPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    GZ_ASSERT(_msg != nullptr, "Message must not be null");

    // Get parameters from message
    double wind_angle = 0.0;
    double wind_speed = 0.0;
    wind_angle = Utilities::MsgParamDouble(*_msg, "wind_angle", wind_angle);
    wind_speed = Utilities::MsgParamDouble(*_msg, "wind_speed", wind_speed);

    // Convert from polar to cartesian
    double wind_vel_x = wind_speed * std::cos(wind_angle);
    double wind_vel_y = wind_speed * std::sin(wind_angle);

    // @DEBUG_INFO
    gzmsg << "OceanVisual received message on topic ["
      << this->data->waveWindSub->GetTopic() << "]" << std::endl;
    gzmsg << "wind_angle: " << wind_angle << std::endl;
    gzmsg << "wind_speed: " << wind_speed << std::endl;
    gzmsg << "wind_vel_x: " << wind_vel_x << std::endl;
    gzmsg << "wind_vel_y: " << wind_vel_y << std::endl;

    // Update simulation
    this->data->oceanTile->SetWindVelocity(wind_vel_x, wind_vel_y);
  }

  void OceanVisual::OnStatsMsg(ConstWorldStatisticsPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    this->data->simTime = ignition::msgs::Convert(_msg->sim_time()).Double();
    this->data->realTime = ignition::msgs::Convert(_msg->real_time()).Double();
    this->data->pauseTime = ignition::msgs::Convert(_msg->pause_time()).Double();
    this->data->paused = _msg->paused();
  }

}
}
