#include "asv_wave_sim_gazebo_plugins/OceanVisual.hh"
#include "asv_wave_sim_gazebo_plugins/OceanTile.hh"

#include "asv_wave_sim_gazebo_plugins/Gazebo.hh"

#include <gazebo/common/common.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Visual.hh>

#include <cmath>
#include <memory>
#include <thread>
#include <vector>

using namespace gazebo;

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// OceanVisualPrivate

  class OceanVisualPrivate
  {
    /// \brief The visuals to attach
    public: rendering::VisualPtr aboveOceanVisual;
    public: rendering::VisualPtr belowOceanVisual;

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

    /// \brief Subscribe to gztopic "~/world_stats".
    public: transport::SubscriberPtr statsSub;
  };

///////////////////////////////////////////////////////////////////////////////
// OceanVisual

  OceanVisual::~OceanVisual()
  {
    this->data->connection.reset();
    this->data->statsSub.reset();
    this->data->node.reset();
  }

  OceanVisual::OceanVisual(
    const std::string &_name,
    rendering::VisualPtr _parent) :
    rendering::Visual(_name, _parent),
    data(new OceanVisualPrivate)
  {
    gzmsg << "Constructing OceanVisual..." << std::endl;

    rendering::Visual::SetType(VT_VISUAL);
  }

  void OceanVisual::Load(sdf::ElementPtr _sdf)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    gzmsg << "Loading OceanVisual..." << std::endl;
 
    GZ_ASSERT(_sdf != nullptr, "SDF Element is NULL");

    // Capture the sdf
    this->data->sdf = _sdf;

    this->Load();

    gzmsg << "Done loading OceanVisual." << std::endl;
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
      this->data->statsSub = this->data->node->Subscribe(
        "~/world_stats", &OceanVisual::OnStatsMsg, this);

      // Bind the update method to ConnectPreRender events
      this->data->connection = event::Events::ConnectPreRender(
          std::bind(&OceanVisual::OnUpdate, this));

      // Shader visuals
      std::string aboveOceanVisualName  = this->Name() + "_ABOVE_OCEAN";
      std::string belowOceanVisualName  = this->Name() + "_BELOW_OCEAN";

      // @TODO: pass name to OceanTile where it is created independently but must match.
      this->data->aboveOceanMeshName = "AboveOceanTileMesh";
      this->data->belowOceanMeshName = "BelowOceanTileMesh";

      // @TODO Synchronise visual with physics...
      int N = 128;
      double L = 256.0;
      double u = 5.0;

      this->data->oceanTile.reset(new OceanTile(N, L));
      this->data->oceanTile->SetWindVelocity(u, 0.0);
      this->data->oceanTile->Create();
      this->data->oceanTile->Update(0.0);

      // Mesh Visual: Above
      {
        this->data->aboveOceanVisual.reset(new rendering::Visual(aboveOceanVisualName, shared_from_this()));
        this->data->aboveOceanVisual->Load();
        gazebo::rendering::AttachMesh(*this->data->aboveOceanVisual, this->data->aboveOceanMeshName);
        this->data->aboveOceanVisual->SetPosition(this->Position());
        this->data->aboveOceanVisual->SetType(rendering::Visual::VT_VISUAL);

        // Set the material from the parent visual
        auto materialName = this->GetMaterialName();
        // std::string materialName("Gazebo/Green");
        this->data->aboveOceanVisual->SetMaterial(materialName);
      }

      // Mesh Visual: Below
      {
        this->data->belowOceanVisual.reset(new rendering::Visual(belowOceanVisualName, shared_from_this()));
        this->data->belowOceanVisual->Load();
        gazebo::rendering::AttachMesh(*this->data->belowOceanVisual, this->data->belowOceanMeshName);
        this->data->belowOceanVisual->SetPosition(this->Position());
        this->data->belowOceanVisual->SetType(rendering::Visual::VT_VISUAL);

        // Set the material from the parent visual
        auto materialName = this->GetMaterialName();
        // std::string materialName("Gazebo/Orange");
        this->data->belowOceanVisual->SetMaterial(materialName);
      }

#if DEBUG
      gzmsg << "AboveOceanVisual..." << std::endl;
      gzmsg << "Name: "                 << this->data->aboveOceanVisual->Name() << std::endl;
      gzmsg << "Id: "                   << this->data->aboveOceanVisual->GetId() << std::endl;
      gzmsg << "MaterialName: "         << this->data->aboveOceanVisual->GetMaterialName() << std::endl;
      gzmsg << "MeshName: "             << this->data->aboveOceanVisual->GetMeshName() << std::endl;
      gzmsg << "ShaderType: "           << this->data->aboveOceanVisual->GetShaderType() << std::endl;
      gzmsg << "AttachedObjectCount: "  << this->data->aboveOceanVisual->GetAttachedObjectCount() << std::endl;

      gzmsg << "BelowOceanVisual..." << std::endl;
      gzmsg << "Name: "                 << this->data->belowOceanVisual->Name() << std::endl;
      gzmsg << "Id: "                   << this->data->belowOceanVisual->GetId() << std::endl;
      gzmsg << "MaterialName: "         << this->data->belowOceanVisual->GetMaterialName() << std::endl;
      gzmsg << "MeshName: "             << this->data->belowOceanVisual->GetMeshName() << std::endl;
      gzmsg << "ShaderType: "           << this->data->belowOceanVisual->GetShaderType() << std::endl;
      gzmsg << "AttachedObjectCount: "  << this->data->belowOceanVisual->GetAttachedObjectCount() << std::endl;
#endif

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

    // gzmsg << "Updating OceanVisual..." << std::endl;

    if (this->data->paused)
      return;

    double time = this->data->simTime;
    // double time = std::fmod(this->data->simTime, _cycleTime);
    // gzmsg << "Time: " << time << std::endl;

    this->data->oceanTile->Update(time);

    // gzmsg << "Done updating OceanVisual." << std::endl;
  }

  void OceanVisual::OnStatsMsg(ConstWorldStatisticsPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    this->data->simTime = gazebo::msgs::Convert(_msg->sim_time()).Double();
    this->data->realTime = gazebo::msgs::Convert(_msg->real_time()).Double();
    this->data->pauseTime = gazebo::msgs::Convert(_msg->pause_time()).Double();
    this->data->paused = _msg->paused();
  }

} // namespace asv
