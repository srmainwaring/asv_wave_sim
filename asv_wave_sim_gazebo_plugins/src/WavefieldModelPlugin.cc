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

#include "asv_wave_sim_gazebo_plugins/WavefieldModelPlugin.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/msgs/any.pb.h>
#include <gazebo/msgs/empty.pb.h>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/msgs/param.pb.h>
#include <gazebo/msgs/param_v.pb.h>

#include <ignition/math/Vector3.hh>

#include <algorithm>
#include <iostream>
#include <string>
#include <thread>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_MODEL_PLUGIN(WavefieldModelPlugin)

///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPluginPrivate

  /// \internal
  /// \brief Private data for the WavefieldModelPlugin
  class WavefieldModelPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief Model pointer.
    public: physics::ModelPtr model;

    /// \brief WavefieldEntity pointer.
    public: boost::shared_ptr<::asv::WavefieldEntity> wavefieldEntity;

    /// \brief Wavefield Marker
    public: ignition::msgs::Marker wavefieldMsg;

    /// \brief Set the wavefield to be static [false].
    public: bool isStatic;

    /// \brief Update rate [30].
    public: double updateRate;

    /// \brief Set to true to show the marker [false].
    public: bool showWavePatch;

    /// \brief Number of grid points to show along each axis [4 4].
    public: Vector2 wavePatchSize;

    /// \brief Previous update time.
    public: common::Time prevTime;

    /// \brief Connection to the World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Ignition transport node for igntopic "/marker".
    public: ignition::transport::Node ignNode;

    /// \brief Gazebo transport node.
    public: transport::NodePtr gzNode;

    /// \brief Publish to gztopic "~/reponse".
    public: transport::PublisherPtr responsePub;

    /// \brief Subscribe to gztopic "~/request".
    public: transport::SubscriberPtr requestSub;

    /// \brief Subscribe to gztopic "~/wave".
    public: transport::SubscriberPtr waveSub;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPlugin

  WavefieldModelPlugin::~WavefieldModelPlugin()
  {
    // Remove node from the topic manager.
    this->data->requestSub.reset();
    this->data->waveSub.reset();
    this->data->responsePub.reset();
    this->data->gzNode->Fini();
    this->Fini();
  }

  WavefieldModelPlugin::WavefieldModelPlugin() : 
    ModelPlugin(), 
    data(new WavefieldModelPluginPrivate())
  {
  }

  void WavefieldModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Load WavefieldModelPlugin [thread: " << threadId << "]" << std::endl;

    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Capture the Model pointer
    this->data->model = _model;
    this->data->world = _model->GetWorld();
    GZ_ASSERT(this->data->world != nullptr, "Model has invalid World");

    // Transport
    this->data->gzNode = transport::NodePtr(new transport::Node());
    this->data->gzNode->Init(this->data->world->Name());

    // Publishers
    this->data->responsePub 
      = this->data->gzNode->Advertise<msgs::Response>("~/response");

    // Subscribers
    this->data->requestSub = this->data->gzNode->Subscribe(
      "~/request", &WavefieldModelPlugin::OnRequest, this);

    this->data->waveSub = this->data->gzNode->Subscribe(
      "~/wave", &WavefieldModelPlugin::OnWaveMsg, this);

    // Bind the update callback to the world update event 
    this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WavefieldModelPlugin::OnUpdate, this));

    // Parameters
    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);
    this->data->updateRate = Utilities::SdfParamDouble(*_sdf, "update_rate", 30.0);
    if (_sdf->HasElement("markers"))
    {
      sdf::ElementPtr sdfMarkers = _sdf->GetElement("markers");
      this->data->showWavePatch = Utilities::SdfParamBool(*sdfMarkers, "wave_patch", false);
      this->data->wavePatchSize = Utilities::SdfParamVector2(*sdfMarkers, "wave_patch_size", Vector2(4, 4));
    }

    // Wavefield
    this->data->wavefieldEntity.reset(new ::asv::WavefieldEntity(this->data->model));
    this->data->wavefieldEntity->Load(_sdf);
    this->data->wavefieldEntity->Init();

    // Generate the entity name and add as a child
    this->data->wavefieldEntity->SetName(
      WavefieldEntity::MakeName(this->data->model->GetName()));
    this->data->model->AddChild(this->data->wavefieldEntity);
  }

  void WavefieldModelPlugin::Init()
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Init WavefieldModelPlugin [thread: " << threadId << "]" << std::endl;

    if (this->data->showWavePatch)
      this->InitMarker();
  }

  void WavefieldModelPlugin::Fini()
  {
    if (this->data->showWavePatch)
      this->FiniMarker();
  }

  void WavefieldModelPlugin::Reset()
  {
    // Reset time
    this->data->prevTime = this->data->world->SimTime(); 

    // Reset markers
    if (this->data->showWavePatch)
      this->ResetMarker();
  }

  void WavefieldModelPlugin::OnUpdate()
  {
    GZ_ASSERT(this->data->world != nullptr, "World is NULL");
    GZ_ASSERT(this->data->model != nullptr, "Model is NULL");
    GZ_ASSERT(this->data->wavefieldEntity != nullptr, "Wavefield Entity is NULL");

    if (!this->data->isStatic)
    {
      // Throttle update [30 FPS by default]
      auto updatePeriod = 1.0/this->data->updateRate;
      auto currentTime = this->data->world->SimTime();
      if ((currentTime - this->data->prevTime).Double() < updatePeriod)
      {
        return;
      }
      this->data->prevTime = currentTime; 

      // Wavefield
      this->data->wavefieldEntity->Update();
      if (this->data->showWavePatch)
        this->UpdateMarker();
    }
  }

  // See for example: gazebo/physics/Wind.cc
  void WavefieldModelPlugin::OnRequest(ConstRequestPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Request message must not be null");
    
    if (_msg->request() == "wave_param")
    {
      auto waveParams = this->data->wavefieldEntity->GetWavefield()->GetParameters();

      msgs::Param_V waveMsg;
      waveParams->FillMsg(waveMsg);

      msgs::Response response;
      response.set_id(_msg->id());
      response.set_request(_msg->request());
      response.set_response("success");
      std::string *serializedData = response.mutable_serialized_data();
      response.set_type(waveMsg.GetTypeName());
      waveMsg.SerializeToString(serializedData);
      this->data->responsePub->Publish(response);
    }
  }

  // @TODO_FRAGILE - the Entity needs proper clone and set methods to be safe
  void WavefieldModelPlugin::OnWaveMsg(ConstParam_VPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Wave message must not be null");

    // Update wave params 
    auto constWaveParams = this->data->wavefieldEntity->GetWavefield()->GetParameters();
    GZ_ASSERT(constWaveParams != nullptr, "WaveParameters must not be null");
    auto& waveParams = const_cast<WaveParameters&>(*constWaveParams);
    waveParams.SetFromMsg(*_msg);

    // @DEBUG_INFO
    gzmsg << "Wavefield Model received message on topic ["
      << this->data->waveSub->GetTopic() << "]" << std::endl;
    waveParams.DebugPrint();
  }

  void WavefieldModelPlugin::InitMarker()
  {
    auto& wavefieldMsg = this->data->wavefieldMsg;

    int markerId = 0;
    wavefieldMsg.set_ns("wave");
    wavefieldMsg.set_id(markerId++);
    wavefieldMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    wavefieldMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
    std::string waveMat("Gazebo/RedTransparent");
    ignition::msgs::Material *waveMatMsg = wavefieldMsg.mutable_material();
    GZ_ASSERT(waveMatMsg != nullptr, "Invalid Material pointer from wavefieldMsg");
    waveMatMsg->mutable_script()->set_name(waveMat);
  }

  void WavefieldModelPlugin::FiniMarker()
  {
    std::string topicName("/marker");

    auto& ignNode = this->data->ignNode;
    auto& wavefieldMsg = this->data->wavefieldMsg;

    wavefieldMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
    ignNode.Request(topicName, wavefieldMsg);
  }

  void WavefieldModelPlugin::ResetMarker()
  {
    std::string topicName("/marker");

    auto& ignNode = this->data->ignNode;
    auto& wavefieldMsg = this->data->wavefieldMsg;
 
    wavefieldMsg.mutable_point()->Clear();
    ignition::msgs::Set(wavefieldMsg.add_point(), ignition::math::Vector3d::Zero);
    ignition::msgs::Set(wavefieldMsg.add_point(), ignition::math::Vector3d::Zero);
    ignition::msgs::Set(wavefieldMsg.add_point(), ignition::math::Vector3d::Zero);
    ignNode.Request(topicName, wavefieldMsg);
  }

  void WavefieldModelPlugin::UpdateMarker()
  {
    std::string topicName("/marker");

    auto grid = this->data->wavefieldEntity->GetWavefield()->GetGrid();
    auto& wavefieldMsg = this->data->wavefieldMsg;
    auto& ignNode = this->data->ignNode;
 
    // Determine marker extents and set bound using grid dimensions.
    int xext = static_cast<int>(this->data->wavePatchSize[0])/2;
    int xmid = grid->GetCellCount()[0]/2;
    int xbeg = std::max(0, xmid-xext);
    int xend = std::min(static_cast<int>(grid->GetCellCount()[0]), xmid+xext);

    int yext = static_cast<int>(this->data->wavePatchSize[1])/2;
    int ymid = grid->GetCellCount()[1]/2;
    int ybeg = std::max(0, ymid-yext);
    int yend = std::min(static_cast<int>(grid->GetCellCount()[1]), ymid+yext);

    wavefieldMsg.mutable_point()->Clear();
    for (int ix=xbeg; ix<xend; ++ix)
    {
      for (int iy=ybeg; iy<yend; ++iy)
      {
        for (int k=0; k<2; ++k)
        {
          Triangle tri(grid->GetTriangle(ix, iy, k));
          ignition::msgs::Set(wavefieldMsg.add_point(), ToIgn(tri[0]));
          ignition::msgs::Set(wavefieldMsg.add_point(), ToIgn(tri[1]));
          ignition::msgs::Set(wavefieldMsg.add_point(), ToIgn(tri[2]));
        }
      }
    }
    ignNode.Request(topicName, wavefieldMsg);
  }

} // namespace gazebo
