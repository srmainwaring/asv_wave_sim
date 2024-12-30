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


// Code for retrieving shader params copied from
// gz-sim/src/systems/shader_param

/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "WavesVisual.hh"

#include <gz/msgs/any.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>

#include <algorithm>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/Grid.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ShaderParams.hh>
#include <gz/rendering/Visual.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/SourceFilePath.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/rendering/RenderUtil.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/waves/OceanTile.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/Utilities.hh"
#include "gz/waves/WaveParameters.hh"
#include "gz/waves/WaveSimulation.hh"
#include "gz/waves/LinearRandomFFTWaveSimulation.hh"

#include "DisplacementMap.hh"
#include "OceanGeometry.hh"
#include "OceanVisual.hh"
#include "RenderEngineExtension.hh"
#include "RenderEngineExtensionManager.hh"
#include "SceneNodeFactory.hh"

// using namespace rendering;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

class WavesVisualPrivate
{
  /// \brief Data structure for storing shader param info
  public: class ShaderParamValue
  {
    /// \brief shader type: vertex or fragment
    public: std::string shader;

    /// \brief variable type: int, float, float_array, int_array,
    /// texture, texture_cube
    public: std::string type;

    /// \brief variable name of param
    public: std::string name;

    /// \brief param value
    public: std::string value;

    /// \brief Any additional arguments
    public: std::vector<std::string> args;
  };

  /// \brief Data structure for storing shader files uri
  public: class ShaderUri
  {
    /// \brief Shader language: glsl or metal
    public: std::string language;

    /// \brief Path to vertex shader
    public: std::string vertexShaderUri;

    /// \brief Path to fragment shader
    public: std::string fragmentShaderUri;
  };

  /// \brief Enum to set the method used for updating shader vertices
  public: enum class MeshDeformationMethod : uint16_t
  {
    /// \internal
    /// \brief Indicator used to create an iterator over the
    /// enum. Do not use this.
    MESH_DEFORMATION_METHOD_BEGIN = 0,

    /// \brief Unknown graphics interface
    UNKNOWN = MESH_DEFORMATION_METHOD_BEGIN,

    /// \brief Update displacement map textures
    DYNAMIC_TEXTURE = 1,

    /// \brief Update vertex, normal and tangent arrays
    DYNAMIC_GEOMETRY = 2,

    /// \internal
    /// \brief Indicator used to create an iterator over the
    /// enum. Do not use this.
    MESH_DEFORMATION_METHOD_END
  };

  /// \brief Set an enum from a string.
  /// \param[in] _str String value to convert to enum value.
  /// \return MeshDeformationMethod enum
  public: static MeshDeformationMethod SetMeshDeformationMethod(
      const std::string &_str);

  /// \brief Destructor
  public: ~WavesVisualPrivate();

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();

  /// \brief Callback for topic "/world/<world>/waves".
  ///
  /// \param[in] _msg Wave parameters message.
  public: void OnWaveMsg(const gz::msgs::Param &_msg);

  /// \brief Name of the world
  public: std::string worldName;

  /// \brief Name of visual this plugin is attached to
  public: std::string visualName;

  /// \brief Pointer to visual
  public: rendering::VisualPtr visual;

  /// \brief Entity id of the visual
  public: Entity entity{kNullEntity};

  /// \brief Pointer to scene
  public: rendering::ScenePtr scene;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;
  public: double                              currentSimTimeSeconds;

  /// \brief Set the wavefield to be static [false].
  public: bool isStatic{false};

  /// \brief The number of tiles in the x-direction given as an offset range.
  /// Default [lower=0, upper=0].
  public: gz::math::Vector2i tiles_x = {0, 0};

  /// \brief The number of tiles in the y-direction given as an offset range.
  /// Default [lower=0, upper=0].
  public: gz::math::Vector2i tiles_y = {0, 0};

  /// \brief Material used by the ocean visual
  public: rendering::MaterialPtr oceanMaterial;

  /// \brief Pointer to ocean visual
  public: std::vector<rendering::OceanVisualPtr> oceanVisuals;
  public: std::vector<rendering::VisualPtr> oceanVisuals2;
  public: std::vector<rendering::OceanGeometryPtr> oceanGeometries;

  public: MeshDeformationMethod meshDeformationMethod{
      MeshDeformationMethod::DYNAMIC_GEOMETRY};

  /////////////////
  // DYNAMIC_TEXTURE
  public: rendering::DisplacementMapPtr displacementMap;

  std::unique_ptr<gz::waves::IWaveSimulation> mWaveSim;
  Eigen::ArrayXd mHeights;
  Eigen::ArrayXd mDhdx;
  Eigen::ArrayXd mDhdy;
  Eigen::ArrayXd mDisplacementsX;
  Eigen::ArrayXd mDisplacementsY;
  Eigen::ArrayXd mDxdx;
  Eigen::ArrayXd mDydy;
  Eigen::ArrayXd mDxdy;

  public: void CreateShaderMaterial();

  public: void InitWaveSim();
  public: void InitUniforms();
  public: void InitTextures();

  public: void UpdateWaveSim();
  public: void UpdateUniforms();
  public: void UpdateTextures();

  /////////////////
  // DYNAMIC_GEOMETRY

  /// \brief Ocean mesh (not stored in the mesh manager)
  public: gz::common::MeshPtr oceanTileMesh;

  /////////////////
  // OceanTile

  /// \brief The wave parameters.
  public: waves::WaveParametersPtr waveParams;
  public: bool waveParamsDirty{false};

  /// \brief The ocean tile managing the wave simulation
  public: waves::visual::OceanTilePtr oceanTile;

  /////////////////
  // ShaderParams (from gz-sim/src/systems/shader_param)

  /// \brief A map of shader language to shader program files
  public: std::map<std::string, ShaderUri> shaders;

  /// \brief A list of shader params
  public: std::vector<ShaderParamValue> shaderParams;

  /// \brief A list of shader params that will be updated every iteration
  public: std::vector<ShaderParamValue> timeParams;

  /// \brief Path to model
  public: std::string modelPath;

  /////////////////
  // Transport and connections

  /// \brief Mutex to protect sim time and parameter updates.
  public: std::mutex mutex;

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Connection to pre-render event callback
  public: gz::common::ConnectionPtr connection{nullptr};

  /// \brief RenderEngineExtension
  public: rendering::RenderEngineExtension *extension{nullptr};

};

/////////////////////////////////////////////////
WavesVisual::WavesVisual()
    : System(), dataPtr(std::make_unique<WavesVisualPrivate>())
{
  gzmsg << "WavesVisual: constructing\n";
}

/////////////////////////////////////////////////
WavesVisual::~WavesVisual()
{
  gzmsg << "WavesVisual: destructing\n";
}

/////////////////////////////////////////////////
void WavesVisual::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  GZ_PROFILE("WavesVisual::Configure");

  gzmsg << "WavesVisual: configuring\n";

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  // Capture entity
  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // Update parameters
  {
    this->dataPtr->isStatic = waves::Utilities::SdfParamBool(
        *sdf,  "static", this->dataPtr->isStatic);

    this->dataPtr->tiles_x = waves::Utilities::SdfParamVector2i(
        *sdf,  "tiles_x", this->dataPtr->tiles_x);

    this->dataPtr->tiles_y = waves::Utilities::SdfParamVector2i(
        *sdf,  "tiles_y", this->dataPtr->tiles_y);

    std::string meshDeformationMethodStr = waves::Utilities::SdfParamString(
        *sdf,  "mesh_deformation_method", "DYNAMIC_GEOMETRY");
    this->dataPtr->meshDeformationMethod =
        this->dataPtr->SetMeshDeformationMethod(meshDeformationMethodStr);
  }

  // Wave parameters
  this->dataPtr->waveParams.reset(new waves::WaveParameters());
  this->dataPtr->waveParamsDirty = true;
  if (sdf->HasElement("wave"))
  {
    auto sdfWave = sdf->GetElement("wave");
    this->dataPtr->waveParams->SetFromSDF(*sdfWave);
  }

  // Shader params
  if (sdf->HasElement("param"))
  {
    // loop and parse all shader params
    sdf::ElementPtr paramElem = sdf->GetElement("param");
    while (paramElem)
    {
      if (!paramElem->HasElement("shader") ||
          !paramElem->HasElement("name"))
      {
        gzerr << "<param> must have <shader> and <name> sdf elements"
               << std::endl;
        paramElem = paramElem->GetNextElement("param");
        continue;
      }
      std::string shaderType = paramElem->Get<std::string>("shader");
      std::string paramName = paramElem->Get<std::string>("name");

      std::string type = paramElem->Get<std::string>("type", "float").first;
      std::string value = paramElem->Get<std::string>("value", "").first;

      WavesVisualPrivate::ShaderParamValue spv;
      spv.shader = shaderType;
      spv.name = paramName;
      spv.value = value;
      spv.type = type;

      if (paramElem->HasElement("arg"))
      {
        sdf::ElementPtr argElem = paramElem->GetElement("arg");
        while (argElem)
        {
          spv.args.push_back(argElem->Get<std::string>());
          argElem = argElem->GetNextElement("arg");
        }
      }

      this->dataPtr->shaderParams.push_back(spv);
      paramElem = paramElem->GetNextElement("param");
    }
  }

  // Model path
  if (this->dataPtr->modelPath.empty())
  {
    auto modelEntity = topLevelModel(_entity, _ecm);
    this->dataPtr->modelPath =
        _ecm.ComponentData<components::SourceFilePath>(modelEntity).value();
  }

  // Shader programs
  {
    if (!sdf->HasElement("shader"))
    {
      gzerr << "Unable to load shader param system. "
            << "Missing <shader> SDF element." << std::endl;
      return;
    }

    // Allow multiple shader SDF element for different shader languages
    sdf::ElementPtr shaderElem = sdf->GetElement("shader");
    while (shaderElem)
    {
      if (!shaderElem->HasElement("vertex") ||
          !shaderElem->HasElement("fragment"))
      {
        gzerr << "<shader> must have <vertex> and <fragment> sdf elements"
              << std::endl;
      }
      else
      {
        // default to glsl
        std::string api = "glsl";
        if (shaderElem->HasAttribute("language"))
          api = shaderElem->GetAttribute("language")->GetAsString();

        WavesVisualPrivate::ShaderUri shader;
        shader.language = api;

        sdf::ElementPtr vertexElem = shaderElem->GetElement("vertex");
        shader.vertexShaderUri = gz::common::findFile(
            asFullPath(vertexElem->Get<std::string>(),
            this->dataPtr->modelPath));
        sdf::ElementPtr fragmentElem = shaderElem->GetElement("fragment");
        shader.fragmentShaderUri = gz::common::findFile(
            asFullPath(fragmentElem->Get<std::string>(),
            this->dataPtr->modelPath));
        this->dataPtr->shaders[api] = shader;
        shaderElem = shaderElem->GetNextElement("shader");
      }
    }
    if (this->dataPtr->shaders.empty())
    {
      gzerr << "Unable to load shader param system. "
            << "No valid shaders." << std::endl;
      return;
    }
  }

  // Connect to the SceneUpdate event
  //
  // The callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<gz::sim::events::SceneUpdate>(
      std::bind(&WavesVisualPrivate::OnUpdate, this->dataPtr.get()));

  // World name
  _ecm.Each<components::World, components::Name>(
    [&](const Entity &,
        const components::World *,
        const components::Name *_name) -> bool
    {
      // Assume there's only one world
      this->dataPtr->worldName = _name->Data();
      return false;
    });

  // Subscribe to wave parameter updates
  std::string topic("/world/" + this->dataPtr->worldName + "/waves");
  this->dataPtr->node.Subscribe(
      topic, &WavesVisualPrivate::OnWaveMsg, this->dataPtr.get());

  gzmsg << "WavesVisual: subscribing to [" << topic << "]\n";
}

//////////////////////////////////////////////////
void WavesVisual::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &)
{
  GZ_PROFILE("WavesVisual::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;

  this->dataPtr->currentSimTimeSeconds =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(
          this->dataPtr->currentSimTime).count()) * 1e-9;
}

/////////////////////////////////////////////////
GZ_ENUM(meshDeformationMethodIface,
    WavesVisualPrivate::MeshDeformationMethod,
    WavesVisualPrivate::MeshDeformationMethod::MESH_DEFORMATION_METHOD_BEGIN,
    WavesVisualPrivate::MeshDeformationMethod::MESH_DEFORMATION_METHOD_END,
    "UNKNOWN", "DYNAMIC_TEXTURE", "DYNAMIC_GEOMETRY"
)

//////////////////////////////////////////////////
WavesVisualPrivate::MeshDeformationMethod
WavesVisualPrivate::SetMeshDeformationMethod(const std::string &_str)
{
  // Convert to upper case
  std::string str(_str);
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);

  // Set the enum
  MeshDeformationMethod e{MeshDeformationMethod::UNKNOWN};
  meshDeformationMethodIface.Set(e, str);
  return e;
}

//////////////////////////////////////////////////
WavesVisualPrivate::~WavesVisualPrivate()
{
  // Remove visuals
  for (auto& vis : this->oceanVisuals2)
  {
    if (vis != nullptr)
    {
      vis->Destroy();
      vis.reset();
    }
  }
};

//////////////////////////////////////////////////
void WavesVisualPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;

  if (!this->scene)
  {
    gzmsg << "WavesVisual: retrieving scene from render engine\n";
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene)
    return;

  // load extensions
  if (!this->extension)
  {
    extension = rendering::RenderEngineExtensionManager::Instance()->
        Extension("ogre2");
  }

  if (!this->extension)
    return;

  if (!this->visual)
  {
    gzmsg << "WavesVisual: searching for visual\n";

    // this does a breadth first search for visual with the entity id
    // \todo(anyone) provide a helper function in RenderUtil to search for
    // visual by entity id?
    auto rootVis = scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        // RenderUtil stores gazebo-entity user data as uint64_t
        auto variant = n->UserData("gazebo-entity");
        const uint64_t *value = std::get_if<uint64_t>(&variant);
        if (value && *value == static_cast<uint64_t>(this->entity))
        {
          this->visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          break;
        }
      }
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }
  }

  if (!this->visual)
    return;

  if (!this->scene->MaterialRegistered("OceanBlue"))
  {
    gzmsg << "WavesVisual: creating material `OceanBlue`\n";

    auto mat = this->scene->CreateMaterial("OceanBlue");
    mat->SetAmbient(0.0, 0.0, 0.3);
    mat->SetDiffuse(0.0, 0.0, 0.8);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  double simTime = this->currentSimTimeSeconds;

  // ocean tile parameters
  auto [lx, ly] = this->waveParams->TileSize();
  double ux = this->waveParams->WindVelocity().X();
  double uy = this->waveParams->WindVelocity().Y();

  switch (this->meshDeformationMethod)
  {
    case MeshDeformationMethod::DYNAMIC_GEOMETRY:
    {
      // Test attaching another visual to the entity
      if (this->oceanVisuals2.empty() && oceanVisuals.empty())
      {
        gzmsg << "WavesVisual: creating dynamic geometry ocean visual\n";

        // retrive the material from the visual's geometry
        // (it's not set on the visual)
        gzmsg << "WavesVisual: Visual Name:          "
            << this->visual->Name() << "\n";
        gzmsg << "WavesVisual: Visual GeometryCount: "
            << this->visual->GeometryCount() << "\n";
        auto visualGeometry = this->visual->GeometryByIndex(0);
        this->oceanMaterial = visualGeometry->Material();
        if (!this->oceanMaterial)
        {
          gzerr << "WavesVisual: invalid material\n";
          return;
        }

        // create ocean tile
        this->oceanTile.reset(new waves::visual::OceanTile(this->waveParams));
        this->oceanTile->SetWindVelocity(ux, uy);

        // create mesh - do not store in MeshManager as it will be modified
        this->oceanTileMesh.reset(this->oceanTile->CreateMesh());

        // Hide the primary visual
        this->visual->SetVisible(false);

        /// \note: how feasible? - multiple copies of the mesh
        // rather than multiple visuals referencing one mesh and instancing?

        // Water tiles: tiles_x[0], tiles_x[0] + 1, ..., tiles_x[1], etc.
        rendering::SceneNodeFactoryPtr sceneNodeFactory =
            this->extension->SceneNodeFactory();

        unsigned int objId = 50000;
        auto position = this->visual->LocalPosition();
        for (waves::Index iy=this->tiles_y[0]; iy <= this->tiles_y[1]; ++iy)
        {
          for (waves::Index ix=this->tiles_x[0]; ix <= this->tiles_x[1]; ++ix)
          {
            // tile position
            gz::math::Vector3d tilePosition(
              position.X() + ix * lx,
              position.Y() + iy * ly,
              position.Z() + 0.0);

#define GZ_WAVES_UPDATE_VISUALS 1
#if GZ_WAVES_UPDATE_VISUALS
            // create name
            std::stringstream ss;
            ss << "OceanVisual(" << objId++ << ")";
            std::string objName = ss.str();

            // create visual
            rendering::OceanVisualPtr oceanVisual =
                sceneNodeFactory->CreateOceanVisual(this->scene);

            oceanVisual->LoadMesh(this->oceanTileMesh);
            oceanVisual->SetLocalPosition(tilePosition);
            oceanVisual->SetMaterial(this->oceanMaterial, false);
            oceanVisual->SetVisible(true);

            // add visual to parent
            auto parent = this->visual->Parent();
            parent->AddChild(oceanVisual);
            this->oceanVisuals.push_back(oceanVisual);
#else
            // create name
            std::stringstream ss;
            ss << "OceanGeometry(" << objId++ << ")";
            std::string objName = ss.str();

            // create visual
            auto oceanVisual = this->scene->CreateVisual();
            oceanVisual->SetLocalPosition(tilePosition);

            // create geometry
            rendering::OceanGeometryPtr geometry =
                sceneNodeFactory->CreateOceanGeometry(this->scene);

            geometry->LoadMesh(this->oceanTileMesh);
            geometry->SetMaterial(this->oceanMaterial, false);
            oceanVisual->AddGeometry(geometry);

            // add visual to parent
            auto parent = this->visual->Parent();
            parent->AddChild(oceanVisual);
            this->oceanVisuals2.push_back(oceanVisual);
            this->oceanGeometries.push_back(geometry);
#endif
          }
        }
      }

      if ((this->oceanVisuals2.empty() && oceanVisuals.empty()) ||
          this->isStatic)
        return;

      if (this->waveParamsDirty)
      {
        this->oceanTile->SetWindVelocity(
            this->waveParams->WindVelocity().X(),
            this->waveParams->WindVelocity().Y());
        this->oceanTile->SetSteepness(
            this->waveParams->Steepness());
        this->waveParamsDirty = false;
      }

      // update the tile (recalculates vertices)
      this->oceanTile->UpdateMesh(simTime, this->oceanTileMesh.get());

#if GZ_WAVES_UPDATE_VISUALS
      for (auto& vis : this->oceanVisuals)
      {
        vis->UpdateMesh(this->oceanTileMesh);
      }
#else
      for (auto& geom : this->oceanGeometries)
      {
        geom->UpdateMesh(this->oceanTileMesh);
      }
#endif
      break;
    }
    case MeshDeformationMethod::DYNAMIC_TEXTURE:
    {
      // Test attaching a gz::common::Mesh to the entity
      if (this->oceanVisuals2.empty())
      {
        gzmsg << "WavesVisual: creating dynamic texture ocean visual\n";

        // create shader material
        this->CreateShaderMaterial();

        // Hide the primary visual
        this->visual->SetVisible(false);

        // Get geometry (should be a plane mesh)
        auto geometry = this->visual->GeometryByIndex(0);
        if (!geometry)
        {
          gzerr << "Waves visual has invalid geometry\n";
          return;
        }

        // Water tiles: tiles_x[0], tiles_x[0] + 1, ..., tiles_x[1], etc.
        auto position = this->visual->LocalPosition();
        for (waves::Index iy=this->tiles_y[0]; iy <= this->tiles_y[1]; ++iy)
        {
          for (waves::Index ix=this->tiles_x[0]; ix <= this->tiles_x[1]; ++ix)
          {
            // tile position
            gz::math::Vector3d tilePosition(
              position.X() + ix * lx,
              position.Y() + iy * ly,
              position.Z() + 0.0);

            /// \note: replaced with cloned geometry from primary visual
            // auto geometry = this->scene->CreateMesh(descriptor);

            // create ocean visual
            auto oceanVisual = this->scene->CreateVisual();
            oceanVisual->AddGeometry(geometry->Clone());
            oceanVisual->SetMaterial(this->oceanMaterial, false);
            oceanVisual->SetLocalPosition(tilePosition);

            // add visual to parent
            auto parent = this->visual->Parent();
            parent->AddChild(oceanVisual);
            this->oceanVisuals2.push_back(oceanVisual);
         }
        }

        this->InitWaveSim();
        this->InitUniforms();
        this->InitTextures();
      }

      if (this->oceanVisuals2.empty() || this->isStatic)
        return;

      if (this->waveParamsDirty)
      {
        this->mWaveSim->SetWindVelocity(
            this->waveParams->WindVelocity().X(),
            this->waveParams->WindVelocity().Y());
        this->mWaveSim->SetSteepness(
            this->waveParams->Steepness());
        this->waveParamsDirty = false;
      }

      this->UpdateWaveSim();
      this->UpdateUniforms();
      this->UpdateTextures();
      break;
    }
    default:
    {
      gzerr << "Invalid mesh deformation method\n";
      break;
    }
  }
}

//////////////////////////////////////////////////
void WavesVisualPrivate::OnWaveMsg(const gz::msgs::Param &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // gzmsg << _msg.DebugString();

  // current wind speed and angle
  double windSpeed = this->waveParams->WindSpeed();
  double windAngleRad = this->waveParams->WindAngleRad();
  double steepness = this->waveParams->Steepness();

  // extract parameters
  {
    auto it = _msg.params().find("wind_speed");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is double
      auto param = it->second;
      // auto type = param.type();
      auto value = param.double_value();
      windSpeed = value;
    }
  }
  {
    auto it = _msg.params().find("wind_angle");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is double
      auto param = it->second;
      // auto type = param.type();
      auto value = param.double_value();
      windAngleRad = M_PI/180.0*value;
    }
  }
  {
    auto it = _msg.params().find("steepness");
    if (it != _msg.params().end())
    {
      /// \todo: assert the type is double
      auto param = it->second;
      // auto type = param.type();
      auto value = param.double_value();
      steepness = value;
    }
  }

  /// \note: oceanTile cannot be updated in this function as it
  //  is created on the render thread and is not available here.
  this->waveParams->SetWindSpeedAndAngle(windSpeed, windAngleRad);
  this->waveParams->SetSteepness(steepness);

  this->waveParamsDirty = true;
}

//////////////////////////////////////////////////
void WavesVisualPrivate::CreateShaderMaterial()
{
  this->oceanMaterial = this->scene->CreateMaterial();

  // default to glsl
  auto it = this->shaders.find("glsl");

  // prefer metal over glsl on macOS
  /// \todo(anyone) instead of using ifdef to check for macOS,
  // expose add an accessor function to get the GraphicsApi
  // from rendering::RenderEngine
#ifdef __APPLE__
  auto metalIt = this->shaders.find("metal");
  if (metalIt != this->shaders.end())
  {
    this->oceanMaterial->SetVertexShader(metalIt->second.vertexShaderUri);
    this->oceanMaterial->SetFragmentShader(metalIt->second.fragmentShaderUri);
    // if both glsl and metal are specified, print a msg to inform that
    // metal is used instead of glsl
    if (it != this->shaders.end())
    {
      gzmsg << "Using metal shaders. " << std::endl;
    }
  }
#else
  if (it != this->shaders.end())
  {
    this->oceanMaterial->SetVertexShader(it->second.vertexShaderUri);
    this->oceanMaterial->SetFragmentShader(it->second.fragmentShaderUri);
  }
#endif
}

//////////////////////////////////////////////////
void WavesVisualPrivate::InitWaveSim()
{
  auto [nx, ny] = this->waveParams->CellCount();
  auto [lx, ly] = this->waveParams->TileSize();
  double ux  = this->waveParams->WindVelocity().X();
  double uy  = this->waveParams->WindVelocity().Y();
  double s   = this->waveParams->Steepness();

  // create wave model
  std::unique_ptr<gz::waves::LinearRandomFFTWaveSimulation> waveSim(
      new gz::waves::LinearRandomFFTWaveSimulation(lx, ly, nx, ny));

  // set params
  waveSim->SetWindVelocity(ux, uy);
  waveSim->SetLambda(s);

  waves::Index N2 = nx * ny;
  this->mHeights = Eigen::ArrayXd::Zero(N2);
  this->mDisplacementsX = Eigen::ArrayXd::Zero(N2);
  this->mDisplacementsY = Eigen::ArrayXd::Zero(N2);
  this->mDhdx = Eigen::ArrayXd::Zero(N2);
  this->mDhdy = Eigen::ArrayXd::Zero(N2);
  this->mDxdx = Eigen::ArrayXd::Zero(N2);
  this->mDydy = Eigen::ArrayXd::Zero(N2);
  this->mDxdy = Eigen::ArrayXd::Zero(N2);

  // move
  this->mWaveSim = std::move(waveSim);
}

//////////////////////////////////////////////////
void WavesVisualPrivate::InitUniforms()
{
  /// \note: Adapted from gz-sim/src/systems/shader_param/ShaderParam.cc
  /// ShaderParamPrivate::OnUpdate

  // set the shader params read from SDF
  // this is only done once
  for (const auto & spv : this->shaderParams)
  {
    // TIME is reserved keyword for sim time
    if (spv.value == "TIME")
    {
      this->timeParams.push_back(spv);
      continue;
    }

    rendering::ShaderParamsPtr params;
    if (spv.shader == "fragment")
    {
      params = this->oceanMaterial->FragmentShaderParams();
    }
    else if (spv.shader == "vertex")
    {
      params = this->oceanMaterial->VertexShaderParams();
    }

    // if no <value> is specified, this could be a constant
    if (spv.value.empty())
    {
      // \todo handle args for constants in gz-rendering
      (*params)[spv.name] = 1;
      continue;
    }

    // handle texture params
    if (spv.type == "texture")
    {
      unsigned int uvSetIndex = spv.args.empty() ? 0u :
          static_cast<unsigned int>(std::stoul(spv.args[0]));
      std::string texPath = gz::common::findFile(
          asFullPath(spv.value, this->modelPath));
      (*params)[spv.name].SetTexture(texPath,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE, uvSetIndex);

      gzmsg << "Shader param [" << spv.name << "]"
          << ", type: " << spv.type
          << ", tex coord set: " << uvSetIndex << "\n";
    }
    else if (spv.type == "texture_cube")
    {
      unsigned int uvSetIndex = spv.args.empty() ? 0u :
          static_cast<unsigned int>(std::stoul(spv.args[0]));
      std::string texPath = gz::common::findFile(
          asFullPath(spv.value, this->modelPath));
      (*params)[spv.name].SetTexture(texPath,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, uvSetIndex);

      gzmsg << "Shader param [" << spv.name << "]"
          << ", type: " << spv.type
          << ", tex coord set: " << uvSetIndex << "\n";
    }
    // handle int, float, int_array, and float_array params
    else
    {
      std::vector<std::string> values = gz::common::split(spv.value, " ");

      int intValue = 0;
      float floatValue = 0;
      std::vector<float> floatArrayValue;

      rendering::ShaderParam::ParamType paramType =
          rendering::ShaderParam::PARAM_NONE;

      // float / int
      if (values.size() == 1u)
      {
        std::string str = values[0];

        // if <type> is not empty, respect the specified type
        if (!spv.type.empty())
        {
          if (spv.type == "int")
          {
            intValue = std::stoi(str);
            paramType = rendering::ShaderParam::PARAM_INT;
          }
          else if (spv.type == "float")
          {
            floatValue = std::stof(str);
            paramType = rendering::ShaderParam::PARAM_FLOAT;
          }
        }
        // else do our best guess at what the type is
        else
        {
          std::string::size_type sz;
          int n = std::stoi(str, &sz);
          if ( sz == str.size())
          {
            intValue = n;
            paramType = rendering::ShaderParam::PARAM_INT;
          }
          else
          {
            floatValue = std::stof(str);
            paramType = rendering::ShaderParam::PARAM_FLOAT;
          }
        }
      }
      // arrays
      else
      {
        // int array
        if (!spv.type.empty() && spv.type == "int_array")
        {
          for (const auto &v : values)
            floatArrayValue.push_back(std::stoi(v));
          paramType = rendering::ShaderParam::PARAM_INT_BUFFER;
        }
        // treat everything else as float_array
        else
        {
          for (const auto &v : values)
            floatArrayValue.push_back(std::stof(v));
          paramType = rendering::ShaderParam::PARAM_FLOAT_BUFFER;
        }
      }

      // set the params
      if (paramType == rendering::ShaderParam::PARAM_INT)
      {
        (*params)[spv.name] = intValue;
      }
      else if (paramType == rendering::ShaderParam::PARAM_FLOAT)
      {
        (*params)[spv.name] = floatValue;
      }
      else if (paramType == rendering::ShaderParam::PARAM_INT_BUFFER ||
          paramType == rendering::ShaderParam::PARAM_FLOAT_BUFFER)
      {
        (*params)[spv.name].InitializeBuffer(floatArrayValue.size());
        float *fv = &floatArrayValue[0];
        (*params)[spv.name].UpdateBuffer(fv);
      }
    }
  }
  this->shaderParams.clear();
}

//////////////////////////////////////////////////
void WavesVisualPrivate::InitTextures()
{
  gzmsg << "WavesVisualPrivate::InitTextures\n";

  // ocean tile parameters
  auto [nx, ny] = this->waveParams->CellCount();

  rendering::SceneNodeFactoryPtr sceneNodeFactory =
      this->extension->SceneNodeFactory();
  this->displacementMap = sceneNodeFactory->CreateDisplacementMap(
    this->scene, this->oceanMaterial, this->entity, nx, ny);
  this->displacementMap->InitTextures();
}

//////////////////////////////////////////////////
void WavesVisualPrivate::UpdateWaveSim()
{
  double simTime = this->currentSimTimeSeconds;

  mWaveSim->SetTime(simTime);
  mWaveSim->DisplacementAndDerivAt(
      mHeights, mDisplacementsX, mDisplacementsY,
      mDhdx, mDhdy, mDxdx, mDydy, mDxdy);
}

//////////////////////////////////////////////////
void WavesVisualPrivate::UpdateUniforms()
{
  /// \note: Adapted from gz-sim/src/systems/shader_param/ShaderParam.cc
  /// ShaderParamPrivate::OnUpdate

  float simTime = static_cast<float>(this->currentSimTimeSeconds);

  // time variables need to be updated every iteration
  for (const auto & spv : this->timeParams)
  {
    rendering::ShaderParamsPtr params;
    if (spv.shader == "fragment")
    {
      params = this->oceanMaterial->FragmentShaderParams();
    }
    else if (spv.shader == "vertex")
    {
      params = this->oceanMaterial->VertexShaderParams();
    }
    (*params)[spv.name] = simTime;
  }
}

// //////////////////////////////////////////////////
void WavesVisualPrivate::UpdateTextures()
{
  this->displacementMap->UpdateTextures(
    this->mHeights,
    this->mDhdx,
    this->mDhdy,
    this->mDisplacementsX,
    this->mDisplacementsY,
    this->mDxdx,
    this->mDydy,
    this->mDxdy);
}

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(gz::sim::systems::WavesVisual,
              gz::sim::System,
              gz::sim::systems::WavesVisual::ISystemConfigure,
              gz::sim::systems::WavesVisual::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::WavesVisual,
                   "gz::sim::systems::WavesVisual")
