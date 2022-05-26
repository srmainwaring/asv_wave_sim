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


// Code for retrieving shader params from gz-sim/src/systems/shader_param

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

#include "Ogre2OceanVisual.hh"
#include "Ogre2OceanGeometry.hh"

#include "gz/marine/OceanTile.hh"
#include "gz/marine/Utilities.hh"
#include "gz/marine/WaveParameters.hh"

#include "gz/marine/WaveSimulation.hh"
#include "gz/marine/WaveSimulationFFT2.hh"

#include <gz/msgs/any.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ShaderParams.hh>
#include <gz/rendering/Visual.hh>

#include <gz/rendering/Grid.hh>

#include <gz/rendering/ogre2.hh>
#include <gz/rendering/ogre2/Ogre2MeshFactory.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include <gz/transport/Node.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Element.hh>

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

  // Subclass from Ogre2Mesh and Ogre2MeshFactory to get
  // indirect access to protected members and override any
  // behaviour that tries to load a common::Mesh which we
  // are not using.

  //////////////////////////////////////////////////
  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshExt :
      public Ogre2Mesh
  {
    /// \brief Destructor
    public: virtual ~Ogre2MeshExt() {}

    /// \brief Constructor
    protected: explicit Ogre2MeshExt() : Ogre2Mesh() {}

    /// \brief Allow intercept of pre-render call for this mesh
    public: virtual void PreRender() override
    {
      Ogre2Mesh::PreRender();
    }

    /// \brief Work-around the protected accessors and protected methods in Scene
    public: void InitObject(Ogre2ScenePtr _scene, unsigned int _id, const std::string &_name)
    {
      this->id = _id;
      this->name = _name;
      this->scene = _scene;

      // initialize object
      this->Load();
      this->Init();
    }

    /// \brief Used by friend class (Ogre2MeshFactoryExt)
    protected: void SetOgreItem(Ogre::Item *_ogreItem)
    {
      this->ogreItem = _ogreItem;
    }

    private: friend class Ogre2MeshFactoryExt;
  };

  typedef std::shared_ptr<Ogre2MeshExt> Ogre2MeshExtPtr;

  //////////////////////////////////////////////////
  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshFactoryExt :
      public Ogre2MeshFactory
  {
    /// \brief Destructor
    public: virtual ~Ogre2MeshFactoryExt() {}

    /// \brief Constructor - construct from an Ogre2ScenePtr
    public: explicit Ogre2MeshFactoryExt(Ogre2ScenePtr _scene) :
        Ogre2MeshFactory(_scene), ogre2Scene(_scene) {}

    /// \brief Override - use an extension of Ogre2Mesh
    public: virtual Ogre2MeshPtr Create(const MeshDescriptor &_desc) override
    {
      // create ogre entity
      Ogre2MeshExtPtr mesh(new Ogre2MeshExt);
      MeshDescriptor normDesc = _desc;
      // \todo do this? override MeshDescriptor behaviour as we're not using common::Mesh
      normDesc.Load();
      mesh->SetOgreItem(this->OgreItem(normDesc));

      // check if invalid mesh
      if (!mesh->ogreItem)
      {
        ignerr << "Failed to get Ogre item for [" << _desc.meshName << "]"
                << std::endl;
        return nullptr;
      }

      // create sub-mesh store
      Ogre2SubMeshStoreFactory subMeshFactory(this->scene, mesh->ogreItem);
      mesh->subMeshes = subMeshFactory.Create();
      for (unsigned int i = 0; i < mesh->subMeshes->Size(); i++)
      {
        Ogre2SubMeshPtr submesh =
            std::dynamic_pointer_cast<Ogre2SubMesh>(mesh->subMeshes->GetById(i));
        submesh->SetMeshName(this->MeshName(_desc));
      }
      return mesh;
    }

    /// \brief Override \todo: may be able to use base class implementation...
    protected: virtual Ogre::Item * OgreItem(const MeshDescriptor &_desc) override
    {
      if (!this->Load(_desc))
      {
        return nullptr;
      }

      std::string name = this->MeshName(_desc);
      ignmsg << "Get Ogre::SceneManager\n";
      Ogre::SceneManager *sceneManager = this->scene->OgreSceneManager();

      ignmsg << "Check for v2 mesh\n";
      // check if a v2 mesh already exists
      Ogre::MeshPtr mesh =
          Ogre::MeshManager::getSingleton().getByName(name);

      // if not, it probably has not been imported from v1 yet
      if (!mesh)
      {
        ignmsg << "Check for v1 mesh\n";
        Ogre::v1::MeshPtr v1Mesh =
            Ogre::v1::MeshManager::getSingleton().getByName(name);
        if (!v1Mesh)
        {
          ignerr << "Did not find v1 mesh [" << name << "]\n";
          return nullptr;
        }

        // examine v1 mesh properties
        v1Mesh->load();
        ignmsg << "v1 mesh: isLoaded: " << v1Mesh->isLoaded() << "\n";

        ignmsg << "Creating v2 mesh\n";
        // create v2 mesh from v1
        mesh = Ogre::MeshManager::getSingleton().createManual(
            name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        ignmsg << "Importing v2 mesh\n";
        mesh->importV1(v1Mesh.get(), false, true, true);
        this->ogreMeshes.push_back(name);
      }

      return sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
    }

    /// \brief Override
    protected: virtual bool LoadImpl(const MeshDescriptor &_desc) override
    {
      /// \todo: currently assume we've already loaded from the tile
      /// but should handle better
      return true;
    }

    /// \brief Override \todo: may be able to use base class implementation...
    public: virtual bool Validate(const MeshDescriptor &_desc) override
    {
      if (!_desc.mesh && _desc.meshName.empty())
      {
        ignerr << "Invalid mesh-descriptor, no mesh specified" << std::endl;
        return false;
      }

      if (!_desc.mesh)
      {
        ignerr << "Cannot load null mesh [" << _desc.meshName << "]" << std::endl;
        return false;
        // Override MeshDescriptor behaviour as we're not using common::Mesh
        // return true;
      }

      if (_desc.mesh->SubMeshCount() == 0)
      {
        ignerr << "Cannot load mesh with zero sub-meshes" << std::endl;
        return false;
      }

      return true;
    }

    /// \brief Pointer to the derived Ogre2Scene
    private: rendering::Ogre2ScenePtr ogre2Scene;

  };

  typedef std::shared_ptr<Ogre2MeshFactoryExt> Ogre2MeshFactoryExtPtr;
}
}
}

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::WavesVisualPrivate
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

  /// \brief Destructor
  public: ~WavesVisualPrivate();

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();

  /// \brief Callback for topic "/world/<world>/waves".
  ///
  /// \param[in] _msg Wave parameters message.
  public: void OnWaveMsg(const ignition::msgs::Param &_msg);

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
  public: math::Vector2i tiles_x = {0, 0};

  /// \brief The number of tiles in the y-direction given as an offset range.
  /// Default [lower=0, upper=0].
  public: math::Vector2i tiles_y = {0, 0};

  /// \brief Material used by the ocean visual
  public: rendering::MaterialPtr oceanMaterial;

  /// \brief Pointer to ocean visual
  public: std::vector<rendering::Ogre2OceanVisualPtr> oceanOgreVisuals;
  public: std::vector<rendering::VisualPtr> oceanVisuals;
  public: std::vector<rendering::Ogre2OceanGeometryPtr> oceanGeometries;

  /////////////////
  /// OGRE2_DYNAMIC_TEXTURE
  /// \note: new code adapted from ogre ocean materials sample
  // textures for displacement, normal and tangent maps
  Ogre::Image2       *mHeightMapImage;
  Ogre::Image2       *mNormalMapImage;
  Ogre::Image2       *mTangentMapImage;
  Ogre::TextureGpu   *mHeightMapTex;
  Ogre::TextureGpu   *mNormalMapTex;
  Ogre::TextureGpu   *mTangentMapTex;

  std::unique_ptr<ignition::marine::WaveSimulation> mWaveSim;
  std::vector<double> mHeights;
  std::vector<double> mDhdx;
  std::vector<double> mDhdy;
  std::vector<double> mDisplacementsX;
  std::vector<double> mDisplacementsY;
  std::vector<double> mDxdx;
  std::vector<double> mDydy;
  std::vector<double> mDxdy;

  public: void CreateShaderMaterial();

  public: void InitWaveSim();
  public: void InitUniforms();
  public: void InitTextures();

  public: void UpdateWaveSim();
  public: void UpdateUniforms();
  public: void UpdateTextures();

  /////////////////
  /// OGRE2_DYNAMIC_GEOMETRY
  /// \brief Ocean mesh (not stored in the mesh manager)
  public: common::MeshPtr oceanTileMesh;

  /////////////////
  /// OceanTile
  /// \brief The wave parameters.
  public: marine::WaveParametersPtr waveParams;
  public: bool waveParamsDirty{false};

  // std::string mAboveOceanMeshName = "AboveOceanTileMesh";
  // std::string mBelowOceanMeshName = "BelowOceanTileMesh";

  /// \brief The ocean tile managing the wave simulation
  public: marine::visual::OceanTilePtr oceanTile;

  /////////////////
  /// ShaderParams (from gz-sim/src/systems/shader_param)

  /// \brief A map of shader language to shader program files
  public: std::map<std::string, ShaderUri> shaders;

  /// \brief A list of shader params
  public: std::vector<ShaderParamValue> shaderParams;

  /// \brief A list of shader params that will be updated every iteration
  public: std::vector<ShaderParamValue> timeParams;

  /// \brief Path to model
  public: std::string modelPath;

  /// \brief Mutex to protect sim time and parameter updates.
  public: std::mutex mutex;

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection{nullptr};
};

/////////////////////////////////////////////////
WavesVisual::WavesVisual()
    : System(), dataPtr(std::make_unique<WavesVisualPrivate>())
{
}

/////////////////////////////////////////////////
WavesVisual::~WavesVisual()
{
}

/////////////////////////////////////////////////
void WavesVisual::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  IGN_PROFILE("WavesVisual::Configure");

  ignmsg << "WavesVisual: configuring\n";

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  // Capture entity 
  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // Update parameters
  {
    this->dataPtr->isStatic = marine::Utilities::SdfParamBool(
        *sdf,  "static", this->dataPtr->isStatic);

    this->dataPtr->tiles_x = marine::Utilities::SdfParamVector2i(
        *sdf,  "tiles_x", this->dataPtr->tiles_x);

    this->dataPtr->tiles_y = marine::Utilities::SdfParamVector2i(
        *sdf,  "tiles_y", this->dataPtr->tiles_y);
  }

  // Wave parameters
  this->dataPtr->waveParams.reset(new marine::WaveParameters());
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
        ignerr << "<param> must have <shader> and <name> sdf elements"
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
      ignerr << "Unable to load shader param system. "
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
        ignerr << "<shader> must have <vertex> and <fragment> sdf elements"
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
        shader.vertexShaderUri = common::findFile(
            asFullPath(vertexElem->Get<std::string>(),
            this->dataPtr->modelPath));
        sdf::ElementPtr fragmentElem = shaderElem->GetElement("fragment");
        shader.fragmentShaderUri = common::findFile(
            asFullPath(fragmentElem->Get<std::string>(),
            this->dataPtr->modelPath));
        this->dataPtr->shaders[api] = shader;
        shaderElem = shaderElem->GetNextElement("shader");
      }
    }
    if (this->dataPtr->shaders.empty())
    {
      ignerr << "Unable to load shader param system. "
            << "No valid shaders." << std::endl;
      return;
    }
  }

  // Connect to the SceneUpdate event
  //
  // The callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<ignition::gazebo::events::SceneUpdate>(
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

  ignmsg << "WavesVisual: subscribing to [" << topic << "]\n";
}

//////////////////////////////////////////////////
void WavesVisual::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &)
{
  IGN_PROFILE("WavesVisual::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;

  this->dataPtr->currentSimTimeSeconds =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(
          this->dataPtr->currentSimTime).count()) * 1e-9;
}

/////////////////////////////////////////////////
//////////////////////////////////////////////////
WavesVisualPrivate::~WavesVisualPrivate()
{
  for (auto& ogreVisual : this->oceanVisuals)
  {
    auto vis = ogreVisual;
    if (vis != nullptr)
    {
      vis->Destroy();
      vis.reset();
    }
  }
};

//////////////////////////////////////////////////
enum class OceanVisualMethod : uint16_t
{
  /// \internal
  /// \brief Indicator used to create an iterator over the
  /// enum. Do not use this.
  OCEAN_VISUAL_METHOD_BEGIN = 0,

  /// \brief Unknown graphics interface
  UNKNOWN = OCEAN_VISUAL_METHOD_BEGIN,

  /// \brief Ogre::Mesh (v2)
  OGRE2_DYNAMIC_TEXTURE = 2,

  /// \brief Ogre2DynamicGeometry
  OGRE2_DYNAMIC_GEOMETRY = 3,

  /// \internal
  /// \brief Indicator used to create an iterator over the
  /// enum. Do not use this.
  OCEAN_VISUAL_METHOD_END
};

//////////////////////////////////////////////////
void WavesVisualPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;

  if (!this->scene)
  {
    ignmsg << "WavesVisual: retrieving scene from render engine\n";
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene)
    return;

  if (!this->visual)
  {
    ignmsg << "WavesVisual: searching for visual\n";

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
        if (value && *value == static_cast<int>(this->entity))
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
    ignmsg << "WavesVisual: creating material `OceanBlue`\n";

    auto mat = this->scene->CreateMaterial("OceanBlue");
    mat->SetAmbient(0.0, 0.0, 0.3);
    mat->SetDiffuse(0.0, 0.0, 0.8);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  double simTime = this->currentSimTimeSeconds;

  // ocean tile parameters
  size_t N = this->waveParams->CellCount();
  double L = this->waveParams->TileSize();
  double ux = this->waveParams->WindVelocity().X();
  double uy = this->waveParams->WindVelocity().Y();

  OceanVisualMethod method = OceanVisualMethod::OGRE2_DYNAMIC_TEXTURE;
  switch (method)
  {
    case OceanVisualMethod::OGRE2_DYNAMIC_GEOMETRY:
    {
      // Test attaching another visual to the entity
      if (this->oceanVisuals.empty() && oceanOgreVisuals.empty())
      {
        ignmsg << "WavesVisual: creating dynamic geometry ocean visual\n";

        // retrive the material from the visual's geometry (it's not set on the visual)
        ignmsg << "WavesVisual: Visual Name:          " << this->visual->Name() << "\n";
        ignmsg << "WavesVisual: Visual GeometryCount: " << this->visual->GeometryCount() << "\n";
        auto visualGeometry = this->visual->GeometryByIndex(0);
        this->oceanMaterial = visualGeometry->Material();
        if (!this->oceanMaterial)
        {
          ignerr << "WavesVisual: invalid material\n";
          return;
        }

        // create ocean tile
        this->oceanTile.reset(new marine::visual::OceanTile(this->waveParams));
        this->oceanTile->SetWindVelocity(ux, uy);

        // create mesh - do not store in MeshManager as it will be modified
        this->oceanTileMesh.reset(this->oceanTile->CreateMesh());

        // scene: use in object initialisation work-around
        rendering::Ogre2ScenePtr ogre2Scene =
            std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);

        // Hide the primary visual
        this->visual->SetVisible(false);

        /// \note: how feasible? - multiple copies of the mesh
        // rather than multiple visuals referencing one mesh and instancing?

        // Water tiles: tiles_x[0], tiles_x[0] + 1, ..., tiles_x[1], etc.
        unsigned int objId = 50000;
        auto position = this->visual->LocalPosition();
        for (int iy=this->tiles_y[0]; iy<=this->tiles_y[1]; ++iy)
        {
          for (int ix=this->tiles_x[0]; ix<=this->tiles_x[1]; ++ix)
          {
            // tile position 
            ignition::math::Vector3d tilePosition(
              position.X() + ix * L,
              position.Y() + iy * L,
              position.Z() + 0.0
            );

#define GZ_MARINE_UPDATE_VISUALS 1
#if GZ_MARINE_UPDATE_VISUALS
            // create name
            std::stringstream ss;
            ss << "OceanVisual(" << objId++ << ")";
            std::string objName = ss.str();

            // create visual
            // ignmsg << "Creating visual: tile: ["
            //     << ix << ", " << iy << "]"
            //     << ", name: " << objName << "\n";
            rendering::Ogre2OceanVisualPtr ogreVisual =
                std::make_shared<rendering::Ogre2OceanVisual>(); 
            ogreVisual->InitObject(ogre2Scene, objId, objName);
            ogreVisual->LoadMesh(this->oceanTileMesh);

            ogreVisual->SetLocalPosition(tilePosition);
            ogreVisual->SetMaterial(this->oceanMaterial, false);
            ogreVisual->SetVisible(true);

            // add visual to parent
            auto parent = this->visual->Parent();
            parent->AddChild(ogreVisual);
            this->oceanOgreVisuals.push_back(ogreVisual);
#else
            // create name
            std::stringstream ss;
            ss << "OceanGeometry(" << objId++ << ")";
            std::string objName = ss.str();

            // create visual
            auto oceanVisual = this->scene->CreateVisual();
            oceanVisual->SetLocalPosition(tilePosition);

            // create geometry
            // ignmsg << "Creating geometry: tile: ["
            //     << ix << ", " << iy << "]"
            //     << ", name: " << objName << "\n";
            auto geometry =
                std::make_shared<rendering::Ogre2OceanGeometry>();
            geometry->InitObject(ogre2Scene, objId, objName);
            geometry->LoadMesh(this->oceanTileMesh);

            geometry->SetMaterial(this->oceanMaterial, false);

            oceanVisual->AddGeometry(geometry);

            // add visual to parent
            auto parent = this->visual->Parent();
            parent->AddChild(oceanVisual);
            this->oceanVisuals.push_back(oceanVisual);
            this->oceanGeometries.push_back(geometry);
#endif
          }
        }
      }

      if ((this->oceanVisuals.empty() && oceanOgreVisuals.empty()) || this->isStatic)
        return;

      if (this->waveParamsDirty)
      {
        double ux = this->waveParams->WindVelocity().X();
        double uy = this->waveParams->WindVelocity().Y();
        this->oceanTile->SetWindVelocity(ux, uy);
        this->waveParamsDirty = false;
      }

      // update the tile (recalculates vertices)
      this->oceanTile->UpdateMesh(simTime, this->oceanTileMesh.get());

#if GZ_MARINE_UPDATE_VISUALS
      for (auto& vis : this->oceanOgreVisuals)
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
    case OceanVisualMethod::OGRE2_DYNAMIC_TEXTURE:
    {
      // Test attaching a common::Mesh to the entity
      if (this->oceanVisuals.empty())
      {
        ignmsg << "WavesVisual: creating Ogre::Mesh ocean visual\n";

        // create shader material
        this->CreateShaderMaterial();

        /// \note: replaced with cloned geometry from primary visual 
        // load mesh
        // std::string meshPath = common::findFile(
        //     asFullPath("materials/mesh_256x256.dae", this->modelPath));
        // rendering::MeshDescriptor descriptor;
        // descriptor.meshName = meshPath;
        // common::MeshManager *meshManager = common::MeshManager::Instance();
        // descriptor.mesh = meshManager->Load(descriptor.meshName);
 
        // Hide the primary visual
        this->visual->SetVisible(false);

        // Get geometry (should be a plane mesh)
        auto geometry = this->visual->GeometryByIndex(0);
        if (!geometry)
        {
          ignerr << "Waves visual has invalid geometry\n";
          return;
        }

        // Water tiles: tiles_x[0], tiles_x[0] + 1, ..., tiles_x[1], etc.
        auto position = this->visual->LocalPosition();
        for (int iy=this->tiles_y[0]; iy<=this->tiles_y[1]; ++iy)
        {
          for (int ix=this->tiles_x[0]; ix<=this->tiles_x[1]; ++ix)
          {
            // tile position 
            ignition::math::Vector3d tilePosition(
              position.X() + ix * L,
              position.Y() + iy * L,
              position.Z() + 0.0
            );

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
            this->oceanVisuals.push_back(oceanVisual);
         }
        }

        this->InitWaveSim();
        this->InitUniforms();
        this->InitTextures();
      }

      if (this->oceanVisuals.empty() || this->isStatic)
        return;

      if (this->waveParamsDirty)
      {
        double ux = this->waveParams->WindVelocity().X();
        double uy = this->waveParams->WindVelocity().Y();
        double s  = this->waveParams->Steepness();

        // set params
        this->mWaveSim->SetWindVelocity(ux, uy);
        // waveSim->SetLambda(s);

        this->waveParamsDirty = false;
      }

      this->UpdateWaveSim();
      this->UpdateUniforms();
      this->UpdateTextures();
      break;
    }
    default:
    {
      // Test attaching another visual to the entity
      if (this->oceanVisuals.empty())
      {
        ignmsg << "WavesVisual: creating default ocean visual\n";

        // create plane
        auto geometry = this->scene->CreatePlane();

        // create visual
        auto oceanVisual = this->scene->CreateVisual("ocean-tile");
        oceanVisual->AddGeometry(geometry);
        oceanVisual->SetLocalPosition(0.0, 0.0, 0.0);
        oceanVisual->SetLocalRotation(0.0, 0.0, 0.0);
        oceanVisual->SetLocalScale(100.0, 100.0, 100.0);
        oceanVisual->SetMaterial("OceanBlue");
        this->oceanVisuals.push_back(oceanVisual);
      }
      break;
    }
  }
}

//////////////////////////////////////////////////
void WavesVisualPrivate::OnWaveMsg(const ignition::msgs::Param &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // ignmsg << _msg.DebugString();

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
      auto type = param.type();
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
      auto type = param.type();
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
      auto type = param.type();
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
  if (it != this->shaders.end())
  {
    this->oceanMaterial->SetVertexShader(it->second.vertexShaderUri);
    this->oceanMaterial->SetFragmentShader(it->second.fragmentShaderUri);
  }
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
      ignmsg << "Using metal shaders. " << std::endl;
    }
  }
#endif
}

//////////////////////////////////////////////////
void WavesVisualPrivate::InitWaveSim()
{
  int N      = this->waveParams->CellCount();
  double L   = this->waveParams->TileSize();
  double ux  = this->waveParams->WindVelocity().X();
  double uy  = this->waveParams->WindVelocity().Y();
  double s   = this->waveParams->Steepness();

  // create wave model
  std::unique_ptr<ignition::marine::WaveSimulationFFT2> waveSim(
      new ignition::marine::WaveSimulationFFT2(N, L));

  // set params
  waveSim->SetWindVelocity(ux, uy);
  waveSim->SetLambda(s);

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
      // \todo handle args for constants in ign-rendering
      (*params)[spv.name] = 1;
      continue;
    }

    // handle texture params
    if (spv.type == "texture")
    {
      unsigned int uvSetIndex = spv.args.empty() ? 0u :
          static_cast<unsigned int>(std::stoul(spv.args[0]));
      std::string texPath = common::findFile(
          asFullPath(spv.value, this->modelPath));
      (*params)[spv.name].SetTexture(texPath,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE, uvSetIndex);
      
      ignmsg << "Shader param [" << spv.name << "]" 
          << ", type: " << spv.type
          << ", tex coord set: " << uvSetIndex << "\n";
    }
    else if (spv.type == "texture_cube")
    {
      unsigned int uvSetIndex = spv.args.empty() ? 0u :
          static_cast<unsigned int>(std::stoul(spv.args[0]));
      std::string texPath = common::findFile(
          asFullPath(spv.value, this->modelPath));
      (*params)[spv.name].SetTexture(texPath,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, uvSetIndex);

      ignmsg << "Shader param [" << spv.name << "]" 
          << ", type: " << spv.type
          << ", tex coord set: " << uvSetIndex << "\n";
    }
    // handle int, float, int_array, and float_array params
    else
    {
      std::vector<std::string> values = common::split(spv.value, " ");

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
  ignmsg << "WavesVisualPrivate::InitTextures\n";

  // ocean tile parameters
  uint32_t N = static_cast<uint32_t>(this->waveParams->CellCount());
  double L   = this->waveParams->TileSize();
  double ux  = this->waveParams->WindVelocity().X();
  double uy  = this->waveParams->WindVelocity().Y();

  auto shader = this->oceanMaterial;
  if (!shader)
  {
    ignerr << "Invalid Ocean Material\n";
    return;
  }

  ignition::rendering::Ogre2ScenePtr ogre2Scene =
    std::dynamic_pointer_cast<ignition::rendering::Ogre2Scene>(
        this->scene);

  ignition::rendering::Ogre2MaterialPtr ogre2Material =
    std::dynamic_pointer_cast<ignition::rendering::Ogre2Material>(
        shader);

  Ogre::SceneManager *ogre2SceneManager = ogre2Scene->OgreSceneManager();

  Ogre::TextureGpuManager *ogre2TextureManager =
      ogre2SceneManager->getDestinationRenderSystem()->getTextureGpuManager();

  // Create empty image
  uint32_t width{N};
  uint32_t height{N};
  uint32_t depthOrSlices{1};
  Ogre::TextureTypes::TextureTypes textureType{
      Ogre::TextureTypes::TextureTypes::Type2D};
  Ogre::PixelFormatGpu format{
      Ogre::PixelFormatGpu::PFG_RGBA32_FLOAT};
  uint8_t numMipmaps{1u};

  ignmsg << "Create HeightMap image\n";
  mHeightMapImage = new Ogre::Image2();
  mHeightMapImage->createEmptyImage(width, height, depthOrSlices,
      textureType, format, numMipmaps);

  ignmsg << "Create NormalMap image\n";
  mNormalMapImage = new Ogre::Image2();
  mNormalMapImage->createEmptyImage(width, height, depthOrSlices,
      textureType, format, numMipmaps);

  ignmsg << "Create TangentMap image\n";
  mTangentMapImage = new Ogre::Image2();
  mTangentMapImage->createEmptyImage(width, height, depthOrSlices,
      textureType, format, numMipmaps);

  ignmsg << "Initialising images\n";
  memset(mHeightMapImage->getRawBuffer(), 0, sizeof(float) * 4 * width * height);
  memset(mNormalMapImage->getRawBuffer(), 0, sizeof(float) * 4 * width * height);
  memset(mTangentMapImage->getRawBuffer(), 0, sizeof(float) * 4 * width * height);

  // Create displacement texture
  ignmsg << "Create HeightMap texture\n";
  mHeightMapTex = ogre2TextureManager->createTexture(
      "HeightMapTex(" + std::to_string(this->entity) + ")",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D );

  mHeightMapTex->setResolution(mHeightMapImage->getWidth(),
      mHeightMapImage->getHeight());
  mHeightMapTex->setPixelFormat(mHeightMapImage->getPixelFormat());
  mHeightMapTex->setNumMipmaps(Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
      mHeightMapTex->getWidth(), mHeightMapTex->getHeight()));

  // Create normal texture
  ignmsg << "Create NormalMap texture\n";
  mNormalMapTex = ogre2TextureManager->createTexture(
      "NormalMapTex(" + std::to_string(this->entity) + ")",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D );

  mNormalMapTex->setResolution(mNormalMapImage->getWidth(),
      mNormalMapImage->getHeight());
  mNormalMapTex->setPixelFormat(mNormalMapImage->getPixelFormat());
  mNormalMapTex->setNumMipmaps(Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
      mNormalMapTex->getWidth(), mNormalMapTex->getHeight()));

  // Create tangent texture
  ignmsg << "Create TangentMap texture\n";
  mTangentMapTex = ogre2TextureManager->createTexture(
      "TangentMapTex(" + std::to_string(this->entity) + ")",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D );

  mTangentMapTex->setResolution(mTangentMapImage->getWidth(),
      mTangentMapImage->getHeight());
  mTangentMapTex->setPixelFormat(mTangentMapImage->getPixelFormat());
  mTangentMapTex->setNumMipmaps(Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
      mTangentMapTex->getWidth(), mTangentMapTex->getHeight()));

  // Set texture on wave material
  ignmsg << "Setting HeightMapTex\n";
  auto mat = ogre2Material->Material();
  auto pass = mat->getTechnique(0u)->getPass(0);

  {
    auto texUnit = pass->getTextureUnitState("heightMap");
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("heightMap");
    }
    texUnit->setTexture(mHeightMapTex);
  }

  {
    auto texUnit = pass->getTextureUnitState("normalMap");
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("normalMap");
    }
    texUnit->setTexture(mNormalMapTex);
  }

  {
    auto texUnit = pass->getTextureUnitState("tangentMap");
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("tangentMap");
    }
    texUnit->setTexture(mTangentMapTex);
  }
}

//////////////////////////////////////////////////
void WavesVisualPrivate::UpdateWaveSim()
{
  double simTime = this->currentSimTimeSeconds;

  mWaveSim->SetTime(simTime);
  mWaveSim->ComputeDisplacementsAndDerivatives(
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

//////////////////////////////////////////////////
void WavesVisualPrivate::UpdateTextures()
{
  ignition::rendering::Ogre2ScenePtr ogre2Scene =
    std::dynamic_pointer_cast<ignition::rendering::Ogre2Scene>(
        this->scene);

  Ogre::SceneManager *ogre2SceneManager = ogre2Scene->OgreSceneManager();

  Ogre::TextureGpuManager *ogre2TextureManager =
      ogre2SceneManager->getDestinationRenderSystem()->getTextureGpuManager();

  // update the image data
  uint32_t width  = mHeightMapImage->getWidth();
  uint32_t height = mHeightMapImage->getHeight();

  Ogre::TextureBox heightBox  = mHeightMapImage->getData(0);
  Ogre::TextureBox normalBox  = mNormalMapImage->getData(0);
  Ogre::TextureBox tangentBox = mTangentMapImage->getData(0);

  for (uint32_t iv=0; iv < height; ++iv)
  {
      /// \todo: coordinates are flipped in the vertex shader
      // texture index to vertex index
      int32_t iy = /*height - 1 - */ iv;
      for (uint32_t iu=0; iu < width; ++iu)
      {
          // texture index to vertex index
          int32_t ix = /* width - 1 - */ iu;

          float Dx{0.0}, Dy{0.0}, Dz{0.0};
          float Tx{1.0}, Ty{0.0}, Tz{0.0};
          float Bx{0.0}, By{1.0}, Bz{0.0};
          float Nx{0.0}, Ny{0.0}, Nz{1.0};

          int32_t idx = iy * width + ix;
          double h  = mHeights[idx];
          double sx = mDisplacementsX[idx];
          double sy = mDisplacementsY[idx];
          double dhdx  = mDhdx[idx]; 
          double dhdy  = mDhdy[idx]; 
          double dsxdx = mDxdx[idx]; 
          double dsydy = mDydy[idx]; 
          double dsxdy = mDxdy[idx]; 

          // vertex displacements
          Dx += sy;
          Dy += sx;
          Dz  = h;

          // tangents
          Tx = dsydy + 1.0;
          Ty = dsxdy;
          Tz = dhdy;

          // bitangents
          Bx = dsxdy;
          By = dsxdx + 1.0;
          Bz = dhdx;

          // normals N = T x B
          Nx = 1.0 * (Ty*Bz - Tz*Bx);
          Ny = 1.0 * (Tz*Bx - Tx*Bz);
          Nz = 1.0 * (Tx*By - Ty*Bx);

          heightBox.setColourAt(Ogre::ColourValue(Dx, Dy, Dz, 0.0), iu, iv, 0,
              mHeightMapImage->getPixelFormat());
          normalBox.setColourAt(Ogre::ColourValue(Nx, Ny, Nz, 0.0), iu, iv, 0,
              mNormalMapImage->getPixelFormat());
          tangentBox.setColourAt(Ogre::ColourValue(Tx, Ty, Tz, 0.0), iu, iv, 0,
              mTangentMapImage->getPixelFormat());
      }
  }

  // schedule update to GPU
  mHeightMapTex->scheduleTransitionTo(Ogre::GpuResidency::Resident);
  mNormalMapTex->scheduleTransitionTo(Ogre::GpuResidency::Resident);
  mTangentMapTex->scheduleTransitionTo(Ogre::GpuResidency::Resident);

  // Staging texture is required for upload from CPU -> GPU
  {
    Ogre::StagingTexture *stagingTexture = ogre2TextureManager->getStagingTexture(
        mHeightMapImage->getWidth(), mHeightMapImage->getHeight(), 1u, 1u, mHeightMapImage->getPixelFormat());
    stagingTexture->startMapRegion();
    Ogre::TextureBox texBox = stagingTexture->mapRegion(
        mHeightMapImage->getWidth(), mHeightMapImage->getHeight(), 1u, 1u, mHeightMapImage->getPixelFormat());

    texBox.copyFrom(mHeightMapImage->getData(0));
    stagingTexture->stopMapRegion();
    stagingTexture->upload(texBox, mHeightMapTex, 0, 0, 0);
    ogre2TextureManager->removeStagingTexture(stagingTexture);
    stagingTexture = nullptr;
    mHeightMapTex->notifyDataIsReady();
  }

  {
    Ogre::StagingTexture *stagingTexture = ogre2TextureManager->getStagingTexture(
        mNormalMapImage->getWidth(), mNormalMapImage->getHeight(), 1u, 1u, mNormalMapImage->getPixelFormat());
    stagingTexture->startMapRegion();
    Ogre::TextureBox texBox = stagingTexture->mapRegion(
        mNormalMapImage->getWidth(), mNormalMapImage->getHeight(), 1u, 1u, mNormalMapImage->getPixelFormat());

    texBox.copyFrom(mNormalMapImage->getData(0));
    stagingTexture->stopMapRegion();
    stagingTexture->upload(texBox, mNormalMapTex, 0, 0, 0);
    ogre2TextureManager->removeStagingTexture(stagingTexture);
    stagingTexture = nullptr;
    mNormalMapTex->notifyDataIsReady();
  }

  {
    Ogre::StagingTexture *stagingTexture = ogre2TextureManager->getStagingTexture(
        mTangentMapImage->getWidth(), mTangentMapImage->getHeight(), 1u, 1u, mTangentMapImage->getPixelFormat());
    stagingTexture->startMapRegion();
    Ogre::TextureBox texBox = stagingTexture->mapRegion(
        mTangentMapImage->getWidth(), mTangentMapImage->getHeight(), 1u, 1u, mTangentMapImage->getPixelFormat());

    texBox.copyFrom(mTangentMapImage->getData(0));
    stagingTexture->stopMapRegion();
    stagingTexture->upload(texBox, mTangentMapTex, 0, 0, 0);
    ogre2TextureManager->removeStagingTexture(stagingTexture);
    stagingTexture = nullptr;
    mTangentMapTex->notifyDataIsReady();
  }
}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(WavesVisual,
                    ignition::gazebo::System,
                    WavesVisual::ISystemConfigure,
                    WavesVisual::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WavesVisual,
  "ignition::gazebo::systems::WavesVisual")
