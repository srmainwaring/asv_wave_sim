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

#include "Waves.hh"

#include "OceanTile.hh"
#include "Ogre2OceanTile.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/ShaderParams.hh>
#include <ignition/rendering/Visual.hh>

#include <ignition/rendering/Grid.hh>

#include <ignition/rendering/ogre2.hh>
#include <ignition/rendering/ogre2/Ogre2MeshFactory.hh>
#include <ignition/rendering/ogre2/Ogre2Scene.hh>

#include <ignition/gazebo/components/Name.hh>
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

class ignition::gazebo::systems::WavesPrivate
{
  /// \brief Path to the model
  public: std::string modelPath;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection{nullptr};

  /// \brief Name of visual this plugin is attached to
  public: std::string visualName;

  /// \brief Pointer to visual
  public: rendering::VisualPtr visual;

  /// \brief Pointer to ocean visual
  public: rendering::VisualPtr oceanVisual;

  /// \brief Material used by this visual
  public: rendering::MaterialPtr material;

  /// \brief Pointer to scene
  public: rendering::ScenePtr scene;

  /// \brief Entity id of the visual
  public: Entity entity = kNullEntity;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;

  /////////////////
  /// OceanTile (both common::Mesh and Ogre2 versions)
  // std::string mAboveOceanMeshName = "AboveOceanTileMesh";
  // std::string mBelowOceanMeshName = "BelowOceanTileMesh";

  // using custom rendering::Ogre2Mesh
  public: std::unique_ptr<rendering::Ogre2OceanTile> ogre2OceanTile;

  // using standard (static) common::Mesh
  public: std::unique_ptr<rendering::OceanTile> oceanTile;

  /// \brief Destructor
  public: ~WavesPrivate();

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();
};

/////////////////////////////////////////////////
Waves::Waves()
    : System(), dataPtr(std::make_unique<WavesPrivate>())
{
}

/////////////////////////////////////////////////
Waves::~Waves()
{
}

/////////////////////////////////////////////////
void Waves::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  IGN_PROFILE("Waves::Configure");

  ignmsg << "Waves: configuring\n";

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  // capture entity 
  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<ignition::gazebo::events::SceneUpdate>(
      std::bind(&WavesPrivate::OnUpdate, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void Waves::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &)
{
  IGN_PROFILE("Waves::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
}

//////////////////////////////////////////////////
WavesPrivate::~WavesPrivate()
{
  if (this->oceanVisual != nullptr)
  {
    this->oceanVisual->Destroy();
    this->oceanVisual.reset();
  }
};

//////////////////////////////////////////////////
void WavesPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;

  if (!this->scene)
  {
    ignmsg << "Waves: retrieving scene from render engine\n";
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene)
    return;

  if (!this->visual)
  {
    ignmsg << "Waves: searching for visual\n";

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
        // RenderUtil stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Ignition H?
        auto variant = n->UserData("gazebo-entity");
        const int *value = std::get_if<int>(&variant);
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
    ignmsg << "Waves: creating material `OceanBlue`\n";

    auto mat = this->scene->CreateMaterial("OceanBlue");
    mat->SetAmbient(0.0, 0.0, 0.3);
    mat->SetDiffuse(0.0, 0.0, 0.8);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

#if 0
  // Test attaching another visual to the entity
  if (!this->oceanVisual)
  {
    ignmsg << "Waves: creating ocean visual\n";

    // create plane
    auto geometry = this->scene->CreatePlane();

    // create visual
    auto visual = this->scene->CreateVisual("ocean-tile");
    visual->AddGeometry(geometry);
    visual->SetLocalPosition(0.0, 0.0, 0.0);
    visual->SetLocalRotation(0.0, 0.0, 0.0);
    visual->SetLocalScale(100.0, 100.0, 100.0);
    visual->SetMaterial("OceanBlue");

    // add visual to parent
    auto parent = this->visual->Parent();
    parent->AddChild(visual);

    // keep reference
    this->oceanVisual = visual;
  }

  if (!this->oceanVisual)
    return;
#endif

  double simTime = (std::chrono::duration_cast<std::chrono::nanoseconds>(
      this->currentSimTime).count()) * 1e-9;

#if 1
  // Test attaching an Ogre2 mesh to the entity
  if (!this->ogre2OceanTile)
  {
    ignmsg << "Waves: creating Ogre2 Ocean Tile\n";

    // \todo(srmainwaring) synchronise visual with physics...
    int N = 128;
    double L = 256.0;
    double u = 5.0;

    // create static ocean tile using common::mesh - this to populate
    // an object in the MeshManager with the correct name
    this->oceanTile.reset(new rendering::OceanTile(N, L));
    this->oceanTile->SetWindVelocity(u, 0.0);
    this->oceanTile->Create();
    common::Mesh *mesh = this->oceanTile->Mesh();
    ignmsg << "Waves: mesh name " << mesh->Name() << "\n";
    ignmsg << "Waves: mesh resource path " << mesh->Path() << "\n";

    this->ogre2OceanTile.reset(new rendering::Ogre2OceanTile(N, L));
    this->ogre2OceanTile->SetWindVelocity(u, 0.0);
    this->ogre2OceanTile->Create();
    // this->ogre2OceanTile->Update(0.0);

    // ogre2 specific
    rendering::Ogre2ScenePtr ogre2Scene =
        std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);    

    auto meshFactory = rendering::Ogre2MeshFactoryExtPtr(
          new rendering::Ogre2MeshFactoryExt(ogre2Scene));

    rendering::MeshDescriptor meshDescriptor;
    meshDescriptor.mesh = mesh;
    meshDescriptor.meshName = "AboveOceanTileMesh"; // mesh->Name();

    /////////////////////////////////////////////
    // BEGIN_LAMDAS
    //
    // Override functions from Scene, BaseScene, 
    // and Ogre2Scene
    //
    /////////////////////////////////////////////

    // bool Ogre2Scene::InitObject(Ogre2ObjectPtr _object, unsigned int _id,
    //     const std::string &_name)
    auto Ogre2Scene_InitObject = [&](
        rendering::Ogre2ScenePtr _scene,
        rendering::Ogre2ObjectPtr _object,
        unsigned int _id,
        const std::string &_name) -> bool
    {
      // assign needed varibles
      // _object->id = _id;
      // _object->name = _name;
      // _object->scene = this->SharedThis();

      // // initialize object
      // _object->Load();
      // _object->Init();

      rendering::Ogre2MeshExtPtr derived =
          std::dynamic_pointer_cast<rendering::Ogre2MeshExt>(_object);

      if (!derived)
        return false;
      else
        derived->InitObject(_scene, _id, _name);
  
      return true;
    };

    // Ogre2Scene::CreateMeshImpl(unsigned int _id,
    //     const std::string &_name, const MeshDescriptor &_desc) -> rendering::MeshPtr 
    auto Ogre2Scene_CreateMeshImpl = [&](
        rendering::Ogre2ScenePtr _scene,
        unsigned int _id,
        const std::string &_name,
        const rendering::MeshDescriptor &_desc)
        -> rendering::MeshPtr
    {
      // rendering::Ogre2MeshPtr mesh = this->meshFactory->Create(_desc);
      rendering::Ogre2MeshPtr mesh = meshFactory->Create(_desc);
      if (nullptr == mesh)
        return nullptr;
      mesh->SetDescriptor(_desc);

      // bool result = this->InitObject(mesh, _id, _name);
      bool result = Ogre2Scene_InitObject(_scene, mesh, _id, _name);
      return (result) ? mesh : nullptr;
    };

    // BaseScene::CreateMesh(const MeshDescriptor &_desc) -> rendering::MeshPtr 
    auto BaseScene_CreateMesh = [&](
        rendering::Ogre2ScenePtr _scene,
        const rendering::MeshDescriptor &_desc)
        -> rendering::MeshPtr 
    {
      std::string meshName = (_desc.mesh) ?
          _desc.mesh->Name() : _desc.meshName;

      // \todo: implement equivalents
      // unsigned int objId = this->CreateObjectId();
      // std::string objName = this->CreateObjectName(objId, "Mesh-" + meshName);
      unsigned int objId = 50000;
      std::string objName = "Mesh-" + meshName;
      // return this->CreateMeshImpl(objId, objName, _desc);
      return Ogre2Scene_CreateMeshImpl(_scene, objId, objName, _desc);
    };

    /////////////////////////////////////////////
    // END_LAMDAS
    /////////////////////////////////////////////

    // create the rendering mesh
    rendering::MeshPtr renderingMesh = BaseScene_CreateMesh(
        ogre2Scene, meshDescriptor);

    // attach mesh to visuals
    auto visual = this->scene->CreateVisual("ocean-tile");
    visual->AddGeometry(renderingMesh);
    visual->SetLocalPosition(0.0, 0.0, 0.0);
    visual->SetLocalRotation(0.0, 0.0, 0.0);
    visual->SetLocalScale(1.0, 1.0, 1.0);
    visual->SetMaterial("OceanBlue");
    visual->SetVisible(true);

    // add visual to parent
    auto parent = this->visual->Parent();
    parent->AddChild(visual);

    // keep reference
    this->oceanVisual = visual;
  }

  if (!this->ogre2OceanTile)
    return;

  // Update the tile
  this->ogre2OceanTile->Update(simTime);

#else

  // Test attaching a common::Mesh to the entity
  if (!this->oceanTile)
  {
    ignmsg << "Waves: creating Ocean Tile\n";

    // create ocean tile
    int N = 128;
    double L = 256.0;
    double u = 5.0;

    this->oceanTile.reset(new rendering::OceanTile(N, L));
    this->oceanTile->SetWindVelocity(u, 0.0);
    this->oceanTile->Create();
    // this->oceanTile->Update(0.0);

    // get the common::Mesh
    common::Mesh *mesh = this->oceanTile->Mesh();

    //convert common::Mesh to rendering::Mesh
    auto geometry = this->scene->CreateMesh(mesh);

    // create ocean tile visual
    auto visual = this->scene->CreateVisual("ocean-tile");
    visual->AddGeometry(geometry);
    visual->SetLocalPosition(0.0, 0.0, 0.0);
    visual->SetLocalRotation(0.0, 0.0, 0.0);
    visual->SetLocalScale(1.0, 1.0, 1.0);
    // visual->SetMaterial("OceanBlue");
    visual->SetVisible(true);

    // retrive the material from the visual's geometry (it's not set on the visual)
    ignmsg << "Waves: Visual Name:          " << this->visual->Name() << "\n";
    ignmsg << "Waves: Visual GeometryCount: " << this->visual->GeometryCount() << "\n";
    auto visualGeometry = this->visual->GeometryByIndex(0);

    auto material = visualGeometry->Material();
    // auto material = this->visual->Material();
    if (!material)
      ignerr << "Waves: invalid material\n";
    else
      visual->SetMaterial(material);

    // add visual to parent
    auto parent = this->visual->Parent();
    parent->AddChild(visual);

    // keep reference
    this->oceanVisual = visual;
  }

  if (!this->oceanTile)
    return;

  // Update the tile
  // this->oceanTile->Update(simTime);
#endif

}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(Waves,
                    ignition::gazebo::System,
                    Waves::ISystemConfigure,
                    Waves::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Waves,
  "ignition::gazebo::systems::Waves")
