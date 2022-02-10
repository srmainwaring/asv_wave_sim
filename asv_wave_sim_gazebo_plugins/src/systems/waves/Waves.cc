
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

  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshExt :
      public Ogre2Mesh
  {
    public: virtual ~Ogre2MeshExt() {}

    protected: explicit Ogre2MeshExt() : Ogre2Mesh() {}

    protected: void SetOgreItem(Ogre::Item *_ogreItem)
    {
      this->ogreItem = _ogreItem;
    }

    // Work-around the protected accessors and protected methods in Scene
    public: void InitObject(Ogre2ScenePtr _scene, unsigned int _id, const std::string &_name)
    {
      // would like to call these methods
      // unsigned int objId = _scene->CreateObjectId();
      // std::string objName = _scene->CreateObjectName(objId, "Mesh-" + meshName);
      this->id = _id;
      this->name = _name;
      this->scene = _scene;

      // initialize object
      this->Load();
      this->Init();
    }

    private: friend class Ogre2MeshFactoryExt;
  };
  typedef std::shared_ptr<Ogre2MeshExt> Ogre2MeshExtPtr;

  class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshFactoryExt :
      public Ogre2MeshFactory
  {
    public: virtual ~Ogre2MeshFactoryExt() {}

    public: explicit Ogre2MeshFactoryExt(Ogre2ScenePtr _scene) :
        Ogre2MeshFactory(_scene) {}

    public: virtual Ogre2MeshPtr Create(const MeshDescriptor &_desc) override
    {
      // Override dropping the attempt to load the common::Mesh

      // create ogre entity
      Ogre2MeshExtPtr mesh(new Ogre2MeshExt);
      MeshDescriptor normDesc = _desc;
      
      // Override MeshDescriptor behaviour as we're not using common::Mesh
      // normDesc.Load();

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

    public: virtual bool Validate(const MeshDescriptor &_desc) override
    {
      if (!_desc.mesh && _desc.meshName.empty())
      {
        ignerr << "Invalid mesh-descriptor, no mesh specified" << std::endl;
        return false;
      }

      if (!_desc.mesh)
      {
        // Override MeshDescriptor behaviour as we're not using common::Mesh
        // ignerr << "Cannot load null mesh [" << _desc.meshName << "]" << std::endl;
        // return false;
        return true;
      }

      if (_desc.mesh->SubMeshCount() == 0)
      {
        ignerr << "Cannot load mesh with zero sub-meshes" << std::endl;
        return false;
      }

      return true;
    }
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
  std::string mAboveOceanMeshName = "AboveOceanTileMesh";
  std::string mBelowOceanMeshName = "BelowOceanTileMesh";

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

#if 0
  // Test attaching an Ogre2 mesh to the entity
  if (!this->ogre2OceanTile)
  {
    ignmsg << "Waves: creating Ogre2 Ocean Tile\n";

    // \todo(srmainwaring) synchronise visual with physics...
    // int N = 128;
    // double L = 256.0;
    int N = 2;
    double L = 256.0;
    double u = 5.0;

    this->ogre2OceanTile.reset(new rendering::Ogre2OceanTile(N, L));
    this->ogre2OceanTile->SetWindVelocity(u, 0.0);
    this->ogre2OceanTile->Create();
    this->ogre2OceanTile->Update(0.0);

    // ogre2 specific
    rendering::Ogre2ScenePtr ogre2Scene =
        std::dynamic_pointer_cast<rendering::Ogre2Scene>(this->scene);    

    auto ogre2MeshFactory = rendering::Ogre2MeshFactoryExtPtr(
          new rendering::Ogre2MeshFactoryExt(ogre2Scene));

    rendering::MeshDescriptor meshDescriptor;
    meshDescriptor.mesh = nullptr;
    meshDescriptor.meshName = this->mAboveOceanMeshName;

    // \todo: move to derived class Ogre2MeshFactoryExt and override
    // Replace std::string Ogre2MeshFactory::MeshName(const MeshDescriptor &_desc);
    auto MeshNameFunction = [&](const rendering::MeshDescriptor &_desc) -> std::string
    {
      std::stringstream ss;
      ss << _desc.meshName << "::";
      ss << _desc.subMeshName << "::";
      ss << ((_desc.centerSubMesh) ? "CENTERED" : "ORIGINAL");
      return ss.str();
    };

    ignmsg << "Mesh name: " << MeshNameFunction(meshDescriptor) << "\n";

    ignmsg << "Create Ogre::Item\n";

    // \todo: move to derived class Ogre2MeshFactoryExt and override
    // Replace Ogre::Item *Ogre2MeshFactory::OgreItem(const MeshDescriptor &_desc)
    // Required to ensure Ogre v1 mesh is imported to Ogre v2
    auto OgreItemFunction = [&](const rendering::MeshDescriptor &_desc) -> Ogre::Item *
    {
      // if (!this->Load(_desc))
      // {
      //   return nullptr;
      // }

      std::string name = MeshNameFunction(_desc);
      ignmsg << "Get Ogre::SceneManager\n";
      Ogre::SceneManager *sceneManager = ogre2Scene->OgreSceneManager();

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
          ignerr << "Did not find v1 mesh\n";
          // return nullptr
        // create v2 mesh from v1

        // Examine v1 mesh properties - this is a bit hacky to prevent a segfault if `load` is called
        // v1Mesh->load(true);
        v1Mesh->setToLoaded();
        // ignmsg << "v1 mesh: calculateSize:  " << v1Mesh->calculateSize() << "\n"; // protected
        ignmsg << "v1 mesh: isLoaded:       " << v1Mesh->isLoaded() << "\n";

        ignmsg << "Creating v2 mesh\n";
        mesh = Ogre::MeshManager::getSingleton().createManual(
            name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        ignmsg << "Importing v2 mesh\n";
        mesh->importV1(v1Mesh.get(), false, true, true);
        // this->ogreMeshes.push_back(name);
      }

      return sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
    };

    ignmsg << "Creating ogre2 item\n";
    auto ogreItem = OgreItemFunction(meshDescriptor);
    if (!ogreItem)
      ignerr << "Failed to get ogre2 item for [" << MeshNameFunction(meshDescriptor) << "]\n";

    ignmsg << "Creating ogre2 mesh\n";
    auto ogre2Mesh = ogre2MeshFactory->Create(meshDescriptor);
    if (!ogre2Mesh)
      ignerr << "Failed create ogre2 mesh for [" << MeshNameFunction(meshDescriptor) << "]\n";

    // Equivalent to Ogre2Scene::InitObject 
    unsigned int objId = 50000;
    std::string objName = "ocean-tile";
    rendering::Ogre2MeshExtPtr ogre2MeshExt =
        std::dynamic_pointer_cast<rendering::Ogre2MeshExt>(ogre2Mesh);
    ogre2MeshExt->InitObject(ogre2Scene, objId, objName);

    // attach mesh to visuals
    auto visual = this->scene->CreateVisual("ocean-tile");
    visual->AddGeometry(ogre2MeshExt);
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
#endif

#if 1
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
    this->oceanTile->Update(0.0);

    // create the common::Mesh
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
    // auto parent = this->visual->Parent();
    // parent->AddChild(visual);

    // keep reference
    this->oceanVisual = visual;
  }

  if (!this->oceanTile)
    return;

  // Update the tile
  this->oceanTile->Update(simTime);
#endif

}

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(Waves,
                    ignition::gazebo::System,
                    Waves::ISystemConfigure,
                    Waves::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Waves,
  "ignition::gazebo::systems::Waves")
