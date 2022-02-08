
#include "Waves.hh"

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/ShaderParams.hh>
#include <ignition/rendering/Visual.hh>

#include <ignition/rendering/Grid.hh>
#include <ignition/rendering/COMVisual.hh>

#include "ignition/rendering/OceanVisual.hh"

#include <sdf/Element.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/gazebo/Util.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::WavesPrivate
{
  /// \brief Data structure for storing param info
  public: class WavesParamValue
  {
  };

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

  if (!this->oceanVisual)
  {
    ignmsg << "Waves: creating ocean visual\n";

    // create material
    if (!this->scene->MaterialRegistered("Blue"))
    {
      auto mat = this->scene->CreateMaterial("Blue");
      mat->SetAmbient(0.0, 0.0, 0.3);
      mat->SetDiffuse(0.0, 0.0, 0.8);
      mat->SetSpecular(0.8, 0.8, 0.8);
      mat->SetShininess(50);
      mat->SetReflectivity(0);
    }

    // create plane
    // auto geometry = this->scene->CreatePlane()

    // create grid (a dynamic renderable)
    auto geometry = this->scene->CreateGrid();
    geometry->SetCellCount(100);
    geometry->SetCellLength(0.01);
    geometry->SetVerticalCellCount(0);

    // create visual
    auto visual = this->scene->CreateVisual("ocean-tile");
    visual->AddGeometry(geometry);
    visual->SetLocalPosition(0.0, 0.0, 0.0);
    visual->SetLocalRotation(0.0, 0.0, 0.0);
    visual->SetLocalScale(100.0, 100.0, 100.0);
    visual->SetMaterial("Blue");

    // add visual to parent
    auto parent = this->visual->Parent();
    parent->AddChild(visual);

    // keep reference
    this->oceanVisual = visual;
  }

  if (!this->oceanVisual)
    return;

  // get the material and set shaders
  if (!this->material)
  {
    ignmsg << "Waves: creating material\n";

    auto mat = scene->CreateMaterial();
    // mat->SetVertexShader(this->vertexShaderUri);
    // mat->SetFragmentShader(this->fragmentShaderUri);
    this->visual->SetMaterial(mat);
    scene->DestroyMaterial(mat);
    this->material = this->visual->Material();
  }

  if (!this->material)
    return;

}

#if 0
//////////////////////////////////////////////////
COMVisualPtr BaseScene::CreateCOMVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateCOMVisual(objId, _name);
}

COMVisualPtr BaseScene::CreateCOMVisual(unsigned int _id,
    const std::string &_name)
{
  COMVisualPtr visual = this->CreateCOMVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
COMVisualPtr Ogre2Scene::CreateCOMVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2COMVisualPtr visual(new Ogre2COMVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}
#endif

//////////////////////////////////////////////////
IGNITION_ADD_PLUGIN(Waves,
                    ignition::gazebo::System,
                    Waves::ISystemConfigure,
                    Waves::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Waves,
  "ignition::gazebo::systems::Waves")
