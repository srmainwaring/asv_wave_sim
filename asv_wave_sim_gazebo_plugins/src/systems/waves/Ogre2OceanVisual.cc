#include "Ogre2OceanVisual.hh"

#include <ignition/rendering/ogre2/Ogre2Material.hh>
#include <ignition/rendering/ogre2/Ogre2DynamicRenderable.hh>

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneNode.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace ignition;
using namespace rendering;

class ignition::rendering::Ogre2OceanVisualPrivate
{
  /// \brief visual materal
  public: Ogre2MaterialPtr material = nullptr;

  /// \brief Ogre renderable used to render the ocean tile lines.
  public: std::shared_ptr<Ogre2DynamicRenderable> oceanTile = nullptr;
};

//////////////////////////////////////////////////
Ogre2OceanVisual::Ogre2OceanVisual()
  : dataPtr(std::make_unique<Ogre2OceanVisualPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2OceanVisual::~Ogre2OceanVisual()
{
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::PreRender()
{
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::Init()
{
  BaseVisual::Init();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::Destroy()
{
  if (this->dataPtr->oceanTile)
  {
    this->dataPtr->oceanTile->Destroy();
    this->dataPtr->oceanTile.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::Load2()
{
  if (!this->dataPtr->oceanTile)
  {
    this->dataPtr->oceanTile.reset(
      new Ogre2DynamicRenderable(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->oceanTile->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->oceanTile->Clear();
  this->dataPtr->oceanTile->Update();

  this->dataPtr->oceanTile->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // Position indicator - with dynamic renderable
  // must specify vertices for each face and get
  // the orientation correct.
  ignition::math::Vector3d p0(-1, -1,  1);
  ignition::math::Vector3d p1( 1, -1,  1);
  ignition::math::Vector3d p2( 1,  1,  1);
  ignition::math::Vector3d p3(-1,  1,  1);
  ignition::math::Vector3d p4(-1, -1, -1);
  ignition::math::Vector3d p5( 1, -1, -1);
  ignition::math::Vector3d p6( 1,  1, -1);
  ignition::math::Vector3d p7(-1,  1, -1);

  // front face
  this->dataPtr->oceanTile->AddPoint(p0);
  this->dataPtr->oceanTile->AddPoint(p1);
  this->dataPtr->oceanTile->AddPoint(p2);
  this->dataPtr->oceanTile->AddPoint(p2);
  this->dataPtr->oceanTile->AddPoint(p3);
  this->dataPtr->oceanTile->AddPoint(p0);

  // back face
  this->dataPtr->oceanTile->AddPoint(p6);
  this->dataPtr->oceanTile->AddPoint(p5);
  this->dataPtr->oceanTile->AddPoint(p4);
  this->dataPtr->oceanTile->AddPoint(p4);
  this->dataPtr->oceanTile->AddPoint(p7);
  this->dataPtr->oceanTile->AddPoint(p6);

  // top face
  this->dataPtr->oceanTile->AddPoint(p3);
  this->dataPtr->oceanTile->AddPoint(p2);
  this->dataPtr->oceanTile->AddPoint(p6);
  this->dataPtr->oceanTile->AddPoint(p6);
  this->dataPtr->oceanTile->AddPoint(p7);
  this->dataPtr->oceanTile->AddPoint(p3);

  // bottom face
  this->dataPtr->oceanTile->AddPoint(p5);
  this->dataPtr->oceanTile->AddPoint(p1);
  this->dataPtr->oceanTile->AddPoint(p0);
  this->dataPtr->oceanTile->AddPoint(p0);
  this->dataPtr->oceanTile->AddPoint(p4);
  this->dataPtr->oceanTile->AddPoint(p5);

  // left face
  this->dataPtr->oceanTile->AddPoint(p4);
  this->dataPtr->oceanTile->AddPoint(p0);
  this->dataPtr->oceanTile->AddPoint(p3);
  this->dataPtr->oceanTile->AddPoint(p3);
  this->dataPtr->oceanTile->AddPoint(p7);
  this->dataPtr->oceanTile->AddPoint(p4);

  // right face
  this->dataPtr->oceanTile->AddPoint(p6);
  this->dataPtr->oceanTile->AddPoint(p2);
  this->dataPtr->oceanTile->AddPoint(p1);
  this->dataPtr->oceanTile->AddPoint(p1);
  this->dataPtr->oceanTile->AddPoint(p5);
  this->dataPtr->oceanTile->AddPoint(p6);

  this->dataPtr->oceanTile->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  Ogre2MaterialPtr derived =
      std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  this->dataPtr->oceanTile->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2OceanVisual::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::InitObject(Ogre2ScenePtr _scene,
    unsigned int _id, const std::string &_name)
{
  this->id = _id;
  this->name = _name;
  this->scene = _scene;

  // initialize object
  this->Load();
  this->Init();
}
