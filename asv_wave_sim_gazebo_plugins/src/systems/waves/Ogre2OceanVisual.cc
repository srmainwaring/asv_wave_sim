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

  /// \brief Ogre renderable used to render the ocean tile.
  public: std::shared_ptr<Ogre2DynamicRenderable> tile = nullptr;
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
  if (this->dataPtr->tile)
  {
    this->dataPtr->tile->Destroy();
    this->dataPtr->tile.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadCube()
{
  if (!this->dataPtr->tile)
  {
    this->dataPtr->tile.reset(
      new Ogre2DynamicRenderable(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->tile->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->tile->Clear();
  this->dataPtr->tile->Update();

  this->dataPtr->tile->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
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
  this->dataPtr->tile->AddPoint(p0);
  this->dataPtr->tile->AddPoint(p1);
  this->dataPtr->tile->AddPoint(p2);
  this->dataPtr->tile->AddPoint(p2);
  this->dataPtr->tile->AddPoint(p3);
  this->dataPtr->tile->AddPoint(p0);

  // back face
  this->dataPtr->tile->AddPoint(p6);
  this->dataPtr->tile->AddPoint(p5);
  this->dataPtr->tile->AddPoint(p4);
  this->dataPtr->tile->AddPoint(p4);
  this->dataPtr->tile->AddPoint(p7);
  this->dataPtr->tile->AddPoint(p6);

  // top face
  this->dataPtr->tile->AddPoint(p3);
  this->dataPtr->tile->AddPoint(p2);
  this->dataPtr->tile->AddPoint(p6);
  this->dataPtr->tile->AddPoint(p6);
  this->dataPtr->tile->AddPoint(p7);
  this->dataPtr->tile->AddPoint(p3);

  // bottom face
  this->dataPtr->tile->AddPoint(p5);
  this->dataPtr->tile->AddPoint(p1);
  this->dataPtr->tile->AddPoint(p0);
  this->dataPtr->tile->AddPoint(p0);
  this->dataPtr->tile->AddPoint(p4);
  this->dataPtr->tile->AddPoint(p5);

  // left face
  this->dataPtr->tile->AddPoint(p4);
  this->dataPtr->tile->AddPoint(p0);
  this->dataPtr->tile->AddPoint(p3);
  this->dataPtr->tile->AddPoint(p3);
  this->dataPtr->tile->AddPoint(p7);
  this->dataPtr->tile->AddPoint(p4);

  // right face
  this->dataPtr->tile->AddPoint(p6);
  this->dataPtr->tile->AddPoint(p2);
  this->dataPtr->tile->AddPoint(p1);
  this->dataPtr->tile->AddPoint(p1);
  this->dataPtr->tile->AddPoint(p5);
  this->dataPtr->tile->AddPoint(p6);

  this->dataPtr->tile->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadOceanTile(OceanTilePtr _oceanTile)
{
  if (!this->dataPtr->tile)
  {
    this->dataPtr->tile.reset(
      new Ogre2DynamicRenderable(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->tile->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->tile->Clear();
  this->dataPtr->tile->Update();

  this->dataPtr->tile->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // Add points for each face
  for (auto i=0; i < _oceanTile->FaceCount(); ++i)
  {
    auto face = _oceanTile->Face(i);
    this->dataPtr->tile->AddPoint(_oceanTile->Vertex(face.X()));
    this->dataPtr->tile->AddPoint(_oceanTile->Vertex(face.Y()));
    this->dataPtr->tile->AddPoint(_oceanTile->Vertex(face.Z()));
  }

  this->dataPtr->tile->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::UpdateOceanTile(OceanTilePtr _oceanTile)
{
  // Update points for each face
  for (auto i=0, v=0; i < _oceanTile->FaceCount(); ++i)
  {
    auto face = _oceanTile->Face(i);
    this->dataPtr->tile->SetPoint(v++, _oceanTile->Vertex(face.X()));
    this->dataPtr->tile->SetPoint(v++, _oceanTile->Vertex(face.Y()));
    this->dataPtr->tile->SetPoint(v++, _oceanTile->Vertex(face.Z()));
  }

  this->dataPtr->tile->Update();
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

  this->dataPtr->tile->SetMaterial(_material, false);
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
