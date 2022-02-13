#include "Ogre2OceanVisual.hh"
#include "Ogre2DynamicMesh.hh"

#include <ignition/common.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/rendering/ogre2/Ogre2Material.hh>

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
  public: std::shared_ptr<Ogre2DynamicMesh> mesh = nullptr;
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
  if (this->dataPtr->mesh)
  {
    this->dataPtr->mesh->Destroy();
    this->dataPtr->mesh.reset();
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
  if (!this->dataPtr->mesh)
  {
    this->dataPtr->mesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->mesh->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->mesh->Clear();
  this->dataPtr->mesh->Update();

  this->dataPtr->mesh->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // Position indicator - with dynamic geometry
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
  this->dataPtr->mesh->AddPoint(p0);
  this->dataPtr->mesh->AddPoint(p1);
  this->dataPtr->mesh->AddPoint(p2);
  this->dataPtr->mesh->AddPoint(p2);
  this->dataPtr->mesh->AddPoint(p3);
  this->dataPtr->mesh->AddPoint(p0);

  // back face
  this->dataPtr->mesh->AddPoint(p6);
  this->dataPtr->mesh->AddPoint(p5);
  this->dataPtr->mesh->AddPoint(p4);
  this->dataPtr->mesh->AddPoint(p4);
  this->dataPtr->mesh->AddPoint(p7);
  this->dataPtr->mesh->AddPoint(p6);

  // top face
  this->dataPtr->mesh->AddPoint(p3);
  this->dataPtr->mesh->AddPoint(p2);
  this->dataPtr->mesh->AddPoint(p6);
  this->dataPtr->mesh->AddPoint(p6);
  this->dataPtr->mesh->AddPoint(p7);
  this->dataPtr->mesh->AddPoint(p3);

  // bottom face
  this->dataPtr->mesh->AddPoint(p5);
  this->dataPtr->mesh->AddPoint(p1);
  this->dataPtr->mesh->AddPoint(p0);
  this->dataPtr->mesh->AddPoint(p0);
  this->dataPtr->mesh->AddPoint(p4);
  this->dataPtr->mesh->AddPoint(p5);

  // left face
  this->dataPtr->mesh->AddPoint(p4);
  this->dataPtr->mesh->AddPoint(p0);
  this->dataPtr->mesh->AddPoint(p3);
  this->dataPtr->mesh->AddPoint(p3);
  this->dataPtr->mesh->AddPoint(p7);
  this->dataPtr->mesh->AddPoint(p4);

  // right face
  this->dataPtr->mesh->AddPoint(p6);
  this->dataPtr->mesh->AddPoint(p2);
  this->dataPtr->mesh->AddPoint(p1);
  this->dataPtr->mesh->AddPoint(p1);
  this->dataPtr->mesh->AddPoint(p5);
  this->dataPtr->mesh->AddPoint(p6);

  this->dataPtr->mesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadOceanTile(OceanTilePtr _oceanTile)
{
  if (!this->dataPtr->mesh)
  {
    this->dataPtr->mesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->mesh->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->mesh->Clear();
  this->dataPtr->mesh->Update();

  this->dataPtr->mesh->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // Add points and texture coordinates for each face
  for (auto i=0, v=0; i < _oceanTile->FaceCount(); i++, v+=3)
  {
    auto face = _oceanTile->Face(i);
    // positions
    this->dataPtr->mesh->AddPoint(_oceanTile->Vertex(face.X()));
    this->dataPtr->mesh->AddPoint(_oceanTile->Vertex(face.Y()));
    this->dataPtr->mesh->AddPoint(_oceanTile->Vertex(face.Z()));

    // uv0s
    this->dataPtr->mesh->SetUV0(v+0, _oceanTile->UV0(face.X()));
    this->dataPtr->mesh->SetUV0(v+1, _oceanTile->UV0(face.Y()));
    this->dataPtr->mesh->SetUV0(v+2, _oceanTile->UV0(face.Z()));
  }

  this->dataPtr->mesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::UpdateOceanTile(OceanTilePtr _oceanTile)
{
  // Update positions and texture coordinates for each face
  for (auto i=0, v=0; i < _oceanTile->FaceCount(); i++, v+=3)
  {
    auto face = _oceanTile->Face(i);
    // positions
    this->dataPtr->mesh->SetPoint(v+0, _oceanTile->Vertex(face.X()));
    this->dataPtr->mesh->SetPoint(v+1, _oceanTile->Vertex(face.Y()));
    this->dataPtr->mesh->SetPoint(v+2, _oceanTile->Vertex(face.Z()));
    // uv0s
    this->dataPtr->mesh->SetUV0(v+0, _oceanTile->UV0(face.X()));
    this->dataPtr->mesh->SetUV0(v+1, _oceanTile->UV0(face.Y()));
    this->dataPtr->mesh->SetUV0(v+2, _oceanTile->UV0(face.Z()));
  }

  this->dataPtr->mesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadMesh(common::MeshPtr _mesh)
{
  if (!this->dataPtr->mesh)
  {
    this->dataPtr->mesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->mesh->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->mesh->Clear();
  this->dataPtr->mesh->Update();

  this->dataPtr->mesh->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // \todo add checks
  // \todo: handle more than one submesh
  auto subMesh = _mesh->SubMeshByIndex(0);

  // Loop over all indices
  for (auto i=0; i < subMesh.lock()->IndexCount(); ++i)
  {
    auto index = subMesh.lock()->Index(i);
    auto vertex = subMesh.lock()->Vertex(index);
    auto normal = subMesh.lock()->Normal(index);
    auto uv0 = subMesh.lock()->TexCoord(index);

    this->dataPtr->mesh->AddPoint(vertex);
    this->dataPtr->mesh->SetNormal(i, normal);
    this->dataPtr->mesh->SetUV0(i, uv0);
  }

  this->dataPtr->mesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::UpdateMesh(common::MeshPtr _mesh)
{
  // \todo add checks
  // \todo: handle more than one submesh
  auto subMesh = _mesh->SubMeshByIndex(0);

  // Loop over all indices
  for (auto i=0; i < subMesh.lock()->IndexCount(); ++i)
  {
    auto index = subMesh.lock()->Index(i);
    auto vertex = subMesh.lock()->Vertex(index);
    auto normal = subMesh.lock()->Normal(index);
    auto uv0 = subMesh.lock()->TexCoord(index);

    this->dataPtr->mesh->SetPoint(i, vertex);
    this->dataPtr->mesh->SetNormal(i, normal);
    this->dataPtr->mesh->SetUV0(i, uv0);
  }

  this->dataPtr->mesh->Update();
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

  this->dataPtr->mesh->SetMaterial(_material, false);
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
