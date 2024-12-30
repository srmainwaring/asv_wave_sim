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

#include "Ogre2OceanVisual.hh"

#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/SubMesh.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneNode.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

#include "gz/common/SubMeshWithTangents.hh"

#include "Ogre2DynamicMesh.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

class Ogre2OceanVisualPrivate
{
  /// \brief visual materal
  public: Ogre2MaterialPtr material = nullptr;

  /// \brief Ogre renderable used to render the ocean tile.
  public: std::shared_ptr<Ogre2DynamicMesh> dynMesh = nullptr;
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
  if (this->dataPtr->dynMesh)
  {
    this->dataPtr->dynMesh->Destroy();
    this->dataPtr->dynMesh.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
MaterialPtr Ogre2OceanVisual::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  Ogre2MaterialPtr derived =
      std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    gzerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  this->dataPtr->dynMesh->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadCube()
{
  if (!this->dataPtr->dynMesh)
  {
    this->dataPtr->dynMesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->dynMesh->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->dynMesh->Clear();
  this->dataPtr->dynMesh->Update();

  this->dataPtr->dynMesh->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // Position indicator - with dynamic geometry
  // must specify vertices for each face and get
  // the orientation correct.
  gz::math::Vector3d p0(-1, -1,  1);
  gz::math::Vector3d p1(+1, -1,  1);
  gz::math::Vector3d p2(+1,  1,  1);
  gz::math::Vector3d p3(-1,  1,  1);
  gz::math::Vector3d p4(-1, -1, -1);
  gz::math::Vector3d p5(+1, -1, -1);
  gz::math::Vector3d p6(+1,  1, -1);
  gz::math::Vector3d p7(-1,  1, -1);

  // front face
  this->dataPtr->dynMesh->AddPoint(p0);
  this->dataPtr->dynMesh->AddPoint(p1);
  this->dataPtr->dynMesh->AddPoint(p2);
  this->dataPtr->dynMesh->AddPoint(p2);
  this->dataPtr->dynMesh->AddPoint(p3);
  this->dataPtr->dynMesh->AddPoint(p0);

  // back face
  this->dataPtr->dynMesh->AddPoint(p6);
  this->dataPtr->dynMesh->AddPoint(p5);
  this->dataPtr->dynMesh->AddPoint(p4);
  this->dataPtr->dynMesh->AddPoint(p4);
  this->dataPtr->dynMesh->AddPoint(p7);
  this->dataPtr->dynMesh->AddPoint(p6);

  // top face
  this->dataPtr->dynMesh->AddPoint(p3);
  this->dataPtr->dynMesh->AddPoint(p2);
  this->dataPtr->dynMesh->AddPoint(p6);
  this->dataPtr->dynMesh->AddPoint(p6);
  this->dataPtr->dynMesh->AddPoint(p7);
  this->dataPtr->dynMesh->AddPoint(p3);

  // bottom face
  this->dataPtr->dynMesh->AddPoint(p5);
  this->dataPtr->dynMesh->AddPoint(p1);
  this->dataPtr->dynMesh->AddPoint(p0);
  this->dataPtr->dynMesh->AddPoint(p0);
  this->dataPtr->dynMesh->AddPoint(p4);
  this->dataPtr->dynMesh->AddPoint(p5);

  // left face
  this->dataPtr->dynMesh->AddPoint(p4);
  this->dataPtr->dynMesh->AddPoint(p0);
  this->dataPtr->dynMesh->AddPoint(p3);
  this->dataPtr->dynMesh->AddPoint(p3);
  this->dataPtr->dynMesh->AddPoint(p7);
  this->dataPtr->dynMesh->AddPoint(p4);

  // right face
  this->dataPtr->dynMesh->AddPoint(p6);
  this->dataPtr->dynMesh->AddPoint(p2);
  this->dataPtr->dynMesh->AddPoint(p1);
  this->dataPtr->dynMesh->AddPoint(p1);
  this->dataPtr->dynMesh->AddPoint(p5);
  this->dataPtr->dynMesh->AddPoint(p6);

  this->dataPtr->dynMesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadOceanTile(waves::visual::OceanTilePtr _oceanTile)
{
  if (!this->dataPtr->dynMesh)
  {
    this->dataPtr->dynMesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->dynMesh->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->dynMesh->Clear();
  this->dataPtr->dynMesh->Update();

  this->dataPtr->dynMesh->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // Add points and texture coordinates for each face
  for (auto i = 0, v = 0; i < _oceanTile->FaceCount(); i++, v+=3)
  {
    auto face = _oceanTile->Face(i);
    // positions
    this->dataPtr->dynMesh->AddPoint(_oceanTile->Vertex(face.X()));
    this->dataPtr->dynMesh->AddPoint(_oceanTile->Vertex(face.Y()));
    this->dataPtr->dynMesh->AddPoint(_oceanTile->Vertex(face.Z()));

    // uv0s
    this->dataPtr->dynMesh->SetUV0(v+0, _oceanTile->UV0(face.X()));
    this->dataPtr->dynMesh->SetUV0(v+1, _oceanTile->UV0(face.Y()));
    this->dataPtr->dynMesh->SetUV0(v+2, _oceanTile->UV0(face.Z()));
  }

  this->dataPtr->dynMesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::UpdateOceanTile(waves::visual::OceanTilePtr _oceanTile)
{
  // Update positions and texture coordinates for each face
  for (auto i = 0, v = 0; i < _oceanTile->FaceCount(); i++, v+=3)
  {
    auto face = _oceanTile->Face(i);
    // positions
    this->dataPtr->dynMesh->SetPoint(v+0, _oceanTile->Vertex(face.X()));
    this->dataPtr->dynMesh->SetPoint(v+1, _oceanTile->Vertex(face.Y()));
    this->dataPtr->dynMesh->SetPoint(v+2, _oceanTile->Vertex(face.Z()));
    // uv0s
    this->dataPtr->dynMesh->SetUV0(v+0, _oceanTile->UV0(face.X()));
    this->dataPtr->dynMesh->SetUV0(v+1, _oceanTile->UV0(face.Y()));
    this->dataPtr->dynMesh->SetUV0(v+2, _oceanTile->UV0(face.Z()));
  }

  this->dataPtr->dynMesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::LoadMesh(gz::common::MeshPtr _mesh)
{
  if (!this->dataPtr->dynMesh)
  {
    this->dataPtr->dynMesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->dynMesh->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->dynMesh->Clear();
  this->dataPtr->dynMesh->Update();

  this->dataPtr->dynMesh->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransBlue")->Clone();
    this->SetMaterial(defaultMat, false);
  }

  // \todo add checks
  // \todo: handle more than one submesh

  // Get the submesh
  auto baseSubMesh = _mesh->SubMeshByIndex(0).lock();
  auto subMesh = std::dynamic_pointer_cast<
      gz::common::SubMeshWithTangents>(baseSubMesh);
  if (!subMesh)
  {
    gzwarn << "Ogre2OceanVisual: submesh does not support tangents\n";
    return;
  }

  // Loop over all indices
  for (size_t i=0; i < subMesh->IndexCount(); ++i)
  {
    auto index = subMesh->Index(i);
    auto vertex = subMesh->Vertex(index);
    auto normal = subMesh->Normal(index);
    auto tangent = subMesh->Tangent(index);
    auto uv0 = subMesh->TexCoord(index);

    this->dataPtr->dynMesh->AddPoint(vertex);
    this->dataPtr->dynMesh->SetNormal(i, normal);
    this->dataPtr->dynMesh->SetTangent(i, tangent);
    this->dataPtr->dynMesh->SetUV0(i, uv0);
  }

  this->dataPtr->dynMesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::UpdateMesh(gz::common::MeshPtr _mesh)
{
  // \todo add checks
  // \todo: handle more than one submesh

  // Get the submesh
  auto baseSubMesh = _mesh->SubMeshByIndex(0).lock();
  auto subMesh = std::dynamic_pointer_cast<
      gz::common::SubMeshWithTangents>(baseSubMesh);
  if (!subMesh)
  {
    gzwarn << "Ogre2OceanVisual: submesh does not support tangents\n";
    return;
  }

  // Loop over all indices
  for (size_t i=0; i < subMesh->IndexCount(); ++i)
  {
    auto index = subMesh->Index(i);
    auto vertex = subMesh->Vertex(index);
    auto normal = subMesh->Normal(index);
    auto tangent = subMesh->Tangent(index);
    auto uv0 = subMesh->TexCoord(index);

    this->dataPtr->dynMesh->SetPoint(i, vertex);
    this->dataPtr->dynMesh->SetNormal(i, normal);
    this->dataPtr->dynMesh->SetTangent(i, tangent);
    this->dataPtr->dynMesh->SetUV0(i, uv0);
  }

  this->dataPtr->dynMesh->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
void Ogre2OceanVisual::InitObject(ScenePtr _scene,
    unsigned int _id, const std::string &_name)
{
  rendering::Ogre2ScenePtr ogre2Scene =
      std::dynamic_pointer_cast<rendering::Ogre2Scene>(_scene);

  this->id = _id;
  this->name = _name;
  this->scene = ogre2Scene;

  // initialize object
  this->Load();
  this->Init();
}

}
}  // namespace rendering
}  // namespace gz
