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

#include "Ogre2OceanGeometry.hh"

#include <string>

#include <gz/common/Console.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include "gz/common/SubMeshWithTangents.hh"

#include "Ogre2DynamicMesh.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

/// \brief Private implementation
class Ogre2OceanGeometryPrivate
{
  /// \brief OceanGeometry materal
  public: Ogre2MaterialPtr material{nullptr};

  /// \brief OceanGeometry dynamic mesh
  public: Ogre2DynamicMeshPtr dynMesh{nullptr};

  /// \brief Lazy evaluation flag
  // public: bool dirty{true};
};

//////////////////////////////////////////////////
Ogre2OceanGeometry::Ogre2OceanGeometry() :
    Ogre2Geometry(),
    dataPtr(std::make_unique<Ogre2OceanGeometryPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2OceanGeometry::~Ogre2OceanGeometry() = default;

//////////////////////////////////////////////////
void Ogre2OceanGeometry::Init()
{
  Ogre2Geometry::Init();
  // this->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanGeometry::Destroy()
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
Ogre::MovableObject *Ogre2OceanGeometry::OgreObject() const
{
  if (this->dataPtr->dynMesh)
    return this->dataPtr->dynMesh->OgreObject();
  else
    return nullptr;
}

//////////////////////////////////////////////////
void Ogre2OceanGeometry::PreRender()
{
  // if (this->dataPtr->dirty)
  // {
  //   this->Update();
  //   this->dataPtr->dirty = false;
  // }
}

//////////////////////////////////////////////////
MaterialPtr Ogre2OceanGeometry::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void Ogre2OceanGeometry::SetMaterial(MaterialPtr _material, bool _unique)
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

  // Set material for the underlying dynamic renderable
  this->dataPtr->dynMesh->SetMaterial(derived, false);
  this->dataPtr->material = derived;
}

//////////////////////////////////////////////////
void Ogre2OceanGeometry::LoadMesh(gz::common::MeshPtr _mesh)
{
  if (!this->dataPtr->dynMesh)
  {
    this->dataPtr->dynMesh.reset(
      new Ogre2DynamicMesh(this->Scene()));
    // this->ogreNode->attachObject(this->dataPtr->dynMesh->OgreObject());
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
    gzwarn << "Ogre2OceanGeometry: submesh does not support tangents\n";
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
void Ogre2OceanGeometry::UpdateMesh(gz::common::MeshPtr _mesh)
{
  // \todo add checks
  // \todo: handle more than one submesh

  // Get the submesh
  auto baseSubMesh = _mesh->SubMeshByIndex(0).lock();
  auto subMesh = std::dynamic_pointer_cast<
      gz::common::SubMeshWithTangents>(baseSubMesh);
  if (!subMesh)
  {
    gzwarn << "Ogre2OceanGeometry: submesh does not support tangents\n";
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
void Ogre2OceanGeometry::InitObject(ScenePtr _scene,
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
