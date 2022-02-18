/*
 * Copyright (C) 2022  Rhys Mainwaring
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
*/

#include "Ogre2OceanGeometry.hh"

#include "ignition/common/SubMeshWithTangents.hh"

/// \brief Private implementation
class ignition::rendering::Ogre2OceanGeometryPrivate
{
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2OceanGeometry::Ogre2OceanGeometry(ScenePtr _scene) :
  Ogre2DynamicMesh(_scene),
  dataPtr(std::make_unique<Ogre2OceanGeometryPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2OceanGeometry::~Ogre2OceanGeometry() = default;

//////////////////////////////////////////////////
void Ogre2OceanGeometry::LoadMesh(common::MeshPtr _mesh)
{
  // if (!this->dataPtr->dynMesh)
  // {
  //   this->dataPtr->dynMesh.reset(
  //     new Ogre2DynamicMesh(this->Scene()));
  //   this->ogreNode->attachObject(this->dataPtr->dynMesh->OgreObject());
  // }

  // Clear any previous data from the grid and update
  this->Clear();
  this->Update();

  this->SetOperationType(MarkerType::MT_TRIANGLE_LIST);
  if (this->Material() == nullptr)
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
      common::SubMeshWithTangents>(baseSubMesh);
  if (!subMesh)
  {
    ignwarn << "Ogre2OceanGeometry: submesh does not support tangents\n";
    return;
  }

  // Loop over all indices
  for (auto i=0; i < subMesh->IndexCount(); ++i)
  {
    auto index = subMesh->Index(i);
    auto vertex = subMesh->Vertex(index);
    auto normal = subMesh->Normal(index);
    auto tangent = subMesh->Tangent(index);
    auto uv0 = subMesh->TexCoord(index);

    this->AddPoint(vertex);
    this->SetNormal(i, normal);
    this->SetTangent(i, tangent);
    this->SetUV0(i, uv0);
  }

  this->Update();
}

//////////////////////////////////////////////////
void Ogre2OceanGeometry::UpdateMesh(common::MeshPtr _mesh)
{
  // \todo add checks
  // \todo: handle more than one submesh

  // Get the submesh
  auto baseSubMesh = _mesh->SubMeshByIndex(0).lock();
  auto subMesh = std::dynamic_pointer_cast<
      common::SubMeshWithTangents>(baseSubMesh);
  if (!subMesh)
  {
    ignwarn << "Ogre2OceanGeometry: submesh does not support tangents\n";
    return;
  }

  // Loop over all indices
  for (auto i=0; i < subMesh->IndexCount(); ++i)
  {
    auto index = subMesh->Index(i);
    auto vertex = subMesh->Vertex(index);
    auto normal = subMesh->Normal(index);
    auto tangent = subMesh->Tangent(index);
    auto uv0 = subMesh->TexCoord(index);

    this->SetPoint(i, vertex);
    this->SetNormal(i, normal);
    this->SetTangent(i, tangent);
    this->SetUV0(i, uv0);
  }

  this->Update();
}
