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

// Adapted from gz-rendering DynamicGeometry

/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "Ogre2DynamicMesh.hh"

#include <vector>

// Note this include is placed in the src file because
// otherwise ogre produces compile errors
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/common/Console.hh>
#include <gz/rendering/ogre2/Ogre2Conversions.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreMeshManager2.h>
#include <OgreSceneManager.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreVaoManager.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

/// \brief Private implementation
class Ogre2DynamicMeshPrivate
{
  /// \brief list of colors at each point
  public: std::vector<gz::math::Color> colors;

  /// \brief List of vertices for the mesh
  public: std::vector<gz::math::Vector3d> vertices;

  /// \brief List of normals for the mesh
  public: std::vector<gz::math::Vector3d> normals;

  /// \brief List of tangents for the mesh
  public: std::vector<gz::math::Vector3d> tangents;

  /// \brief List of binormals for the mesh
  // public: std::vector<gz::math::Vector3d> binormals;

  /// \brief List of uv0 coordinates for the mesh
  public: std::vector<gz::math::Vector2d> uv0s;

  /// \brief Used to indicate if the lines require an update
  public: bool dirty {false};

  /// \brief Render operation type
  public: Ogre::OperationType operationType;

  /// \brief Ogre mesh
  public: Ogre::MeshPtr mesh;

  /// \brief Ogre submesh
  public: Ogre::SubMesh *subMesh {nullptr};

  /// \brief Ogre vertex buffer data structure
  public: Ogre::VertexBufferPacked *vertexBuffer {nullptr};

  /// \brief Ogre vertex array object which binds the index and vertex buffers
  public: Ogre::VertexArrayObject *vao {nullptr};

  /// \brief Ogre item created from the dynamic geometry
  public: Ogre::Item *ogreItem {nullptr};

  /// \todo if we use binormals then update teh stride and ensure
  ///       they are set elsewhere
  /// \brief vertex buffer stride =
  ///             vertex.xyz
  ///             + normal.xyz + tangent.xyz + binormal.xyz
  ///             + uv0.xy
  // public: unsigned int stride {3 + 3 + 3 + 3 + 2};

  /// \brief vertex buffer stride =
  ///             vertex.xyz
  ///             + normal.xyz + tangent.xyz
  ///             + uv0.xy
  public: unsigned int stride {3 + 3 + 3 + 2};

  /// \brief raw vertex buffer
  public: float *vbuffer {nullptr};

  /// \brief Maximum capacity of the currently allocated vertex buffer.
  public: size_t vertexBufferCapacity {0};

  /// \brief Pointer to the dynamic mesh's material
  public: Ogre2MaterialPtr material;

  /// \brief Flag to indicate whether or not this mesh should be
  /// responsible for destroying the material
  public: bool ownsMaterial {false};

  /// \brief Pointer to scene
  public: ScenePtr scene;

  /// \brief Pointer to the ogre scene manager
  public: Ogre::SceneManager *sceneManager = nullptr;
};

//////////////////////////////////////////////////
Ogre2DynamicMesh::Ogre2DynamicMesh(ScenePtr _scene)
    : Ogre2Geometry(),
    dataPtr(new Ogre2DynamicMeshPrivate)
{
  this->dataPtr->scene = _scene;

  Ogre2ScenePtr s = std::dynamic_pointer_cast<Ogre2Scene>(this->dataPtr->scene);
  this->dataPtr->sceneManager = s->OgreSceneManager();

  this->SetOperationType(MT_TRIANGLE_LIST);
  this->CreateDynamicMesh();
}

//////////////////////////////////////////////////
Ogre2DynamicMesh::~Ogre2DynamicMesh()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::Destroy()
{
  if (!this->dataPtr->scene->IsInitialized())
    return;

  if (!this->dataPtr->ogreItem)
    return;

  this->DestroyBuffer();

  // destroy ogre item
  this->dataPtr->sceneManager->destroyItem(this->dataPtr->ogreItem);
  this->dataPtr->ogreItem = nullptr;

  // remove mesh from mesh manager
  if (this->dataPtr->subMesh &&
      Ogre::MeshManager::getSingleton().resourceExists(
      this->dataPtr->subMesh->mParent->getName()))
  {
    Ogre::MeshManager::getSingleton().remove(
        this->dataPtr->subMesh->mParent->getName());
    this->dataPtr->subMesh = nullptr;
  }

  if (this->dataPtr->material && this->dataPtr->ownsMaterial)
  {
    this->dataPtr->scene->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2DynamicMesh::OgreObject() const
{
  return this->dataPtr->ogreItem;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2DynamicMesh::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::SetMaterial(MaterialPtr _material, bool _unique)
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

  if (this->dataPtr->material && this->dataPtr->ownsMaterial)
    this->dataPtr->scene->DestroyMaterial(this->dataPtr->material);

  this->dataPtr->ownsMaterial = _unique;

  this->dataPtr->material = derived;

  this->dataPtr->ogreItem->getSubItem(0)->setDatablock(
      static_cast<Ogre::HlmsPbsDatablock *>(derived->Datablock()));

  // set cast shadows
  this->dataPtr->ogreItem->setCastShadows(_material->CastShadows());
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::DestroyBuffer()
{
  if (this->dataPtr->vbuffer)
    delete [] this->dataPtr->vbuffer;

  Ogre::RenderSystem *renderSystem =
      this->dataPtr->sceneManager->getDestinationRenderSystem();
  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

  if (!vaoManager)
    return;

  if (this->dataPtr->subMesh)
  {
    if (!this->dataPtr->subMesh->mVao[Ogre::VpNormal].empty())
    {
      this->dataPtr->subMesh->destroyVaos(
          this->dataPtr->subMesh->mVao[Ogre::VpNormal], vaoManager);
    }
    if (!this->dataPtr->subMesh->mVao[Ogre::VpShadow].empty())
      this->dataPtr->subMesh->mVao[Ogre::VpShadow].clear();
  }

  this->dataPtr->vertexBuffer = nullptr;
  this->dataPtr->vao = nullptr;
  this->dataPtr->vbuffer = nullptr;
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::Update()
{
  this->UpdateBuffer();
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::CreateDynamicMesh()
{
  if (this->dataPtr->ogreItem)
    return;

  static int dynamicMeshId = 0;
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
              "dynamic_mesh_" + std::to_string(dynamicMeshId++),
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  this->dataPtr->subMesh = mesh->createSubMesh();
  this->dataPtr->dirty = true;

  // this creates the ogre2 dynamic geometry buffer
  this->UpdateBuffer();

  this->dataPtr->ogreItem =
      this->dataPtr->sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::UpdateBuffer()
{
  if (!this->dataPtr->dirty)
    return;

  unsigned int stride = this->dataPtr->stride;

  Ogre::RenderSystem *renderSystem =
      this->dataPtr->sceneManager->getDestinationRenderSystem();

  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();
  if (!vaoManager)
    return;

  // Prepare vertex buffer
  unsigned int newVertCapacity = this->dataPtr->vertexBufferCapacity;

  unsigned int vertexCount = this->dataPtr->vertices.size();
  if ((vertexCount > this->dataPtr->vertexBufferCapacity) ||
      (!this->dataPtr->vertexBufferCapacity))
  {
    // vertexCount exceeds current capacity!
    // It is necessary to reallocate the buffer.

    // Check if this is the first call
    if (!newVertCapacity)
      newVertCapacity = 1;

    // Make capacity the next power of two
    while (newVertCapacity < vertexCount)
      newVertCapacity <<= 1;
  }
  else if (vertexCount < this->dataPtr->vertexBufferCapacity>>1)
  {
    // Make capacity the previous power of two
    unsigned int newCapacity = newVertCapacity >>1;
    while (vertexCount < newCapacity)
    {
      newVertCapacity = newCapacity;
      newCapacity >>= 1;
    }
  }

  // recreate vao if needed
  if (newVertCapacity != this->dataPtr->vertexBufferCapacity)
  {
    this->dataPtr->vertexBufferCapacity = newVertCapacity;

    this->DestroyBuffer();

    unsigned int size = this->dataPtr->vertexBufferCapacity * stride;
    this->dataPtr->vbuffer = new float[size];
    memset(this->dataPtr->vbuffer, 0, size * sizeof(float));

    this->dataPtr->subMesh->mVao[Ogre::VpNormal].clear();
    this->dataPtr->subMesh->mVao[Ogre::VpShadow].clear();

    // recreate the vao data structures
    Ogre::VertexElement2Vec vertexElements;
    vertexElements.push_back(
        Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_POSITION));
    vertexElements.push_back(
        Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_NORMAL));
    vertexElements.push_back(
        Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_TANGENT));
    // vertexElements.push_back(
    //     Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_BINORMAL));
    vertexElements.push_back(
        Ogre::VertexElement2(Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES));

    // create vertex buffer
    this->dataPtr->vertexBuffer = vaoManager->createVertexBuffer(
        vertexElements, this->dataPtr->vertexBufferCapacity,
        Ogre::BT_DYNAMIC_PERSISTENT, this->dataPtr->vbuffer, false);

    Ogre::VertexBufferPackedVec vertexBuffers;
    vertexBuffers.push_back(this->dataPtr->vertexBuffer);

    // it is ok to use null index buffer
    Ogre::IndexBufferPacked *indexBuffer = nullptr;

    this->dataPtr->vao = vaoManager->createVertexArrayObject(vertexBuffers,
        indexBuffer, this->dataPtr->operationType);

    this->dataPtr->subMesh->mVao[Ogre::VpNormal].push_back(this->dataPtr->vao);
    // Use the same geometry for shadow casting.
    this->dataPtr->subMesh->mVao[Ogre::VpShadow].push_back(this->dataPtr->vao);
  }

  // map buffer and update the geometry
  Ogre::Aabb bbox;
  float * RESTRICT_ALIAS vertices = reinterpret_cast<float * RESTRICT_ALIAS>(
      this->dataPtr->vertexBuffer->map(
      0, this->dataPtr->vertexBuffer->getNumElements()));

  // fill positions, normals and texture coords
  for (unsigned int i = 0; i < vertexCount; ++i)
  {
    unsigned int idx = i*stride;

    // position
    Ogre::Vector3 v = Ogre2Conversions::Convert(this->dataPtr->vertices[i]);
    vertices[idx++] = v.x;
    vertices[idx++] = v.y;
    vertices[idx++] = v.z;

    // normal
    Ogre::Vector3 n = Ogre2Conversions::Convert(this->dataPtr->normals[i]);
    vertices[idx++] = n.x;
    vertices[idx++] = n.y;
    vertices[idx++] = n.z;

    // tangent
    Ogre::Vector3 t = Ogre2Conversions::Convert(this->dataPtr->tangents[i]);
    vertices[idx++] = t.x;
    vertices[idx++] = t.y;
    vertices[idx++] = t.z;

    // // binormal
    // Ogre::Vector3 b = Ogre2Conversions::Convert(this->dataPtr->binormals[i]);
    // vertices[idx++] = b.x;
    // vertices[idx++] = b.y;
    // vertices[idx++] = b.z;

    // uv0
    Ogre::Vector2 uv0(this->dataPtr->uv0s[i].X(), this->dataPtr->uv0s[i].Y());
    vertices[idx++] = uv0.x;
    vertices[idx++] = uv0.y;

    bbox.merge(v);
  }

  // fill the rest of the buffer with the position of the last vertex to avoid
  // the geometry connecting back to 0, 0, 0
  if (vertexCount > 0 && vertexCount < this->dataPtr->vertexBufferCapacity)
  {
    math::Vector3d lastVertex = this->dataPtr->vertices[vertexCount-1];
    math::Vector3d lastNormal = this->dataPtr->normals[vertexCount-1];
    math::Vector3d lastTangent = this->dataPtr->tangents[vertexCount-1];
    // math::Vector3d lastBinormal = this->dataPtr->binormals[vertexCount-1];
    math::Vector2d lastUv0 = this->dataPtr->uv0s[vertexCount-1];
    for (unsigned int i = vertexCount; i < this->dataPtr->vertexBufferCapacity;
        ++i)
    {
      unsigned int idx = i * stride;
      // vertex
      vertices[idx++] = lastVertex.X();
      vertices[idx++] = lastVertex.Y();
      vertices[idx++] = lastVertex.Z();

      // normal
      vertices[idx++] = lastNormal.X();
      vertices[idx++] = lastNormal.Y();
      vertices[idx++] = lastNormal.Z();

      // tangent
      vertices[idx++] = lastTangent.X();
      vertices[idx++] = lastTangent.Y();
      vertices[idx++] = lastTangent.Z();

      // binormal
      // vertices[idx++] = lastBinormal.X();
      // vertices[idx++] = lastBinormal.Y();
      // vertices[idx++] = lastBinormal.Z();

      // uv0
      vertices[idx++] = lastUv0.X();
      vertices[idx++] = lastUv0.Y();
    }
  }

  /// \todo add condition to check if normals are to be set or generated,
  ///       ditto colors

  // fill normals
  // this->GenerateNormals(this->dataPtr->operationType,
  //    this->dataPtr->vertices, vertices);

  // fill colors for points
  // this->GenerateColors(this->dataPtr->operationType, this->dataPtr->vertices,
  //     vertices);

  // unmap buffer
  this->dataPtr->vertexBuffer->unmap(Ogre::UO_KEEP_PERSISTENT);

  // Set the bounds to get frustum culling and LOD to work correctly.
  Ogre::Mesh *mesh = this->dataPtr->subMesh->mParent;
  mesh->_setBounds(bbox, true);

  // update item aabb
  if (this->dataPtr->ogreItem)
  {
    bool castShadows = this->dataPtr->ogreItem->getCastShadows();
    auto lowLevelMat = this->dataPtr->ogreItem->getSubItem(0)->getMaterial();

    // need to rebuild ogre [sub]item because the vao was destroyed
    // this updates the item's bounding box and fixes occasional crashes
    // from invalid access to old vao
    this->dataPtr->ogreItem->_initialise(true);

    // set material
    if (this->dataPtr->material)
    {
      this->dataPtr->ogreItem->getSubItem(0)->setDatablock(
          static_cast<Ogre::HlmsPbsDatablock *>(
          this->dataPtr->material->Datablock()));
      this->dataPtr->ogreItem->setCastShadows(
          this->dataPtr->material->CastShadows());
    }
    if (lowLevelMat)
    {
      // the _initialise call above resets the ogre item properties so set
      // them again
      this->dataPtr->ogreItem->getSubItem(0)->setMaterial(lowLevelMat);
      this->dataPtr->ogreItem->setCastShadows(castShadows);
    }
  }

  this->dataPtr->dirty = false;
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::SetOperationType(MarkerType _opType)
{
  switch (_opType)
  {
    case MT_POINTS:
      this->dataPtr->operationType = Ogre::OperationType::OT_POINT_LIST;
      break;

    case MT_LINE_LIST:
      this->dataPtr->operationType = Ogre::OperationType::OT_LINE_LIST;
      break;

    case MT_LINE_STRIP:
      this->dataPtr->operationType = Ogre::OperationType::OT_LINE_STRIP;
      break;

    case MT_TRIANGLE_LIST:
      this->dataPtr->operationType = Ogre::OperationType::OT_TRIANGLE_LIST;
      break;

    case MT_TRIANGLE_STRIP:
      this->dataPtr->operationType = Ogre::OperationType::OT_TRIANGLE_STRIP;
      break;

    case MT_TRIANGLE_FAN:
      this->dataPtr->operationType = Ogre::OperationType::OT_TRIANGLE_FAN;
      break;

    default:
      gzerr << "Unknown render operation type[" << _opType << "]\n";
      return;
  }
}

//////////////////////////////////////////////////
MarkerType Ogre2DynamicMesh::OperationType() const
{
  MarkerType opType;
  switch (this->dataPtr->operationType)
  {
    case Ogre::OperationType::OT_LINE_LIST:
      opType = MT_LINE_LIST;
      break;

    case Ogre::OperationType::OT_LINE_STRIP:
      opType = MT_LINE_STRIP;
      break;

    case Ogre::OperationType::OT_TRIANGLE_LIST:
      opType = MT_TRIANGLE_LIST;
      break;

    case Ogre::OperationType::OT_TRIANGLE_STRIP:
      opType = MT_TRIANGLE_STRIP;
      break;

    case Ogre::OperationType::OT_TRIANGLE_FAN:
      opType = MT_TRIANGLE_FAN;
      break;

    default:
    case Ogre::OperationType::OT_POINT_LIST:
      opType = MT_POINTS;
      break;
  }

  return opType;
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::AddPoint(const math::Vector3d &_pt,
    const math::Color &_color)
{
  this->dataPtr->vertices.push_back(_pt);

  // todo(anyone)
  // setting material works but vertex coloring only works for points
  // It requires using an unlit datablock:
  // https://forums.ogre3d.org/viewtopic.php?t=93627#p539276
  this->dataPtr->colors.push_back(_color);

  this->dataPtr->normals.push_back(math::Vector3d::Zero);
  this->dataPtr->tangents.push_back(math::Vector3d::Zero);
  // this->dataPtr->binormals.push_back(math::Vector3d::Zero);
  this->dataPtr->uv0s.push_back(math::Vector2d::Zero);

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::AddPoint(double _x, double _y, double _z,
    const math::Color &_color)
{
  this->AddPoint(math::Vector3d(_x, _y, _z), _color);
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::SetPoint(unsigned int _index,
    const math::Vector3d &_value)
{
  if (_index >= this->dataPtr->vertices.size())
  {
    gzerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->vertices.size()-1 << "]\n";
    return;
  }

  this->dataPtr->vertices[_index] = _value;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::SetColor(unsigned int _index,
    const math::Color &_color)
{
  if (_index >= this->dataPtr->colors.size())
  {
    gzerr << "Point color index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->colors.size()-1 << "]\n";
    return;
  }


  // todo(anyone)
  // vertex coloring only works for points.
  // Full implementation requires using an unlit datablock:
  // https://forums.ogre3d.org/viewtopic.php?t=93627#p539276
  this->dataPtr->colors[_index] = _color;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::SetNormal(unsigned int _index,
                              const math::Vector3d &_normal)
{
  if (_index >= this->dataPtr->normals.size())
  {
    gzerr << "Point normal index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->normals.size()-1 << "]\n";
    return;
  }

  this->dataPtr->normals[_index] = _normal;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::SetTangent(unsigned int _index,
                              const math::Vector3d &_tangent)
{
  if (_index >= this->dataPtr->tangents.size())
  {
    gzerr << "Point tangent index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->tangents.size()-1 << "]\n";
    return;
  }

  this->dataPtr->tangents[_index] = _tangent;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::SetUV0(unsigned int _index,
                              const math::Vector2d &_uv0)
{
  if (_index >= this->dataPtr->uv0s.size())
  {
    gzerr << "Point uv0 index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->uv0s.size()-1 << "]\n";
    return;
  }

  this->dataPtr->uv0s[_index] = _uv0;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
math::Vector3d Ogre2DynamicMesh::Point(
    const unsigned int _index) const
{
  if (_index >= this->dataPtr->vertices.size())
  {
    gzerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->vertices.size()-1 << "]\n";

    return math::Vector3d(math::INF_D,
                          math::INF_D,
                          math::INF_D);
  }

  return this->dataPtr->vertices[_index];
}

/////////////////////////////////////////////////
unsigned int Ogre2DynamicMesh::PointCount() const
{
  return this->dataPtr->vertices.size();
}

/////////////////////////////////////////////////
void Ogre2DynamicMesh::Clear()
{
  if (this->dataPtr->vertices.empty() && this->dataPtr->colors.empty())
    return;

  this->dataPtr->vertices.clear();
  this->dataPtr->normals.clear();
  this->dataPtr->tangents.clear();
  // this->dataPtr->binormals.clear();
  this->dataPtr->colors.clear();
  this->dataPtr->uv0s.clear();
  this->dataPtr->dirty = true;
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::GenerateNormals(Ogre::OperationType _opType,
  const std::vector<math::Vector3d> &_vertices, float *_vbuffer)
{
  unsigned int vertexCount = _vertices.size();
  unsigned int stride = this->dataPtr->stride;

  // Each vertex occupies 3 + 3 + 2 elements in the vbuffer float array:
  // vbuffer[i++] : position x
  // vbuffer[i++] : position y
  // vbuffer[i++] : position z
  // vbuffer[i++] : normal x
  // vbuffer[i++] : normal y
  // vbuffer[i++] : normal z
  // vbuffer[i++] : tangent x
  // vbuffer[i++] : tangent y
  // vbuffer[i++] : tangent z
  // vbuffer[i++] : binormal x // disabled
  // vbuffer[i++] : binormal y // disabled
  // vbuffer[i++] : binormal z // disabled
  // vbuffer[i++] : uv x
  // vbuffer[i++] : uv y
  switch (_opType)
  {
    case Ogre::OperationType::OT_POINT_LIST:
    case Ogre::OperationType::OT_LINE_LIST:
    case Ogre::OperationType::OT_LINE_STRIP:
      return;
    case Ogre::OperationType::OT_TRIANGLE_LIST:
    {
      if (vertexCount < 3)
        return;

      // NOTE: because each face has it's own vertices they each get
      //       the normal for the face. When using indexed vertices
      //       we average over the normals from easch face shared by
      //       a vertex, which results in a smoother looking surface
      for (unsigned int i = 0; i < vertexCount / 3; ++i)
      {
        unsigned int idx = i*3;
        unsigned int idx1 = idx * stride;
        unsigned int idx2 = idx1 + stride;
        unsigned int idx3 = idx2 + stride;
        math::Vector3d v1 = _vertices[idx+0];
        math::Vector3d v2 = _vertices[idx+1];
        math::Vector3d v3 = _vertices[idx+2];
        math::Vector3d n = (v1 - v2).Cross((v1 - v3));
        // math::Vector3d n = math::Vector3d::Normal(v1, v2, v3);

        _vbuffer[idx1+3] = n.X();
        _vbuffer[idx1+4] = n.Y();
        _vbuffer[idx1+5] = n.Z();

        _vbuffer[idx2+3] = n.X();
        _vbuffer[idx2+4] = n.Y();
        _vbuffer[idx2+5] = n.Z();

        _vbuffer[idx3+3] = n.X();
        _vbuffer[idx3+4] = n.Y();
        _vbuffer[idx3+5] = n.Z();
      }

      break;
    }
    case Ogre::OperationType::OT_TRIANGLE_STRIP:
    {
      if (vertexCount < 3)
        return;

      bool even = false;
      for (unsigned int i = 0; i < vertexCount - 2; ++i)
      {
        math::Vector3d v1;
        math::Vector3d v2;
        math::Vector3d v3 = _vertices[i+2];

        // For odd n, vertices n, n+1, and n+2 define triangle n.
        // For even n, vertices n+1, n, and n+2 define triangle n.
        unsigned int idx1;
        unsigned int idx2;
        unsigned int idx3 = (i+2) * stride;
        if (even)
        {
          v1 = _vertices[i+1];
          v2 = _vertices[i];
          idx1 = (i+1) * stride;
          idx2 = i*stride;
        }
        else
        {
          v1 = _vertices[i];
          v2 = _vertices[i+1];
          idx1 = i*stride;
          idx2 = (i+1) * stride;
        }
        even = !even;

        math::Vector3d n = (v1 - v2).Cross((v1 - v3));
        math::Vector3d n1(_vbuffer[idx1+3], _vbuffer[idx1+4], _vbuffer[idx1+5]);
        math::Vector3d n2(_vbuffer[idx2+3], _vbuffer[idx2+4], _vbuffer[idx2+5]);
        math::Vector3d n3(_vbuffer[idx3+3], _vbuffer[idx3+4], _vbuffer[idx3+5]);

        math::Vector3d n1a = ((n1 + n)/2);
        n1a.Normalize();
        math::Vector3d n2a = ((n2 + n)/2);
        n2a.Normalize();
        math::Vector3d n3a = ((n3 + n)/2);
        n3a.Normalize();

        _vbuffer[idx1+3] = n1a.X();
        _vbuffer[idx1+4] = n1a.Y();
        _vbuffer[idx1+5] = n1a.Z();
        _vbuffer[idx2+3] = n2a.X();
        _vbuffer[idx2+4] = n2a.Y();
        _vbuffer[idx2+5] = n2a.Z();
        _vbuffer[idx3+3] = n3a.X();
        _vbuffer[idx3+4] = n3a.Y();
        _vbuffer[idx3+5] = n3a.Z();
      }

      break;
    }
    case Ogre::OperationType::OT_TRIANGLE_FAN:
    {
      if (vertexCount < 3)
        return;

      unsigned int idx1 = 0;
      math::Vector3d v1 = _vertices[0];

      for (unsigned int i = 0; i < vertexCount - 2; ++i)
      {
        unsigned int idx2 = (i+1) * stride;
        unsigned int idx3 = idx2 + stride;
        math::Vector3d v2 = _vertices[i+1];
        math::Vector3d v3 = _vertices[i+2];
        math::Vector3d n = (v1 - v2).Cross((v1 - v3));

        math::Vector3d n1(_vbuffer[idx1+3], _vbuffer[idx1+4], _vbuffer[idx1+5]);
        math::Vector3d n2(_vbuffer[idx2+3], _vbuffer[idx2+4], _vbuffer[idx2+5]);
        math::Vector3d n3(_vbuffer[idx3+3], _vbuffer[idx3+4], _vbuffer[idx3+5]);

        math::Vector3d n1a = ((n1 + n)/2);
        n1a.Normalize();
        math::Vector3d n2a = ((n2 + n)/2);
        n2a.Normalize();
        math::Vector3d n3a = ((n3 + n)/2);
        n3a.Normalize();

        _vbuffer[idx1+3] = n1a.X();
        _vbuffer[idx1+4] = n1a.Y();
        _vbuffer[idx1+5] = n1a.Z();
        _vbuffer[idx2+3] = n2a.X();
        _vbuffer[idx2+4] = n2a.Y();
        _vbuffer[idx2+5] = n2a.Z();
        _vbuffer[idx3+3] = n3a.X();
        _vbuffer[idx3+4] = n3a.Y();
        _vbuffer[idx3+5] = n3a.Z();
      }

      break;
    }
    default:
      break;
  }
}

//////////////////////////////////////////////////
void Ogre2DynamicMesh::GenerateColors(Ogre::OperationType _opType,
  const std::vector<math::Vector3d> &_vertices, float *_vbuffer)
{
  // Skip if colors haven't been setup per-vertex correctly.
  if (_vertices.size() != this->dataPtr->colors.size())
    return;

  unsigned int stride = this->dataPtr->stride;

  // Each vertex occupies 3 + 3 + 2 elements in the vbuffer float array.
  // Normally, the last 3 are reserved for normals. But for types that
  // don't use normals, we use them for per-vertex coloring.
  // vbuffer[i]   : position x
  // vbuffer[i+1] : position y
  // vbuffer[i+2] : position z
  // vbuffer[i+3] : color r
  // vbuffer[i+4] : color g
  // vbuffer[i+5] : color b
  // ...
  switch (_opType)
  {
    case Ogre::OperationType::OT_POINT_LIST:
    {
      for (unsigned int i = 0; i < this->dataPtr->vertexBufferCapacity; ++i)
      {
        math::Color color = i < this->dataPtr->colors.size() ?
            this->dataPtr->colors[i] : this->dataPtr->colors.back();

        unsigned int idx = i * stride;
        _vbuffer[idx+3] = color.R();
        _vbuffer[idx+4] = color.G();
        _vbuffer[idx+5] = color.B();
      }

      break;
    }
    default:
      break;
  }
}

}
}  // namespace rendering
}  // namespace gz
