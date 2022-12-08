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
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_RENDERING_OGRE2_OGRE2DYNAMICMESH_HH_
#define GZ_RENDERING_OGRE2_OGRE2DYNAMICMESH_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/math.hh>

#include <gz/rendering/ogre2/Export.hh>
#include <gz/rendering/ogre2/Ogre2Geometry.hh>
#include <gz/rendering/ogre2/Ogre2RenderTypes.hh>
#include <gz/rendering/Marker.hh>

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreHlmsPso.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace Ogre
{
class MovableObject;
}  // namespace Ogre

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

// forward declarations
class Ogre2DynamicMeshPrivate;

/*  \class Ogre2DynamicMesh Ogre2DynamicMesh.hh \
  *  gz/rendering/ogre2/Ogre2DynamicMesh.hh
  */
/// \brief Dynamic mesh class that manages hardware buffers for
/// a dynamic mesh
class GZ_RENDERING_OGRE2_VISIBLE Ogre2DynamicMesh :
    public Ogre2Geometry
{
  /// \brief Constructor
  /// \param[in] _scene Pointer to scene
  public: explicit Ogre2DynamicMesh(ScenePtr _scene);

  /// \brief Virtual destructor
  public: virtual ~Ogre2DynamicMesh();

  // Documentation inherited.
  public: virtual void Destroy() override;

  // Documentation inherited.
  public: virtual Ogre::MovableObject *OgreObject() const override;

  // Documentation inherited.
  public: virtual MaterialPtr Material() const override;

  // Documentation inherited.
  public: virtual void
      SetMaterial(MaterialPtr _material, bool _unique) override;

  /////////// FROM DYNAMIC RENDERABLE

  /// \brief Set the render operation type
  /// \param[in] _opType The type of render operation to perform.
  public: void SetOperationType(MarkerType _opType);

  /// \brief Get the render operation type
  /// \return The render operation type.
  public: MarkerType OperationType() const;

  /// \brief Update the dynamic renderable
  public: void Update();

  /// \brief Add a point to the point list
  /// \param[in] _pt gz::math::Vector3d point
  /// \param[in] _color gz::math::Color Point color
  public: void AddPoint(const gz::math::Vector3d &_pt,
        const gz::math::Color &_color = gz::math::Color::White);

  /// \brief Add a point to the point list.
  /// \param[in] _x X position
  /// \param[in] _y Y position
  /// \param[in] _z Z position
  /// \param[in] _color Point color
  public: void AddPoint(const double _x, const double _y, const double _z,
        const gz::math::Color &_color = gz::math::Color::White);

  /// \brief Change the location of an existing point in the point list
  /// \param[in] _index Index of the point to set
  /// \param[in] _value Position of the point
  public: void SetPoint(unsigned int _index,
                        const gz::math::Vector3d &_value);

  /// \brief Change the color of an existing point in the point list
  /// \param[in] _index Index of the point to set
  /// \param[in] _color color to set the point to
  public: void SetColor(unsigned int _index,
                        const gz::math::Color &_color);

  /// \brief Change the normal vector of an existing point in the point list
  /// \param[in] _index Index of the point to set
  /// \param[in] _normal Normal of the point
  public: void SetNormal(unsigned int _index,
                        const gz::math::Vector3d &_normal);

  /// \brief Change the tangent vector of an existing point in the point list
  /// \param[in] _index Index of the point to set
  /// \param[in] _normal Tangent of the point
  public: void SetTangent(unsigned int _index,
                        const gz::math::Vector3d &_tangent);

  /// \brief Change the texture coordinate of an existing point
  ///        in the point list
  /// \param[in] _index Index of the point to set
  /// \param[in] _uv0 Texture coordinate of the point
  public: void SetUV0(unsigned int _index,
                        const gz::math::Vector2d &_uv0);

  /// \brief Return the position of an existing point in the point list
  /// \param[in] _index Get the point at this index
  /// \return position of point. A vector of
  /// [gz::math::INF_D, gz::math::INF_D, gz::math::INF_D]
  /// is returned when then the _index is out of bounds.
  /// gz::math::INF_D==std::numeric_limits<double>::infinity()
  public: gz::math::Vector3d Point(unsigned int _index) const;

  /// \brief Return the total number of points in the point list
  /// \return Number of points
  public: unsigned int PointCount() const;

  /// \brief Remove all points from the point list
  public: void Clear();

  /// \brief Create the dynamic mesh
  private: void CreateDynamicMesh();

  /// \brief Update vertex buffer if vertices have changes
  private: void UpdateBuffer();

  /// \brief Helper function to generate normals
  /// \param[in] _opType Ogre render operation type
  /// \param[in] _vertices a list of vertices
  /// \param[in,out] _vbuffer vertex buffer to be filled
  private: void GenerateNormals(Ogre::OperationType _opType,
      const std::vector<gz::math::Vector3d> &_vertices, float *_vbuffer);

  /// \brief Helper function to generate colors per-vertex. Only applies
  /// to points. The colors fill the normal slots on the vertex buffer.
  /// \param[in] _opType Ogre render operation type
  /// \param[in] _vertices a list of vertices
  /// \param[in,out] _vbuffer vertex buffer to be filled
  private: void GenerateColors(Ogre::OperationType _opType,
      const std::vector<gz::math::Vector3d> &_vertices, float *_vbuffer);

  /// \brief Destroy the vertex buffer
  private: void DestroyBuffer();

  /// \brief Pointer to private data
  private: std::unique_ptr<Ogre2DynamicMeshPrivate> dataPtr;
};

typedef std::shared_ptr<Ogre2DynamicMesh> Ogre2DynamicMeshPtr;

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_OGRE2_OGRE2DYNAMICMESH_HH_
