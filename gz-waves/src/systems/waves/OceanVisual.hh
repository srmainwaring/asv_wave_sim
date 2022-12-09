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

#ifndef GZ_RENDERING_OCEANVISUAL_HH_
#define GZ_RENDERING_OCEANVISUAL_HH_

#include <memory>
#include <string>

#include <gz/common/graphics/Types.hh>

#include <gz/rendering/config.hh>
#include <gz/rendering/Object.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Visual.hh>

#include "gz/waves/OceanTile.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

/// \brief Ocean visual using a dynamic mesh
class GZ_RENDERING_VISIBLE OceanVisual :
  public virtual Visual
{
  /// \brief Destructor
  public: virtual ~OceanVisual();

  /// \brief Load a dynamic cube (example)
  public: virtual void LoadCube() = 0;

  /// \brief Load from an ocean tile
  public: virtual void LoadOceanTile(
      waves::visual::OceanTilePtr _oceanTile) = 0;

  /// \brief Update from an ocean tile
  public: virtual void UpdateOceanTile(
      waves::visual::OceanTilePtr _oceanTile) = 0;

  /// \brief Load from a mesh
  public: virtual void LoadMesh(gz::common::MeshPtr _mesh) = 0;

  /// \brief Update from a mesh
  public: virtual void UpdateMesh(gz::common::MeshPtr _mesh) = 0;

  /// \brief Work-around the protected accessors and methods in Scene
  public: virtual void InitObject(ScenePtr _scene,
      unsigned int _id, const std::string &_name) = 0;

};

typedef std::shared_ptr<OceanVisual> OceanVisualPtr;

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_OCEANVISUAL_HH_
