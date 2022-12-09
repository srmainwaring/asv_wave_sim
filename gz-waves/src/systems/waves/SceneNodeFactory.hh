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

#ifndef GZ_RENDERING_SCENENODEFACTORY_HH_
#define GZ_RENDERING_SCENENODEFACTORY_HH_

#include <memory>

#include <gz/rendering/config.hh>
#include <gz/rendering/Scene.hh>
#include "gz/rendering/Export.hh"

#include "DisplacementMap.hh"
#include "OceanGeometry.hh"
#include "OceanVisual.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

class GZ_RENDERING_VISIBLE SceneNodeFactory
{
  /// \brief Destructor
  public: virtual ~SceneNodeFactory();

  /// \brief Create an ocean visual
  public: virtual OceanVisualPtr CreateOceanVisual(ScenePtr _scene) = 0;

  /// \brief Create an ocean geometry
  public: virtual OceanGeometryPtr CreateOceanGeometry(ScenePtr _scene) = 0;

  /// \brief Create a displacement map
  public: virtual DisplacementMapPtr CreateDisplacementMap(
      ScenePtr _scene,
      MaterialPtr _material,
      uint64_t _entity,
      uint32_t _width,
      uint32_t _height) = 0;
};

typedef std::shared_ptr<SceneNodeFactory> SceneNodeFactoryPtr;

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_SCENENODEFACTORY_HH_
