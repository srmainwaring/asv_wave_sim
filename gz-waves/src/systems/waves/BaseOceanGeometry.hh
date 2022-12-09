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

#ifndef GZ_RENDERING_BASE_BASEOCEANGEOMETRY_HH_
#define GZ_RENDERING_BASE_BASEOCEANGEOMETRY_HH_

#include <string>

#include "OceanGeometry.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

/// \brief Base implementation of an Ocean geometry
template <class T>
class BaseOceanGeometry :
    public virtual OceanGeometry,
    public virtual T
{
  /// \brief Constructor
  public: BaseOceanGeometry();

  /// \brief Destructor
  public: virtual ~BaseOceanGeometry();

  /// \brief Load from a mesh
  public: virtual void LoadMesh(gz::common::MeshPtr _mesh) override;

  /// \brief Update from a mesh
  public: virtual void UpdateMesh(gz::common::MeshPtr _mesh) override;

  /// \brief Work-around the protected accessors and methods in Scene
  public: virtual void InitObject(ScenePtr _scene,
      unsigned int _id, const std::string &_name) override;
};

/////////////////////////////////////////////////
// BaseOceanGeometry
/////////////////////////////////////////////////
template <class T>
BaseOceanGeometry<T>::BaseOceanGeometry()
{
}

/////////////////////////////////////////////////
template <class T>
BaseOceanGeometry<T>::~BaseOceanGeometry()
{
}

/////////////////////////////////////////////////
template <class T>
void BaseOceanGeometry<T>::LoadMesh(gz::common::MeshPtr /*_mesh*/)
{
  // no default implementation
}

/////////////////////////////////////////////////
template <class T>
void BaseOceanGeometry<T>::UpdateMesh(gz::common::MeshPtr /*_mesh*/)
{
  // no default implementation
}

/////////////////////////////////////////////////
template <class T>
void BaseOceanGeometry<T>::InitObject(ScenePtr /*_scene*/,
    unsigned int /*_id*/, const std::string &/*_name*/)
{
  // no default implementation
}

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_BASE_BASEOCEANGEOMETRY_HH_
