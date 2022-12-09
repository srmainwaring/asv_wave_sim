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

#ifndef GZ_RENDERING_BASE_BASEOCEANVISUAL_HH_
#define GZ_RENDERING_BASE_BASEOCEANVISUAL_HH_

#include <string>

#include <gz/rendering/base/BaseObject.hh>
#include <gz/rendering/base/BaseRenderTypes.hh>
#include <gz/rendering/Scene.hh>

#include "OceanVisual.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

/// \brief Base implementation of an Ocean visual
template <class T>
class BaseOceanVisual :
  public virtual OceanVisual,
  public virtual T
{
  /// \brief Constructor
  public: BaseOceanVisual();

  /// \brief Destructor
  public: virtual ~BaseOceanVisual();

  // Documentation inherited.
  public: virtual void Init() override;

  // Documentation inherited.
  public: virtual void PreRender() override;

  // Documentation inherited.
  protected: virtual void Destroy() override;

  // Documentation inherited.
  public: virtual MaterialPtr Material() const override;

  // Documentation inherited.
  public: virtual void SetMaterial(
    MaterialPtr _material, bool _unique) override;

  /// \brief Load a dynamic cube (example)
  public: virtual void LoadCube() override;

  /// \brief Load from an ocean tile
  public: virtual void LoadOceanTile(
      waves::visual::OceanTilePtr _oceanTile) override;

  /// \brief Update from an ocean tile
  public: virtual void UpdateOceanTile(
      waves::visual::OceanTilePtr _oceanTile) override;

  /// \brief Load from a mesh
  public: virtual void LoadMesh(gz::common::MeshPtr _mesh) override;

  /// \brief Update from a mesh
  public: virtual void UpdateMesh(gz::common::MeshPtr _mesh) override;

  /// \brief Work-around the protected accessors and protected methods in Scene
  public: virtual void InitObject(ScenePtr _scene,
      unsigned int _id, const std::string &_name) override;

};

//////////////////////////////////////////////////
template <class T>
BaseOceanVisual<T>::BaseOceanVisual()
{
}

//////////////////////////////////////////////////
template <class T>
BaseOceanVisual<T>::~BaseOceanVisual()
{
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::PreRender()
{
  T::PreRender();
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::Init()
{
  T::Init();
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::Destroy()
{
  T::Destroy();
}

//////////////////////////////////////////////////
template <class T>
MaterialPtr BaseOceanVisual<T>::Material() const
{
  return T::Material();
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::SetMaterial(
  MaterialPtr _material, bool _unique)
{
  T::SetMaterial(_material, _unique);
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::LoadCube()
{
  // no default implementation
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::LoadOceanTile(
  waves::visual::OceanTilePtr /*_oceanTile*/)
{
  // no default implementation
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::UpdateOceanTile(
  waves::visual::OceanTilePtr /*_oceanTile*/)
{
  // no default implementation
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::LoadMesh(gz::common::MeshPtr /*_mesh*/)
{
  // no default implementation
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::UpdateMesh(gz::common::MeshPtr /*_mesh*/)
{
  // no default implementation
}

//////////////////////////////////////////////////
template <class T>
void BaseOceanVisual<T>::InitObject(ScenePtr /*_scene*/,
    unsigned int /*_id*/, const std::string &/*_name*/)
{
  // no default implementation
}

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_BASE_BASEOCEANVISUAL_HH_
