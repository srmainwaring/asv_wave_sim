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

#ifndef IGNITION_GAZEBO_SYSTEMS_OCEANTILE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OCEANTILE_HH_

#include <ignition/math.hh>
#include <ignition/common.hh>
#include <ignition/common/Mesh.hh>

#include <memory>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

  class OceanTilePrivate;

  class OceanTile
  {
    public: virtual ~OceanTile();

    public: OceanTile(unsigned int _N, double _L, bool _hasVisuals=true);

    /// \brief The tile size (or length) L. 
    double TileSize() const;

    /// \brief The tile resolution (N). The tile contains (N + 1)**2 vertices. 
    unsigned int Resolution() const;

    public: void SetWindVelocity(double _ux, double _uy);

    // Returns a new common::Mesh. The caller must take ownership.
    public: common::Mesh* CreateMesh();

    public: void Update(double _time);

    public: void UpdateMesh(double _time, common::Mesh *_mesh);


    ////////////////////////////////////////
    // Access to vertices, texture coordinates and faces
    public: unsigned int VertexCount() const;
    public: math::Vector3d Vertex(unsigned int _index) const;
    public: math::Vector2d UV0(unsigned int _index) const;

    public: unsigned int FaceCount() const;
    public: math::Vector3i Face(unsigned int _index) const;

    private: std::unique_ptr<OceanTilePrivate> dataPtr;
  };

  typedef std::shared_ptr<OceanTile> OceanTilePtr;

}
}
}

#endif
