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

#ifndef GZ_MARINE_OCEANTILE_HH_
#define GZ_MARINE_OCEANTILE_HH_

#include "gz/marine/CGALTypes.hh"
#include "gz/marine/WaveParameters.hh"

#include <gz/math.hh>
#include <gz/common.hh>
#include <gz/common/Mesh.hh>

#include <memory>

namespace ignition
{
namespace marine
{
  template <typename Vector3>
  class OceanTilePrivate;

  template <typename Vector3>
  class OceanTileT
  {
    public: virtual ~OceanTileT();

    public: OceanTileT(unsigned int _N, double _L, bool _hasVisuals=true);

    public: OceanTileT(WaveParametersPtr _params, bool _hasVisuals=true);

    /// \brief The tile size (or length) L. 
    double TileSize() const;

    /// \brief The tile resolution (N). The tile contains (N + 1)**2 vertices. 
    unsigned int Resolution() const;

    public: void SetWindVelocity(double _ux, double _uy);

    public: void Create();

    // Returns a new common::Mesh. The caller must take ownership.
    public: common::Mesh* CreateMesh();

    public: void Update(double _time);

    public: void UpdateMesh(double _time, common::Mesh *_mesh);


    ////////////////////////////////////////
    // Access to vertices, texture coordinates and faces
    public: unsigned int VertexCount() const;
    public: Vector3 Vertex(unsigned int _index) const;
    public: math::Vector2d UV0(unsigned int _index) const;
    public: unsigned int FaceCount() const;
    public: math::Vector3i Face(unsigned int _index) const;
    public: const std::vector<Vector3>& Vertices() const;

    private: std::unique_ptr<OceanTilePrivate<Vector3>> dataPtr;
  };

  namespace visual
  {
    typedef OceanTileT<math::Vector3d> OceanTile;
    typedef std::shared_ptr<OceanTile> OceanTilePtr;
  }
  namespace physics
  {
    typedef OceanTileT<cgal::Point3>   OceanTile;
    typedef std::shared_ptr<OceanTile> OceanTilePtr;
  }
}
}

#endif
