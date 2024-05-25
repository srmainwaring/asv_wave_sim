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

#ifndef GZ_WAVES_OCEANTILE_HH_
#define GZ_WAVES_OCEANTILE_HH_

#include <array>
#include <memory>
#include <vector>

#include <gz/common/Mesh.hh>

#include "gz/waves/CGALTypes.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/WaveParameters.hh"

namespace gz
{
namespace waves
{
template <typename Vector3>
class OceanTilePrivate;

template <typename Vector3>
class OceanTileT
{
 public:
  virtual ~OceanTileT();

  explicit OceanTileT(Index nx, Index ny, double lx, double ly,
      bool has_visuals = true);

  explicit OceanTileT(WaveParametersPtr params, bool has_visuals = true);

  /// \brief The size of the wave tile (m).
  std::array<double, 2> TileSize() const;

  /// \brief The number of cells in the wave tile in each direction.
  /// The tile contains (nx + 1) * (ny + 1) vertices.
  std::array<Index, 2> CellCount() const;

  void SetWindVelocity(double ux, double uy);

  void SetSteepness(double value);

  void Create();

  // Returns a new gz::common::Mesh. The caller must take ownership.
  gz::common::Mesh* CreateMesh();

  void Update(double time);

  void UpdateMesh(double time, gz::common::Mesh* mesh);

  // Access to vertices, texture coordinates and faces
  Index VertexCount() const;

  Vector3 Vertex(Index index) const;

  gz::math::Vector2d UV0(Index index) const;

  Index FaceCount() const;

  gz::math::Vector3i Face(Index index) const;

  const std::vector<Vector3>& Vertices() const;

 private:
  std::unique_ptr<OceanTilePrivate<Vector3>> impl_;
};

namespace visual
{
typedef OceanTileT<gz::math::Vector3d> OceanTile;
typedef std::shared_ptr<OceanTile> OceanTilePtr;
}  // namespace visual

namespace physics
{
typedef OceanTileT<cgal::Point3>   OceanTile;
typedef std::shared_ptr<OceanTile> OceanTilePtr;
}  // namespace physics

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_OCEANTILE_HH_
