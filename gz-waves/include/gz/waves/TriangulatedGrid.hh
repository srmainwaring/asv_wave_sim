// Copyright (C) 2019  Rhys Mainwaring
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

/// \file TriangulatedGrid.hh

#ifndef GZ_WAVES_TRIANGULATEDGRID_HH_
#define GZ_WAVES_TRIANGULATEDGRID_HH_

#include <array>
#include <memory>
#include <vector>

#include "gz/waves/CGALTypes.hh"
#include "gz/waves/Types.hh"

#include <gz/math.hh>

namespace gz
{
namespace waves
{

typedef std::array<int64_t, 3>    Index3;
typedef std::vector<cgal::Point3> Point3Range;
typedef std::vector<Index3>       Index3Range;

class TriangulatedGrid
{
 public:
  virtual ~TriangulatedGrid();
  TriangulatedGrid(Index nx, Index ny, double lx, double ly);
  void CreateMesh();
  void CreateTriangulation();
  static std::unique_ptr<TriangulatedGrid> Create(
      Index nx, Index ny, double lx, double ly);

  bool Locate(const cgal::Point3& query, int64_t& faceIndex) const;
  bool Height(const cgal::Point3& query, double& height) const;
  bool Height(const std::vector<cgal::Point3>& queries,
      std::vector<double>& heights) const;

  bool Interpolate(TriangulatedGrid& patch) const;

  const Point3Range& Points() const;
  const Index3Range& Indices() const;
  const cgal::Point3& Origin() const;
  void ApplyPose(const math::Pose3d& pose);

  bool IsValid(bool verbose = false) const;
  void DebugPrintMesh() const;
  void DebugPrintTriangulation() const;
  void UpdatePoints(const std::vector<cgal::Point3>& from);
  void UpdatePoints(const std::vector<math::Vector3d>& from);
  void UpdatePoints(const cgal::Mesh& from);

  std::array<double, 2> TileSize() const;
  std::array<Index, 2> CellCount() const;

 private:
  class Private;
  std::unique_ptr<Private> impl_;
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_TRIANGULATEDGRID_HH_
