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

/// \file Grid.hh
/// \brief This file contains classes to construct planar meshes and to
/// find line intersections with them.

#ifndef GZ_WAVES_GRID_HH_
#define GZ_WAVES_GRID_HH_

#include <array>
#include <memory>

#include "gz/waves/CGALTypes.hh"
#include "gz/waves/Types.hh"

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
/// \internal Class to hold private data for Grid.
class GridPrivate;

/// \brief An accessing / partitioning scheme for a planar SurfaceMesh.
class Grid
{
 public:
  /// \brief Constructor. Initialise the SurfaceMesh comprising the grid.
  ///
  /// \param[in] _size        The size of the grid in the x, y directions.
  /// \param[in] _cellCount   The number of cells in each direction.
  Grid(
    const std::array<double, 2>& _size,
    const std::array<Index, 2>& _cellCount);

  /// \brief Copy constructor. Performs a deep copy.
  ///
  /// \param[in] _other   The source grid to copy.
  Grid(const Grid& _other);

  /// \brief Copy assignment. Performs a deep copy.
  ///
  /// \param[in] _other   The source grid to copy.
  /// \return             A reference to this, the copy.
  Grid& operator=(const Grid& _other);

  /// \brief Get the CGAL SurfaceMesh comprising the grid.
  ///
  /// \return             A pointer to the mesh.
  std::shared_ptr<const cgal::Mesh> GetMesh() const;

  /// \brief Get the CGAL SurfaceMesh comprising the grid (mutable).
  ///
  /// \return             A mutable pointer to the mesh.
  std::shared_ptr<cgal::Mesh> GetMesh();

  /// \brief Get the CGAL SurfaceMesh comprising the grid.
  ///
  /// \return             An immutable reference to the mesh.
  const cgal::Mesh& GetMeshByRef() const;

  /// \brief Get the size of the grid in each direction.
  ///
  /// \return             An array containing the grid size.
  const std::array<double, 2>& GetSize() const;

  /// \brief Get the number of cells the partioning the grid.
  ///
  /// \return             An array containing the number of grid cells.
  const std::array<Index, 2>& GetCellCount() const;

  /// \brief Get the number of vertices (and points).
  ///
  /// \return             The number of grid vertices.
  Index GetVertexCount() const;

  /// \brief Get the number of faces (and triangles, normals).
  ///
  /// \return             The number of grid faces (triangles).
  Index GetFaceCount() const;

  /// \brief Get the _i-th point.
  ///
  /// \return             A point.
  const cgal::Point3& GetPoint(Index _i) const;

  /// \brief Set the _i-th point.
  ///
  /// \param[in] _i       Index to the i-th point. Must be less than
  ///                     GetVertexCount().
  /// \param[in] _point   The point to set.
  void SetPoint(Index _i, const cgal::Point3& _point);

  /// \brief Get the _k-th Triangle in cell(_ix, _iy), _k = 0, 1.
  ///
  /// \param[in] _ix      Index to the ix-th grid cell.
  /// \param[in] _iy      Index to the iy-th grid cell.
  /// \param[in] _k       Index to the k-th face in cell (ix, iy).
  /// \return             A triangle.
  cgal::Triangle GetTriangle(Index _ix, Index _iy, Index _k) const;

  /// \brief Get the _k-th Face (triangle) in cell(_ix, _iy), _k = 0, 1.
  ///
  /// \param[in] _ix      Index to the ix-th grid cell.
  /// \param[in] _iy      Index to the iy-th grid cell.
  /// \param[in] _k       Index to the k-th face in cell (ix, iy).
  /// \return             A face index.
  cgal::FaceIndex GetFace(Index _ix, Index _iy, Index _k) const;

  /// \brief Get the _k-th Triangle normal in cell(_ix, _iy), _k = 0, 1.
  ///
  /// \param[in] _ix      Index to the ix-th grid cell.
  /// \param[in] _iy      Index to the iy-th grid cell.
  /// \param[in] _k       Index to the k-th face in cell (ix, iy).
  /// \return             The face normal vector.
  const cgal::Vector3& GetNormal(Index _ix, Index _iy, Index _k) const;

  /// \brief Get the Triangle normal in face _idx (face indexing).
  ///
  /// \param[in] _idx     A face index.
  /// \return             The face normal vector.
  const cgal::Vector3& GetNormal(Index _idx) const;

  /// \brief Recalculate the normals.
  void RecalculateNormals();

  /// \brief Get the position of the grid center.
  const cgal::Point3& GetCenter() const;

  /// \brief Set the position of the grid center.
  ///
  /// \param[in] _center  Set the location of the center
  ///                     of the grid (xy-plane only).
  void SetCenter(const cgal::Point3& _center);

  /// \brief Output the grid properties to std::cout.
  ///
  void DebugPrint() const;

 private:
  /// \internal Private implementation.
  std::shared_ptr<GridPrivate> data;
};

//////////////////////////////////////////////////
/// \brief A collection of methods for finding line intersections with a Grid.
class GridTools
{
 public:
  /// \brief Find the index of a grid cell and triangle that contains the
  /// coordinate (_x, _y)
  ///
  /// \param[in] _grid          The grid to search.
  /// \param[in] _x             The x coordinate.
  /// \param[in] _y             The y coordinate.
  /// \param[out] _index        The grid cell and face index (ix, iy, k).
  /// \return                   True if an intersection is found.
  static bool FindIntersectionIndex(
    const Grid& _grid,
    double _x, double _y,
    std::array<Index, 3>& _index);

  /// \brief Search one triangle for an intersection with the line
  ///        defined by origin and direction.
  ///
  /// \param[in] _grid          The grid to search.
  /// \param[in] _origin        The origin of the line.
  /// \param[in] _direction     The direction of the line.
  /// \param[in] _index         The grid cell and face/triangle
  ///                           index (ix, iy, k).
  /// \param[out] _intersection The intersection point if found.
  /// \return                   True if an intersection is found.
  static bool FindIntersectionTriangle(
    const Grid& _grid,
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    const std::array<Index, 3>& _index,
    cgal::Point3& _intersection);

  /// \brief Search the two triangles in a cell for an intersection with
  ///        the line defined by origin and direction.
  ///
  /// \param[in] _grid          The grid to search.
  /// \param[in] _origin        The origin of the line.
  /// \param[in] _direction     The direction of the line.
  /// \param[in, out] _index    [in] The initial (ix, iy, k) indices of
  ///                           the cell,
  ///                           [out] The index of the face where the
  ///                           intersection occurred.
  /// \param[out] _intersection The intersection point if found.
  /// \return                   True if an intersection is found.
  static bool FindIntersectionCell(
    const Grid& _grid,
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    std::array<Index, 3>& _index,
    cgal::Point3& _intersection);

  /// Search a grid for an intersection with the line defined by origin
  /// and direction. The search starts with a initial guess cell, then
  /// expands in shells one cell wide about the initial cell
  /// (adjusted for boundaries).
  //
  /// \param[in] _grid          The grid to search.
  /// \param[in] _origin        The origin of the line.
  /// \param[in] _direction     The direction of the line.
  /// \param[in, out] _index    [in] The initial (ix, iy, k) indices
  ///                           of the cell,
  ///                           [out] The index of the face where the
  ///                           intersection occurred.
  /// \param[out] _intersection The intersection point if found.
  /// \return                   True if an intersection is found.
  static bool FindIntersectionGrid(
    const Grid& _grid,
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    std::array<Index, 3>& _index,
    cgal::Point3& _intersection);
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_GRID_HH_
