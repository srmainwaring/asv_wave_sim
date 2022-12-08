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

/// \file Geometry.hh
/// \brief This file contains methods to calculate properties of simple
///        geometrical objects.

#ifndef GZ_WAVES_GEOMETRY_HH_
#define GZ_WAVES_GEOMETRY_HH_

#include <array>
#include <memory>

#include "gz/waves/CGALTypes.hh"

namespace gz
{
namespace waves
{
class Grid;

/// \brief A collection of static methods concerning linear geometry.
class Geometry
{
 public:
  /// \brief Calculate the area of a triangle.
  ///
  /// \param[in] _p0    Point at the first vertex.
  /// \param[in] _p1    Point at the second vertex.
  /// \param[in] _p2    Point at the third vertex.
  /// \return           The triangle area
  static double TriangleArea(
    const cgal::Point3& _p0,
    const cgal::Point3& _p1,
    const cgal::Point3& _p2);

  /// \brief Calculate the area of a triangle.
  ///
  /// \param[in] _tri   A triangle.
  /// \return           The triangle area
  static double TriangleArea(
    const cgal::Triangle& _tri);

  /// \brief Calculate the centroid of a triangle.
  ///
  /// \param[in] _p0    Point at the first vertex.
  /// \param[in] _p1    Point at the second vertex.
  /// \param[in] _p2    Point at the third vertex.
  /// \return           The triangle centroid
  static cgal::Point3 TriangleCentroid(
    const cgal::Point3& _p0,
    const cgal::Point3& _p1,
    const cgal::Point3& _p2);

  /// \brief Calculate the centroid of a triangle.
  ///
  /// \param[in] _tri   A triangle.
  /// \return           The triangle centroid
  static cgal::Point3 TriangleCentroid(
    const cgal::Triangle& _tri);

  /// \brief Find the mid-point of a line between points _p0 and _p1.
  ///
  /// \param[in] _p0    First point.
  /// \param[in] _p1    Second point.
  /// \return           The midpoint
  static cgal::Point3 MidPoint(
    const cgal::Point3& _p0,
    const cgal::Point3& _p1);

  /// \brief Calculate the point on a line from the origin passing through _p
  /// such that the vector from the origin to returned point has unit length.
  ///
  /// \param[in] _p     A point.
  /// \return           The point that normalises the vector from
  ///                   the origin to _p.
  static cgal::Point3 Normalize(const cgal::Point3& _p);

  /// \brief Normalise a Vector2 (i.e. ensure it has unit length)
  ///
  /// \param[in] _v     The vector to normalise.
  /// \return           The normalized vector.
  static cgal::Vector2 Normalize(const cgal::Vector2& _v);

  /// \brief Normalise a Vector3 (i.e. ensure it has unit length)
  ///
  /// \param[in] _v     The vector to normalise.
  /// \return           The normalized vector.
  static cgal::Vector3 Normalize(const cgal::Vector3& _v);

  /// \brief Compute the (normalised) normal to the plane defined
  ///        by a triangle.
  ///
  /// \param[in] _p0    Point at the first vertex.
  /// \param[in] _p1    Point at the second vertex.
  /// \param[in] _p2    Point at the third vertex.
  /// \return           The normal vector.
  static cgal::Vector3 Normal(
    const cgal::Point3& _v0,
    const cgal::Point3& _v1,
    const cgal::Point3& _v2);

  /// \brief Compute the (normalized) normal to the plane defined
  ///        by a triangle.
  ///
  /// \param[in] _tri   A triangle.
  /// \return           The normal vector.
  static cgal::Vector3 Normal(
    const cgal::Triangle& _tri);

  /// \brief Compute the (normalized) normal to the plane defined
  ///        by a triangle face.
  ///
  /// \param[in] _mesh  A surface mesh.
  /// \param[in] _face  A face index for the surface mesh.
  /// \return           The normal vector.
  static cgal::Vector3 Normal(
    const cgal::Mesh& _mesh,
    cgal::FaceIndex _face);

  /// Given a triangle (_high, _mid, _low) with
  /// _low.z() <= _mid.z() <= _high.z(), find the intercept of a line
  /// from _mid, normal to z and the line between vertex _low and
  /// vertex _mid.
  ///
  /// \param[in] _high  The point with greatest z-component _high.z().
  /// \param[in] _mid   The point with z-component lying between _high.z()
  ///                    and _low.z().
  /// \param[in] _low   The point with least z-component _low.z().
  /// \return           The intercept point.
  static cgal::Point3 HorizontalIntercept(
    const cgal::Point3& _high,
    const cgal::Point3& _mid,
    const cgal::Point3& _low);

  /// \brief Fast Minimum Storage Ray-Triangle Interesection
  /// <http://webserver2.tecgraf.puc-rio.br/~mgattass/cg/trbRR/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf>
  ///
  /// Möller–Trumbore intersection algorithm
  /// <https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm>
  ///
  /// \param[in] _origin          The origin of the ray.
  /// \param[in] _direction       The direction of the ray.
  /// \param[in] _tri             A triangle.
  /// \param[out] _intersection   The intersection point.
  static bool RayIntersectsTriangle(
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    const cgal::Triangle& _tri,
    cgal::Point3& _intersection);

  /// \brief Fast Minimum Storage Ray-Triangle Interesection
  /// <http://webserver2.tecgraf.puc-rio.br/~mgattass/cg/trbRR/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf>
  ///
  /// Möller–Trumbore intersection algorithm
  /// <https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm>
  ///
  /// The same algorithm as above, but omitting the last check for
  /// the sign of t.
  ///
  /// \param[in] _origin          The origin of the ray.
  /// \param[in] _direction       The direction of the ray.
  /// \param[in] _tri             A triangle.
  /// \param[out] _intersection   The intersection point.
  static bool LineIntersectsTriangle(
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    const cgal::Triangle& _tri,
    cgal::Point3& _intersection);

  /// \brief Fast Minimum Storage Ray-Triangle Interesection
  /// <http://webserver2.tecgraf.puc-rio.br/~mgattass/cg/trbRR/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf>
  ///
  /// Möller–Trumbore intersection algorithm
  /// <https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm>
  ///
  /// The same algorithm as above, but using a triangle's points.
  ///
  /// \param[in] _origin          The origin of the ray.
  /// \param[in] _direction       The direction of the ray.
  /// \param[in] _p0              Point at the first vertex.
  /// \param[in] _p1              Point at the second vertex.
  /// \param[in] _p2              Point at the third vertex.
  /// \param[out] _intersection   The intersection point.
  static bool LineIntersectsTriangle(
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    const cgal::Point3& _p0,
    const cgal::Point3& _p1,
    const cgal::Point3& _p2,
    cgal::Point3& _intersection);

  /// \brief Utility to make a triangle given a mesh and face index.
  ///
  /// \param[in] _mesh    A surface mesh.
  /// \param[in] _face    A face index for the surface mesh.
  /// \return             A triangle.
  static cgal::Triangle MakeTriangle(const cgal::Mesh& _mesh,
      cgal::FaceIndex _face);

  /// \brief Create an AABB tree for the input mesh.
  ///
  /// \param[in] _mesh            The mesh to search.
  /// \return                     The built AABB tree.
  static std::shared_ptr<cgal::AABBTree> MakeAABBTree(
      const cgal::Mesh& _mesh);

  /// \brief Search a mesh for an intersection with the line defined by
  ///        origin and direction.
  /// NOTE: This version is slow because it builds a AABB tree for each query.
  ///
  /// \param[in] _mesh            The mesh to search.
  /// \param[in] _origin          The origin of the line.
  /// \param[in] _direction       The direction of the line (both directions
  ///                             are searched).
  /// \param[out] _intersection   The intersection point if found.
  static bool SearchMesh(
    const cgal::Mesh& _mesh,
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    cgal::Point3& _intersection);

  /// \brief Search a mesh for an intersection with the line defined by
  ///       origin and direction.
  /// NOTE: This version is fast because it uses a pre-built AABB tree.
  ///
  /// \param[in] _tree            The AABB tree for the mesh being searched.
  /// \param[in] _origin          The origin of the line.
  /// \param[in] _direction       The direction of the line (both directions
  ///                             are searched).
  /// \param[out] _intersection   The intersection point if found.
  static bool SearchMesh(
    const cgal::AABBTree& _tree,
    const cgal::Point3& _origin,
    const cgal::Direction3& _direction,
    cgal::Point3& _intersection);
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_GEOMETRY_HH_
