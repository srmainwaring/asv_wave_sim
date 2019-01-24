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
/// \brief This file contains methods to calculate properties of simple geometrical objects.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_GEOMETRY_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_GEOMETRY_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <array>

namespace asv
{
  class Grid;

///////////////////////////////////////////////////////////////////////////////
// Geometry

  /// \brief A collection of static methods concerning linear geometry.
  class Geometry
  {
    /// \brief Calculate the area of a triangle.
    ///
    /// \param[in] _p0    Point at the first vertex.
    /// \param[in] _p1    Point at the second vertex.
    /// \param[in] _p2    Point at the third vertex.
    /// \return           The triangle area
    public: static double TriangleArea(
      const Point3& _p0,
      const Point3& _p1,
      const Point3& _p2
    );

    /// \brief Calculate the area of a triangle.
    ///
    /// \param[in] _tri   A triangle.
    /// \return           The triangle area
    public: static double TriangleArea(
      const Triangle& _tri
    );

    /// \brief Calculate the centroid of a triangle.
    ///
    /// \param[in] _p0    Point at the first vertex.
    /// \param[in] _p1    Point at the second vertex.
    /// \param[in] _p2    Point at the third vertex.
    /// \return           The triangle centroid
    public: static Point3 TriangleCentroid(
      const Point3& _p0,
      const Point3& _p1,
      const Point3& _p2
    );

    /// \brief Calculate the centroid of a triangle.
    ///
    /// \param[in] _tri   A triangle.
    /// \return           The triangle centroid
    public: static Point3 TriangleCentroid(
      const Triangle& _tri
    );

    /// \brief Find the mid-point of a line between points _p0 and _p1.
    ///
    /// \param[in] _p0    First point.
    /// \param[in] _p1    Second point.
    /// \return           The midpoint
    public: static Point3 MidPoint(
      const Point3& _p0,
      const Point3& _p1
    );

    /// \brief Calculate the point on a line from the origin passing through _p
    /// such that the vector from the origin to returned point has unit length.
    ///
    /// \param[in] _p     A point.
    /// \return           The point that normalises the vector from the origin to _p.
    public: static Point3 Normalize(const Point3& _p);

    /// \brief Normalise a Vector2 (i.e. ensure it has unit length)
    ///
    /// \param[in] _v     The vector to normalise.
    /// \return           The normalized vector.
    public: static Vector2 Normalize(const Vector2& _v);

    /// \brief Normalise a Vector3 (i.e. ensure it has unit length)
    ///
    /// \param[in] _v     The vector to normalise.
    /// \return           The normalized vector.
    public: static Vector3 Normalize(const Vector3& _v);

    /// \brief Compute the (normalised) normal to the plane defined by a triangle.
    ///
    /// \param[in] _p0    Point at the first vertex.
    /// \param[in] _p1    Point at the second vertex.
    /// \param[in] _p2    Point at the third vertex.
    /// \return           The normal vector.
    public: static Vector3 Normal(
      const Point3& _v0,
      const Point3& _v1,
      const Point3& _v2
    );

    /// \brief Compute the (normalized) normal to the plane defined by a triangle.
    ///
    /// \param[in] _tri   A triangle.
    /// \return           The normal vector.
    public: static Vector3 Normal(
      const Triangle& _tri
    );

    /// \brief Compute the (normalized) normal to the plane defined by a triangle face.
    ///
    /// \param[in] _mesh  A surface mesh.
    /// \param[in] _face  A face index for the surface mesh.
    /// \return           The normal vector.
    public: static Vector3 Normal(
      const Mesh& _mesh,
      FaceIndex _face
    );

    /// Given a triangle (_high, _mid, _low) with _low.z() <= _mid.z() <= _high.z(), find the intercept of a line 
    /// from _mid, normal to z and the line between vertex _low and vertex _mid.
    ///
    /// \param[in] _high  The point with greatest z-component _high.z().
    /// \param[in] _mid   The point with z-component lying between _high.z() and _low.z().
    /// \param[in] _low   The point with least z-component _low.z().
    /// \return           The intercept point.
    public: static Point3 HorizontalIntercept(
      const Point3& _high,
      const Point3& _mid,
      const Point3& _low
    );

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
    public: static bool RayIntersectsTriangle(
      const Point3& _origin,
      const Direction3& _direction,
      const Triangle& _tri,
      Point3& _intersection
    );

    /// \brief Fast Minimum Storage Ray-Triangle Interesection
    /// <http://webserver2.tecgraf.puc-rio.br/~mgattass/cg/trbRR/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf>
    ///
    /// Möller–Trumbore intersection algorithm
    /// <https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm>
    ///
    /// The same algorithm as above, but omitting the last check for the sign of t.
    ///
    /// \param[in] _origin          The origin of the ray.
    /// \param[in] _direction       The direction of the ray.
    /// \param[in] _tri             A triangle.
    /// \param[out] _intersection   The intersection point.
    public: static bool LineIntersectsTriangle(
      const Point3& _origin,
      const Direction3& _direction,
      const Triangle& _tri,
      Point3& _intersection
    );

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
    public: static bool LineIntersectsTriangle(
      const Point3& _origin,
      const Direction3& _direction,
      const Point3& _p0,
      const Point3& _p1,
      const Point3& _p2,
      Point3& _intersection
    );

    /// \brief Utility to make a triangle given a mesh and face index.
    ///
    /// \param[in] _mesh    A surface mesh.
    /// \param[in] _face    A face index for the surface mesh.
    /// \return             A triangle.
    public: static Triangle MakeTriangle(const Mesh& _mesh, FaceIndex _face);

    /// \brief Create an AABB tree for the input mesh.
    ///
    /// \param[in] _mesh            The mesh to search.
    /// \return                     The built AABB tree.
    public: static std::shared_ptr<AABBTree> MakeAABBTree(const Mesh& _mesh);

    /// \brief Search a mesh for an intersection with the line defined by origin and direction.
    /// NOTE: This version is slow because it builds a AABB tree for each query.
    ///
    /// \param[in] _mesh            The mesh to search.
    /// \param[in] _origin          The origin of the line.
    /// \param[in] _direction       The direction of the line (both directions are searched).
    /// \param[out] _intersection   The intersection point if found.
    public: static bool SearchMesh(
      const Mesh& _mesh,
      const Point3& _origin,
      const Direction3& _direction,
      Point3& _intersection
    );
  
    /// \brief Search a mesh for an intersection with the line defined by origin and direction.
    /// NOTE: This version is fast because it uses a pre-built AABB tree.
    ///
    /// \param[in] _tree            The AABB tree for the mesh being searched.
    /// \param[in] _origin          The origin of the line.
    /// \param[in] _direction       The direction of the line (both directions are searched).
    /// \param[out] _intersection   The intersection point if found.
    public: static bool SearchMesh(
      const AABBTree& _tree,
      const Point3& _origin,
      const Direction3& _direction,
      Point3& _intersection
    );

  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_GEOMETRY_HH_
