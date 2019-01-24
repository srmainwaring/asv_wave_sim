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

#include "asv_wave_sim_gazebo_plugins/Geometry.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/MeshTools.hh"

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>

#include <array>
#include <functional>
#include <iostream>
#include <cmath>
#include <limits>
#include <string>

namespace asv 
{
  // Typedefs
  typedef boost::optional<AABBTree::Intersection_and_primitive_id<Ray>::Type> RayIntersection;

  double Geometry::TriangleArea(
    const Point3& _p0,
    const Point3& _p1,
    const Point3& _p2
  )
  {
    Vector3 e1 = _p1 - _p0;
    Vector3 e2 = _p2 - _p0;
    return 0.5 * std::sqrt(CGAL::cross_product(e1, e2).squared_length());
  }

  double Geometry::TriangleArea(
    const Triangle& _tri
  )
  {
    return TriangleArea(_tri[0], _tri[1], _tri[2]);
  }

  Point3 Geometry::TriangleCentroid(
    const Point3& _p0,
    const Point3& _p1,
    const Point3& _p2
  )
  {
    return CGAL::centroid(_p0, _p1, _p2);
  }

  Point3 Geometry::TriangleCentroid(
    const Triangle& _tri
  )
  {
    return CGAL::centroid(_tri[0], _tri[1], _tri[2]);
  }

  Point3 Geometry::MidPoint(
    const Point3& _p0,
    const Point3& _p1
  )
  {
    return CGAL::midpoint(_p0, _p1);
  }

  Point3 Geometry::Normalize(const Point3& _p)
  {
    if (_p == CGAL::ORIGIN)
      return _p;
    else
    {
      Vector3 v = _p - CGAL::ORIGIN;
      double norm = std::sqrt(v.squared_length());
      return Point3(_p.x()/norm, _p.y()/norm, _p.z()/norm);
    }
  }

  Vector2 Geometry::Normalize(const Vector2& _v)
  {
    if (_v == CGAL::NULL_VECTOR)
      return _v;
    else
      return _v/std::sqrt(_v.squared_length()); 
  }

  Vector3 Geometry::Normalize(const Vector3& _v)
  {
    if (_v == CGAL::NULL_VECTOR)
      return _v;
    else
      return _v/std::sqrt(_v.squared_length()); 
  }

  Vector3 Geometry::Normal(
    const Point3& _p0,
    const Point3& _p1,
    const Point3& _p2
  )
  {
    auto n = CGAL::normal(_p0, _p1, _p2);
    if (n == CGAL::NULL_VECTOR)
      return n;
    else
      return n/std::sqrt(n.squared_length()); 
  }

  Vector3 Geometry::Normal(
    const Triangle& _tri
  )
  {
    auto n = CGAL::normal(_tri[0], _tri[1], _tri[2]);
    if (n == CGAL::NULL_VECTOR)
      return n;
    else
      return n/std::sqrt(n.squared_length()); 
  }

  Vector3 Geometry::Normal(const Mesh& _mesh, FaceIndex _face)
  {
    HalfedgeIndex hf = _mesh.halfedge(_face);
    const Point3& p0 = _mesh.point(_mesh.target(hf));
    hf = _mesh.next(hf);
    const Point3& p1= _mesh.point(_mesh.target(hf));
    hf = _mesh.next(hf);
    const Point3& p2 = _mesh.point(_mesh.target(hf));
    hf = _mesh.next(hf);

    auto n = CGAL::normal(p0, p1, p2);

    if (n == CGAL::NULL_VECTOR)
      return n;
    else
      return n/std::sqrt(n.squared_length()); 
  }

  Point3 Geometry::HorizontalIntercept(
    const Point3& _high,
    const Point3& _mid,
    const Point3& _low
  )
  {
    // If _high.Z() = _low().Z() then _high.Z() = _low.Z() = _mid.Z()
    // so any point on the line LH will satisfy the intercept condition,
    // including t=0, which we set as default (and so return _low).
    double t = 0.0;
    double div = _high.z() - _low.z();
    if (std::abs(div) > std::numeric_limits<double>::epsilon())
    {
      t = (_mid.z() - _low.z()) / div;
    }
    return Point3(
      _low.x() + t * (_high.x() - _low.x()),
      _low.y() + t * (_high.y() - _low.y()),
      _mid.z()
    );
  }

  bool Geometry::RayIntersectsTriangle(
    const Point3& _origin,
    const Direction3& _direction,
    const Triangle& _tri,
    Point3& _intersection
  )
  {    
    const Point3& p0 = _tri[0];
    const Point3& p1 = _tri[1];  
    const Point3& p2 = _tri[2];
    Vector3 _ray(_direction.vector());
    Vector3 e1 = p1 - p0;
    Vector3 e2 = p2 - p0;
    Vector3 h = CGAL::cross_product(_ray, e2);
    double a = CGAL::scalar_product(e1, h);
    if (a > -std::numeric_limits<double>::epsilon()
      && a < std::numeric_limits<double>::epsilon())
        return false;    // This ray is parallel to this triangle.
    double f = 1.0 / a;
    Vector3 s = _origin - p0;
    double u = f * CGAL::scalar_product(s, h);
    if (u < 0.0 || u > 1.0)
        return false;
    Vector3 q = CGAL::cross_product(s, e1);
    double v = f * CGAL::scalar_product(_ray, q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    double t = f * CGAL::scalar_product(e2, q);
    if (t > std::numeric_limits<double>::epsilon()) 
    {
      // ray intersection
      _intersection = _origin + _ray * t;
      return true;
    }
    else 
    {
      // This means that there is a line intersection but not a ray intersection.
      return false;
    }
  }

  bool Geometry::LineIntersectsTriangle(
    const Point3& _origin,
    const Direction3& _direction,
    const Triangle& _tri,
    Point3& _intersection
  )
  {
    return LineIntersectsTriangle(
      _origin, _direction, _tri[0], _tri[1], _tri[2], _intersection);
  }

  bool Geometry::LineIntersectsTriangle(
    const Point3& _origin,
    const Direction3& _direction,
    const Point3& _p0,
    const Point3& _p1,  
    const Point3& _p2,
    Point3& _intersection
  )
  {
    Vector3 _ray(_direction.vector());
    Vector3 e1 = _p1 - _p0;
    Vector3 e2 = _p2 - _p0;
    Vector3 h = CGAL::cross_product(_ray, e2);
    double a = CGAL::scalar_product(e1, h);
    if (a > -std::numeric_limits<double>::epsilon()
      && a < std::numeric_limits<double>::epsilon())
        return false;    // This ray is parallel to this triangle.
    double f = 1.0 / a;
    Vector3 s = _origin - _p0;
    double u = f * CGAL::scalar_product(s, h);
    if (u < 0.0 || u > 1.0)
        return false;
    Vector3 q = CGAL::cross_product(s, e1);
    double v = f * CGAL::scalar_product(_ray, q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    double t = f * CGAL::scalar_product(e2, q);
    _intersection = _origin + _ray * t;
    return true;
  }

  Triangle Geometry::MakeTriangle(const Mesh& _mesh, FaceIndex _face)
  {
    HalfedgeIndex hf = _mesh.halfedge(_face);
    const Point3& p0 = _mesh.point(_mesh.target(hf));
    hf = _mesh.next(hf);
    const Point3& p1 = _mesh.point(_mesh.target(hf));
    hf = _mesh.next(hf);
    const Point3& p2 = _mesh.point(_mesh.target(hf));
    hf = _mesh.next(hf);
    return Triangle(p0, p1, p2);
  }

  std::shared_ptr<AABBTree> Geometry::MakeAABBTree(const Mesh& _mesh)
  {
    std::shared_ptr<AABBTree> tree(
      new AABBTree(faces(_mesh).first, faces(_mesh).second, _mesh));
    tree->build();
    return tree;
  }

  bool Geometry::SearchMesh(
    const Mesh& _mesh,
    const Point3& _origin,
    const Direction3& _direction,
    Point3& _intersection
  )
  {
    // AABB Tree
    AABBTree tree(faces(_mesh).first, faces(_mesh).second, _mesh);
    tree.build();
    
    // Query
    Ray query(_origin, _direction);
    RayIntersection intersection = tree.first_intersection(query);

    // Search both directions
    if (!intersection)
      intersection = tree.first_intersection(query.opposite());

    // Retrieve intersection point
    if (intersection)
    {
      if (boost::get<Point3>(&(intersection->first)))
      {
        const Point3* p =  boost::get<Point3>(&(intersection->first));
        _intersection = *p;
        return true;
      }
    }
    return false;
  }

  bool Geometry::SearchMesh(
    const AABBTree& _tree,
    const Point3& _origin,
    const Direction3& _direction,
    Point3& _intersection
  )
  {
    // Query
    Ray query(_origin, _direction);
    RayIntersection intersection = _tree.first_intersection(query);

    // Search both directions
    if (!intersection)
      intersection = _tree.first_intersection(query.opposite());

    // Retrieve intersection point
    if (intersection)
    {
      if (boost::get<Point3>(&(intersection->first)))
      {
        const Point3* p =  boost::get<Point3>(&(intersection->first));
        _intersection = *p;
        return true;
      }
    }
    return false;
  }

} // namespace asv

