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

/// \file CGALTypes.hh
/// \brief Type definitions for CGAL structures used in the library.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_CGAL_TYPES_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_CGAL_TYPES_HH_

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <memory>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// CGAL Typedefs

  // 2D/3D Linear Geometry
  typedef CGAL::Simple_cartesian<double>  Kernel;
  typedef Kernel::Direction_2             Direction2;
  typedef Kernel::Direction_3             Direction3;
  typedef Kernel::Point_3                 Point3;
  typedef Kernel::Line_3                  Line;
  typedef Kernel::Ray_3                   Ray;
  typedef Kernel::Triangle_3              Triangle;
  typedef Kernel::Vector_2                Vector2;
  typedef Kernel::Vector_3                Vector3;

  // SurfaceMesh
  typedef CGAL::Surface_mesh<Point3>      Mesh;
  typedef Mesh::Face_index                FaceIndex;
  typedef Mesh::Halfedge_index            HalfedgeIndex; 
  typedef Mesh::Vertex_index              VertexIndex; 

  // AABB Tree
  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> AABBTree;

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_CGAL_TYPES_HH_
