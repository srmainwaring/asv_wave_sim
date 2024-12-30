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

#include <gtest/gtest.h>

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#if CGAL_VERSION_MAJOR >= 6
#include <CGAL/AABB_traits_3.h>
#else
#include <CGAL/AABB_traits.h>
#endif
#include <CGAL/AABB_tree.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>

#include <CGAL/algorithm.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/generators.h>
#include <CGAL/number_utils.h>
#include <CGAL/point_generators_2.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gz/waves/Geometry.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/MeshTools.hh"
#include "gz/waves/TriangulatedGrid.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Triangle.hh>

namespace waves
{
using gz::waves::Grid;
using gz::waves::MeshTools;
using gz::waves::TriangulatedGrid;
using gz::waves::Wavefield;
using gz::waves::WaveParameters;
}

//////////////////////////////////////////////////
// Utilities

// Extract a triangle from a face
namespace
{
typedef CGAL::Simple_cartesian<double> K;
typedef K::Triangle_3 Triangle;
typedef K::Point_3 Point3;
typedef K::Vector_3 Vector3;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Face_index Face_descriptor;
typedef Mesh::Halfedge_index Halfedge_descriptor;

std::shared_ptr<gz::cgal::Mesh> CreateSphere(
  const std::string& meshName,
  double radius
)
{
  gz::common::MeshManager::Instance()->CreateSphere(
    meshName,
    radius,                   // radius
    32,                       // rings
    32);                      // segments
  GZ_ASSERT(gz::common::MeshManager::Instance()->HasMesh(meshName),
    "Failed to create Mesh for Cylinder");

  const gz::common::Mesh* source =
      gz::common::MeshManager::Instance()->MeshByName(meshName);
  GZ_ASSERT(source != nullptr, "Invalid Sphere Mesh");
  // std::cout << "Mesh:       " << source->GetName() << "\n";
  // std::cout << "Vertex:     " << source->GetVertexCount() << "\n";

  std::shared_ptr<gz::cgal::Mesh> target = std::make_shared<gz::cgal::Mesh>();
  waves::MeshTools::MakeSurfaceMesh(*source, *target);
  return target;
}
}  // namespace

//////////////////////////////////////////////////
TEST(CGAL, Surprising) {
  typedef CGAL::Simple_cartesian<double> Kernel;
  typedef Kernel::Point_2 Point_2;

  {
    Point_2 p(0, 0.3), q(1, 0.6), r(2, 0.9);
    // std::cout << (CGAL::collinear(p,q,r) ?
    //  "collinear\n" : "not collinear\n");
  }
  {
    Point_2 p(0, 1.0/3.0), q(1, 2.0/3.0), r(2, 1);
    // std::cout << (CGAL::collinear(p,q,r) ?
    //  "collinear\n" : "not collinear\n");
  }
  {
    Point_2 p(0, 0), q(1, 1), r(2, 2);
    // std::cout << (CGAL::collinear(p,q,r) ?
    //  "collinear\n" : "not collinear\n");
  }
}

//////////////////////////////////////////////////
TEST(CGAL, SurfaceMesh) {
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  typedef K::Plane_3 Plane;
  // typedef K::Vector_3 Vector;
  typedef K::Segment_3 Segment;
  typedef CGAL::Surface_mesh<Point3> Mesh;
  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
#if CGAL_VERSION_MAJOR >= 6
  typedef CGAL::AABB_traits_3<K, Primitive> Traits;
#else
  typedef CGAL::AABB_traits<K, Primitive> Traits;
#endif
  typedef CGAL::AABB_tree<Traits> Tree;
#if CGAL_VERSION_MAJOR >= 6
  typedef std::optional<Tree::Intersection_and_primitive_id<Segment>::Type>
  Segment_intersection;
  typedef std::optional<Tree::Intersection_and_primitive_id<Plane>::Type>
  Plane_intersection;
#else
  typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type>
  Segment_intersection;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type>
  Plane_intersection;
#endif
  typedef Tree::Primitive_id Primitive_id;

  Point3 p(1.0, 0.0, 0.0);
  Point3 q(0.0, 1.0, 0.0);
  Point3 r(0.0, 0.0, 1.0);
  Point3 s(0.0, 0.0, 0.0);
  Mesh m;
  CGAL::make_tetrahedron(p, q, r, s, m);

  // constructs AABB tree
  Tree tree(faces(m).first, faces(m).second, m);

  // constructs segment query
  Point3 a(-0.2, 0.2, -0.2);
  Point3 b(1.3, 0.2, 1.3);
  Segment segment_query(a, b);

  // tests intersections with segment query
  if (tree.do_intersect(segment_query)) {
      // std::cout << "intersection(s)" << "\n";
  } else {
    // std::cout << "no intersection" << "\n";
  }

  // computes #intersections with segment query
  // std::cout << tree.number_of_intersected_primitives(segment_query)
  //     << " intersection(s)" << "\n";

  // computes first encountered intersection with segment query
  // (generally a point)
  Segment_intersection intersection =
      tree.any_intersection(segment_query);
  if (intersection) {
    // gets intersection object
#if CGAL_VERSION_MAJOR >= 6
    if (std::get_if<Point3>(&(intersection->first))) {
      // Point3* p = std::get_if<Point3>(&(intersection->first));
#else
    if (boost::get<Point3>(&(intersection->first))) {
      // Point3* p = boost::get<Point3>(&(intersection->first));
#endif
      // std::cout << "intersection object is a point " << *p <<  "\n";
      // std::cout << "with face "<< intersection->second  <<  "\n";
    }
  }

  // computes all intersections with segment query
  // (as pairs object - primitive_id)
  std::list<Segment_intersection> intersections;
  tree.all_intersections(segment_query, std::back_inserter(intersections));

  // computes all intersected primitives with segment query as primitive ids
  std::list<Primitive_id> primitives;
  tree.all_intersected_primitives(segment_query,
      std::back_inserter(primitives));

  // constructs plane query
  Point3 base(0.0, 0.0, 0.5);
  Vector3 vec(0.0, 0.0, 1.0);
  Plane plane_query(base, vec);

  // computes first encountered intersection with plane query
  // (generally a segment)
  Plane_intersection plane_intersection = tree.any_intersection(plane_query);
  if (plane_intersection) {
#if CGAL_VERSION_MAJOR >= 6
    if (std::get_if<Segment>(&(plane_intersection->first))) {
      // Segment* s = std::get_if<Segment>(&(plane_intersection->first));
#else
    if (boost::get<Segment>(&(plane_intersection->first))) {
      // Segment* s = boost::get<Segment>(&(plane_intersection->first));
#endif
      // std::cout << "one intersection object is the segment " << s << "\n";
      // std::cout << "with face "<< intersection->second  <<  "\n";
    }
  }
}

//////////////////////////////////////////////////
/// This example based upon the examples distributed with CGAL in:
/// CGAL-4.13/examples/AABB_tree/AABB_polyhedron_facet_intersection_example.cpp
/// Author(s) : Camille Wormser, Pierre Alliez
///
TEST(CGAL, AABBPolyhedronFacetIntersection) {
  typedef CGAL::Simple_cartesian<double> K;
  // typedef K::FT FT;
  typedef K::Point_3 Point3;
  // typedef K::Segment_3 Segment;
  typedef CGAL::Polyhedron_3<K> Polyhedron;
  typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
#if CGAL_VERSION_MAJOR >= 6
  typedef CGAL::AABB_traits_3<K, Primitive> Traits;
#else
  typedef CGAL::AABB_traits<K, Primitive> Traits;
#endif
  typedef CGAL::AABB_tree<Traits> Tree;
  // typedef Tree::Point_and_primitive_id Point_and_primitive_id;

  Point3 p(1.0, 0.0, 0.0);
  Point3 q(0.0, 1.0, 0.0);
  Point3 r(0.0, 0.0, 1.0);
  Point3 s(0.0, 0.0, 0.0);
  Polyhedron polyhedron;
  polyhedron.make_tetrahedron(p, q, r, s);

  // constructs AABB tree and computes internal KD-tree
  // data structure to accelerate distance queries
  Tree tree(faces(polyhedron).first, faces(polyhedron).second, polyhedron);
  tree.accelerate_distance_queries();

  // query point
  // Point3 query(0.0, 0.0, 3.0);

  // computes squared distance from query
  // FT sqd = tree.squared_distance(query);
  // std::cout << "squared distance: " << sqd << "\n";

  // computes closest point
  // Point3 closest = tree.closest_point(query);
  // std::cout << "closest point: " << closest << "\n";

  // computes closest point and primitive id
  // Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
  // Point3 closest_point = pp.first;
  // Polyhedron::Face_handle f = pp.second; // closest primitive id
  // std::cout << "closest point: " << closest_point << "\n";
  // std::cout << "closest triangle: ( "
  //   << f->halfedge()->vertex()->point() << " , "
  //   << f->halfedge()->next()->vertex()->point() << " , "
  //   << f->halfedge()->next()->next()->vertex()->point()
  //   << " )" << "\n";
}

//////////////////////////////////////////////////
TEST(CGAL, SurfaceMeshGridCell) {
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  // typedef K::Plane_3 Plane;
  typedef K::Ray_3 Ray;
  // typedef K::Vector_3 Vector3;
  // typedef K::Segment_3 Segment;

  typedef CGAL::Surface_mesh<Point3> Mesh;
  // typedef Mesh::Vertex_index vertex_descriptor;
  // typedef Mesh::Face_index face_descriptor;

  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
#if CGAL_VERSION_MAJOR >= 6
  typedef CGAL::AABB_traits_3<K, Primitive> Traits;
#else
  typedef CGAL::AABB_traits<K, Primitive> Traits;
#endif
  typedef CGAL::AABB_tree<Traits> Tree;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type>
  // Segment_intersection;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type>
  // Plane_intersection;
  // typedef Tree::Primitive_id Primitive_id;

#if CGAL_VERSION_MAJOR >= 6
  typedef std::optional<Tree::Intersection_and_primitive_id<Ray>::Type>
  Ray_intersection;
#else
  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type>
  Ray_intersection;
#endif
  typedef CGAL::Timer Timer;

  Point3 p0(-1.0, -1.0, 1.0);
  Point3 p1(1.0, -1.0, 1.5);
  Point3 p2(1.0, 1.0, 1.3);
  Point3 p3(-1.0, 1.0, 1.6);

  Mesh mesh;
  Mesh::Vertex_index v0 = mesh.add_vertex(p0);
  Mesh::Vertex_index v1 = mesh.add_vertex(p1);
  Mesh::Vertex_index v2 = mesh.add_vertex(p2);
  Mesh::Vertex_index v3 = mesh.add_vertex(p3);

  mesh.add_face(v0, v1, v2);
  mesh.add_face(v0, v2, v3);

  {
    // std::cout << "Vertices " << "\n";
    // for(auto&& vertex : mesh.vertices()) {
    //   std::cout << vertex << "\n";
    // }
  }

  {
    // std::cout << "Faces " << "\n";
    // for(auto&& face : mesh.faces()) {
    //   std::cout << face << "\n";

    //   Triangle tri = MakeTriangle(mesh, face);
    //   std::cout << tri << "\n";
    // }
  }

  {
    // std::cout << "AABB Tree " << "\n";

    Timer t;
    t.start();
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();
    t.stop();
    // std::cout << "Build: " << t.time() << " sec" << "\n";

    Point3 r0(0.5, 0.5, 101.0);
    Point3 r1(0.5, 0.5, 100.0);
    Ray ray_query(r0, r1);

    t.reset();
    t.start();
    Ray_intersection intersection;
    for (int i=0; i < 1000; ++i) {
      intersection = tree.first_intersection(ray_query);
    }
    t.stop();
    // std::cout << "Intersect (x1000): " << t.time() << " sec" << "\n";

    // if(intersection) {
#if CGAL_VERSION_MAJOR >= 6
    //   if(std::get_if<Point3>(&(intersection->first))) {
    //     const Point3* p = std::get_if<Point3>(&(intersection->first));
#else
    //   if(boost::get<Point3>(&(intersection->first))) {
    //     const Point3* p = boost::get<Point3>(&(intersection->first));
#endif
    //     std::cout <<  *p << "\n";
    //   }
    // }
  }
}

//////////////////////////////////////////////////
TEST(CGAL, SurfaceMeshGrid) {
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  // typedef K::Plane_3 Plane;
  typedef K::Ray_3 Ray;
  // typedef K::Vector_3 Vector3;
  // typedef K::Segment_3 Segment;

  typedef CGAL::Surface_mesh<Point3> Mesh;
  // typedef Mesh::Vertex_index vertex_descriptor;
  // typedef Mesh::Face_index face_descriptor;

  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
#if CGAL_VERSION_MAJOR >= 6
  typedef CGAL::AABB_traits_3<K, Primitive> Traits;
#else
  typedef CGAL::AABB_traits<K, Primitive> Traits;
#endif
  typedef CGAL::AABB_tree<Traits> Tree;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type>
  // Segment_intersection;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type>
  // Plane_intersection;
  // typedef Tree::Primitive_id Primitive_id;

#if CGAL_VERSION_MAJOR >= 6
  typedef std::optional<Tree::Intersection_and_primitive_id<Ray>::Type>
  Ray_intersection;
#else
  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type>
  Ray_intersection;
#endif
  typedef CGAL::Timer Timer;

  // Create Grid
  waves::Grid grid({ 100, 100 }, { 4, 4 });

  // Convert to SurfaceMesh
  const gz::cgal::Mesh& mesh = *grid.GetMesh();

  // Properties
  // {
  //   std::cout << "Vertices " << "\n";
  //   for(auto&& vertex : mesh.vertices()) {
  //     std::cout << vertex << "\n";
  //   }
  // }

  // {
  //   std::cout << "Faces " << "\n";
  //   for(auto&& face : mesh.faces()) {
  //     std::cout << face << "\n";

  //     Triangle tri = MakeTriangle(mesh, face);
  //     std::cout << tri << "\n";
  //   }
  // }

  // Intersections
  {
    // std::cout << "AABB Tree " << "\n";

    // Create a sphere
    std::shared_ptr<gz::cgal::Mesh> sphere = CreateSphere(
      "TestSurfaceMeshGridCellSphere", 5.25);

    Timer t;
    t.start();
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();
    t.stop();
    // std::cout << "Build: " << t.time() << " sec" << "\n";

    t.reset();
    t.start();
    Ray_intersection intersection;
    for (auto v : sphere->vertices()) {
      Point3 r0 = sphere->point(v);
      Point3 r1(r0.x(), r0.y(), r0.z() - 10.0);
      Ray ray_query(r0, r1);
      intersection = tree.first_intersection(ray_query);
    }
    t.stop();
    // std::cout << "Intersect (x" << sphere->number_of_vertices()
    //    << "): " << t.time() << " sec" << "\n";

    // if(intersection) {
#if CGAL_VERSION_MAJOR >= 6
    //   if(std::get_if<Point3>(&(intersection->first))) {
    //     const Point3* p = std::get_if<Point3>(&(intersection->first));
#else
    //   if(boost::get<Point3>(&(intersection->first))) {
    //     const Point3* p = boost::get<Point3>(&(intersection->first));
#endif
    //     std::cout <<  *p << "\n";
    //   }
    // }
  }
}

//////////////////////////////////////////////////
TEST(CGAL, SurfaceMeshModifyGrid) {
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  // typedef K::Plane_3 Plane;
  // typedef K::Ray_3 Ray;
  typedef K::Vector_3 Vector3;
  // typedef K::Segment_3 Segment;

  // typedef CGAL::Surface_mesh<Point3> Mesh;
  // typedef Mesh::Vertex_index vertex_descriptor;
  // typedef Mesh::Face_index face_descriptor;

  // typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
#if CGAL_VERSION_MAJOR >= 6
  // typedef CGAL::AABB_traits_3<K, Primitive> Traits;
#else
  // typedef CGAL::AABB_traits<K, Primitive> Traits;
#endif
  // typedef CGAL::AABB_tree<Traits> Tree;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type>
  // Segment_intersection;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type>
  // Plane_intersection;
  // typedef Tree::Primitive_id Primitive_id;

  // typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type>
  // Ray_intersection;

  // typedef CGAL::Timer Timer;

  // Create Grid
  waves::Grid grid({ 100, 100 }, { 4, 4 });

  // Convert to SurfaceMesh
  gz::cgal::Mesh& mesh = const_cast<gz::cgal::Mesh&>(*grid.GetMesh());

  // Properties
  {
    // std::cout << "Update Points" << "\n";
    for (auto&& vertex : mesh.vertices()) {
      Point3& point = mesh.point(vertex);
      point += Vector3(0, 0, 10);
    }
  }

  {
    // std::cout << "Faces" << "\n";
    // for(auto&& face : mesh.faces()) {
    //   std::cout << face << "\n";

    //   Triangle tri = MakeTriangle(mesh, face);
    //   std::cout << tri << "\n";
    // }
  }
}

//////////////////////////////////////////////////
TEST(CGAL, SurfaceMeshWavefield) {
  // typedef CGAL::Simple_cartesian<double> K;
  // typedef K::Point_3 Point3;
  // typedef K::Plane_3 Plane;
  // typedef K::Ray_3 Ray;
  // typedef K::Vector_3 Vector3;
  // typedef K::Segment_3 Segment;

  // typedef CGAL::Surface_mesh<Point3> Mesh;
  // typedef Mesh::Vertex_index vertex_descriptor;
  // typedef Mesh::Face_index face_descriptor;

  // typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  // typedef CGAL::AABB_traits_3<K, Primitive> Traits;
  // typedef CGAL::AABB_tree<Traits> Tree;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type>
  // Segment_intersection;
  // typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type>
  // Plane_intersection;
  // typedef Tree::Primitive_id Primitive_id;

  // typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type>
  // Ray_intersection;

  typedef CGAL::Timer Timer;

  // Wavefield Parameters
  std::shared_ptr<waves::WaveParameters> params(
      new waves::WaveParameters());
  params->SetNumber(1);
  params->SetAmplitude(3.0);
  params->SetPeriod(10.0);
  params->SetPhase(0.0);

  // Wavefield
  waves::Wavefield wavefield("waves");
  wavefield.SetParameters(params);

  // Evolve to t=10 with 1000 updates
  Timer t;
  t.start();
  for (size_t i=0; i < 1000; ++i) {
    wavefield.Update(i/100.0);
  }
  t.stop();
  // std::cout << "Update (x1000): " << t.time() << " sec" << "\n";

  // Debug info
  // auto mesh = wavefield.GetMesh();
  // {
  //   std::cout << "Faces" << "\n";
  //   for(auto&& face : mesh->faces()) {
  //     Triangle tri = MakeTriangle(*mesh, face);
  //     std::cout << face << ": " << tri << "\n";
  //   }
  // }
}

//////////////////////////////////////////////////
TEST(CGAL, VertexRangeIterator) {
  // Mesh
  Point3 p0(0, 0, 0);
  Point3 p1(1, 0, 0);
  Point3 p2(0, 1, 0);
  Point3 p3(1, 1, 0);
  Mesh mesh;

  auto v0 = mesh.add_vertex(p0);
  auto v1 = mesh.add_vertex(p1);
  auto v2 = mesh.add_vertex(p2);
  auto v3 = mesh.add_vertex(p3);
  mesh.add_face(v0, v1, v2);
  mesh.add_face(v0, v2, v3);

  // Iterate over one variable
  // for (
  //   auto&& vb = std::begin(mesh.vertices());
  //   vb != std::end(mesh.vertices());
  //   ++vb) {
  //   auto&& v  = *vb;
  //   const Point3& p = mesh.point(v);
  //   std::cout << p << "\n";
  // }

  // Iterate over two variables using std::pair
  // for (
  //   auto&& it = std::make_pair(std::begin(mesh.vertices()), 0);
  //   it.first != std::end(mesh.vertices());
  //   ++it.first, ++it.second) {
  //   auto&& v = *it.first;
  //   const Point3& p = mesh.point(v);
  //   std::cout << it.second << ": " << p << "\n";
  // }
}

//////////////////////////////////////////////////
/// \todo(srmainwaring) resolve why this test is failing on Ubuntu.
#if 0
TEST(CGAL, CreateTriangulationN) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
  typedef CGAL::Triangulation_vertex_base_2<K>                    Vbb;
  typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>        Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<K>          Fbb;
  typedef CGAL::Triangulation_face_base_with_info_2<int, K, Fbb>  Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>            Tds;
  // typedef CGAL::Triangulation_2<K, Tds>                        TBase;
  typedef CGAL::Constrained_triangulation_2<K, Tds>               TBase;

  typedef CGAL::Triangulation_hierarchy_2<TBase>  Triangulation;
  typedef Triangulation::Vertex_circulator        Vertex_circulator;
  typedef Triangulation::Vertex_handle            Vertex_handle;
  typedef Triangulation::Face_handle              Face_handle;
  typedef Triangulation::Point                    Point;
  typedef Triangulation::Face                     Face;
  typedef Triangulation::Triangle                 Triangle;
  typedef Triangulation::Edge                     Edge;
  typedef Triangulation::Segment                  Segment;

  // Create the mesh
  std::vector<Point> points;
  std::vector<std::array<size_t, 3>> indices;
  std::vector<std::array<size_t, 2>> infiniteIndices;

  size_t N = 3;
  size_t NPlus1 = N + 1;
  double L = 10.0;
  double dl = L / N;
  double lm = - L / 2.0;

  // Points - (N+1) points in each row / column
  for (size_t iy=0; iy <= N; ++iy) {
    double py = iy * dl + lm;
    for (size_t ix=0; ix <= N; ++ix) {
      // Vertex position
      double px = ix * dl + lm;
      Point point(px, py);
      points.push_back(point);
    }
  }

  // Face indices
  for (size_t iy=0; iy < N; ++iy) {
    for (size_t ix=0; ix < N; ++ix) {
      // Get the points in the cell coordinates
      size_t idx0 = iy * NPlus1 + ix;
      size_t idx1 = iy * NPlus1 + ix + 1;
      size_t idx2 = (iy+1) * NPlus1 + ix + 1;
      size_t idx3 = (iy+1) * NPlus1 + ix;

      // Face indices
      indices.push_back({ idx0, idx1, idx2 });
      indices.push_back({ idx0, idx2, idx3 });
    }
  }

  // Infinite indices (follow edges counter clockwise around grid)
  infiniteIndices.resize(4*N);
  for (size_t i=0; i < N; ++i) {
    // bottom
    size_t idx = i;
    infiniteIndices[i] = { idx, idx+1 };

    // right
    idx = i * NPlus1 + N;
    infiniteIndices[N+i] = { idx, idx+NPlus1 };

    // top
    idx = N * NPlus1 + i;
    infiniteIndices[3*N-1-i] = { idx+1, idx };

    // left
    idx = i * NPlus1;
    infiniteIndices[4*N-1-i] = { idx+NPlus1, idx };
  }

  // std::cout << "points:" << "\n";
  // for (auto&& p : points)  {
  //   std::cout << p << "\n";
  // }
  // std::cout << "indices:" << "\n";
  // for (auto&& i : indices) {
  //   std::cout << i[0] << " " << i[1] << " " << i[2]  << "\n";
  // }
  // std::cout << "infinite indices:" << "\n";
  // for (auto&& i : infiniteIndices) {
  //   std::cout << i[0] << " " << i[1] << "\n";
  // }

  // Manually create a triangulation
  Triangulation t;
  Triangulation::Triangulation_data_structure& tds = t.tds();
  {
    // Clear all finite faces and vertices
    tds.clear();

    // Set dimension
    tds.set_dimension(2);

    // Infinite Vertex
    Vertex_handle vi = tds.create_vertex();
    t.set_infinite_vertex(vi);
    vi = t.infinite_vertex();

    // Finite vertices
    std::vector<Vertex_handle> vertices;
    for (auto&& p : points) {
      Vertex_handle v = tds.create_vertex();

      // Assign points to finite vertices
      v->set_point(p);
      vertices.push_back(v);
    }

    // Finite faces
    std::vector<Face_handle> faces;
    for (auto&& i : indices) {
      auto& v0 = vertices[i[0]];
      auto& v1 = vertices[i[1]];
      auto& v2 = vertices[i[2]];
      Face_handle f = tds.create_face(v0, v1, v2);
      faces.push_back(f);
    }

    // Infinite faces
    std::vector<Face_handle> infiniteFaces;
    for (auto&& i : infiniteIndices) {
      auto& v0 = vertices[i[0]];
      auto& v2 = vertices[i[1]];
      Face_handle f = tds.create_face(v0, vi, v2);
      infiniteFaces.push_back(f);
    }

    // Set infinite vertex faces
    vi->set_face(infiniteFaces[0]);

    // Set vertex faces excluding top and right
    // @DEBUG_INFO
    std::vector<size_t> debugVertexFaces(NPlus1*NPlus1, -1);
    for (size_t iy=0; iy < N; ++iy) {
      for (size_t ix=0; ix < N; ++ix) {
        size_t vidx = iy * NPlus1 + ix;
        size_t fidx = 2 * (iy * N + ix);
        vertices[vidx]->set_face(faces[fidx]);
        // @DEBUG_INFO
        debugVertexFaces[vidx] = fidx;
      }
    }
    // Set vertex faces : top and right
    for (size_t i=0; i < N; ++i) {
      // right
      size_t vidx = i * NPlus1 + N;
      size_t fidx = 2 * (i * N + N - 1);
      vertices[vidx]->set_face(faces[fidx]);
      // @DEBUG_INFO
      debugVertexFaces[vidx] = fidx;

      // top
      vidx = N * NPlus1 + i;
      fidx = 2 * ((N - 1) * N + i) + 1;
      vertices[vidx]->set_face(faces[fidx]);
      // @DEBUG_INFO
      debugVertexFaces[vidx] = fidx;
    }
    {
      // top-right
      size_t vidx = NPlus1*NPlus1-1;
      size_t fidx = 2 * (N * N - 1);
      vertices[vidx]->set_face(faces[fidx]);
      // @DEBUG_INFO
      debugVertexFaces[vidx] = fidx;
    }

    // std::cout << "vertex : faces" << "\n";
    // for (size_t i=0; i<debugVertexFaces.size(); ++i)
    // {
    //   std::cout << i << " " << debugVertexFaces[i] << "\n";
    // }

    // Set Finite Face neighbours
    for (size_t iy=0; iy < N; ++iy) {
      for (size_t ix=0; ix < N; ++ix) {
        for (size_t k=0; k < 2; ++k) {
          size_t idx = 2 * (iy * N + ix);

          if (k == 0) {
            if (iy == 0) {
              if (ix == N-1) {
                // case 1
                size_t fk = idx;
                size_t f0 = N;
                size_t f1 = idx + 1;
                size_t f2 = N - 1;
                faces[fk]->set_neighbors(infiniteFaces[f0],
                    faces[f1], infiniteFaces[f2]);
              } else {  // ix != N-1
                // case 2
                size_t fk = idx;
                size_t f0 = idx + 3;
                size_t f1 = idx + 1;
                size_t f2 = ix;
                faces[fk]->set_neighbors(faces[f0], faces[f1],
                    infiniteFaces[f2]);
              }
            } else {  // iy != 0
              if (ix == N-1) {
                // case 3
                size_t fk = idx;
                size_t f0 = N + iy;
                size_t f1 = idx + 1;
                size_t f2 = idx + 1 - 2 * N;
                faces[fk]->set_neighbors(infiniteFaces[f0],
                    faces[f1], faces[f2]);
              } else {
                // case 4 (general)
                size_t fk = idx;
                size_t f0 = idx + 3;
                size_t f1 = idx + 1;
                size_t f2 = idx + 1 - 2 * N;
                faces[fk]->set_neighbors(faces[f0], faces[f1], faces[f2]);
              }
            }
          }
          if (k == 1) {
            if (ix == 0){
              if (iy == N-1) {
                // case 5
                size_t fk = idx + 1;
                size_t f0 = 3 * N - 1;
                size_t f1 = 3 * N;
                size_t f2 = idx;
                faces[fk]->set_neighbors(infiniteFaces[f0],
                    infiniteFaces[f1], faces[f2]);
              } else {
                // case 6
                size_t fk = idx + 1;
                size_t f0 = idx + 2 * N;
                size_t f1 = 4 * N - 1 - iy;
                size_t f2 = idx;
                faces[fk]->set_neighbors(faces[f0],
                    infiniteFaces[f1], faces[f2]);
              }
            } else {  // ix != 0
              if (iy == N-1) {
                // case 7
                size_t fk = idx + 1;
                size_t f0 = 3 * N - 1 - ix;
                size_t f1 = idx - 2;
                size_t f2 = idx;
                faces[fk]->set_neighbors(infiniteFaces[f0],
                    faces[f1], faces[f2]);
              } else {
                // case 8 (general)
                size_t fk = idx + 1;
                size_t f0 = idx + 2 * N;
                size_t f1 = idx - 2;
                size_t f2 = idx;
                faces[fk]->set_neighbors(faces[f0], faces[f1], faces[f2]);
              }
            }
          }
        }
      }
    }

    // Set Infinite Face neighbours
    for (size_t i=0; i < N; ++i) {
      // bottom
      {
        size_t fk = i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * i;
        size_t f2 = fk - 1;
        if (i == 0)
          f2 = 4 * N - 1;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0],
            faces[f1], infiniteFaces[f2]);
      }

      // right
      {
        size_t fk = N + i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * (i * N + N - 1);
        size_t f2 = fk - 1;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0],
            faces[f1], infiniteFaces[f2]);
      }

      // top
      {
        size_t fk = 2 * N + i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * ((N - 1) * N + i) + 1;
        size_t f2 = fk - 1;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0],
            faces[f1], infiniteFaces[f2]);
      }

      // left
      {
        size_t fk = 3 * N + i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * (i * N) + 1;
        size_t f2 = fk - 1;
        if (i == N - 1)
          f0 = 0;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0],
            faces[f1], infiniteFaces[f2]);
      }
    }

    // Verify vertices
    EXPECT_TRUE(vi->is_valid());
    // for (auto v = tds.vertices_begin(); v !=  tds.vertices_end(); ++v) {
    //   EXPECT_TRUE(v->is_valid(true));
    // }
    for (auto&& v : vertices) {
      EXPECT_TRUE(v->is_valid());
    }

    // Verify faces
    // for (auto f = tds.faces_begin(); f !=  tds.faces_end(); ++f) {
    //   EXPECT_TRUE(f->is_valid(true));
    // }
    for (auto&& f : faces) {
      EXPECT_TRUE(f->is_valid());
    }
    for (auto&& f : infiniteFaces) {
      EXPECT_TRUE(f->is_valid());
    }

    // Verify triangulation
    // std::cout << "triangulation data structure" << "\n";
    // std::cout << tds << "\n";
    // std::cout << "is valid: " << tds.is_valid() << "\n";
    // std::cout << "dimension: " << tds.dimension() << "\n";
    // std::cout << "number of vertices : " << tds.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << tds.number_of_faces() << "\n";
    // std::cout << "number of edges : " << tds.number_of_edges() << "\n";
    EXPECT_TRUE(tds.is_valid());
    EXPECT_EQ(tds.dimension(), 2);
    // EXPECT_EQ(tds.number_of_vertices(), 5);
    // EXPECT_EQ(tds.number_of_faces(), 6);
    // EXPECT_EQ(tds.number_of_edges(), 9);

    // std::cout << "triangulation" << "\n";
    // std::cout << t << "\n";
    // std::cout << "is valid: " << t.is_valid() << "\n";
    // std::cout << "dimension: " << t.dimension() << "\n";
    // std::cout << "number of vertices : " << t.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << t.number_of_faces() << "\n";
    EXPECT_TRUE(t.is_valid());
    EXPECT_EQ(t.dimension(), 2);
    // EXPECT_EQ(t.number_of_vertices(), 4);
    // EXPECT_EQ(t.number_of_faces(), 2);
  }

  // Manually create a constrained triangulation hierarchy
  Triangulation ct;
  Triangulation::Triangulation_data_structure& ctds = ct.tds();
  {
    // Add all finite edges as constraints.
    for (auto e = t.finite_edges_begin(); e != t.finite_edges_end(); ++e) {
      const auto& f = e->first;
      int i = e->second;
      const auto& v0 = f->vertex(f->cw(i));
      const auto& v1 = f->vertex(f->ccw(i));
      ct.insert_constraint(v0->point(), v1->point());
    }
    // std::cout << "constrained triangulation" << "\n";
    // std::cout << t << "\n";

    // Now insert all finite points (to force building the
    // triangulation hierarchy)
    for (auto v = ct.finite_vertices_begin();
        v != ct.finite_vertices_end(); ++v) {
      ct.insert(v->point());
    }

    // std::cout << "constrained triangulation" << "\n";
    // std::cout << t << "\n";
    // std::cout << "is valid: " << ct.is_valid() << "\n";
    // std::cout << "dimension: " << ct.dimension() << "\n";
    // std::cout << "number of vertices : " << ct.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << ct.number_of_faces() << "\n";
    EXPECT_TRUE(ct.is_valid());

    // Face list mapping

    // Initialise face info
    for (auto f = t.all_faces_begin(); f != t.all_faces_end(); ++f) {
      f->info() = -1;
    }

    for (auto f = ct.all_faces_begin(); f != ct.all_faces_end(); ++f) {
      f->info() = -2;
    }

    // Face matching
    Face_handle fh;
    for (auto f = t.finite_faces_begin(); f != t.finite_faces_end(); ++f) {
      // Compute the centroid of f and locate the ct face containing this point.
      Point p0 = f->vertex(0)->point();
      Point p1 = f->vertex(1)->point();
      Point p2 = f->vertex(2)->point();
      Point p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0);
      fh = ct.locate(p, fh);
      bool found = fh != nullptr;

      // @ISSUE - the problem with using is_face is that fh is a handle
      //          to a Face in triangulation t not ct. It appears that
      //          is_face obtains the face from the vertices rather than
      //          the list of faces stored in the triangular data structure.
      // bool found = ct.is_face(f->vertex(0), f->vertex(1), f->vertex(2), fh);

      // Set the info value on the found face.
      if (fh != nullptr) {
        auto d1 = std::distance(t.finite_faces_begin(), f);
        fh->info() = d1;
        // std::cout << "found: " << found
        //   << ", is_infinite: " << ct.is_infinite(fh)
        //   << ", info: " << fh->info()
        //   << "\n";
      }
    }

    // Display.
    // size_t idx = 0;
    // std::cout << "t info:" << "\n";
    // for (auto f = t.all_faces_begin();
    //    f != t.all_faces_end(); ++f, ++idx) {
    //   std::cout << "idx: " << idx
    //     << ", is_infinite: " << t.is_infinite(f)
    //     << ", info: " << f->info()
    //     << "\n";
    // }

    // idx = 0;
    // std::cout << "ct info:" << "\n";
    // for (auto f = ct.all_faces_begin();
    //    f != ct.all_faces_end(); ++f, ++idx) {
    //   std::cout << "idx: " << idx
    //     << ", is_infinite: " << ct.is_infinite(f)
    //     << ", info: " << f->info()
    //     << "\n";
    // }

    // idx = 0;
    // std::cout << "ct face info:" << "\n";
    // for (auto f = ct.finite_faces_begin();
    //    f != ct.finite_faces_end(); ++f, ++idx) {
    //   std::cout << "ct: " << idx
    //     << ", t: " << f->info()
    //     << "\n";
    // }
  }
}

TEST(CGAL, CreateTriangulationHierarchyN) {
  int N = 4;
  double L = 4.0;
  waves::TriangulatedGrid tri_grid(N, L);
  tri_grid.CreateMesh();
  tri_grid.CreateTriangulation();

  // tri_grid.DebugPrintMesh();
  // tri_grid.DebugPrintTriangulation();

  EXPECT_TRUE(tri_grid.IsValid());

  // Location tests..
  std::vector<Point3> points;
  std::vector<std::array<int64_t, 3>> indices;

  // Create mesh
  {
    int NPlus1 = N + 1;
    double dl = L / N;
    double lm = - L / 2.0;

    // Points - (N+1) points in each row / column
    for (int64_t iy=0; iy <= N; ++iy) {
      double py = iy * dl + lm;
      for (int64_t ix=0; ix <= N; ++ix) {
        // Vertex position
        double px = ix * dl + lm;
        Point3 point(px, py, 0.0);
        points.push_back(point);
      }
    }

    // Face indices
    for (int64_t iy=0; iy < N; ++iy) {
      for (int64_t ix=0; ix < N; ++ix) {
        // Get the points in the cell coordinates
        int64_t idx0 = iy * NPlus1 + ix;
        int64_t idx1 = iy * NPlus1 + ix + 1;
        int64_t idx2 = (iy+1) * NPlus1 + ix + 1;
        int64_t idx3 = (iy+1) * NPlus1 + ix;

        // Face indices
        indices.push_back({ idx0, idx1, idx2 });
        indices.push_back({ idx0, idx2, idx3 });
      }
    }
  }

  // Locate faces
  {
    // CGAL::Timer timer;
    // timer.start();
    for (int64_t i=0; i < indices.size(); ++i) {
      // Get face index.
      auto& f  = indices[i];

      // Compute centroid.
      auto& p0 = points[f[0]];
      auto& p1 = points[f[1]];
      auto& p2 = points[f[2]];
      Point3 p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0,
        (p0.z() + p1.z() + p2.z())/3.0);

      // Locate point.
      int64_t fidx = 0;
      bool found = tri_grid.Locate(p, fidx);
      EXPECT_TRUE(found);
      EXPECT_EQ(fidx, i);
    }
    // timer.stop();
    // std::cout << "Locate " << indices.size() << " points: ("
    //    << timer.time() << " s)" << "\n";
  }

  // Update mesh
  std::vector<Point3> points2(points.size());
  {
    double g = 9.8;
    double amplitude = 5.0;
    double period    = 10.0;
    double omega = 2.0 * M_PI / period;
    double time = 1.0;
    double k = omega * omega / g;
    double kx = k;
    double ky = 0.0;
    EXPECT_LT(amplitude * k, 1.0);

    // Trochoid wave
    for (int64_t i=0; i < points.size(); ++i) {
      const auto& p0 = points[i];
      auto& p = points2[i];
      double theta = kx * p0.x() + ky * p0.y() - omega * time;
      double c = std::cos(theta);
      double s = std::sin(theta);
      // double sx = amplitude * s;
      // double sy = amplitude * s;
      p = Point3(
        p0.x() - kx / k * amplitude * s,
        p0.y() - ky / k * amplitude * s,
        p0.z() + amplitude * c);
    }
  }

  // std::cout << "points2: " << "\n";
  // std::copy(points2.begin(), points2.end(),
  //   std::ostream_iterator<Point3>(std::cout, "\n"));
  // std::cout << "\n";

  tri_grid.UpdatePoints(points2);
  // tri_grid.DebugPrintTriangulation();

  // Locate faces in updated mesh
  {
    CGAL::Timer timer;
    timer.start();
    for (int64_t i=0; i < indices.size(); ++i) {
      // Get face index.
      auto& f  = indices[i];

      // Compute centroid of updated point.
      auto& p0 = points2[f[0]];
      auto& p1 = points2[f[1]];
      auto& p2 = points2[f[2]];
      Point3 p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0,
        (p0.z() + p1.z() + p2.z())/3.0);

      // Locate point.
      int64_t fidx = 0;
      bool found = tri_grid.Locate(p, fidx);
      EXPECT_TRUE(found);
      EXPECT_EQ(fidx, i);
    }
    timer.stop();
    // std::cout << "Locate " << indices.size()
    //    << " points: (" << timer.time() << " s)" << "\n";
  }
}
#endif

//////////////////////////////////////////////////
TEST(CGAL, CreateTriangulation3) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

  // When not using the triangulation hierarchy use the default
  // triangulation data structure
  typedef CGAL::Triangulation_2<K>                         TSlow;

  // Otherwise must use the extended vertex base class
  typedef CGAL::Triangulation_vertex_base_2<K>             Vbb;
  typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb> Vb;
  typedef CGAL::Triangulation_face_base_2<K>               Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>     Tds;
  typedef CGAL::Triangulation_2<K, Tds>                    TBase;

  typedef CGAL::Triangulation_hierarchy_2<TBase>  Triangulation;
  // typedef Triangulation::Vertex_circulator        Vertex_circulator;
  typedef Triangulation::Vertex_handle            Vertex_handle;
  typedef Triangulation::Face_handle              Face_handle;
  typedef Triangulation::Point                    Point;
  // typedef Triangulation::Face                     Face;
  // typedef Triangulation::Triangle                 Triangle;
  // typedef Triangulation::Edge                     Edge;
  // typedef Triangulation::Segment                  Segment;

  // Points
  Point p1(0, 0);
  Point p2(0, 1);
  Point p3(1, 0);
  Point p4(1, 1);

  {
    // Automatically create a triangulation
    Triangulation t;
    // Triangulation::Triangulation_data_structure& tds = t.tds();

    t.insert(p1);
    t.insert(p2);
    t.insert(p3);
    // t.insert(p4);

    // Check valid
    // std::cout << "triangulation data structure" << "\n";
    // std::cout << "is valid: " << tds.is_valid() << "\n";
    // std::cout << "dimension: " << tds.dimension() << "\n";
    // std::cout << "number of vertices : " << tds.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << tds.number_of_faces() << "\n";
    // std::cout << "number of edges : " << tds.number_of_edges() << "\n";
    // std::cout << tds << "\n";

    // std::cout << "triangulation" << "\n";
    // std::cout << "is valid: " << t.is_valid() << "\n";
    // std::cout << "dimension: " << t.dimension() << "\n";
    // std::cout << "number of vertices : " << t.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << t.number_of_faces() << "\n";
    // std::cout << t << "\n";

    // std::cout << "finite vertices" << "\n";
    // for (auto v=t.finite_vertices_begin();
    //    v != t.finite_vertices_end(); ++v) {
    //   std::cout << *v << "\n";
    // }

    // std::cout << "finite faces" << "\n";
    // for (auto f=t.finite_faces_begin(); f != t.finite_faces_end(); ++f) {
    //   std::cout << *f->vertex(0) << ", " << *f->vertex(1)
    //      << ", " << *f->vertex(2) << "\n";
    // }
  }

  if (0) {
    // Manually create a triangulation
    Triangulation t;
    Triangulation::Triangulation_data_structure& tds = t.tds();

    // Clear all finite faces and vertices
    tds.clear();

    // Set dimension
    tds.set_dimension(2);

    // std::cout << "before vertex insertion..." << "\n";
    // std::cout << "is valid: " << tds.is_valid() << "\n";
    // std::cout << "dimension: " << tds.dimension() << "\n";
    // std::cout << "number of vertices : " << tds.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << tds.number_of_faces() << "\n";
    // std::cout << "number of edges : " << tds.number_of_edges() << "\n";
    EXPECT_FALSE(tds.is_valid());
    EXPECT_EQ(tds.dimension(), 2);
    EXPECT_EQ(tds.number_of_vertices(), 0);
    EXPECT_EQ(tds.number_of_faces(), 0);
    EXPECT_EQ(tds.number_of_edges(), 0);

    // Infinite Vertex
    Vertex_handle v0 = tds.create_vertex();
    t.set_infinite_vertex(v0);
    v0 = t.infinite_vertex();

    // Finite Vertices
    Vertex_handle v1 = tds.create_vertex();
    Vertex_handle v2 = tds.create_vertex();
    Vertex_handle v3 = tds.create_vertex();

    // Assign points to finite vertices
    v1->set_point(p1);
    v2->set_point(p2);
    v3->set_point(p3);

    // Add faces
    Face_handle f0 = tds.create_face(v0, v2, v3);
    Face_handle f1 = tds.create_face(v2, v1, v3);
    Face_handle f2 = tds.create_face(v1, v0, v3);
    Face_handle f3 = tds.create_face(v1, v2, v0);

    // Set vertex faces
    v0->set_face(f0);
    v1->set_face(f1);
    v2->set_face(f1);
    v2->set_face(f1);

    // Set face neighbours
    f0->set_neighbors(f1, f2, f3);
    f1->set_neighbors(f2, f0, f3);
    f2->set_neighbors(f0, f1, f3);
    f3->set_neighbors(f0, f2, f1);

    // Check vertices valid
    // std::cout << "v0->is_valid: " << v0->is_valid(true) << "\n";
    // std::cout << "v1->is_valid: " << v1->is_valid(true) << "\n";
    // std::cout << "v2->is_valid: " << v2->is_valid(true) << "\n";
    // std::cout << "v3->is_valid: " << v3->is_valid(true) << "\n";
    EXPECT_TRUE(v0->is_valid());
    EXPECT_TRUE(v1->is_valid());
    EXPECT_TRUE(v2->is_valid());
    EXPECT_TRUE(v3->is_valid());

    // Check faces valid
    // std::cout << "f0->is_valid: " << f0->is_valid(true) << "\n";
    // std::cout << "f1->is_valid: " << f1->is_valid(true) << "\n";
    // std::cout << "f2->is_valid: " << f2->is_valid(true) << "\n";
    // std::cout << "f3->is_valid: " << f3->is_valid(true) << "\n";
    EXPECT_TRUE(f0->is_valid());
    EXPECT_TRUE(f1->is_valid());
    EXPECT_TRUE(f2->is_valid());
    EXPECT_TRUE(f3->is_valid());

    // Check valid
    // std::cout << "is valid: " << tds.is_valid() << "\n";
    // std::cout << "dimension: " << tds.dimension() << "\n";
    // std::cout << "number of vertices : " << tds.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << tds.number_of_faces() << "\n";
    // std::cout << "number of edges : " << tds.number_of_edges() << "\n";
    EXPECT_TRUE(tds.is_valid());
    EXPECT_EQ(tds.dimension(), 2);
    EXPECT_EQ(tds.number_of_vertices(), 4);
    EXPECT_EQ(tds.number_of_faces(), 4);
    EXPECT_EQ(tds.number_of_edges(), 6);

    // std::cout << "triangulation data structure" << "\n";
    // std::cout << tds << "\n";

    // std::cout << "is infinite (T): " << t.is_infinite(v0) << "\n";
    // std::cout << "is infinite (T): " << t.is_infinite(f0) << "\n";
    // std::cout << "is infinite (T): " << t.is_infinite(f2) << "\n";
    // std::cout << "is infinite (T): " << t.is_infinite(f3) << "\n";
    EXPECT_TRUE(t.is_infinite(v0));
    EXPECT_TRUE(t.is_infinite(f0));
    EXPECT_TRUE(t.is_infinite(f2));
    EXPECT_TRUE(t.is_infinite(f3));

    // std::cout << "is infinite (F): " << t.is_infinite(v1) << "\n";
    // std::cout << "is infinite (F): " << t.is_infinite(v2) << "\n";
    // std::cout << "is infinite (F): " << t.is_infinite(v3) << "\n";
    // std::cout << "is infinite (F): " << t.is_infinite(f1) << "\n";
    EXPECT_FALSE(t.is_infinite(v1));
    EXPECT_FALSE(t.is_infinite(v2));
    EXPECT_FALSE(t.is_infinite(v3));
    EXPECT_FALSE(t.is_infinite(f1));

    // std::cout << "is valid: " << t.is_valid() << "\n";
    // std::cout << "dimension: " << t.dimension() << "\n";
    // std::cout << "number of vertices : " << t.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << t.number_of_faces() << "\n";
    EXPECT_TRUE(t.is_valid());
    EXPECT_EQ(t.dimension(), 2);
    EXPECT_EQ(t.number_of_vertices(), 3);
    EXPECT_EQ(t.number_of_faces(), 1);

    // std::cout << "triangulation" << "\n";
    // std::cout << t << "\n";


    // Point location.
    std::cout << "point location..." << "\n";
    Face_handle f = t.locate(Point(0.1, 0.1));
    std::cout << "point found: " << !t.is_infinite(f) << "\n";
    std::cout << *f->vertex(0) << ", " << *f->vertex(1)
        << ", " << *f->vertex(2) << "\n";

    f = t.locate(Point(-1.0, -1.0));
    std::cout << "point found: " << !t.is_infinite(f) << "\n";
    std::cout << *f->vertex(0) << ", " << *f->vertex(1)
        << ", " << *f->vertex(2) << "\n";

    // Now shift the points on the vertices.
    v1->set_point(Point(-10, -10));
    f = t.locate(Point(-1.0, -1.0));
    std::cout << "point found: " << !t.is_infinite(f) << "\n";
    std::cout << *f->vertex(0) << ", " << *f->vertex(1)
        << ", " << *f->vertex(2) << "\n";

    // Point location using triangulation hierarchy
    // Verbose mode of is_valid shows the number of vertices at each level.
    std::cout << "number of vertices at successive levels:" << "\n";
    EXPECT_TRUE(t.is_valid(true));

    {
      typedef CGAL::Creator_uniform_2<double, Point> Creator;
      std::cout << "insertion of random points" << "\n";
      int N = 256;
      Triangulation tt;
      CGAL::Random_points_in_square_2<Point, Creator> g(1.);
      CGAL::cpp11::copy_n(g, (N+1)*(N+1), std::back_inserter(tt));

      // Verbose mode of is_valid shows the number of vertices at each level.
      std::cout << "The number of vertices at successive levels" << "\n";
      EXPECT_TRUE(tt.is_valid(true));

      // Full search
      CGAL::Timer timer;
      timer.start();
      for (int64_t i=0; i < 1000; ++i) {
        f = tt.locate(Point(0.1, 0.7));
      }
      timer.stop();

      std::cout << "point found: " << !tt.is_infinite(f)
          << ", time (1000x): " << timer.time() << "\n";
      std::cout << *f->vertex(0) << ", " << *f->vertex(1)
          << ", " << *f->vertex(2) << "\n";

      // With hint
      timer.reset();
      timer.start();
      for (int64_t i=0; i < 1000; ++i) {
        f = tt.locate(Point(0.1, 0.7), f);
      }
      timer.stop();

      std::cout << "point found: " << !tt.is_infinite(f)
          << ", time (1000x): " << timer.time() << "\n";
      std::cout << *f->vertex(0) << ", " << *f->vertex(1)
          << ", " << *f->vertex(2) << "\n";
    }
    {
      // Point location without triangulation hierarchy.
      typedef CGAL::Creator_uniform_2<double, TSlow::Point> Creator;
      std::cout << "insertion of random points" << "\n";
      int N = 256;
      TSlow tt;
      CGAL::Random_points_in_square_2<TSlow::Point, Creator> g(1.);
      CGAL::cpp11::copy_n(g, (N+1)*(N+1), std::back_inserter(tt));

      // Verbose mode of is_valid shows the number of vertices at each level.
      std::cout << "The number of vertices at successive levels" << "\n";
      EXPECT_TRUE(tt.is_valid(true));

      CGAL::Timer timer;
      timer.start();
      TSlow::Face_handle ff;
      for (int64_t i=0; i < 1000; ++i) {
        ff = tt.locate(TSlow::Point(0.1, 0.7));
      }
      timer.stop();

      std::cout << "point found: " << !tt.is_infinite(ff)
          << ", time (1000x): " << timer.time() << "\n";
      std::cout << *ff->vertex(0) << ", " << *ff->vertex(1)
          << ", " << *ff->vertex(2) << "\n";
    }
  }
}

//////////////////////////////////////////////////
TEST(CGAL, CreateTriangulation4) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Triangulation_2<K>                Triangulation;
  // typedef Triangulation::Vertex_circulator        Vertex_circulator;
  typedef Triangulation::Vertex_handle            Vertex_handle;
  typedef Triangulation::Face_handle              Face_handle;
  typedef Triangulation::Point                    Point;
  // typedef Triangulation::Face                     Face;
  // typedef Triangulation::Triangle                 Triangle;
  // typedef Triangulation::Edge                     Edge;
  // typedef Triangulation::Segment                  Segment;

  // Points
  Point p0(0, 0);
  Point p1(1, 0);
  Point p2(0, 1);
  Point p3(1, 1);

  {
    // Manually create a triangulation
    Triangulation t;
    Triangulation::Triangulation_data_structure& tds = t.tds();

    // Clear all finite faces and vertices
    tds.clear();

    // Set dimension
    tds.set_dimension(2);

    // Infinite Vertex
    Vertex_handle vi = tds.create_vertex();
    t.set_infinite_vertex(vi);
    vi = t.infinite_vertex();

    // Finite Vertices
    Vertex_handle v0 = tds.create_vertex();
    Vertex_handle v1 = tds.create_vertex();
    Vertex_handle v2 = tds.create_vertex();
    Vertex_handle v3 = tds.create_vertex();

    // Assign points to finite vertices
    v0->set_point(p0);
    v1->set_point(p1);
    v2->set_point(p2);
    v3->set_point(p3);

    // Add faces
    Face_handle ff0 = tds.create_face(v0, v1, v3);
    Face_handle ff1 = tds.create_face(v0, v3, v2);

    Face_handle if0 = tds.create_face(v0, vi, v1);
    Face_handle if1 = tds.create_face(v1, vi, v3);
    Face_handle if2 = tds.create_face(v3, vi, v2);
    Face_handle if3 = tds.create_face(v2, vi, v0);

    // Set vertex faces
    vi->set_face(if0);
    v0->set_face(ff0);
    v1->set_face(ff0);
    v2->set_face(ff1);
    v3->set_face(ff1);

    // Set face neighbours
    ff0->set_neighbors(if1, ff1, if0);
    ff1->set_neighbors(if2, if3, ff0);
    if0->set_neighbors(if1, ff0, if3);
    if1->set_neighbors(if2, ff0, if0);
    if2->set_neighbors(if3, ff1, if1);
    if3->set_neighbors(if0, ff1, if2);

    // Check vertices valid
    // std::cout << "vi->is_valid: " << vi->is_valid(true) << "\n";
    // std::cout << "v0->is_valid: " << v0->is_valid(true) << "\n";
    // std::cout << "v1->is_valid: " << v1->is_valid(true) << "\n";
    // std::cout << "v2->is_valid: " << v2->is_valid(true) << "\n";
    // std::cout << "v3->is_valid: " << v3->is_valid(true) << "\n";
    EXPECT_TRUE(vi->is_valid());
    EXPECT_TRUE(v0->is_valid());
    EXPECT_TRUE(v1->is_valid());
    EXPECT_TRUE(v2->is_valid());
    EXPECT_TRUE(v3->is_valid());

    // Check faces valid
    // std::cout << "ff0->is_valid: " << ff0->is_valid(true) << "\n";
    // std::cout << "ff1->is_valid: " << ff1->is_valid(true) << "\n";
    EXPECT_TRUE(ff0->is_valid());
    EXPECT_TRUE(ff1->is_valid());

    // std::cout << "if0->is_valid: " << if0->is_valid(true) << "\n";
    // std::cout << "if1->is_valid: " << if1->is_valid(true) << "\n";
    // std::cout << "if2->is_valid: " << if2->is_valid(true) << "\n";
    // std::cout << "if3->is_valid: " << if3->is_valid(true) << "\n";
    EXPECT_TRUE(if0->is_valid());
    EXPECT_TRUE(if1->is_valid());
    EXPECT_TRUE(if2->is_valid());
    EXPECT_TRUE(if3->is_valid());

    // Check valid
    // std::cout << "is valid: " << tds.is_valid() << "\n";
    // std::cout << "dimension: " << tds.dimension() << "\n";
    // std::cout << "number of vertices : " << tds.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << tds.number_of_faces() << "\n";
    // std::cout << "number of edges : " << tds.number_of_edges() << "\n";
    EXPECT_TRUE(tds.is_valid(true));
    EXPECT_EQ(tds.dimension(), 2);
    EXPECT_EQ(tds.number_of_vertices(), 5);
    EXPECT_EQ(tds.number_of_faces(), 6);
    EXPECT_EQ(tds.number_of_edges(), 9);

    // std::cout << "triangulation data structure" << "\n";
    // std::cout << tds << "\n";

    EXPECT_TRUE(t.is_infinite(vi));
    EXPECT_TRUE(t.is_infinite(if0));
    EXPECT_TRUE(t.is_infinite(if1));
    EXPECT_TRUE(t.is_infinite(if2));
    EXPECT_TRUE(t.is_infinite(if3));

    EXPECT_FALSE(t.is_infinite(v0));
    EXPECT_FALSE(t.is_infinite(v1));
    EXPECT_FALSE(t.is_infinite(v2));
    EXPECT_FALSE(t.is_infinite(v3));
    EXPECT_FALSE(t.is_infinite(ff0));
    EXPECT_FALSE(t.is_infinite(ff1));

    // std::cout << "is valid: " << t.is_valid() << "\n";
    // std::cout << "dimension: " << t.dimension() << "\n";
    // std::cout << "number of vertices : " << t.number_of_vertices() << "\n";
    // std::cout << "number of faces : " << t.number_of_faces() << "\n";
    EXPECT_TRUE(t.is_valid(true));
    EXPECT_EQ(t.dimension(), 2);
    EXPECT_EQ(t.number_of_vertices(), 4);
    EXPECT_EQ(t.number_of_faces(), 2);

    // std::cout << "triangulation" << "\n";
    // std::cout << t << "\n";
  }
}

//////////////////////////////////////////////////
TEST(CGAL, CreateConstrainedTrianguation4) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Triangulation_vertex_base_2<K>                Vbb;
  typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>    Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<K>      Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>        Tds;
  // typedef CGAL::No_constraint_intersection_tag                Itag;
  typedef CGAL::Constrained_triangulation_2<K, Tds>           TBase;

  typedef CGAL::Triangulation_hierarchy_2<TBase>  Triangulation;
  // typedef Triangulation::Vertex_circulator        Vertex_circulator;
  // typedef Triangulation::Vertex_handle            Vertex_handle;
  // typedef Triangulation::Face_handle              Face_handle;
  typedef Triangulation::Point                    Point;
  // typedef Triangulation::Face                     Face;

  // Points
  Point p0(0, 0);
  Point p1(1, 0);
  Point p2(0, 1);
  Point p3(1, 1);

  // Create a triangulation
  Triangulation t;
  // Triangulation::Triangulation_data_structure& tds = t.tds();

  // Insert constraints (i.e. all edges)
  t.insert_constraint(p0, p1);
  t.insert_constraint(p2, p3);
  t.insert_constraint(p0, p2);
  t.insert_constraint(p1, p3);
  t.insert_constraint(p0, p3);


  // std::cout << "triangulation data structure:" << "\n";
  // std::cout << tds << "\n";

  // std::cout << "triangulation:" << "\n";
  // std::cout << t << "\n";

  // std::cout << "is valid: " << t.is_valid(true) << "\n";
}

//////////////////////////////////////////////////
TEST(CGAL, CreateCTAlt) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Triangulation_vertex_base_2<K>                Vbb;
  typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>    Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<K>      Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>        Tds;
  // typedef CGAL::No_constraint_intersection_tag                Itag;
  typedef CGAL::Constrained_triangulation_2<K, Tds>           TBase;

  typedef CGAL::Triangulation_hierarchy_2<TBase>  Triangulation;
  // typedef Triangulation::Vertex_circulator        Vertex_circulator;
  // typedef Triangulation::Vertex_handle            Vertex_handle;
  // typedef Triangulation::Face_handle              Face_handle;
  typedef Triangulation::Point                    Point;
  // typedef Triangulation::Face                     Face;

  // Points
  Point p0(0, 0);
  Point p1(1, 0);
  Point p2(0, 1);
  Point p3(1, 1);

  // Create a triangulation
  Triangulation t;
  // Triangulation::Triangulation_data_structure& tds = t.tds();

  // Insert first row
  t.insert(p0);
  t.insert(p1);

  // Insert second row + diagonal constraints
  t.insert(p2);
  t.insert_constraint(p0, p3);

  // std::cout << "triangulation data structure:" << "\n";
  // std::cout << tds << "\n";

  // std::cout << "triangulation:" << "\n";
  // std::cout << t << "\n";

  // std::cout << "is valid: " << t.is_valid() << "\n";
  EXPECT_TRUE(t.is_valid());
}

//////////////////////////////////////////////////
TEST(CGAL, CreateCTAltN) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Projection_traits_xy_3<K>                     Kp;
  typedef CGAL::Triangulation_vertex_base_2<Kp>               Vbb;
  typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>    Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<Kp>     Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>        Tds;
  // typedef CGAL::No_constraint_intersection_tag                Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<Kp, Tds> TBase;

  typedef CGAL::Triangulation_hierarchy_2<TBase>  Triangulation;
  // typedef Triangulation::Vertex_circulator        Vertex_circulator;
  // typedef Triangulation::Vertex_handle            Vertex_handle;
  // typedef Triangulation::Face_handle              Face_handle;
  typedef K::Point_3                              Point;
  // typedef Triangulation::Face                     Face;

  // Create the mesh
  std::vector<Point> points;
  std::vector<std::array<size_t, 3>> indices;
  std::vector<std::array<size_t, 2>> infiniteIndices;

  size_t N = 256;
  size_t NPlus1 = N + 1;
  double L = 10.0;
  double dl = L / N;
  double lm = - L / 2.0;

  // Create a triangulation
  Triangulation t;
  // Triangulation::Triangulation_data_structure& tds = t.tds();

  // Points - (N+1) points in each row / column
  for (size_t iy=0; iy <= N; ++iy) {
    double py = iy * dl + lm;
    for (size_t ix=0; ix <= N; ++ix) {
      // Vertex position
      double px = ix * dl + lm;
      Point p(px, py, 0.0);
      points.push_back(p);

      // Insert point
      // t.insert(p);

      // Add diagonal constraint
      // if (ix > 0 && iy > 0) {
      //   size_t idx = (iy - 1) * NPlus1 + (ix - 1);
      //   t.insert_constraint(points[idx], p);
      // }
    }
  }

  // Insert points - NOTE: must insert the points to build the
  // triangulation hierarchy.
  CGAL::Timer timer;
  timer.start();
  t.insert(points.begin(), points.end());
  timer.stop();
  // std::cout << "point insertion: " << timer.time() << " s" << "\n";

  // Constraint indices
  std::vector<std::pair<size_t, size_t>> cindices;
  for (size_t iy=0; iy < N; ++iy) {
    for (size_t ix=0; ix < N; ++ix) {
      size_t idx1 = iy * NPlus1 + ix;
      size_t idx2 = (iy + 1) * NPlus1 + (ix + 1);
      cindices.push_back(std::make_pair(idx1, idx2));
    }
  }

  // Insert constraints
  timer.reset();
  timer.start();
  t.insert_constraints(points.begin(), points.end(),
    cindices.begin(), cindices.end());
  timer.stop();
  // std::cout << "constraint insertion: " << timer.time() << " s" << "\n";

  // std::cout << "triangulation data structure:" << "\n";
  // std::cout << tds << "\n";

  // std::cout << "triangulation:" << "\n";
  // std::cout << t << "\n";

  // std::cout << "is valid: " << t.is_valid() << "\n";
  EXPECT_TRUE(t.is_valid());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

