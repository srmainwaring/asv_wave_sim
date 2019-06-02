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
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WaveParameters.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Triangle.hh>

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/number_utils.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>

#include <CGAL/boost/graph/Euler_operations.h>

#include <tbb/tbb.h>

#include <gtest/gtest.h>

#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <string>

///////////////////////////////////////////////////////////////////////////////
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

  std::shared_ptr<asv::Mesh> CreateSphere(
    const std::string& meshName,
    double radius
  )
  {
    gazebo::common::MeshManager::Instance()->CreateSphere(
      meshName,
      radius,                   // radius
      32,                       // rings
      32);                      // segments
    GZ_ASSERT(gazebo::common::MeshManager::Instance()->HasMesh(meshName),
      "Failed to create Mesh for Cylinder");

    const gazebo::common::Mesh* source = gazebo::common::MeshManager::Instance()->GetMesh(meshName);
    GZ_ASSERT(source != nullptr, "Invalid Sphere Mesh");
    // std::cout << "Mesh:       " << source->GetName() << std::endl;
    // std::cout << "Vertex:     " << source->GetVertexCount() << std::endl;

    std::shared_ptr<asv::Mesh> target = std::make_shared<asv::Mesh>();
    asv::MeshTools::MakeSurfaceMesh(*source, *target);
    return target;
  }

} // namespace (anon)

///////////////////////////////////////////////////////////////////////////////
// Define tests
TEST(CGAL, Surprising)
{
  typedef CGAL::Simple_cartesian<double> Kernel;
  typedef Kernel::Point_2 Point_2;

  {
    Point_2 p(0, 0.3), q(1, 0.6), r(2, 0.9);
    // std::cout << (CGAL::collinear(p,q,r) ? "collinear\n" : "not collinear\n");   
  }
  {
    Point_2 p(0, 1.0/3.0), q(1, 2.0/3.0), r(2, 1);
    // std::cout << (CGAL::collinear(p,q,r) ? "collinear\n" : "not collinear\n");   
  }  
  {
    Point_2 p(0,0), q(1, 1), r(2, 2);
    // std::cout << (CGAL::collinear(p,q,r) ? "collinear\n" : "not collinear\n");   
  }  
}

TEST(CGAL, SurfaceMesh)
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  typedef K::Plane_3 Plane;
  typedef K::Vector_3 Vector;
  typedef K::Segment_3 Segment;
  typedef CGAL::Surface_mesh<Point3> Mesh;
  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
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
  Segment segment_query(a,b);

  // tests intersections with segment query
  if(tree.do_intersect(segment_query))
  {
      // std::cout << "intersection(s)" << std::endl;
  }
  else
  {
    // std::cout << "no intersection" << std::endl;
  }

  // computes #intersections with segment query
  // std::cout << tree.number_of_intersected_primitives(segment_query)
  //     << " intersection(s)" << std::endl;

  // computes first encountered intersection with segment query
  // (generally a point)
  Segment_intersection intersection =
      tree.any_intersection(segment_query);
  if(intersection)
  {
    // gets intersection object
    if(boost::get<Point3>(&(intersection->first)))
    {
      Point3* p = boost::get<Point3>(&(intersection->first));
      // std::cout << "intersection object is a point " << *p <<  std::endl;
      // std::cout << "with face "<< intersection->second  <<  std::endl;
    }
  }

  // computes all intersections with segment query (as pairs object - primitive_id)
  std::list<Segment_intersection> intersections;
  tree.all_intersections(segment_query, std::back_inserter(intersections));

  // computes all intersected primitives with segment query as primitive ids
  std::list<Primitive_id> primitives;
  tree.all_intersected_primitives(segment_query, std::back_inserter(primitives));

  // constructs plane query
  Point3 base(0.0,0.0,0.5);
  Vector3 vec(0.0,0.0,1.0);
  Plane plane_query(base,vec);

  // computes first encountered intersection with plane query
  // (generally a segment)
  Plane_intersection plane_intersection = tree.any_intersection(plane_query);
  if(plane_intersection)
  {
    if(boost::get<Segment>(&(plane_intersection->first)))
    {
      Segment* s = boost::get<Segment>(&(plane_intersection->first));
      // std::cout << "one intersection object is the segment " << s << std::endl;
      // std::cout << "with face "<< intersection->second  <<  std::endl;
    }
  }
}

/// This example based upon the examples distributed with CGAL in:
/// CGAL-4.13/examples/AABB_tree/AABB_polyhedron_facet_intersection_example.cpp
/// Author(s) : Camille Wormser, Pierre Alliez
///
TEST(CGAL, AABBPolyhedronFacetIntersection)
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::FT FT;
  typedef K::Point_3 Point3;
  typedef K::Segment_3 Segment;
  typedef CGAL::Polyhedron_3<K> Polyhedron;
  typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef Tree::Point_and_primitive_id Point_and_primitive_id;

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
  Point3 query(0.0, 0.0, 3.0);

  // computes squared distance from query
  FT sqd = tree.squared_distance(query);
  // std::cout << "squared distance: " << sqd << std::endl;

  // computes closest point
  Point3 closest = tree.closest_point(query);
  // std::cout << "closest point: " << closest << std::endl;

  // computes closest point and primitive id
  Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
  Point3 closest_point = pp.first;
  Polyhedron::Face_handle f = pp.second; // closest primitive id
  // std::cout << "closest point: " << closest_point << std::endl;
  // std::cout << "closest triangle: ( "
  //   << f->halfedge()->vertex()->point() << " , " 
  //   << f->halfedge()->next()->vertex()->point() << " , "
  //   << f->halfedge()->next()->next()->vertex()->point()
  //   << " )" << std::endl;
}

TEST(CGAL, SurfaceMeshGridCell)
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  typedef K::Plane_3 Plane;
  typedef K::Ray_3 Ray;
  typedef K::Vector_3 Vector3;
  typedef K::Segment_3 Segment;

  typedef CGAL::Surface_mesh<Point3> Mesh;
  typedef Mesh::Vertex_index vertex_descriptor;
  typedef Mesh::Face_index face_descriptor;

  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
  typedef Tree::Primitive_id Primitive_id;

  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

  typedef CGAL::Timer Timer;

  Point3 p0(-1.0, -1.0, 1.0);
  Point3 p1( 1.0, -1.0, 1.5);
  Point3 p2( 1.0,  1.0, 1.3);
  Point3 p3(-1.0,  1.0, 1.6);
  
  Mesh mesh;
  Mesh::Vertex_index v0 = mesh.add_vertex(p0);
  Mesh::Vertex_index v1 = mesh.add_vertex(p1);
  Mesh::Vertex_index v2 = mesh.add_vertex(p2);
  Mesh::Vertex_index v3 = mesh.add_vertex(p3);

  mesh.add_face(v0, v1, v2);
  mesh.add_face(v0, v2, v3);

 { 
    // std::cout << "Vertices " << std::endl;    
    for(auto&& vertex : mesh.vertices())
    {
      // std::cout << vertex << std::endl;
    }
  }

  { 
    // std::cout << "Faces " << std::endl;
    for(auto&& face : mesh.faces())
    {
      // std::cout << face << std::endl;

      Triangle tri = asv::Geometry::MakeTriangle(mesh, face);
      // std::cout << tri << std::endl;
    }
  }

  {
    // std::cout << "AABB Tree " << std::endl;

    Timer t;
    t.start();
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();
    // std::cout << "Build: " << t.time() << " sec" << std::endl;

    Point3 r0(0.5, 0.5, 101.0);
    Point3 r1(0.5, 0.5, 100.0);

    Ray ray_query(r0, r1);
    t.start();
    Ray_intersection intersection;
    for (int i=0; i<1000; ++i)
    {
      intersection = tree.first_intersection(ray_query);
    }
    // std::cout << "Intersect (x1000): " << t.time() << " sec" << std::endl;

    if(intersection)
    {
      if(boost::get<Point3>(&(intersection->first)))
      {
        const Point3* p =  boost::get<Point3>(&(intersection->first));
        // std::cout <<  *p << std::endl;
      }
    }
  }
}

TEST(CGAL, SurfaceMeshGrid)
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  typedef K::Plane_3 Plane;
  typedef K::Ray_3 Ray;
  typedef K::Vector_3 Vector3;
  typedef K::Segment_3 Segment;

  typedef CGAL::Surface_mesh<Point3> Mesh;
  typedef Mesh::Vertex_index vertex_descriptor;
  typedef Mesh::Face_index face_descriptor;

  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
  typedef Tree::Primitive_id Primitive_id;

  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

  typedef CGAL::Timer Timer;

  // Create Grid
  asv::Grid grid({ 100, 100 }, { 4, 4 });

  // Convert to SurfaceMesh
  const Mesh& mesh = *grid.GetMesh();

  // Properties
  // { 
  //   std::cout << "Vertices " << std::endl;    
  //   for(auto&& vertex : mesh.vertices())
  //   {
  //     std::cout << vertex << std::endl;
  //   }
  // }

  // { 
  //   std::cout << "Faces " << std::endl;
  //   for(auto&& face : mesh.faces())
  //   {
  //     std::cout << face << std::endl;

  //     Triangle tri = asv::Geometry::MakeTriangle(mesh, face);
  //     std::cout << tri << std::endl;
  //   }
  // }

  // Intersections
  {
    // std::cout << "AABB Tree " << std::endl;

    // Create a sphere
    std::shared_ptr<asv::Mesh> sphere = CreateSphere(
      "TestSurfaceMeshGridCellSphere", 5.25);

    Timer t;
    t.start();
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();
    t.stop();
    // std::cout << "Build: " << t.time() << " sec" << std::endl;

    t.reset();
    t.start();
    Ray_intersection intersection;
    for (auto v : sphere->vertices())
    {
      Point3 r0 = sphere->point(v);
      Point3 r1(r0.x(), r0.y(), r0.z() - 10.0);
      Ray ray_query(r0, r1);
      intersection = tree.first_intersection(ray_query);
    }
    t.stop();
    // std::cout << "Intersect (x" << sphere->number_of_vertices() << "): " << t.time() << " sec" << std::endl;

    if(intersection)
    {
      if(boost::get<Point3>(&(intersection->first)))
      {
        const Point3* p =  boost::get<Point3>(&(intersection->first));
        // std::cout <<  *p << std::endl;
      }
    }
  }
}

TEST(CGAL, SurfaceMeshModifyGrid)
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  typedef K::Plane_3 Plane;
  typedef K::Ray_3 Ray;
  typedef K::Vector_3 Vector3;
  typedef K::Segment_3 Segment;

  typedef CGAL::Surface_mesh<Point3> Mesh;
  typedef Mesh::Vertex_index vertex_descriptor;
  typedef Mesh::Face_index face_descriptor;

  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
  typedef Tree::Primitive_id Primitive_id;

  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

  typedef CGAL::Timer Timer;

  // Create Grid
  asv::Grid grid({ 100, 100 }, { 4, 4 });

  // Convert to SurfaceMesh
  Mesh& mesh = const_cast<Mesh&>(*grid.GetMesh());

  // Properties
  { 
    // std::cout << "Update Points" << std::endl;    
    for(auto&& vertex : mesh.vertices())
    {
      Point3& point = mesh.point(vertex);
      point += Vector3(0, 0, 10);  
    }
  }

  { 
    // std::cout << "Faces" << std::endl;
    for(auto&& face : mesh.faces())
    {
      // std::cout << face << std::endl;

      Triangle tri = asv::Geometry::MakeTriangle(mesh, face);
      // std::cout << tri << std::endl;
    }
  }
}

TEST(CGAL, SurfaceMeshWavefield)
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point3;
  typedef K::Plane_3 Plane;
  typedef K::Ray_3 Ray;
  typedef K::Vector_3 Vector3;
  typedef K::Segment_3 Segment;

  typedef CGAL::Surface_mesh<Point3> Mesh;
  typedef Mesh::Vertex_index vertex_descriptor;
  typedef Mesh::Face_index face_descriptor;

  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
  typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
  typedef Tree::Primitive_id Primitive_id;

  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

  typedef CGAL::Timer Timer;

  // Wavefield Parameters
  std::shared_ptr<asv::WaveParameters> params(new asv::WaveParameters());
  params->SetNumber(1);
  params->SetAmplitude(3.0);
  params->SetPeriod(10.0);
  params->SetPhase(0.0);

  // Wavefield
  asv::WavefieldGerstner wavefield("TestSurfaceMeshWavefield"); 
  wavefield.SetParameters(params);

  // Evolve to t=10 with 1000 updates
  Timer t;
  t.start();
  for (size_t i=0; i<1000; ++i)
  {
    wavefield.Update(i/100.0);
  }
  t.stop();
  // std::cout << "Update (x1000): " << t.time() << " sec" << std::endl;

  // Debug info
  // auto mesh = wavefield.GetMesh();
  // { 
  //   std::cout << "Faces" << std::endl;
  //   for(auto&& face : mesh->faces())
  //   {
  //     Triangle tri = asv::Geometry::MakeTriangle(*mesh, face);
  //     std::cout << face << ": " << tri << std::endl;
  //   }
  // }
}

TEST(CGAL, TBBParallelFor)
{
  typedef std::vector<double>::iterator Iterator;

  std::vector<double> a(10);

  // kernel lambda...
  auto kernel = [=](double& a)
  {
    // do stuff with a
    a = 1;
  };

  // loop lambda
  auto applyKernel = [=](const tbb::blocked_range<Iterator>& _r)
  {
    for (Iterator it=_r.begin(); it!=_r.end(); ++it)
    { 
      kernel(*it);
    }
  };

  tbb::parallel_for(tbb::blocked_range<Iterator>(a.begin(), a.end()), applyKernel);
  for (auto v : a)
  {
    // std::cout << v << std::endl;
  }
}

TEST(CGAL, VertexRangeIterator)
{
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
  for (
    auto&& vb = std::begin(mesh.vertices());
    vb != std::end(mesh.vertices());
    ++vb)
  {
    auto&& v  = *vb;
    const Point3& p = mesh.point(v);
    // std::cout << p << std::endl;
  }

  // Iterate over two variables using std::pair
  for (
    auto&& it = std::make_pair(std::begin(mesh.vertices()), 0);
    it.first != std::end(mesh.vertices());
    ++it.first, ++it.second)
  {
    auto&& v = *it.first;
    const Point3& p = mesh.point(v);
    // std::cout << it.second << ": " << p << std::endl;
  }

}

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Regular_triangulation_2.h>

TEST(CGAL, MeshToTriangulation2D)
{
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Triangulation_2<K>         Triangulation;
  typedef Triangulation::Vertex_circulator Vertex_circulator;
  typedef Triangulation::Point             Point;
  typedef Triangulation::Face              Face;
  typedef Triangulation::Triangle          Triangle;
  typedef Triangulation::Edge              Edge;
  typedef Triangulation::Segment           Segment;

  // Create the mesh
  std::vector<Point> points;
  std::vector<std::array<size_t, 3>> faces;

  size_t N = 4;
  size_t NPlus1 = N + 1;
  size_t N2 = N * N;
  double L = 10.0;
  double dl = L / N;
  double lm = - L / 2.0;

  // Points - (N+1) points in each row / column 
  for (size_t iy=0; iy<=N; ++iy)
  {
    double py = iy * dl + lm;
    for (size_t ix=0; ix<=N; ++ix)
    {
      // Vertex position
      double px = ix * dl + lm;
      Point point(px, py);        
      points.push_back(point);
    }
  }

  // Faces
  for (size_t iy=0; iy<N; ++iy)
  {
    for (size_t ix=0; ix<N; ++ix)
    {
      // Get the points in the cell coordinates
      size_t idx0 = iy * NPlus1 + ix;
      size_t idx1 = iy * NPlus1 + ix + 1;
      size_t idx2 = (iy+1) * NPlus1 + ix + 1;
      size_t idx3 = (iy+1) * NPlus1 + ix;

      // Faces
      faces.push_back({ idx0, idx1, idx2 });
      faces.push_back({ idx0, idx2, idx3 });
    }
  }

  for (auto&& p : points)
  {
    // std::cout << p << std::endl;
  }
  for (auto&& f : faces)
  {
    // std::cout << f[0] << " " << f[1] << " " << f[2]  << std::endl;
  }

  // Save as triangulation
  // Triangulation t;
  // t.insert(std::begin(points), std::end(points));

  // std::cout << "number of vertices: " << t.number_of_vertices() << std::endl;
  // std::cout << "number of faces:    " << t.number_of_faces() << std::endl;

  // for (auto&& it = t.all_faces_begin(); it != t.all_faces_end(); ++it)
  // {
  //   Face& face = *it;
  //   std::cout << *face.vertex(0)
  //     << ", " << *face.vertex(1)
  //     << ", " << *face.vertex(2)
  //     << std::endl;
  // }
}

TEST(CGAL, CreateVertex)
{
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Triangulation_2<K>          Triangulation;
  typedef Triangulation::Vertex_circulator  Vertex_circulator;
  typedef Triangulation::Vertex_handle      Vertex_handle;
  typedef Triangulation::Face_handle        Face_handle;
  typedef Triangulation::Point              Point;
  typedef Triangulation::Face               Face;
  typedef Triangulation::Triangle           Triangle;
  typedef Triangulation::Edge               Edge;
  typedef Triangulation::Segment            Segment;

  // Points
  Point p1(0, 0);
  Point p2(0, 1);
  Point p3(1, 0);
  Point p4(1, 1);

  {
    // Automatically create a triangulation
    Triangulation t;
    Triangulation::Triangulation_data_structure& tds = t.tds();

    t.insert(p1);
    t.insert(p2);
    t.insert(p3);
    // t.insert(p4);

    // Check valid
    // std::cout << "triangulation data structure" << std::endl;
    // std::cout << "is valid: " << tds.is_valid() << std::endl;
    // std::cout << "dimension: " << tds.dimension() << std::endl;
    // std::cout << "number of vertices : " << tds.number_of_vertices() << std::endl;
    // std::cout << "number of faces : " << tds.number_of_faces() << std::endl;
    // std::cout << "number of edges : " << tds.number_of_edges() << std::endl;
    // std::cout << tds << std::endl; 

    // std::cout << "triangulation" << std::endl;
    // std::cout << "is valid: " << t.is_valid() << std::endl;
    // std::cout << "dimension: " << t.dimension() << std::endl;
    // std::cout << "number of vertices : " << t.number_of_vertices() << std::endl;
    // std::cout << "number of faces : " << t.number_of_faces() << std::endl;
    // std::cout << t << std::endl; 

    // std::cout << "finite vertices" << std::endl;
    // for (auto v=t.finite_vertices_begin(); v != t.finite_vertices_end(); ++v)
    // {
    //   std::cout << *v << std::endl;
    // }

    // std::cout << "finite faces" << std::endl;
    // for (auto f=t.finite_faces_begin(); f != t.finite_faces_end(); ++f)
    // {
    //   std::cout << *f->vertex(0) << ", " << *f->vertex(1) << ", " << *f->vertex(2) << std::endl;
    // }


  }

  if (1)
  {
    // Manually create a triangulation
    Triangulation t;
    Triangulation::Triangulation_data_structure& tds = t.tds();

    // Clear all finite faces and vertices
    tds.clear();

    // Set dimension
    tds.set_dimension(2);

    // std::cout << "before vertex insertion..." << std::endl;
    // std::cout << "is valid: " << tds.is_valid() << std::endl;
    // std::cout << "dimension: " << tds.dimension() << std::endl;
    // std::cout << "number of vertices : " << tds.number_of_vertices() << std::endl;
    // std::cout << "number of faces : " << tds.number_of_faces() << std::endl;
    // std::cout << "number of edges : " << tds.number_of_edges() << std::endl;
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
    // std::cout << "v0->is_valid: " << v0->is_valid(true) << std::endl;
    // std::cout << "v1->is_valid: " << v1->is_valid(true) << std::endl;
    // std::cout << "v2->is_valid: " << v2->is_valid(true) << std::endl;
    // std::cout << "v3->is_valid: " << v3->is_valid(true) << std::endl;
    EXPECT_TRUE(v0->is_valid());
    EXPECT_TRUE(v1->is_valid());
    EXPECT_TRUE(v2->is_valid());
    EXPECT_TRUE(v3->is_valid());

    // Check faces valid
    // std::cout << "f0->is_valid: " << f0->is_valid(true) << std::endl;
    // std::cout << "f1->is_valid: " << f1->is_valid(true) << std::endl;
    // std::cout << "f2->is_valid: " << f2->is_valid(true) << std::endl;
    // std::cout << "f3->is_valid: " << f3->is_valid(true) << std::endl;
    EXPECT_TRUE(f0->is_valid());
    EXPECT_TRUE(f1->is_valid());
    EXPECT_TRUE(f2->is_valid());
    EXPECT_TRUE(f3->is_valid());

    // Check valid
    // std::cout << "is valid: " << tds.is_valid() << std::endl;
    // std::cout << "dimension: " << tds.dimension() << std::endl;
    // std::cout << "number of vertices : " << tds.number_of_vertices() << std::endl;
    // std::cout << "number of faces : " << tds.number_of_faces() << std::endl;
    // std::cout << "number of edges : " << tds.number_of_edges() << std::endl;
    EXPECT_TRUE(tds.is_valid());
    EXPECT_EQ(tds.dimension(), 2);
    EXPECT_EQ(tds.number_of_vertices(), 4);
    EXPECT_EQ(tds.number_of_faces(), 4);
    EXPECT_EQ(tds.number_of_edges(), 6);

    // std::cout << "triangulation data structure" << std::endl;
    // std::cout << tds << std::endl; 

    // std::cout << "is infinite (T): " << t.is_infinite(v0) << std::endl;
    // std::cout << "is infinite (T): " << t.is_infinite(f0) << std::endl;
    // std::cout << "is infinite (T): " << t.is_infinite(f2) << std::endl;
    // std::cout << "is infinite (T): " << t.is_infinite(f3) << std::endl;
    EXPECT_TRUE(t.is_infinite(v0));
    EXPECT_TRUE(t.is_infinite(f0));
    EXPECT_TRUE(t.is_infinite(f2));
    EXPECT_TRUE(t.is_infinite(f3));

    // std::cout << "is infinite (F): " << t.is_infinite(v1) << std::endl;
    // std::cout << "is infinite (F): " << t.is_infinite(v2) << std::endl;
    // std::cout << "is infinite (F): " << t.is_infinite(v3) << std::endl;
    // std::cout << "is infinite (F): " << t.is_infinite(f1) << std::endl;
    EXPECT_FALSE(t.is_infinite(v1));
    EXPECT_FALSE(t.is_infinite(v2));
    EXPECT_FALSE(t.is_infinite(v3));
    EXPECT_FALSE(t.is_infinite(f1));

    // std::cout << "is valid: " << t.is_valid() << std::endl;
    // std::cout << "dimension: " << t.dimension() << std::endl;
    // std::cout << "number of vertices : " << t.number_of_vertices() << std::endl;
    // std::cout << "number of faces : " << t.number_of_faces() << std::endl;
    EXPECT_TRUE(t.is_valid());
    EXPECT_EQ(t.dimension(), 2);
    EXPECT_EQ(t.number_of_vertices(), 3);
    EXPECT_EQ(t.number_of_faces(), 1);

    // std::cout << "triangulation" << std::endl;
    // std::cout << t << std::endl; 


    // Point location.
    



  }

  // List vertices


  // List faces


}

// TEST(CGAL, PointLocation)
// {
//   typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
//   typedef CGAL::Regular_triangulation_2<K> Regular_triangulation;

//    std::ifstream in("data/regular.cin");
//    Regular_triangulation::Weighted_point wp;
//    int count = 0;
//    std::vector<Regular_triangulation::Weighted_point> wpoints;
//    while(in >> wp){
//        count++;
//      wpoints.push_back(wp);
//    }

//    Regular_triangulation rt(wpoints.begin(), wpoints.end());
//    rt.is_valid();
//    std::cout << "number of inserted points : " << count << std::endl;
//    std::cout << "number of vertices :  " ;
//    std::cout << rt.number_of_vertices() << std::endl;
//    std::cout << "number of hidden vertices :  " ;
//    std::cout << rt.number_of_hidden_vertices() << std::endl;

// }

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

