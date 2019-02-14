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
    std::cout << "Mesh:       " << source->GetName() << std::endl;
    std::cout << "Vertex:     " << source->GetVertexCount() << std::endl;

    std::shared_ptr<asv::Mesh> target = std::make_shared<asv::Mesh>();
    asv::MeshTools::MakeSurfaceMesh(*source, *target);
    return target;
  }

} // namespace (anon)

///////////////////////////////////////////////////////////////////////////////
// Define tests
void TestSurprising()
{
  std::cout << "TestSurprising..." << std::endl;

  typedef CGAL::Simple_cartesian<double> Kernel;
  typedef Kernel::Point_2 Point_2;

  {
    Point_2 p(0, 0.3), q(1, 0.6), r(2, 0.9);
    std::cout << (CGAL::collinear(p,q,r) ? "collinear\n" : "not collinear\n");   
  }
  {
    Point_2 p(0, 1.0/3.0), q(1, 2.0/3.0), r(2, 1);
    std::cout << (CGAL::collinear(p,q,r) ? "collinear\n" : "not collinear\n");   
  }  
  {
    Point_2 p(0,0), q(1, 1), r(2, 2);
    std::cout << (CGAL::collinear(p,q,r) ? "collinear\n" : "not collinear\n");   
  }  
}

void TestSurfaceMesh()
{
  std::cout << "TestSurfaceMesh..." << std::endl;
  
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
      std::cout << "intersection(s)" << std::endl;
  else
      std::cout << "no intersection" << std::endl;

  // computes #intersections with segment query
  std::cout << tree.number_of_intersected_primitives(segment_query)
      << " intersection(s)" << std::endl;

  // computes first encountered intersection with segment query
  // (generally a point)
  Segment_intersection intersection =
      tree.any_intersection(segment_query);
  if(intersection){
    // gets intersection object
    if(boost::get<Point3>(&(intersection->first))){
      Point3* p = boost::get<Point3>(&(intersection->first));
      std::cout << "intersection object is a point " << *p <<  std::endl;
      std::cout << "with face "<< intersection->second  <<  std::endl;
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
  if(plane_intersection){
    if(boost::get<Segment>(&(plane_intersection->first))){
      Segment* s = boost::get<Segment>(&(plane_intersection->first));
      std::cout << "one intersection object is the segment " << s << std::endl;
      std::cout << "with face "<< intersection->second  <<  std::endl;
    }
  }
}

/// This example based upon the examples distributed with CGAL in:
/// CGAL-4.13/examples/AABB_tree/AABB_polyhedron_facet_intersection_example.cpp
/// Author(s) : Camille Wormser, Pierre Alliez
///
void TestAABBPolyhedronFacetIntersection()
{
  std::cout << "TestAABBPolyhedronFacetIntersection..." << std::endl;

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
  std::cout << "squared distance: " << sqd << std::endl;

  // computes closest point
  Point3 closest = tree.closest_point(query);
  std::cout << "closest point: " << closest << std::endl;

  // computes closest point and primitive id
  Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
  Point3 closest_point = pp.first;
  Polyhedron::Face_handle f = pp.second; // closest primitive id
  std::cout << "closest point: " << closest_point << std::endl;
  std::cout << "closest triangle: ( "
    << f->halfedge()->vertex()->point() << " , " 
    << f->halfedge()->next()->vertex()->point() << " , "
    << f->halfedge()->next()->next()->vertex()->point()
    << " )" << std::endl;
}

void TestSurfaceMeshGridCell()
{
  std::cout << "TestSurfaceMeshGridCell..." << std::endl;
  
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
    std::cout << "Vertices " << std::endl;    
    for(auto&& vertex : mesh.vertices())
    {
      std::cout << vertex << std::endl;
    }
  }

  { 
    std::cout << "Faces " << std::endl;
    for(auto&& face : mesh.faces())
    {
      std::cout << face << std::endl;

      Triangle tri = asv::Geometry::MakeTriangle(mesh, face);
      std::cout << tri << std::endl;
    }
  }

  {
    std::cout << "AABB Tree " << std::endl;

    Timer t;
    t.start();
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();
    std::cout << "Build: " << t.time() << " sec" << std::endl;

    Point3 r0(0.5, 0.5, 101.0);
    Point3 r1(0.5, 0.5, 100.0);

    Ray ray_query(r0, r1);
    t.start();
    Ray_intersection intersection;
    for (int i=0; i<1000; ++i)
    {
      intersection = tree.first_intersection(ray_query);
    }
    std::cout << "Intersect (x1000): " << t.time() << " sec" << std::endl;

    if(intersection)
    {
      if(boost::get<Point3>(&(intersection->first)))
      {
        const Point3* p =  boost::get<Point3>(&(intersection->first));
        std::cout <<  *p << std::endl;
      }
    }
  }
}

void TestSurfaceMeshGrid()
{
  std::cout << "TestSurfaceMeshGrid..." << std::endl;
  
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
    std::cout << "AABB Tree " << std::endl;

    // Create a sphere
    std::shared_ptr<asv::Mesh> sphere = CreateSphere(
      "TestSurfaceMeshGridCellSphere", 5.25);

    Timer t;
    t.start();
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();
    t.stop();
    std::cout << "Build: " << t.time() << " sec" << std::endl;

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
    std::cout << "Intersect (x" << sphere->number_of_vertices() << "): " << t.time() << " sec" << std::endl;

    if(intersection)
    {
      if(boost::get<Point3>(&(intersection->first)))
      {
        const Point3* p =  boost::get<Point3>(&(intersection->first));
        std::cout <<  *p << std::endl;
      }
    }
  }
}

void TestSurfaceMeshModifyGrid()
{
  std::cout << "TestSurfaceMeshModifyGrid..." << std::endl;
  
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
    std::cout << "Update Points" << std::endl;    
    for(auto&& vertex : mesh.vertices())
    {
      Point3& point = mesh.point(vertex);
      point += Vector3(0, 0, 10);  
    }
  }

  { 
    std::cout << "Faces" << std::endl;
    for(auto&& face : mesh.faces())
    {
      std::cout << face << std::endl;

      Triangle tri = asv::Geometry::MakeTriangle(mesh, face);
      std::cout << tri << std::endl;
    }
  }
}

void TestSurfaceMeshWavefield()
{
  std::cout << "TestSurfaceMeshWavefield..." << std::endl;
  
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
  asv::Wavefield wavefield("TestSurfaceMeshWavefield"); 
  wavefield.SetParameters(params);

  // Evolve to t=10 with 1000 updates
  Timer t;
  t.start();
  for (size_t i=0; i<1000; ++i)
  {
    wavefield.Update(i/100.0);
  }
  t.stop();
  std::cout << "Update (x1000): " << t.time() << " sec" << std::endl;

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

void TestTBBParallelFor()
{
  std::cout << "TestTBBParallelFor..." << std::endl;

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
    std::cout << v << std::endl;
}

void TestVertexRangeIterator()
{
  std::cout << "TestVertexRangeIterator..." << std::endl;

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
    std::cout << p << std::endl;
  }

  // Iterate over two variables using std::pair
  for (
    auto&& it = std::make_pair(std::begin(mesh.vertices()), 0);
    it.first != std::end(mesh.vertices());
    ++it.first, ++it.second)
  {
    auto&& v = *it.first;
    const Point3& p = mesh.point(v);
    std::cout << it.second << ": " << p << std::endl;
  }

}


///////////////////////////////////////////////////////////////////////////////
// Run tests
void RunCGALTests()
{
  TestSurprising();
  TestSurfaceMesh();
  TestAABBPolyhedronFacetIntersection();
  TestSurfaceMeshGridCell();
  TestSurfaceMeshGrid();
  TestSurfaceMeshModifyGrid();
  TestSurfaceMeshWavefield();
  TestTBBParallelFor();
  TestVertexRangeIterator();
}

