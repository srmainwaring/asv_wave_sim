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

#include "asv_wave_sim_gazebo_plugins/PointLocator.hh"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Timer.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <CGAL/algorithm.h>
#include <CGAL/point_generators_2.h>

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// PointLocatorPrivate

  class PointLocatorPrivate
  {
  public:
    virtual ~PointLocatorPrivate();
    PointLocatorPrivate(size_t _N, double _L);

    bool Locate(const Point3& p, size_t& faceIndex) const;

    void CreateMesh();
    void CreateTriangulation();
    
    bool IsValid(bool verbose=false) const;

    void DebugPrintMesh() const;
    void DebugPrintTriangulation() const;

    // Type definitions
    typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
    typedef CGAL::Projection_traits_xy_3<K>                             Kp;
    typedef CGAL::Triangulation_vertex_base_with_info_2<int, Kp>        Vbb;
    typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>            Vb;
    typedef CGAL::Constrained_triangulation_face_base_2<Kp>             Fbb;
    typedef CGAL::Triangulation_face_base_with_info_2<int, Kp, Fbb>     Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                Tds;
    typedef CGAL::No_intersection_tag                                   Itag;
    typedef CGAL::Constrained_Delaunay_triangulation_2<Kp, Tds, Itag>   Tb;

    typedef CGAL::Triangulation_hierarchy_2<Tb>     Triangulation;
    typedef Triangulation::Vertex_handle            Vertex_handle;
    typedef Triangulation::Face_handle              Face_handle;
    typedef Triangulation::Face                     Face;
    typedef Triangulation::Triangle                 Triangle;
    typedef K::Point_3                              Point;

    // Dimensions
    size_t N;
    double L;

    // Derived dimensions
    size_t NPlus1;

    // Mesh
    std::vector<Point>                  points;
    std::vector<std::array<size_t, 3>>  indices;
    std::vector<std::array<size_t, 2>>  infiniteIndices;

    // Triangulation / Triangulation Hierarchy
    Triangulation tri;
  };

  PointLocatorPrivate::~PointLocatorPrivate()
  {
  }

  PointLocatorPrivate::PointLocatorPrivate(size_t _N, double _L) : N(_N), L(_L), NPlus1(_N + 1)
  {  
  }

  bool PointLocatorPrivate::Locate(const Point3& p, size_t& faceIndex) const
  {
    auto fh = tri.locate(Point(p.x(), p.y(), p.z()));
    if (fh != nullptr)
    {
      faceIndex = fh->info();
      return true;
    }
    return false;
  }

  void PointLocatorPrivate::CreateMesh()
  {
    // size_t N = 3;
    // size_t NPlus1 = N + 1;
    // double L = 10.0;
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
        Point point(px, py, 0.0);
        points.push_back(point);
      }
    }

    // Face indices
    for (size_t iy=0; iy<N; ++iy)
    {
      for (size_t ix=0; ix<N; ++ix)
      {
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
    for (size_t i=0; i<N; ++i)
    {
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
  }

  void PointLocatorPrivate::CreateTriangulation()
  {
    // Insert points - NOTE: must insert the points to build the triangulation hierarchy.
    // CGAL::Timer timer;

    // Point with info
    std::vector<std::pair<Point, int>> pointsWithInfo;
    for (size_t i=0; i<points.size(); ++i)
    {
      pointsWithInfo.push_back(std::make_pair(points[i], i));
    }

    // timer.start();
    // @NOTE insert poinst with info not compiling for a triangulation hierarcy. See below. 
    // tri.insert(pointsWithInfo.begin(), pointsWithInfo.end());
    tri.insert(points.begin(), points.end());
    // timer.stop();
    // std::cout << "insert points: " << timer.time() << " s" << std::endl;

    // Set vertex info
    // @NOTE - use this work around because the insert for points with info
    // does not compile for a triangulation hierarchy.
    // timer.reset();
    // timer.start();
    Face_handle fh;
    for (size_t i=0; i<points.size(); ++i)
    {
      auto vh = tri.insert(points[i] ,fh);
      if (vh != nullptr)
      {
        vh->info() = i;
      }
    }
    // timer.stop();
    // std::cout << "set vertex info: " << timer.time() << " s" << std::endl;

    // Constraint indices 
    std::vector<std::pair<size_t, size_t>> cindices;
    for (size_t iy=0; iy<N; ++iy)
    {
      for (size_t ix=0; ix<N; ++ix)
      {
        size_t idx1 = iy * NPlus1 + ix;
        size_t idx2 = (iy + 1) * NPlus1 + (ix + 1);
        cindices.push_back(std::make_pair(idx1, idx2));
      }
    }

    // Insert constraints
    // timer.reset();
    // timer.start();
    tri.insert_constraints(points.begin(), points.end(), cindices.begin(), cindices.end());   
    // timer.stop();
    // std::cout << "insert constraints: " << timer.time() << " s" << std::endl;

    // Set info on infinite vertex
    tri.infinite_vertex()->info() = -1;

    // Face list mapping  

    // Initialise face info
    // timer.reset();
    // timer.start();
    for (auto f = tri.all_faces_begin(); f != tri.all_faces_end(); ++f)
    {
      f->info() = -1; 
    }
    // timer.stop();
    // std::cout << "initialise face info: (" << timer.time() << " s)" << std::endl;

    // Face matching
    fh = nullptr;
    size_t idx = 0;
    // timer.reset();
    // timer.start();
    for (auto f = indices.begin(); f != indices.end(); ++f, ++idx)
    {
      // Compute the centroid of f and locate the tri face containing this point.
      auto& p0 = points[f->at(0)];
      auto& p1 = points[f->at(1)];
      auto& p2 = points[f->at(2)];
      Point p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0,
        0.0
      );
      fh = tri.locate(p, fh);

      // Set the info value on the found face.
      if (fh != nullptr)
      {
        fh->info() = idx;
      }
    }
    // timer.stop();
    // std::cout << "face mapping: (" << timer.time() << " s)" << std::endl;    
  }

  void PointLocatorPrivate::DebugPrintMesh() const
  {
    std::cout << "N: " << N << std::endl;
    std::cout << "L: " << L << std::endl;

    std::cout << "points: [" << points.size() << "]" << std::endl;
    for (auto&& p : points)
    {
      std::cout << p << std::endl;
    }
    std::cout << "indices: [" << indices.size() << "]" << std::endl;
    for (auto&& i : indices)
    {
      std::cout << i[0] << " " << i[1] << " " << i[2]  << std::endl;
    }
    std::cout << "infinite indices: [" << infiniteIndices.size() << "]" << std::endl;
    for (auto&& i : infiniteIndices)
    {
      std::cout << i[0] << " " << i[1] << std::endl;
    }
  }

  void PointLocatorPrivate::DebugPrintTriangulation() const
  {
    const Triangulation::Triangulation_data_structure& ctds = tri.tds();

    std::cout << "triangulation hierarchy data structure" << std::endl;
    std::cout << ctds << std::endl;

    std::cout << "is valid: " << ctds.is_valid() << std::endl;
    std::cout << "dimension: " << ctds.dimension() << std::endl;
    std::cout << "number of vertices : " << ctds.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << ctds.number_of_faces() << std::endl;
    std::cout << "number of edges : " << ctds.number_of_edges() << std::endl;
    std::cout << std::endl;

    std::cout << "triangulation hierarchy" << std::endl;
    std::cout << tri << std::endl; 
    
    std::cout << "is valid: " << tri.is_valid() << std::endl;
    std::cout << "dimension: " << tri.dimension() << std::endl;
    std::cout << "number of vertices : " << tri.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << tri.number_of_faces() << std::endl;

    std::cout << "hierarchy: " << tri.is_valid() << std::endl;
    tri.is_valid(true);

    size_t idx = 0;
    std::cout << "vertex -> mesh vertex:" << std::endl;
    for (auto v = tri.all_vertices_begin(); v != tri.all_vertices_end(); ++v, ++idx)
    {
      std::cout << "vertex: " << idx
        << ", mesh vertex: " << v->info()
        << std::endl; 
    }

    idx = 0;
    std::cout << "face -> mesh face:" << std::endl;
    for (auto f = tri.all_faces_begin(); f != tri.all_faces_end(); ++f, ++idx)
    {
      std::cout << "face: " << idx
        << ", mesh face: " << f->info()
        << std::endl; 
    }
  }

  bool PointLocatorPrivate::IsValid(bool verbose) const
  {
    bool isValid = true;

    // Triangulation Hierarchy
    const Triangulation::Triangulation_data_structure& tds = tri.tds();

    // Verify infinite vertex
    auto vi = tri.infinite_vertex();
    isValid &= vi->is_valid(verbose);

    // Verify vertices
    for (auto v = tds.vertices_begin(); v !=  tds.vertices_end(); ++v)
    {
      isValid &= v->is_valid(verbose);
    }

    // Verify faces
    for (auto f = tds.faces_begin(); f !=  tds.faces_end(); ++f)
    {
      isValid &= f->is_valid(verbose);
    }

    // Verify triangulation hierarchy data structure
    isValid &= tds.is_valid(verbose);

    // Verify triangulation hierarchy
    isValid &= tri.is_valid(verbose);

    return isValid;
  }

///////////////////////////////////////////////////////////////////////////////

  PointLocator::~PointLocator()
  {  
  }

  PointLocator::PointLocator(size_t _N, double _L) :
    impl(new PointLocatorPrivate(_N, _L))
  {  
  }

  bool PointLocator::Locate(const Point3& p, size_t& faceIndex) const
  {
    return this->impl->Locate(p, faceIndex);
  }

  void PointLocator::CreateMesh()
  {
    // CGAL::Timer timer;
    // timer.start();
    this->impl->CreateMesh();
    // timer.stop();
    // std::cout << "CreateMesh: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocator::CreateTriangulation()
  {
    // CGAL::Timer timer;
    // timer.start();
    this->impl->CreateTriangulation();
    // timer.stop();
    // std::cout << "CreateTriangulation: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocator::DebugPrintMesh() const
  {
    this->impl->DebugPrintMesh();
  }

  void PointLocator::DebugPrintTriangulation() const
  {
    this->impl->DebugPrintTriangulation();
  }

  bool PointLocator::IsValid(bool verbose) const
  {
    return this->impl->IsValid(verbose);
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv
