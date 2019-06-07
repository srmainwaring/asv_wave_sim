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
    void CreateMesh();
    void CreateTriangulation();
    void CreateTriangulationHierarchy();
    void CreateConstrainedDelaunayTriangulationHierarchy();
    
    bool IsValid(bool verbose=false) const;

    void DebugPrintMesh() const;
    void DebugPrintTriangulation() const;
    void DebugPrintTriangulationHierarchy() const;

    // Type definitions
    typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
    typedef CGAL::Projection_traits_xy_3<K>                             Kp;

    // typedef CGAL::Triangulation_vertex_base_2<Kp>                       Vbbb;
    // typedef CGAL::Triangulation_vertex_base_with_info_2<int, Kp, Vbbb>  Vbb;
    typedef CGAL::Triangulation_vertex_base_with_info_2<int, Kp>        Vbb;
    typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>            Vb;

    typedef CGAL::Constrained_triangulation_face_base_2<Kp>             Fbb;
    typedef CGAL::Triangulation_face_base_with_info_2<int, Kp, Fbb>     Fb;

    typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                Tds;
    typedef CGAL::No_intersection_tag                                   Itag;
    typedef CGAL::Constrained_Delaunay_triangulation_2<Kp, Tds, Itag>   Tb;

    // typedef Tb                                      Triangulation;
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
    Triangulation t;
    Triangulation ct;
  };

  PointLocatorPrivate::~PointLocatorPrivate()
  {
  }

  PointLocatorPrivate::PointLocatorPrivate(size_t _N, double _L) : N(_N), L(_L), NPlus1(_N + 1)
  {  
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
    Triangulation::Triangulation_data_structure& tds = t.tds();

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
    for (auto&& p : points)
    {
      Vertex_handle v = tds.create_vertex();

      // Assign points to finite vertices
      v->set_point(p);
      vertices.push_back(v);
    }

    // Finite faces
    std::vector<Face_handle> faces;
    for (auto&& i : indices)
    {
      auto& v0 = vertices[i[0]];
      auto& v1 = vertices[i[1]];
      auto& v2 = vertices[i[2]];
      Face_handle f = tds.create_face(v0, v1, v2);
      faces.push_back(f);
    }

    // Infinite faces
    std::vector<Face_handle> infiniteFaces;
    for (auto&& i : infiniteIndices)
    {
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
    for (size_t iy=0; iy<N; ++iy)
    {
      for (size_t ix=0; ix<N; ++ix)
      {
        size_t vidx = iy * NPlus1 + ix;
        size_t fidx = 2 * (iy * N + ix);
        vertices[vidx]->set_face(faces[fidx]);
        // @DEBUG_INFO
        debugVertexFaces[vidx] = fidx;
      }
    }
    // Set vertex faces : top and right
    for (size_t i=0; i<N; ++i)
    {
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
    // Set vertex face : top - right
    {
      size_t vidx = NPlus1*NPlus1-1;
      size_t fidx = 2 * (N * N - 1);
      vertices[vidx]->set_face(faces[fidx]);    
      // @DEBUG_INFO
      debugVertexFaces[vidx] = fidx;
    }

    // std::cout << "vertex : faces" << std::endl;
    // for (size_t i=0; i<debugVertexFaces.size(); ++i)
    // {
    //   std::cout << i << " " << debugVertexFaces[i] << std::endl;
    // }

    // Set Finite Face neighbours
    for (size_t iy=0; iy<N; ++iy)
    {
      for (size_t ix=0; ix<N; ++ix)
      {
        for (size_t k=0; k<2; ++k)
        {
          size_t idx = 2 * (iy * N + ix);

          if (k == 0)          
          {
            if (iy == 0)
            {
              if (ix == N-1)
              {
                // case 1
                size_t fk = idx;
                size_t f0 = N;
                size_t f1 = idx + 1;
                size_t f2 = N - 1;
                faces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], infiniteFaces[f2]);
              }
              else // ix != N-1
              {
                // case 2
                size_t fk = idx;
                size_t f0 = idx + 3;
                size_t f1 = idx + 1;
                size_t f2 = ix;
                faces[fk]->set_neighbors(faces[f0], faces[f1], infiniteFaces[f2]);
              }
            }
            else // iy != 0
            {
              if (ix == N-1)
              {
                // case 3
                size_t fk = idx;
                size_t f0 = N + iy;
                size_t f1 = idx + 1;
                size_t f2 = idx + 1 - 2 * N;
                faces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], faces[f2]);
              }
              else
              {
                // case 4 (general)
                size_t fk = idx;
                size_t f0 = idx + 3;
                size_t f1 = idx + 1;
                size_t f2 = idx + 1 - 2 * N;
                faces[fk]->set_neighbors(faces[f0], faces[f1], faces[f2]);
              }
            } 
          }
          if (k == 1)          
          {
            if (ix == 0)
            {
              if (iy == N-1)
              {
                // case 5
                size_t fk = idx + 1;
                size_t f0 = 3 * N - 1;
                size_t f1 = 3 * N;
                size_t f2 = idx;
                faces[fk]->set_neighbors(infiniteFaces[f0], infiniteFaces[f1], faces[f2]);
              }
              else
              {
                // case 6
                size_t fk = idx + 1;
                size_t f0 = idx + 2 * N;
                size_t f1 = 4 * N - 1 - iy;
                size_t f2 = idx;
                faces[fk]->set_neighbors(faces[f0], infiniteFaces[f1], faces[f2]);
              }
            }
            else // ix != 0
            {
              if (iy == N-1)
              {
                // case 7
                size_t fk = idx + 1;
                size_t f0 = 3 * N - 1 - ix;
                size_t f1 = idx - 2;
                size_t f2 = idx;
                faces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], faces[f2]);
              }
              else
              {
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
    for (size_t i=0; i<N; ++i)
    {
      // bottom
      {
        size_t fk = i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * i;
        size_t f2 = fk - 1;
        if (i == 0)
          f2 = 4 * N - 1;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], infiniteFaces[f2]);
      }

      // right
      {
        size_t fk = N + i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * (i * N + N - 1);
        size_t f2 = fk - 1;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], infiniteFaces[f2]);        
      }

      // top
      {
        size_t fk = 2 * N + i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * ((N - 1) * N + i) + 1;
        size_t f2 = fk - 1;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], infiniteFaces[f2]);
      }

      // left
      {
        size_t fk = 3 * N + i;
        size_t f0 = fk + 1;
        size_t f1 = 2 * (i * N) + 1;
        size_t f2 = fk - 1;
        if (i == N - 1)
          f0 = 0;
        infiniteFaces[fk]->set_neighbors(infiniteFaces[f0], faces[f1], infiniteFaces[f2]);
      }
    }

    // Initialise face info
    for (auto f = t.all_faces_begin(); f != t.all_faces_end(); ++f)
    {
      f->info() = -2; 
    }
  }

  void PointLocatorPrivate::CreateTriangulationHierarchy()
  {
    CGAL::Timer timer;

    Triangulation::Triangulation_data_structure& ctds = ct.tds();

    // Add all finite edges as constraints.
    std::cout << "adding finite edge constraints: ["
      << std::distance(t.finite_edges_begin(), t.finite_edges_end())
      << "]" << std::endl;
    timer.reset();
    timer.start();
    for (auto e = t.finite_edges_begin(); e != t.finite_edges_end(); ++e)
    {      
      const auto& f = e->first;
      int i = e->second;
      const auto& v0 = f->vertex(f->cw(i));
      const auto& v1 = f->vertex(f->ccw(i));
      ct.insert_constraint(v0->point(), v1->point());
    }
    timer.stop();
    std::cout << "finite edge constraints: (" << timer.time() << " s)" << std::endl;

    // Now insert all finite points (to force building the triangulation hierarchy)
    timer.reset();
    timer.start();
    for (auto v = ct.finite_vertices_begin(); v != ct.finite_vertices_end(); ++v)
    {      
      ct.insert(v->point());
    }
    timer.stop();
    std::cout << "finite points: (" << timer.time() << " s)" << std::endl;

    // Face list mapping  

    // Initialise face info
    timer.reset();
    timer.start();
    for (auto f = ct.all_faces_begin(); f != ct.all_faces_end(); ++f)
    {
      f->info() = -1; 
    }
    timer.stop();
    std::cout << "initialise face info: (" << timer.time() << " s)" << std::endl;

    // Face matching
    Face_handle fh;
    size_t idx = 0;
    timer.reset();
    timer.start();
    for (auto f = t.finite_faces_begin(); f != t.finite_faces_end(); ++f, ++idx)
    {
      // Compute the centroid of f and locate the ct face containing this point.
      Point p0 = f->vertex(0)->point();
      Point p1 = f->vertex(1)->point();
      Point p2 = f->vertex(2)->point();
      Point p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0,
        0.0
      );
      fh = ct.locate(p, fh);
      bool found = fh != nullptr;

      // @ISSUE - the problem with using is_face is that fh is a handle
      //          to a Face in triangulation t not ct. It appears that
      //          is_face obtains the face from the vertices rather than
      //          the list of faces stored in the triangular data structure. 
      // bool found = ct.is_face(f->vertex(0), f->vertex(1), f->vertex(2), fh);

      // Set the info value on the found face.
      if (fh != nullptr)
      {
        // std::distance is O(N) making this loop O(N^2)
        // auto d1 = std::distance(t.finite_faces_begin(), f);
        // fh->info() = d1;
        fh->info() = idx;
        // std::cout << "found: " << found
        //   << ", is_infinite: " << ct.is_infinite(fh)
        //   << ", info: " << fh->info()
        //   << std::endl;
      }
    }
    timer.stop();
    std::cout << "face mapping: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocatorPrivate::CreateConstrainedDelaunayTriangulationHierarchy()
  {
    // Insert points - NOTE: must insert the points to build the triangulation hierarchy.
    CGAL::Timer timer;

    // Point with info
    std::vector<std::pair<Point, int>> pointsWithInfo;
    for (size_t i=0; i<points.size(); ++i)
    {
      pointsWithInfo.push_back(std::make_pair(points[i], i));
    }

    timer.start();
    // @NOTE insert poinst with info not compiling for a triangulation hierarcy. See below. 
    // ct.insert(pointsWithInfo.begin(), pointsWithInfo.end());
    ct.insert(points.begin(), points.end());
    timer.stop();
    std::cout << "insert points: " << timer.time() << " s" << std::endl;

    // Set vertex info
    // @NOTE - use this work around because the insert for points with info
    // does not compile for a triangulation hierarchy.
    timer.reset();
    timer.start();
    Face_handle fh;
    for (size_t i=0; i<points.size(); ++i)
    {
      auto vh = ct.insert(points[i] ,fh);
      if (vh != nullptr)
      {
        vh->info() = i;
      }
    }
    timer.stop();
    std::cout << "set vertex info: " << timer.time() << " s" << std::endl;

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
    timer.reset();
    timer.start();
    ct.insert_constraints(points.begin(), points.end(), cindices.begin(), cindices.end());   
    timer.stop();
    std::cout << "insert constraints: " << timer.time() << " s" << std::endl;

    // Set info on infinite vertex
    ct.infinite_vertex()->info() = -1;

    // Face list mapping  

    // Initialise face info
    timer.reset();
    timer.start();
    for (auto f = ct.all_faces_begin(); f != ct.all_faces_end(); ++f)
    {
      f->info() = -1; 
    }
    timer.stop();
    std::cout << "initialise face info: (" << timer.time() << " s)" << std::endl;

    // Face matching
    // Face_handle fh;
    fh = nullptr;
    size_t idx = 0;
    timer.reset();
    timer.start();
    for (auto f = indices.begin(); f != indices.end(); ++f, ++idx)
    {
      // Compute the centroid of f and locate the ct face containing this point.
      auto& p0 = points[f->at(0)];
      auto& p1 = points[f->at(1)];
      auto& p2 = points[f->at(2)];
      Point p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0,
        0.0
      );
      fh = ct.locate(p, fh);

      // Set the info value on the found face.
      if (fh != nullptr)
      {
        fh->info() = idx;
      }
    }
    timer.stop();
    std::cout << "face mapping: (" << timer.time() << " s)" << std::endl;    
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
    const Triangulation::Triangulation_data_structure& tds = t.tds();

    // std::cout << "vertex : faces" << std::endl;
    // for (size_t i=0; i<debugVertexFaces.size(); ++i)
    // {
    //   std::cout << i << " " << debugVertexFaces[i] << std::endl;
    // }

    std::cout << "triangulation data structure" << std::endl;
    std::cout << tds << std::endl; 

    std::cout << "is valid: " << tds.is_valid() << std::endl;
    std::cout << "dimension: " << tds.dimension() << std::endl;
    std::cout << "number of vertices : " << tds.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << tds.number_of_faces() << std::endl;
    std::cout << "number of edges : " << tds.number_of_edges() << std::endl;
    std::cout << std::endl;

    std::cout << "triangulation" << std::endl;
    std::cout << t << std::endl; 

    std::cout << "is valid: " << t.is_valid() << std::endl;
    std::cout << "dimension: " << t.dimension() << std::endl;
    std::cout << "number of vertices : " << t.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << t.number_of_faces() << std::endl;
    std::cout << std::endl;

    // This would just show the mapping to the default initialised face info (= -2)
    // size_t idx = 0;
    // std::cout << "face -> index:" << std::endl;
    // for (auto f = t.finite_faces_begin(); f != t.finite_faces_end(); ++f, ++idx)
    // {
    //   std::cout << "face: " << idx
    //     << ", index: " << f->info()
    //     << std::endl; 
    // }
  }

  void PointLocatorPrivate::DebugPrintTriangulationHierarchy() const
  {
    const Triangulation::Triangulation_data_structure& ctds = ct.tds();

    std::cout << "triangulation hierarchy data structure" << std::endl;
    std::cout << ctds << std::endl;

    std::cout << "is valid: " << ctds.is_valid() << std::endl;
    std::cout << "dimension: " << ctds.dimension() << std::endl;
    std::cout << "number of vertices : " << ctds.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << ctds.number_of_faces() << std::endl;
    std::cout << "number of edges : " << ctds.number_of_edges() << std::endl;
    std::cout << std::endl;

    std::cout << "triangulation hierarchy" << std::endl;
    std::cout << ct << std::endl; 
    
    std::cout << "is valid: " << ct.is_valid() << std::endl;
    std::cout << "dimension: " << ct.dimension() << std::endl;
    std::cout << "number of vertices : " << ct.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << ct.number_of_faces() << std::endl;

    size_t idx = 0;
    std::cout << "vertex -> mesh vertex:" << std::endl;
    for (auto v = ct.all_vertices_begin(); v != ct.all_vertices_end(); ++v, ++idx)
    {
      std::cout << "vertex: " << idx
        << ", mesh vertex: " << v->info()
        << std::endl; 
    }

    idx = 0;
    std::cout << "face -> mesh face:" << std::endl;
    for (auto f = ct.all_faces_begin(); f != ct.all_faces_end(); ++f, ++idx)
    {
      std::cout << "face: " << idx
        << ", mesh face: " << f->info()
        << std::endl; 
    }
  }

  bool PointLocatorPrivate::IsValid(bool verbose) const
  {
    bool isValid = true;

    // 1. (Mesh) Triangulation
    const Triangulation::Triangulation_data_structure& tds = t.tds();

    // Verify infinite vertex
    auto vi = t.infinite_vertex();
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

    // Verify triangulation data structure
    isValid &= tds.is_valid(verbose);

    // Verify triangulation
    isValid &= t.is_valid();

    // 2. Triangulation Hierarchy
    const Triangulation::Triangulation_data_structure& ctds = ct.tds();

    // Verify infinite vertex
    auto cvi = ct.infinite_vertex();
    isValid &= cvi->is_valid(verbose);

    // Verify vertices
    for (auto v = ctds.vertices_begin(); v !=  ctds.vertices_end(); ++v)
    {
      isValid &= v->is_valid(verbose);
    }

    // Verify faces
    for (auto f = ctds.faces_begin(); f !=  ctds.faces_end(); ++f)
    {
      isValid &= f->is_valid(verbose);
    }

    // Verify triangulation hierarchy data structure
    isValid &= ctds.is_valid(verbose);

    // Verify triangulation hierarchy
    isValid &= ct.is_valid(verbose);

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

  void PointLocator::CreateMesh()
  {
    CGAL::Timer timer;
    timer.start();
    this->impl->CreateMesh();
    timer.stop();
    std::cout << "CreateMesh: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocator::CreateTriangulation()
  {
    CGAL::Timer timer;
    timer.start();
    this->impl->CreateTriangulation();
    timer.stop();
    std::cout << "CreateTriangulation: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocator::CreateTriangulationHierarchy()
  {
    CGAL::Timer timer;
    timer.start();
    this->impl->CreateTriangulationHierarchy();
    timer.stop();
    std::cout << "CreateTriangulationHierarchy: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocator::CreateConstrainedDelaunayTriangulationHierarchy()
  {
    CGAL::Timer timer;
    timer.start();
    this->impl->CreateConstrainedDelaunayTriangulationHierarchy();
    timer.stop();
    std::cout << "CreateConstrainedDelaunayTriangulationHierarchy: (" << timer.time() << " s)" << std::endl;
  }

  void PointLocator::DebugPrintMesh() const
  {
    this->impl->DebugPrintMesh();
  }

  void PointLocator::DebugPrintTriangulation() const
  {
    this->impl->DebugPrintTriangulation();
  }

  void PointLocator::DebugPrintTriangulationHierarchy() const
  {
    this->impl->DebugPrintTriangulationHierarchy();
  }

  bool PointLocator::IsValid(bool verbose) const
  {
    return this->impl->IsValid(verbose);
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv
