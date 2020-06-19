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

#include "asv_wave_sim_gazebo_plugins/TriangulatedGrid.hh"
#include "asv_wave_sim_gazebo_plugins/Geometry.hh"

#include <gazebo/gazebo.hh>

#include <CGAL/Simple_cartesian.h>
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

namespace asv {

///////////////////////////////////////////////////////////////////////////////
// TriangulatedGrid::Private

  class TriangulatedGrid::Private {
   public:
    ~Private();
    Private(int num_segments, double length);
    void CreateMesh();
    void CreateTriangulation();    

    bool Locate(const Point3& query, int64_t& faceIndex) const;
    bool Height(const Point3& query, double& height) const;
    bool Height(const std::vector<Point3>& queries, std::vector<double>& heights) const;
    bool Interpolate(TriangulatedGrid& patch) const;

    const Point3Range& Points() const;
    const Index3Range& Indices() const;
    const Point3& Origin() const;
    void ApplyPose(const ignition::math::Pose3d& pose);

    bool IsValid(bool verbose=false) const;
    void DebugPrintMesh() const;
    void DebugPrintTriangulation() const;
    void UpdatePoints(const std::vector<Point3>& points);
    void UpdatePoints(const std::vector<Ogre::Vector3>& from);
    void UpdatePoints(const Mesh& from);

    // Type definitions - use a consistent Kernel
    typedef asv::Kernel                                                 Kernel;
    // typedef CGAL::Simple_cartesian<double>                              Kernel;
    // typedef CGAL::Exact_predicates_inexact_constructions_kernel         Kernel;
    typedef CGAL::Projection_traits_xy_3<Kernel>                        Gt;
    typedef CGAL::Triangulation_vertex_base_with_info_2<int64_t, Gt>    Vbb;
    typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>            Vb;
    typedef CGAL::Constrained_triangulation_face_base_2<Gt>             Fbb;
    typedef CGAL::Triangulation_face_base_with_info_2<int64_t, Gt, Fbb> Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                Tds;
    typedef CGAL::No_intersection_tag                                   Itag;
    typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, Tds, Itag>   Tb;
    typedef CGAL::Triangulation_hierarchy_2<Tb>                         Triangulation;
    typedef Triangulation::Vertex_handle                                Vertex_handle;
    typedef Triangulation::Face_handle                                  Face_handle;
    typedef Triangulation::Face                                         Face;

    // Dimensions
    int num_segments_;
    double length_;

    // Mesh
    Point3              origin_;
    std::vector<Point3> points0_;
    std::vector<Point3> points_;
    std::vector<Index3> indices_;
    std::vector<Index3> infinite_indices_;

    // Triangulation / Triangulation Hierarchy
    Triangulation       tri_;
  };

  TriangulatedGrid::Private::~Private() {
  }

  TriangulatedGrid::Private::Private(int num_segments, double length) : 
    num_segments_(num_segments), length_(length), origin_(CGAL::ORIGIN) {  
  }

  void TriangulatedGrid::Private::CreateMesh() {
    double dl = length_ / num_segments_;
    double lm = - length_ / 2.0;
    const int nplus1 = num_segments_ + 1; 

    // Points - (num_segments_+1) points_ in each row / column 
    for (int64_t iy=0; iy<=num_segments_; ++iy) {
      double py = iy * dl + lm;
      for (int64_t ix=0; ix<=num_segments_; ++ix) {
        // Vertex position
        double px = ix * dl + lm;
        Point3 point(px, py, 0.0);
        points_.push_back(point);
      }
    }
    // Copy initial points
    points0_ = points_;

    // Face indices
    for (int64_t iy=0; iy<num_segments_; ++iy) {
      for (int64_t ix=0; ix<num_segments_; ++ix) {
        // Get the points in the cell coordinates
        int64_t idx0 = iy * nplus1 + ix;
        int64_t idx1 = iy * nplus1 + ix + 1;
        int64_t idx2 = (iy+1) * nplus1 + ix + 1;
        int64_t idx3 = (iy+1) * nplus1 + ix;

        // Face indices
        indices_.push_back({ idx0, idx1, idx2 });
        indices_.push_back({ idx0, idx2, idx3 });
      }
    }

    // Infinite indices (follow edges counter clockwise around grid)
    infinite_indices_.resize(4*num_segments_);
    for (int64_t i=0; i<num_segments_; ++i) {
      // bottom
      int64_t idx = i;
      infinite_indices_[i] = { idx, idx+1 };

      // right
      idx = i * nplus1 + num_segments_;
      infinite_indices_[num_segments_+i] = { idx, idx+nplus1 };

      // top
      idx = num_segments_ * nplus1 + i;
      infinite_indices_[3*num_segments_-1-i] = { idx+1, idx };

      // left
      idx = i * nplus1;
      infinite_indices_[4*num_segments_-1-i] = { idx+nplus1, idx };
    }
  }

  void TriangulatedGrid::Private::CreateTriangulation() {
    // Insert points - NOTE: must insert the points to build the triangulation hierarchy.
    // CGAL::Timer timer;

    // Point with info
    // std::vector<std::pair<Point3, int64_t>> pointsWithInfo;
    // for (int64_t i=0; i<points_.size(); ++i) {
    //   pointsWithInfo.push_back(std::make_pair(points_[i], i));
    // }

    // timer.start();
    // @NOTE insert poinst with info not compiling for a triangulation hierarcy. See below. 
    // tri_.insert(pointsWithInfo.begin(), pointsWithInfo.end());
    tri_.insert(points_.begin(), points_.end());
    // timer.stop();
    // std::cout << "insert points: " << timer.time() << " s" << std::endl;

    // Set vertex info
    // @NOTE - use this work around because the insert for points with info
    // does not compile for a triangulation hierarchy.
    // timer.reset();
    // timer.start();
    Face_handle fh;
    for (int64_t i=0; i<points_.size(); ++i) {
      auto vh = tri_.insert(points_[i] ,fh);
      if (vh != nullptr) {
        vh->info() = i;
      }
    }
    // timer.stop();
    // std::cout << "set vertex info: " << timer.time() << " s" << std::endl;

    // Constraint indices 
    const int nplus1 = num_segments_ + 1; 
    std::vector<std::pair<size_t, int64_t>> cindices;
    for (int64_t iy=0; iy<num_segments_; ++iy) {
      for (int64_t ix=0; ix<num_segments_; ++ix) {
        int64_t idx1 = iy * nplus1 + ix;
        int64_t idx2 = (iy + 1) * nplus1 + (ix + 1);
        cindices.push_back(std::make_pair(idx1, idx2));
      }
    }

    // Insert constraints
    // timer.reset();
    // timer.start();
    tri_.insert_constraints(points_.begin(), points_.end(), cindices.begin(), cindices.end());   
    // timer.stop();
    // std::cout << "insert constraints: " << timer.time() << " s" << std::endl;

    // Set info on infinite vertex
    tri_.infinite_vertex()->info() = -1;

    // Face list mapping  

    // Initialise face info
    // timer.reset();
    // timer.start();
    for (auto f = tri_.all_faces_begin(); f != tri_.all_faces_end(); ++f) {
      f->info() = -1; 
    }
    // timer.stop();
    // std::cout << "initialise face info: (" << timer.time() << " s)" << std::endl;

    // Face matching
    fh = nullptr;
    int64_t idx = 0;
    // timer.reset();
    // timer.start();
    for (auto f = indices_.begin(); f != indices_.end(); ++f, ++idx) {
      // Compute the centroid of f and locate the tri_ face containing this point.
      auto& p0 = points_[f->at(0)];
      auto& p1 = points_[f->at(1)];
      auto& p2 = points_[f->at(2)];
      Point3 p(
        (p0.x() + p1.x() + p2.x())/3.0,
        (p0.y() + p1.y() + p2.y())/3.0,
        0.0
      );
      fh = tri_.locate(p, fh);

      // Set the info value on the found face.
      if (fh != nullptr)
      {
        fh->info() = idx;
      }
    }
    // timer.stop();
    // std::cout << "face mapping: (" << timer.time() << " s)" << std::endl;    
  }

  bool TriangulatedGrid::Private::Locate(const Point3& query, int64_t& faceIndex) const {
    auto fh = tri_.locate(query);
    if (fh != nullptr) {
      faceIndex = fh->info();
      return true;
    }
    return false;
  }

  bool TriangulatedGrid::Private::Height(const Point3& query, double& height) const {
    bool found = false;
    height = 0.0;
    auto fh = tri_.locate(query);
    if (fh != nullptr) {
      // Triangle vertices
      const auto& p0 = fh->vertex(0)->point();
      const auto& p1 = fh->vertex(1)->point();
      const auto& p2 = fh->vertex(2)->point();

      // Height query
      const Direction3 direction(0, 0, 1);      
      Point3 intersection(query);
      Triangle triangle(p0, p1, p2);

      found = Geometry::LineIntersectsTriangle(
          query, direction, triangle, intersection);

      if (found) {
        height = intersection.z() - query.z();
      }
    }
    return found;
  }

  bool TriangulatedGrid::Private::Height(const std::vector<Point3>& queries, std::vector<double>& heights) const {
    bool foundAll = true;
    Face_handle fh = nullptr;
    for (int64_t i=0; i<heights.size(); ++i)
    {
      double height_i = 0.0;
      const Point3& query = queries[i];
      fh = tri_.locate(query, fh);
      bool found = fh != nullptr;
      if (found) {
        // Triangle vertices
        const auto& p0 = fh->vertex(0)->point();
        const auto& p1 = fh->vertex(1)->point();
        const auto& p2 = fh->vertex(2)->point();

        // Height query
        const Direction3 direction(0, 0, 1);      
        Point3 intersection(query);
        Triangle triangle(p0, p1, p2);

        found = Geometry::LineIntersectsTriangle(
            query, direction, triangle, intersection);

        if (found) {
          height_i = intersection.z() - query.z();
        }
      }
      heights[i] = height_i;
      foundAll &= found;    
    }
    return foundAll;
  }

  bool TriangulatedGrid::Private::Interpolate(TriangulatedGrid& patch) const {
    bool foundAll = true;

    Face_handle fh = nullptr;
    for (auto it = patch.impl_->points_.begin(); it != patch.impl_->points_.end(); ++it) {
      double height = 0.0;        
      const Point3& query = *it;
      fh = tri_.locate(query, fh);
      bool found = fh != nullptr;
      if (found) {
        // Triangle vertices
        const auto& p0 = fh->vertex(0)->point();
        const auto& p1 = fh->vertex(1)->point();
        const auto& p2 = fh->vertex(2)->point();

        // Height query
        const Direction3 direction(0, 0, 1);      
        Point3 intersection(query);
        Triangle triangle(p0, p1, p2);

        found = Geometry::LineIntersectsTriangle(
            query, direction, triangle, intersection);

        // @DEBUG_INFO
        // std::cout << "query:        " << query << std::endl;
        // std::cout << "triangle:     " << triangle << std::endl;
        // std::cout << "intersection: " << intersection << std::endl;

        if (found) {
          height = intersection.z();
        }
      }
      // @NOTE this assumes the patch initially has height = 0.0;
      *it = Point3(query.x(), query.y(), height);

      foundAll &= found;
    }
    return foundAll;
  }

  const Point3Range& TriangulatedGrid::Private::Points() const {
    return points_;
  }

  const Index3Range& TriangulatedGrid::Private::Indices() const {
    return indices_;
  }

  const Point3& TriangulatedGrid::Private::Origin() const {
    return origin_;
  }

  void TriangulatedGrid::Private::ApplyPose(const ignition::math::Pose3d& pose) {
    // Origin - slide the patch in the xy - plane only
    Point3 o = CGAL::ORIGIN;
    origin_ = Point3(o.x() + pose.Pos().X(), o.y() + pose.Pos().Y(), o.z());

    // Mesh points
    for (
      auto&& it = std::make_pair(std::begin(points0_), std::begin(points_));
      it.first != std::end(points0_) && it.second != std::end(points_);
      ++it.first, ++it.second)
    {
      const auto& p0 = *it.first;
      auto& p = *it.second;
      p = Point3(p0.x() + pose.Pos().X(), p0.y() + pose.Pos().Y(), p0.z());
    }

    // Triangulation points
    for (auto v = tri_.finite_vertices_begin(); v != tri_.finite_vertices_end(); ++v) {
      int64_t idx = v->info();      
      v->set_point(points_[idx]);
    }
  }

  bool TriangulatedGrid::Private::IsValid(bool verbose) const {
    bool isValid = true;

    // Triangulation Hierarchy
    const Triangulation::Triangulation_data_structure& tds = tri_.tds();

    // Verify infinite vertex
    auto vi = tri_.infinite_vertex();
    isValid &= vi->is_valid(verbose);

    // Verify vertices
    for (auto v = tds.vertices_begin(); v !=  tds.vertices_end(); ++v) {
      isValid &= v->is_valid(verbose);
    }

    // Verify faces
    for (auto f = tds.faces_begin(); f !=  tds.faces_end(); ++f) {
      isValid &= f->is_valid(verbose);
    }

    // Verify triangulation hierarchy data structure
    isValid &= tds.is_valid(verbose);

    // Verify triangulation hierarchy
    isValid &= tri_.is_valid(verbose);

    return isValid;
  }

  void TriangulatedGrid::Private::DebugPrintMesh() const {
    std::cout << "num_segments: " << num_segments_ << std::endl;
    std::cout << "length: " << length_ << std::endl;

    std::cout << "points: [" << points_.size() << "]" << std::endl;
    for (auto&& p : points_) {
      std::cout << p << std::endl;
    }
    std::cout << "indices: [" << indices_.size() << "]" << std::endl;
    for (auto&& i : indices_) {
      std::cout << i[0] << " " << i[1] << " " << i[2]  << std::endl;
    }
    std::cout << "infinite indices: [" << infinite_indices_.size() << "]" << std::endl;
    for (auto&& i : infinite_indices_) {
      std::cout << i[0] << " " << i[1] << std::endl;
    }
  }

  void TriangulatedGrid::Private::DebugPrintTriangulation() const {
    const Triangulation::Triangulation_data_structure& ctds = tri_.tds();

    std::cout << "triangulation hierarchy data structure" << std::endl;
    std::cout << ctds << std::endl;

    std::cout << "is valid: " << ctds.is_valid() << std::endl;
    std::cout << "dimension: " << ctds.dimension() << std::endl;
    std::cout << "number of vertices : " << ctds.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << ctds.number_of_faces() << std::endl;
    std::cout << "number of edges : " << ctds.number_of_edges() << std::endl;
    std::cout << std::endl;

    std::cout << "triangulation hierarchy" << std::endl;
    std::cout << tri_ << std::endl; 
    
    std::cout << "is valid: " << tri_.is_valid() << std::endl;
    std::cout << "dimension: " << tri_.dimension() << std::endl;
    std::cout << "number of vertices : " << tri_.number_of_vertices() << std::endl;
    std::cout << "number of faces : " << tri_.number_of_faces() << std::endl;

    std::cout << "hierarchy: " << tri_.is_valid() << std::endl;
    tri_.is_valid(true);

    int64_t idx = 0;
    std::cout << "vertex -> mesh vertex:" << std::endl;
    for (auto v = tri_.all_vertices_begin(); v != tri_.all_vertices_end(); ++v, ++idx) {
      std::cout << "vertex: " << idx
        << ", mesh vertex: " << v->info()
        << std::endl; 
    }

    idx = 0;
    std::cout << "face -> mesh face:" << std::endl;
    for (auto f = tri_.all_faces_begin(); f != tri_.all_faces_end(); ++f, ++idx) {
      std::cout << "face: " << idx
        << ", mesh face: " << f->info()
        << std::endl; 
    }
  }

  void TriangulatedGrid::Private::UpdatePoints(const std::vector<Point3>& from) {
    // Mesh points
    points_ = from;

    // Triangulation points
    for (auto v = tri_.finite_vertices_begin(); v != tri_.finite_vertices_end(); ++v) {
      int64_t idx = v->info();      
      v->set_point(points_[idx]);
    }
  }

  void TriangulatedGrid::Private::UpdatePoints(const std::vector<Ogre::Vector3>& from) {
    // Mesh points
    auto it_to = points_.begin();
    auto it_from = from.begin();
    for ( ; it_to != points_.end() && it_from != from.end(); ++it_to, ++it_from) {
      *it_to = Point3(it_from->x, it_from->y, it_from->z);
    }

    // Triangulation points
    for (auto v = tri_.finite_vertices_begin(); v != tri_.finite_vertices_end(); ++v) {
      int64_t idx = v->info();      
      v->set_point(points_[idx]);
    }
  }

  void TriangulatedGrid::Private::UpdatePoints(const Mesh& from) {
    // Mesh points
    auto it_to = points_.begin();
    auto it_from = std::begin(from.vertices());
    for ( ; it_to != points_.end() && it_from != std::end(from.vertices()); ++it_to, ++it_from) {
      const Point3& p = from.point(*it_from);
      *it_to = Point3(p.x(), p.y(), p.z());
    }

    // Triangulation points
    for (auto v = tri_.finite_vertices_begin(); v != tri_.finite_vertices_end(); ++v) {
      int64_t idx = v->info();      
      v->set_point(points_[idx]);
    }
  }

///////////////////////////////////////////////////////////////////////////////
// TriangulatedGrid

  TriangulatedGrid::~TriangulatedGrid() {  
  }

  TriangulatedGrid::TriangulatedGrid(int num_segments, double length) :
    impl_(new TriangulatedGrid::Private(num_segments, length))
  {  
  }

  void TriangulatedGrid::CreateMesh() {
    impl_->CreateMesh();
  }

  void TriangulatedGrid::CreateTriangulation() {
    impl_->CreateTriangulation();
  }

  std::unique_ptr<TriangulatedGrid> TriangulatedGrid::Create(int num_segments, double length) {
    std::unique_ptr<TriangulatedGrid> instance = std::make_unique<TriangulatedGrid>(num_segments, length);
    instance->CreateMesh();
    instance->CreateTriangulation();
    return instance;
  }

  bool TriangulatedGrid::Locate(const Point3& query, int64_t& faceIndex) const {
    return impl_->Locate(query, faceIndex);
  }

  bool TriangulatedGrid::Height(const Point3& query, double& height) const {
    return impl_->Height(query, height);
  }

  bool TriangulatedGrid::Height(const std::vector<Point3>& queries, std::vector<double>& heights) const {
    return impl_->Height(queries, heights);
  }

  bool TriangulatedGrid::Interpolate(TriangulatedGrid& patch) const {
    return impl_->Interpolate(patch);
  }

  const Point3Range& TriangulatedGrid::Points() const {
    return impl_->Points();
  }

  const Index3Range& TriangulatedGrid::Indices() const {
    return impl_->Indices();
  }

  const Point3& TriangulatedGrid::Origin() const {
    return impl_->Origin();    
  }

  void TriangulatedGrid::ApplyPose(const ignition::math::Pose3d& pose) {
    impl_->ApplyPose(pose);    
  }

  bool TriangulatedGrid::IsValid(bool verbose) const {
    return impl_->IsValid(verbose);
  }

  void TriangulatedGrid::DebugPrintMesh() const {
    impl_->DebugPrintMesh();
  }

  void TriangulatedGrid::DebugPrintTriangulation() const {
    impl_->DebugPrintTriangulation();
  }

  void TriangulatedGrid::UpdatePoints(const std::vector<Point3>& from) {
    impl_->UpdatePoints(from);
  }

  void TriangulatedGrid::UpdatePoints(const std::vector<Ogre::Vector3>& from) {
    impl_->UpdatePoints(from);
  }

  void TriangulatedGrid::UpdatePoints(const Mesh& from) {
    impl_->UpdatePoints(from);
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv
