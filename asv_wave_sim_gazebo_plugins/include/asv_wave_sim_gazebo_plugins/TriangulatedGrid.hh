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

/// \file TriangulatedGrid.hh

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_TRIANGULATED_GRID_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_TRIANGULATED_GRID_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <gazebo/rendering/ogre_gazebo.h>
#include <ignition/math/Pose3.hh>

#include <memory>
#include <vector>

namespace asv {

///////////////////////////////////////////////////////////////////////////////
// TriangulatedGrid
  
  
  typedef std::array<int64_t, 3>  Index3;
  typedef std::vector<Point3>     Point3Range;
  typedef std::vector<Index3>     Index3Range;

  class TriangulatedGrid {
   public:

    virtual ~TriangulatedGrid();    
    TriangulatedGrid(int num_segments, double length);
    void CreateMesh();
    void CreateTriangulation();
    static std::unique_ptr<TriangulatedGrid> Create(int num_segments, double length);

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
    void UpdatePoints(const std::vector<Point3>& from);
    void UpdatePoints(const std::vector<Ogre::Vector3>& from);
    void UpdatePoints(const Mesh& from);

   private:
    class Private;
    std::unique_ptr<Private> impl_;
  };

///////////////////////////////////////////////////////////////////////////////

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_TRIANGULATED_GRID_HH_
