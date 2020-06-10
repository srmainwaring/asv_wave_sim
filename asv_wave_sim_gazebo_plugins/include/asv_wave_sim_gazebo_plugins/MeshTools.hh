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

/// \file MeshTools.hh
/// \brief This file defines methods used to convert between CGAL
/// and Gazebo meshes.     

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_MESH_TOOLS_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_MESH_TOOLS_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <memory>

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// MeshTools

  /// \brief A collection of static methods for switching between Gazebo and CGAL meshes.
  class MeshTools
  {
    /// \brief Wrapper around gazebo::common::Mesh::FillArrays to populate vectors instead of raw arrays.
    ///
    /// \param[in] _source      The source mesh (a Gazebo Mesh).
    /// \param[out] _vertices   The vector of vertices to populate.
    /// \param[out] _indices    The vector of indices to populate.
    public: static void FillArrays(
      const gazebo::common::Mesh& _source,
      std::vector<float>& _vertices,
      std::vector<int>& _indices
    );

    /// \brief Make a SurfaceMesh from a Gazebo Mesh.
    /// 
    /// \param[in] _source      The source mesh (a Gazebo Mesh).
    /// \param[out] _target     The target mesg (a CGAL SurfaceMesh).
    public: static void MakeSurfaceMesh(
      const gazebo::common::Mesh& _source,
      Mesh& _target
    );
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_MESH_TOOLS_HH_
