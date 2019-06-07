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

/// \file PointLocator.hh

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_POINT_LOCATOR_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_POINT_LOCATOR_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <memory>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// PointLocator
  
  class PointLocatorPrivate;
  
  class PointLocator
  {
    public: virtual ~PointLocator();
    
    public: PointLocator(size_t _N, double _L);
    
    public: bool Locate(const Point3& p, size_t& faceIndex) const;

    public: void CreateMesh();
    
    public: void CreateTriangulation();
        
    public: bool IsValid(bool verbose=false) const;

    public: void DebugPrintMesh() const;
    
    public: void DebugPrintTriangulation() const;
    
    private: std::unique_ptr<PointLocatorPrivate> impl;
  };

///////////////////////////////////////////////////////////////////////////////

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_POINT_LOCATOR_HH_
