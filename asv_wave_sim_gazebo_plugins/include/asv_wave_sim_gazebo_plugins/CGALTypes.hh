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

#include <CGAL/Simple_cartesian.h>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// CGAL Typedefs

  // 2D/3D Linear Geometry
  typedef CGAL::Simple_cartesian<double>  Kernel;
  typedef Kernel::Direction_2             Direction2;
  typedef Kernel::Direction_3             Direction3;
  typedef Kernel::Point_3                 Point3;
  typedef Kernel::Vector_2                Vector2;
  typedef Kernel::Vector_3                Vector3;

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_CGAL_TYPES_HH_
