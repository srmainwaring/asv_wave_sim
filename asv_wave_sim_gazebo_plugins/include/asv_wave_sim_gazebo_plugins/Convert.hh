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

/// \file Convert.hh
/// \brief Utility methods for converting between CGAL and Gazebo / Ignition types.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_CONVERT_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_CONVERT_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// Conversions

  /// \brief Convert a CGAL Point3 to an ignition Vector3d
  /// \param[in] _point   The point to convert
  /// \return             The converted point 
  ignition::math::Vector3d ToIgn(const Point3& _point);

  /// \brief Convert a CGAL Vector2 to an ignition Vector2d
  /// \param[in] _vector  The vector to convert
  /// \return             The converted vector 
  ignition::math::Vector2d ToIgn(const Vector2& _vector);

  /// \brief Convert a CGAL Vector3 to an ignition Vector3d
  /// \param[in] _vector  The vector to convert
  /// \return             The converted vector 
  ignition::math::Vector3d ToIgn(const Vector3& _vector);

  /// \brief Convert an ignition Vector3d to a CGAL Point3
  /// \param[in] _vector  The vector to convert
  /// \return             The converted point
  Point3 ToPoint3(const ignition::math::Vector3d& _vector);

  /// \brief Convert an ignition Vector2d to a CGAL Vector2
  /// \param[in] _vector  The vector to convert
  /// \return             The converted vector
  Vector2 ToVector2(const ignition::math::Vector2d& _vector);

  /// \brief Convert an ignition Vector3d to a CGAL Vector3
  /// \param[in] _vector  The vector to convert
  /// \return             The converted vector
  Vector3 ToVector3(const ignition::math::Vector3d& _vector);

///////////////////////////////////////////////////////////////////////////////
} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_CONVERT_HH_
