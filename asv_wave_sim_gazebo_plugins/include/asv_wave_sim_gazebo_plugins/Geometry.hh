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

/// \file Geometry.hh
/// \brief This file contains methods to calculate properties of simple geometrical objects.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_GEOMETRY_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_GEOMETRY_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <array>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// Geometry

  /// \brief A collection of static methods concerning linear geometry.
  class Geometry
  {
    /// \brief Calculate the point on a line from the origin passing through _p
    /// such that the vector from the origin to returned point has unit length.
    ///
    /// \param[in] _p     A point.
    /// \return           The point that normalises the vector from the origin to _p.
    public: static Point3 Normalize(const Point3& _p);

    /// \brief Normalise a Vector2 (i.e. ensure it has unit length)
    ///
    /// \param[in] _v     The vector to normalise.
    /// \return           The normalized vector.
    public: static Vector2 Normalize(const Vector2& _v);

    /// \brief Normalise a Vector3 (i.e. ensure it has unit length)
    ///
    /// \param[in] _v     The vector to normalise.
    /// \return           The normalized vector.
    public: static Vector3 Normalize(const Vector3& _v);

    /// \brief Compute the (normalised) normal to the plane defined by a triangle.
    ///
    /// \param[in] _p0    Point at the first vertex.
    /// \param[in] _p1    Point at the second vertex.
    /// \param[in] _p2    Point at the third vertex.
    /// \return           The normal vector.
    public: static Vector3 Normal(
      const Point3& _v0,
      const Point3& _v1,
      const Point3& _v2
    );
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_GEOMETRY_HH_
