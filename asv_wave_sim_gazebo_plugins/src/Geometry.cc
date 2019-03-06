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

#include <array>
#include <functional>
#include <iostream>
#include <cmath>
#include <limits>
#include <string>

namespace asv 
{
  Point3 Geometry::Normalize(const Point3& _p)
  {
    if (_p == CGAL::ORIGIN)
      return _p;
    else
    {
      Vector3 v = _p - CGAL::ORIGIN;
      double norm = std::sqrt(v.squared_length());
      return Point3(_p.x()/norm, _p.y()/norm, _p.z()/norm);
    }
  }

  Vector2 Geometry::Normalize(const Vector2& _v)
  {
    if (_v == CGAL::NULL_VECTOR)
      return _v;
    else
      return _v/std::sqrt(_v.squared_length()); 
  }

  Vector3 Geometry::Normalize(const Vector3& _v)
  {
    if (_v == CGAL::NULL_VECTOR)
      return _v;
    else
      return _v/std::sqrt(_v.squared_length()); 
  }

  Vector3 Geometry::Normal(
    const Point3& _p0,
    const Point3& _p1,
    const Point3& _p2
  )
  {
    auto n = CGAL::normal(_p0, _p1, _p2);
    if (n == CGAL::NULL_VECTOR)
      return n;
    else
      return n/std::sqrt(n.squared_length()); 
  }
  
} // namespace asv

