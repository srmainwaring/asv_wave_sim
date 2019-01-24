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

#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// Conversions
  
  ignition::math::Vector3d ToIgn(const Point3& _point)
  {
    return ignition::math::Vector3d(_point.x(), _point.y(), _point.z());
  }

  ignition::math::Vector2d ToIgn(const Vector2& _vector)
  {
    return ignition::math::Vector2d(_vector.x(), _vector.y());
  }

  ignition::math::Vector3d ToIgn(const Vector3& _vector)
  {
    return ignition::math::Vector3d(_vector.x(), _vector.y(), _vector.z());
  }

  Point3 ToPoint3(const ignition::math::Vector3d& _vector)
  {
    return Point3(_vector.X(), _vector.Y(), _vector.Z());
  }

  Vector2 ToVector2(const ignition::math::Vector2d& _vector)
  {
    return Vector2(_vector.X(), _vector.Y());
  }

  Vector3 ToVector3(const ignition::math::Vector3d& _vector)
  {
    return Vector3(_vector.X(), _vector.Y(), _vector.Z());
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv