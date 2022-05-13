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

#include "gz/marine/Convert.hh"
#include "gz/marine/CGALTypes.hh"

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

namespace ignition
{
namespace marine
{

///////////////////////////////////////////////////////////////////////////////
// Conversions
  
  math::Vector3d ToIgn(const cgal::Point3& _point)
  {
    return math::Vector3d(_point.x(), _point.y(), _point.z());
  }

  math::Vector2d ToIgn(const cgal::Vector2& _vector)
  {
    return math::Vector2d(_vector.x(), _vector.y());
  }

  math::Vector3d ToIgn(const cgal::Vector3& _vector)
  {
    return math::Vector3d(_vector.x(), _vector.y(), _vector.z());
  }

  cgal::Point3 ToPoint3(const math::Vector3d& _vector)
  {
    return cgal::Point3(_vector.X(), _vector.Y(), _vector.Z());
  }

  cgal::Vector2 ToVector2(const math::Vector2d& _vector)
  {
    return cgal::Vector2(_vector.X(), _vector.Y());
  }

  cgal::Vector3 ToVector3(const math::Vector3d& _vector)
  {
    return cgal::Vector3(_vector.X(), _vector.Y(), _vector.Z());
  }

///////////////////////////////////////////////////////////////////////////////

}
}
