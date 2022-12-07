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

#include "gz/waves/Convert.hh"
#include "gz/waves/CGALTypes.hh"

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
// Conversions

gz::math::Vector3d ToGz(const cgal::Point3& _point)
{
  return gz::math::Vector3d(_point.x(), _point.y(), _point.z());
}

gz::math::Vector2d ToGz(const cgal::Vector2& _vector)
{
  return gz::math::Vector2d(_vector.x(), _vector.y());
}

gz::math::Vector3d ToGz(const cgal::Vector3& _vector)
{
  return gz::math::Vector3d(_vector.x(), _vector.y(), _vector.z());
}

cgal::Point3 ToPoint3(const gz::math::Vector3d& _vector)
{
  return cgal::Point3(_vector.X(), _vector.Y(), _vector.Z());
}

cgal::Vector2 ToVector2(const gz::math::Vector2d& _vector)
{
  return cgal::Vector2(_vector.X(), _vector.Y());
}

cgal::Vector3 ToVector3(const gz::math::Vector3d& _vector)
{
  return cgal::Vector3(_vector.X(), _vector.Y(), _vector.Z());
}

///////////////////////////////////////////////////////////////////////////////

}  // namespace waves
}  // namespace gz
