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
/// \brief Utility methods for converting between CGAL and Gazebo types.

#ifndef GZ_WAVES_CONVERT_HH_
#define GZ_WAVES_CONVERT_HH_

#include "gz/waves/CGALTypes.hh"

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

namespace gz
{
namespace waves
{
/// \brief Convert a CGAL Point3 to a gazebo Vector3d
/// \param[in] point    The point to convert
/// \return             The converted point
gz::math::Vector3d ToGz(const cgal::Point3& point);

/// \brief Convert a CGAL Vector2 to a gazebo Vector2d
/// \param[in] vector   The vector to convert
/// \return             The converted vector
gz::math::Vector2d ToGz(const cgal::Vector2& vector);

/// \brief Convert a CGAL Vector3 to a gazebo Vector3d
/// \param[in] vector   The vector to convert
/// \return             The converted vector
gz::math::Vector3d ToGz(const cgal::Vector3& vector);

/// \brief Convert a gazebo Vector3d to a CGAL Point3
/// \param[in] vector   The vector to convert
/// \return             The converted point
cgal::Point3 ToPoint3(const gz::math::Vector3d& vector);

/// \brief Convert a gazebo Vector2d to a CGAL Vector2
/// \param[in] vector   The vector to convert
/// \return             The converted vector
cgal::Vector2 ToVector2(const gz::math::Vector2d& vector);

/// \brief Convert a gazebo Vector3d to a CGAL Vector3
/// \param[in] vector   The vector to convert
/// \return             The converted vector
cgal::Vector3 ToVector3(const gz::math::Vector3d& vector);

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_CONVERT_HH_
