// Copyright (C) 2022  Rhys Mainwaring
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

#ifndef GZ_RENDERING_DISPLACEMENTMAP_HH_
#define GZ_RENDERING_DISPLACEMENTMAP_HH_

#include <Eigen/Dense>

#include <memory>

#include <gz/rendering/config.hh>
#include <gz/rendering/Export.hh>

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

class GZ_RENDERING_VISIBLE DisplacementMap
{
  public: virtual ~DisplacementMap();

  public: virtual void InitTextures() = 0;

  public: virtual void UpdateTextures(
    const Eigen::Ref<const Eigen::ArrayXXd> &mHeights,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDhdx,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDhdy,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDisplacementsX,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDisplacementsY,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDxdx,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDydy,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDxdy) = 0;
};

typedef std::shared_ptr<DisplacementMap> DisplacementMapPtr;

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_DISPLACEMENTMAP_HH_
