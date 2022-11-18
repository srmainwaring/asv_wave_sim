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

#include <gz/rendering/config.hh>
#include <gz/rendering/Export.hh>

#include <Eigen/Dense>

#include <memory>

using Eigen::MatrixXf;

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
        const Eigen::Ref<const Eigen::MatrixXd> &mHeights,
        const Eigen::Ref<const Eigen::MatrixXd> &mDhdx,
        const Eigen::Ref<const Eigen::MatrixXd> &mDhdy,
        const Eigen::Ref<const Eigen::MatrixXd> &mDisplacementsX,
        const Eigen::Ref<const Eigen::MatrixXd> &mDisplacementsY,
        const Eigen::Ref<const Eigen::MatrixXd> &mDxdx,
        const Eigen::Ref<const Eigen::MatrixXd> &mDydy,
        const Eigen::Ref<const Eigen::MatrixXd> &mDxdy) = 0;
    };

    typedef std::shared_ptr<DisplacementMap> DisplacementMapPtr;

    }
  }
}
#endif
