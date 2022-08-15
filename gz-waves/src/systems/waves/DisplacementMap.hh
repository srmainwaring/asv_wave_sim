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
#include "gz/rendering/Export.hh"

#include <memory>
#include <vector>

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
        const std::vector<double> &mHeights,
        const std::vector<double> &mDhdx,
        const std::vector<double> &mDhdy,
        const std::vector<double> &mDisplacementsX,
        const std::vector<double> &mDisplacementsY,
        const std::vector<double> &mDxdx,
        const std::vector<double> &mDydy,
        const std::vector<double> &mDxdy) = 0;
    };

    typedef std::shared_ptr<DisplacementMap> DisplacementMapPtr;

    }
  }
}
#endif
