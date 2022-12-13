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

#ifndef GZ_RENDERING_OGRE2_OGRE2DISPLACEMENTMAP_HH_
#define GZ_RENDERING_OGRE2_OGRE2DISPLACEMENTMAP_HH_

#include <Eigen/Dense>

#include <memory>

#include <gz/rendering/config.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Export.hh>

#include <gz/rendering/ogre2.hh>
#include <gz/rendering/ogre2/Export.hh>

#include "DisplacementMap.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

class GZ_RENDERING_OGRE2_VISIBLE Ogre2DisplacementMap :
    public DisplacementMap
{
  public: Ogre2DisplacementMap(
    ScenePtr _scene,
    MaterialPtr _material,
    uint64_t _entity,
    uint32_t _width,
    uint32_t _height);

  public: virtual ~Ogre2DisplacementMap();

  public: virtual void InitTextures() override;

  public: virtual void UpdateTextures(
    const Eigen::Ref<const Eigen::ArrayXXd> &mHeights,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDhdx,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDhdy,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDisplacementsX,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDisplacementsY,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDxdx,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDydy,
    const Eigen::Ref<const Eigen::ArrayXXd> &mDxdy) override;

  private: ScenePtr scene;
  private: MaterialPtr material;
  private: uint64_t entity{0};
  private: uint32_t width{0};
  private: uint32_t height{0};

  private:
  /// \note: new code adapted from ogre ocean materials sample
  // textures for displacement, normal and tangent maps
  Ogre::Image2       *mHeightMapImage{nullptr};
  Ogre::Image2       *mNormalMapImage{nullptr};
  Ogre::Image2       *mTangentMapImage{nullptr};
  Ogre::TextureGpu   *mHeightMapTex{nullptr};
  Ogre::TextureGpu   *mNormalMapTex{nullptr};
  Ogre::TextureGpu   *mTangentMapTex{nullptr};

  /// \note Triple buffer staging textures
  Ogre::StagingTexture* mHeightMapStagingTextures[3] =
      {nullptr, nullptr, nullptr};
  Ogre::StagingTexture* mNormalMapStagingTextures[3] =
      {nullptr, nullptr, nullptr};
  Ogre::StagingTexture* mTangentMapStagingTextures[3] =
      {nullptr, nullptr, nullptr};

  uint8_t mHeightMapFrameIdx{0};
  uint8_t mNormalMapFrameIdx{0};
  uint8_t mTangentMapFrameIdx{0};
};

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_OGRE2_OGRE2DISPLACEMENTMAP_HH_
