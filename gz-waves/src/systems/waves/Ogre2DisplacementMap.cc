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

#include "Ogre2DisplacementMap.hh"

#include <gz/rendering/RenderingIface.hh>

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

//////////////////////////////////////////////////
Ogre2DisplacementMap::Ogre2DisplacementMap(
  ScenePtr _scene,
  MaterialPtr _material,
  uint64_t _entity,
  uint32_t _width,
  uint32_t _height) :
  scene(_scene),
  material(_material),
  entity(_entity),
  width(_width),
  height(_height)
{
}

//////////////////////////////////////////////////
Ogre2DisplacementMap::~Ogre2DisplacementMap()
{
  gzmsg << "Ogre2DisplacementMap::Destructor\n";

  // Remove staging textures
  gz::rendering::Ogre2ScenePtr ogre2Scene =
    std::dynamic_pointer_cast<gz::rendering::Ogre2Scene>(
        this->scene);

  if (ogre2Scene != nullptr)
  {
    Ogre::SceneManager *ogre2SceneManager = ogre2Scene->OgreSceneManager();

    if (ogre2SceneManager != nullptr)
    {
      Ogre::TextureGpuManager *ogre2TextureManager =
          ogre2SceneManager->getDestinationRenderSystem()->
              getTextureGpuManager();

      if (ogre2TextureManager != nullptr)
      {
        for (uint8_t i=0; i < 3; ++i)   {
          if (mHeightMapStagingTextures[i]) {
              ogre2TextureManager->removeStagingTexture(
                  mHeightMapStagingTextures[i]);
              mHeightMapStagingTextures[i] = nullptr;
          }

          if (mNormalMapStagingTextures[i]) {
              ogre2TextureManager->removeStagingTexture(
                  mNormalMapStagingTextures[i]);
              mNormalMapStagingTextures[i] = nullptr;
          }

          if (mTangentMapStagingTextures[i]) {
              ogre2TextureManager->removeStagingTexture(
                  mTangentMapStagingTextures[i]);
              mTangentMapStagingTextures[i] = nullptr;
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Ogre2DisplacementMap::InitTextures()
{
  gzmsg << "Ogre2DisplacementMap::InitTextures\n";

  if (!this->material)
  {
    gzerr << "Invalid Material\n";
    return;
  }

  gz::rendering::Ogre2ScenePtr ogre2Scene =
    std::dynamic_pointer_cast<gz::rendering::Ogre2Scene>(
        this->scene);

  gz::rendering::Ogre2MaterialPtr ogre2Material =
    std::dynamic_pointer_cast<gz::rendering::Ogre2Material>(
        this->material);

  Ogre::SceneManager *ogre2SceneManager = ogre2Scene->OgreSceneManager();

  Ogre::TextureGpuManager *ogre2TextureManager =
      ogre2SceneManager->getDestinationRenderSystem()->getTextureGpuManager();

  // Create empty image
  uint32_t depthOrSlices{1};
  Ogre::TextureTypes::TextureTypes textureType{
      Ogre::TextureTypes::TextureTypes::Type2D};
  Ogre::PixelFormatGpu format{
      Ogre::PixelFormatGpu::PFG_RGBA32_FLOAT};
  uint8_t numMipmaps{1u};

  gzmsg << "Create HeightMap image\n";
  mHeightMapImage = new Ogre::Image2();
  mHeightMapImage->createEmptyImage(this->width, this->height, depthOrSlices,
      textureType, format, numMipmaps);

  gzmsg << "Create NormalMap image\n";
  mNormalMapImage = new Ogre::Image2();
  mNormalMapImage->createEmptyImage(this->width, this->height, depthOrSlices,
      textureType, format, numMipmaps);

  gzmsg << "Create TangentMap image\n";
  mTangentMapImage = new Ogre::Image2();
  mTangentMapImage->createEmptyImage(this->width, this->height, depthOrSlices,
      textureType, format, numMipmaps);

  gzmsg << "Initialising images\n";
  uint32_t bufLen = sizeof(float) * 4 * this->width * this->height;
  memset(mHeightMapImage->getRawBuffer(), 0, bufLen);
  memset(mNormalMapImage->getRawBuffer(), 0, bufLen);
  memset(mTangentMapImage->getRawBuffer(), 0, bufLen);

  // Create displacement texture
  gzmsg << "Create HeightMap texture\n";
  mHeightMapTex = ogre2TextureManager->createOrRetrieveTexture(
      "HeightMapTex(" + std::to_string(this->entity) + ")",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D);

  mHeightMapTex->setResolution(mHeightMapImage->getWidth(),
      mHeightMapImage->getHeight());
  mHeightMapTex->setPixelFormat(mHeightMapImage->getPixelFormat());
  mHeightMapTex->setNumMipmaps(Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
      mHeightMapTex->getWidth(), mHeightMapTex->getHeight()));

  // Create normal texture
  gzmsg << "Create NormalMap texture\n";
  mNormalMapTex = ogre2TextureManager->createOrRetrieveTexture(
      "NormalMapTex(" + std::to_string(this->entity) + ")",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D);

  mNormalMapTex->setResolution(mNormalMapImage->getWidth(),
      mNormalMapImage->getHeight());
  mNormalMapTex->setPixelFormat(mNormalMapImage->getPixelFormat());
  mNormalMapTex->setNumMipmaps(Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
      mNormalMapTex->getWidth(), mNormalMapTex->getHeight()));

  // Create tangent texture
  gzmsg << "Create TangentMap texture\n";
  mTangentMapTex = ogre2TextureManager->createOrRetrieveTexture(
      "TangentMapTex(" + std::to_string(this->entity) + ")",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D);

  mTangentMapTex->setResolution(mTangentMapImage->getWidth(),
      mTangentMapImage->getHeight());
  mTangentMapTex->setPixelFormat(mTangentMapImage->getPixelFormat());
  mTangentMapTex->setNumMipmaps(Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
      mTangentMapTex->getWidth(), mTangentMapTex->getHeight()));

  // Set texture on wave material
  gzmsg << "Assign dynamic textures to material\n";
  auto mat = ogre2Material->Material();
  auto pass = mat->getTechnique(0u)->getPass(0);
  auto ogreParams = pass->getVertexProgramParameters();

  /// \todo understand why using the render engine instance
  /// directly gives an incorrect result?
  // {
  //   auto engine = rendering::Ogre2RenderEngine::Instance();
  //   auto graphicsAPI = engine->GraphicsAPI();
  //   gzdbg << "Using graphicsAPI: "
  //       << GraphicsAPIUtils::Str(graphicsAPI) << "\n";
  // }

  auto engine = rendering::engine("ogre2");
  auto graphicsAPI = engine->GraphicsAPI();
  gzdbg << "Using graphicsAPI: "
      << GraphicsAPIUtils::Str(graphicsAPI) << "\n";

  {
    auto texUnit = pass->getTextureUnitState("heightMap");
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("heightMap");
    }
    texUnit->setTexture(mHeightMapTex);
    texUnit->setTextureCoordSet(0);
    int texIndex = static_cast<int>(pass->getTextureUnitStateIndex(texUnit));

    gzmsg << "texNameStr:   " << mHeightMapTex->getNameStr() << "\n";
    gzmsg << "texCoordSet:  " << 0 << "\n";
    gzmsg << "texIndex:     " << texIndex << "\n";

    // set to wrap mode otherwise default is clamp mode
    Ogre::HlmsSamplerblock samplerBlockRef;
    samplerBlockRef.mU = Ogre::TAM_WRAP;
    samplerBlockRef.mV = Ogre::TAM_WRAP;
    samplerBlockRef.mW = Ogre::TAM_WRAP;
    texUnit->setSamplerblock(samplerBlockRef);

    if (graphicsAPI == rendering::GraphicsAPI::OPENGL)
    {
      // set the texture map index
      ogreParams->setNamedConstant("heightMap", &texIndex, 1, 1);
    }
  }

  {
    auto texUnit = pass->getTextureUnitState("normalMap");
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("normalMap");
    }
    texUnit->setTexture(mNormalMapTex);
    texUnit->setTextureCoordSet(0);
    int texIndex = static_cast<int>(pass->getTextureUnitStateIndex(texUnit));

    gzmsg << "texNameStr:   " << mNormalMapTex->getNameStr() << "\n";
    gzmsg << "texCoordSet:  " << 0 << "\n";
    gzmsg << "texIndex:     " << texIndex << "\n";

    // set to wrap mode otherwise default is clamp mode
    Ogre::HlmsSamplerblock samplerBlockRef;
    samplerBlockRef.mU = Ogre::TAM_WRAP;
    samplerBlockRef.mV = Ogre::TAM_WRAP;
    samplerBlockRef.mW = Ogre::TAM_WRAP;
    texUnit->setSamplerblock(samplerBlockRef);

    if (graphicsAPI == rendering::GraphicsAPI::OPENGL)
    {
      // set the texture map index
      ogreParams->setNamedConstant("normalMap", &texIndex, 1, 1);
    }
  }

  {
    auto texUnit = pass->getTextureUnitState("tangentMap");
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("tangentMap");
    }
    texUnit->setTexture(mTangentMapTex);
    texUnit->setTextureCoordSet(0);
    int texIndex = static_cast<int>(pass->getTextureUnitStateIndex(texUnit));

    gzmsg << "texNameStr:   " << mTangentMapTex->getNameStr() << "\n";
    gzmsg << "texCoordSet:  " << 0 << "\n";
    gzmsg << "texIndex:     " << texIndex << "\n";

    // set to wrap mode otherwise default is clamp mode
    Ogre::HlmsSamplerblock samplerBlockRef;
    samplerBlockRef.mU = Ogre::TAM_WRAP;
    samplerBlockRef.mV = Ogre::TAM_WRAP;
    samplerBlockRef.mW = Ogre::TAM_WRAP;
    texUnit->setSamplerblock(samplerBlockRef);

    if (graphicsAPI == rendering::GraphicsAPI::OPENGL)
    {
      // set the texture map index
      ogreParams->setNamedConstant("tangentMap", &texIndex, 1, 1);
    }
  }
}

//////////////////////////////////////////////////
void Ogre2DisplacementMap::UpdateTextures(
  const Eigen::Ref<const Eigen::ArrayXXd> &mHeights,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDhdx,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDhdy,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDisplacementsX,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDisplacementsY,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDxdx,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDydy,
  const Eigen::Ref<const Eigen::ArrayXXd> &mDxdy
)
{
  gz::rendering::Ogre2ScenePtr ogre2Scene =
    std::dynamic_pointer_cast<gz::rendering::Ogre2Scene>(
        this->scene);

  Ogre::SceneManager *ogre2SceneManager = ogre2Scene->OgreSceneManager();

  Ogre::TextureGpuManager *ogre2TextureManager =
      ogre2SceneManager->getDestinationRenderSystem()->getTextureGpuManager();

  // update the image data
  uint32_t mapWidth  = mHeightMapImage->getWidth();
  uint32_t mapHeight = mHeightMapImage->getHeight();

  Ogre::TextureBox heightBox  = mHeightMapImage->getData(0);
  Ogre::TextureBox normalBox  = mNormalMapImage->getData(0);
  Ogre::TextureBox tangentBox = mTangentMapImage->getData(0);

  for (uint32_t iv=0; iv < mapHeight; ++iv)
  {
      /// \todo: coordinates are flipped in the vertex shader
      // texture index to vertex index
      int32_t iy = /*mapHeight - 1 - */ iv;
      for (uint32_t iu=0; iu < mapWidth; ++iu)
      {
          // texture index to vertex index
          int32_t ix = /* mapWidth - 1 - */ iu;

          float Dx{0.0}, Dy{0.0}, Dz{0.0};
          float Tx{1.0}, Ty{0.0}, Tz{0.0};
          float Bx{0.0}, By{1.0}, Bz{0.0};
          float Nx{0.0}, Ny{0.0}, Nz{1.0};

          int32_t idx = iy * mapWidth + ix;
          double h  = mHeights(idx, 0);
          double sx = mDisplacementsX(idx, 0);
          double sy = mDisplacementsY(idx, 0);
          double dhdx  = mDhdx(idx, 0);
          double dhdy  = mDhdy(idx, 0);
          double dsxdx = mDxdx(idx, 0);
          double dsydy = mDydy(idx, 0);
          double dsxdy = mDxdy(idx, 0);

          // vertex displacements
          Dx += sy;
          Dy += sx;
          Dz  = h;

          // tangents
          Tx = dsydy + 1.0;
          Ty = dsxdy;
          Tz = dhdy;

          // bitangents
          Bx = dsxdy;
          By = dsxdx + 1.0;
          Bz = dhdx;

          // normals N = T x B
          Nx = 1.0 * (Ty*Bz - Tz*Bx);
          Ny = 1.0 * (Tz*Bx - Tx*Bz);
          Nz = 1.0 * (Tx*By - Ty*Bx);

          heightBox.setColourAt(Ogre::ColourValue(Dx, Dy, Dz, 0.0), iu, iv, 0,
              mHeightMapImage->getPixelFormat());
          normalBox.setColourAt(Ogre::ColourValue(Nx, Ny, Nz, 0.0), iu, iv, 0,
              mNormalMapImage->getPixelFormat());
          tangentBox.setColourAt(Ogre::ColourValue(Tx, Ty, Tz, 0.0), iu, iv, 0,
              mTangentMapImage->getPixelFormat());
      }
  }

  // schedule update to GPU
  mHeightMapTex->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);
  mNormalMapTex->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);
  mTangentMapTex->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  // Staging texture is required for upload from CPU -> GPU
  {
    if (!mHeightMapStagingTextures[mHeightMapFrameIdx]) {
        mHeightMapStagingTextures[mHeightMapFrameIdx] =
            ogre2TextureManager->getStagingTexture(
                mHeightMapImage->getWidth(),
                mHeightMapImage->getHeight(),
                1u, 1u,
                mHeightMapImage->getPixelFormat(),
                100u);
    }

    Ogre::StagingTexture *stagingTexture =
      mHeightMapStagingTextures[mHeightMapFrameIdx];
    mHeightMapFrameIdx = (mHeightMapFrameIdx + 1) % 3;

    stagingTexture->startMapRegion();
    Ogre::TextureBox texBox = stagingTexture->mapRegion(
        mHeightMapImage->getWidth(), mHeightMapImage->getHeight(), 1u, 1u,
        mHeightMapImage->getPixelFormat());

    texBox.copyFrom(mHeightMapImage->getData(0));
    stagingTexture->stopMapRegion();
    stagingTexture->upload(texBox, mHeightMapTex, 0, 0, 0);
    if (!mHeightMapTex->isDataReady()) {
        mHeightMapTex->notifyDataIsReady();
    }
  }

  {
    if (!mNormalMapStagingTextures[mNormalMapFrameIdx]) {
        mNormalMapStagingTextures[mNormalMapFrameIdx] =
            ogre2TextureManager->getStagingTexture(
                mNormalMapImage->getWidth(),
                mNormalMapImage->getHeight(),
                1u, 1u,
                mNormalMapImage->getPixelFormat(),
                100u);
    }

    Ogre::StagingTexture *stagingTexture =
      mNormalMapStagingTextures[mNormalMapFrameIdx];
    mNormalMapFrameIdx = (mNormalMapFrameIdx + 1) % 3;

    stagingTexture->startMapRegion();
    Ogre::TextureBox texBox = stagingTexture->mapRegion(
        mNormalMapImage->getWidth(), mNormalMapImage->getHeight(), 1u, 1u,
        mNormalMapImage->getPixelFormat());

    texBox.copyFrom(mNormalMapImage->getData(0));
    stagingTexture->stopMapRegion();
    stagingTexture->upload(texBox, mNormalMapTex, 0, 0, 0);
    if (!mNormalMapTex->isDataReady()) {
        mNormalMapTex->notifyDataIsReady();
    }
  }

  {
    if (!mTangentMapStagingTextures[mTangentMapFrameIdx]) {
        mTangentMapStagingTextures[mTangentMapFrameIdx] =
            ogre2TextureManager->getStagingTexture(
                mTangentMapImage->getWidth(),
                mTangentMapImage->getHeight(),
                1u, 1u,
                mTangentMapImage->getPixelFormat(),
                100u);
    }

    Ogre::StagingTexture *stagingTexture =
      mTangentMapStagingTextures[mTangentMapFrameIdx];
    mTangentMapFrameIdx = (mTangentMapFrameIdx + 1) % 3;

    stagingTexture->startMapRegion();
    Ogre::TextureBox texBox = stagingTexture->mapRegion(
        mTangentMapImage->getWidth(), mTangentMapImage->getHeight(), 1u, 1u,
        mTangentMapImage->getPixelFormat());

    texBox.copyFrom(mTangentMapImage->getData(0));
    stagingTexture->stopMapRegion();
    stagingTexture->upload(texBox, mTangentMapTex, 0, 0, 0);
    if (!mTangentMapTex->isDataReady()) {
      mTangentMapTex->notifyDataIsReady();
    }
  }
}

}
}  // namespace rendering
}  // namespace gz
