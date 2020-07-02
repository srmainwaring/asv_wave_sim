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

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_OCEAN_TILE_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_OCEAN_TILE_HH_

#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/ogre_gazebo.h>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

// #include <Ogre.h>
// #include <OgreMesh.h>
// #include <OgreMeshManager.h>
// #include <OgrePlane.h>
// #include <OgreVector.h>

#include <cmath>
#include <memory>
#include <iostream>

namespace asv
{
  class WaveSimulation;

  class OceanTile
  {
  public:

    virtual ~OceanTile();

    OceanTile(size_t _N, double _L, bool _hasVisuals=true);

    void SetWindVelocity(double _ux, double _uy);

    // See:
    //  osgOcean/OceanTile.
    //  ocean_gazebo_plugins/ShaderVisual.cc
    //
    // The texture coordinates (u,v) span the entire tile.
    // The Ogre convention for texture coordinates has:
    // (u, v) = (0, 0) at the top left 
    // (u, v) = (1, 1) at the bottom right 
    // The tangent space basis calculation is adjusted to 
    // conform with this convention.
    void Create();

    void ComputeNormals();

    void ComputeTangentSpace();

    // Compute the tangent space vectors (Tanget, Bitangent, Normal)
    //
    // Adapted from:
    // https://learnopengl.com/Advanced-Lighting/Normal-Mapping
    // Retrieved: 03 April 2019
    //
    // Resources:
    // Learn OpenGL: https://learnopengl.com/Advanced-Lighting/Normal-Mapping
    // Bumpmapping with GLSL: http://fabiensanglard.net/bumpMapping/index.php
    // Lesson 8: Tangent Space: http://jerome.jouvie.free.fr/opengl-tutorials/Lesson8.php
    //
    static void ComputeTBN(
        const Ogre::Vector3& _p0, 
        const Ogre::Vector3& _p1, 
        const Ogre::Vector3& _p2, 
        const Ogre::Vector2& _uv0, 
        const Ogre::Vector2& _uv1, 
        const Ogre::Vector2& _uv2, 
        Ogre::Vector3& _tangent, 
        Ogre::Vector3& _bitangent, 
        Ogre::Vector3& _normal);

    static void ComputeTBN(
        const std::vector<Ogre::Vector3>& _vertices,
        const std::vector<Ogre::Vector2>& _texCoords,
        const std::vector<ignition::math::Vector3i>& _faces, 
        std::vector<Ogre::Vector3>& _tangents,
        std::vector<Ogre::Vector3>& _bitangents,
        std::vector<Ogre::Vector3>& _normals);

    void Update(double _time);

    void UpdateVertices(double _time);

    Ogre::SubMesh* CreateMesh(const Ogre::String &_name, double _offsetZ=0.0, bool _reverseOrientation=false);

    void UpdateMesh(Ogre::SubMesh *_subMesh, double _offsetZ=0.0, bool _reverseOrientation=false);

    void DebugPrintVertexBuffers(Ogre::SubMesh *_subMesh) const;

    const std::vector<Ogre::Vector3>& Vertices() const;

    const std::vector<ignition::math::Vector3i>& Faces() const;

  private:
    bool    mHasVisuals;

    size_t  mResolution;                      /// \brief FFT size (N = 2^n)
    size_t  mRowLength;                       /// \brief Number of vertices per row (N+1)
    size_t  mNumVertices;                     /// \brief Total number of vertices (N+1)^2
    size_t  mNumFaces;                        /// \brief Total number of faces 2 * N^2
    double  mTileSize;                        /// \brief Tile size in world units
    double  mSpacing;                         /// \brief Space between vertices

    std::vector<Ogre::Vector3>  mVertices0;
    std::vector<Ogre::Vector3>  mVertices;
    std::vector<ignition::math::Vector3i>  mFaces;

    std::vector<Ogre::Vector2>  mTexCoords;
    std::vector<Ogre::Vector3>  mTangents;
    std::vector<Ogre::Vector3>  mBitangents;
    std::vector<Ogre::Vector3>  mNormals;

    std::string                 mAboveOceanMeshName = "AboveOceanTileMesh";
    std::string                 mBelowOceanMeshName = "BelowOceanTileMesh";
    Ogre::SubMesh*              mAboveOceanSubMesh;
    Ogre::SubMesh*              mBelowOceanSubMesh;

    std::unique_ptr<WaveSimulation> mWaveSim;
    std::vector<double>         mHeights;
    std::vector<double>         mDhdx;
    std::vector<double>         mDhdy;

    std::vector<double>         mDisplacementsX;
    std::vector<double>         mDisplacementsY;
    std::vector<double>         mDxdx;
    std::vector<double>         mDydy;
    std::vector<double>         mDxdy;
  };

}

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_OCEAN_TILE_HH_
