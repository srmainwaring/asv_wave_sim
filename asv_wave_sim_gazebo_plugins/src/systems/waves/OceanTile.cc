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

#include "OceanTile.hh"

#include "../../../include/asv_wave_sim_gazebo_plugins/WaveSimulation.hh"
#include "../../../include/asv_wave_sim_gazebo_plugins/WaveSimulationFFTW.hh"
#include "../../../include/asv_wave_sim_gazebo_plugins/WaveSimulationSinusoidal.hh"
#include "../../../include/asv_wave_sim_gazebo_plugins/WaveSimulationTrochoid.hh"
#include "../../../include/asv_wave_sim_gazebo_plugins/WaveParameters.hh"

#include <ignition/common.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/rendering.hh>

#include <cmath>
#include <iostream>

using namespace asv;

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

// from OceanTile.hh
class OceanTilePrivate
{
public:
  ~OceanTilePrivate();

  OceanTilePrivate(size_t _N, double _L, bool _hasVisuals=true);
  
  void SetWindVelocity(double _ux, double _uy);

  bool                        mHasVisuals;
  size_t                      mResolution;      /// \brief FFT size (N = 2^n)
  size_t                      mRowLength;       /// \brief Number of vertices per row (N+1)
  size_t                      mNumVertices;     /// \brief Total number of vertices (N+1)^2
  size_t                      mNumFaces;        /// \brief Total number of faces 2 * N^2
  double                      mTileSize;        /// \brief Tile size in world units
  double                      mSpacing;         /// \brief Space between vertices

  std::vector<math::Vector3d> mVertices0;
  std::vector<math::Vector3d> mVertices;
  std::vector<math::Vector3i> mFaces;

  std::vector<math::Vector3d> mTangents;
  std::vector<math::Vector2d> mTexCoords;
  std::vector<math::Vector3d> mBitangents;
  std::vector<math::Vector3d> mNormals;

  std::string                 mAboveOceanMeshName = "AboveOceanTileMesh";
  std::string                 mBelowOceanMeshName = "BelowOceanTileMesh";

  std::unique_ptr<WaveSimulation> mWaveSim;
  std::vector<double>         mHeights;
  std::vector<double>         mDhdx;
  std::vector<double>         mDhdy;

  std::vector<double>         mDisplacementsX;
  std::vector<double>         mDisplacementsY;
  std::vector<double>         mDxdx;
  std::vector<double>         mDydy;
  std::vector<double>         mDxdy;

  common::Mesh * CreateMesh();

  void ComputeNormals();

  void ComputeTangentSpace();

  static void ComputeTBN(
      const math::Vector3d& _p0, 
      const math::Vector3d& _p1, 
      const math::Vector3d& _p2, 
      const math::Vector2d& _uv0, 
      const math::Vector2d& _uv1, 
      const math::Vector2d& _uv2, 
      math::Vector3d& _tangent, 
      math::Vector3d& _bitangent, 
      math::Vector3d& _normal);

  static void ComputeTBN(
      const std::vector<math::Vector3d>& _vertices,
      const std::vector<math::Vector2d>& _texCoords,
      const std::vector<math::Vector3i>& _faces, 
      std::vector<math::Vector3d>& _tangents,
      std::vector<math::Vector3d>& _bitangents,
      std::vector<math::Vector3d>& _normals);

  void Update(double _time);

  void UpdateVertices(double _time);

  common::Mesh * CreateMesh(const std::string &_name, double _offsetZ,
      bool _reverseOrientation);

  void UpdateMesh(double _time, common::Mesh *_mesh);

  // void UpdateMesh(Ogre::v1::SubMesh *_subMesh, double _offsetZ=0.0, bool _reverseOrientation=false);

  const std::vector<math::Vector3d>& Vertices() const;
  const std::vector<math::Vector3i>& Faces() const;
};

//////////////////////////////////////////////////
OceanTilePrivate::~OceanTilePrivate()
{
}

//////////////////////////////////////////////////
OceanTilePrivate::OceanTilePrivate(
size_t _N,
double _L,
bool _hasVisuals) :
    mHasVisuals(_hasVisuals),
    mResolution(_N),
    mRowLength(_N + 1),
    mNumVertices((_N + 1) * (_N + 1)),
    mNumFaces(2 * _N * _N),
    mTileSize(_L),
    mSpacing(_L / static_cast<double>(_N)),
    mHeights(_N * _N, 0.0),
    mDhdx(_N * _N, 0.0),
    mDhdy(_N * _N, 0.0),
    mDisplacementsX(_N * _N, 0.0),
    mDisplacementsY(_N * _N, 0.0),
    mDxdx(_N * _N, 0.0),
    mDydy(_N * _N, 0.0),
    mDxdy(_N * _N, 0.0)
{
  // Different types of wave simulator are supported...
  // 0 - WaveSimulationSinusoidal
  // 1 - WaveSimulationTrochoid
  // 2 - WaveSimulationFFTW
  // 3 - WaveSimulationOpenCL
  //
  const int wave_sim_type = 2;
  switch (wave_sim_type)
  {
    case 0:
    {
      // Simple
      double amplitude = 0.0;
      double period = 10.0;
      std::unique_ptr<WaveSimulationSinusoidal> waveSim(new WaveSimulationSinusoidal(_N, _L));
      waveSim->SetParameters(amplitude, period);
      mWaveSim = std::move(waveSim);
      break;
    }
    case 1:
    {
      // Trochoid
      std::shared_ptr<WaveParameters> waveParams(new WaveParameters());
      waveParams->SetNumber(3);
      waveParams->SetAngle(0.6);
      waveParams->SetScale(1.2);
      waveParams->SetSteepness(1.0);
      waveParams->SetAmplitude(3.0);
      waveParams->SetPeriod(7.0);
      waveParams->SetDirection(Vector2(1.0, 0.0));
      mWaveSim.reset(new WaveSimulationTrochoid(_N, _L, waveParams));
      break;
    }
    case 2:
    {
      // FFTW
      mWaveSim.reset(new WaveSimulationFFTW(_N, _L));
      break;
    }
    // case 3:
    // {
    //   // OpenCL
    //   mWaveSim.reset(new WaveSimulationOpenCL(_N, _L));
    //   break;
    // }
    default:
      break;
  }
}

//////////////////////////////////////////////////
void OceanTilePrivate::SetWindVelocity(double _ux, double _uy)
{
  mWaveSim->SetWindVelocity(_ux, _uy);
}

//////////////////////////////////////////////////
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
common::Mesh * OceanTilePrivate::CreateMesh()
{
  ignmsg << "OceanTile: create tile\n";
  ignmsg << "Resolution:    " << mResolution  << "\n";
  ignmsg << "RowLength:     " << mRowLength   << "\n";
  ignmsg << "NumVertices:   " << mNumVertices << "\n";
  ignmsg << "NumFaces:      " << mNumFaces    << "\n";
  ignmsg << "TileSize:      " << mTileSize    << "\n";
  ignmsg << "Spacing:       " << mSpacing     << "\n";

  // Grid dimensions
  const size_t nx = this->mResolution;
  const size_t ny = this->mResolution;
  const double Lx = this->mTileSize;
  const double Ly = this->mTileSize;
  const double lx = this->mSpacing;
  const double ly = this->mSpacing;
  const double xTex = 1.0 * lx;
  const double yTex = 1.0 * ly;

  ignmsg << "OceanTile: calculating vertices\n";
  // Vertices - (N+1) vertices in each row / column
  for (size_t iy=0; iy<=ny; ++iy)
  {
    double py = iy * ly - Ly/2.0;
    for (size_t ix=0; ix<=nx; ++ix)
    {
      // Vertex position
      double px = ix * lx - Lx/2.0;

      math::Vector3d vertex(px, py, 0);
      mVertices0.push_back(vertex);
      mVertices.push_back(vertex);
      // Texture coordinates (u, v): top left: (0, 0), bottom right: (1, 1)
      math::Vector2d texCoord(ix * xTex, 1.0 - (iy * yTex));
      mTexCoords.push_back(texCoord);
    }
  }

  ignmsg << "OceanTile: calculating indices\n";
  // Indices
  for (size_t iy=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix)
    {
      // Get the vertices in the cell coordinates
      const size_t idx0 = iy * (nx+1) + ix;
      const size_t idx1 = iy * (nx+1) + ix + 1;
      const size_t idx2 = (iy+1) * (nx+1) + ix + 1;
      const size_t idx3 = (iy+1) * (nx+1) + ix;

      // Indices
      mFaces.push_back(ignition::math::Vector3i(idx0, idx1, idx2));
      mFaces.push_back(ignition::math::Vector3i(idx0, idx2, idx3));
    }
  }

  ignmsg << "OceanTile: assigning texture coords\n";
  // Texture Coordinates
  mTangents.assign(mVertices.size(), math::Vector3d::Zero);
  mBitangents.assign(mVertices.size(), math::Vector3d::Zero);
  mNormals.assign(mVertices.size(), math::Vector3d::Zero);

  // \todo(srmainwaring): remove - this to test static model
  UpdateVertices(5.0);

  if (mHasVisuals) 
  {
    ComputeNormals();
    ComputeTangentSpace();
#if 0
    CreateMesh(this->mAboveOceanMeshName, 0.0, false);
    CreateMesh(this->mBelowOceanMeshName, -0.05, true);
#endif
  }

  return CreateMesh(this->mAboveOceanMeshName, 0.0, false);
}

//////////////////////////////////////////////////
void OceanTilePrivate::ComputeNormals()
{
  // ignmsg << "OceanTile: compute normals\n";

  // 0. Reset normals.
  mNormals.assign(mVertices.size(), math::Vector3d::Zero);

  // 1. For each face calculate the normal and add to each vertex in the face
  for (size_t i=0; i<mNumFaces; ++i)
  {
    // Vertices
    auto v0Idx = mFaces[i][0];
    auto v1Idx = mFaces[i][1];
    auto v2Idx = mFaces[i][2];
    auto&& v0 = mVertices[v0Idx];
    auto&& v1 = mVertices[v1Idx];
    auto&& v2 = mVertices[v2Idx];

    // Normal
    math::Vector3d normal(math::Vector3d::Normal(v0, v1, v2));

    // Add to vertices
    mNormals[v0Idx] += normal;
    mNormals[v1Idx] += normal;
    mNormals[v2Idx] += normal;
  }

  // 2. Normalise each vertex normal.
  for (auto&& normal : mNormals)
  {
      normal.Normalize();
  }

  // ignmsg << "OceanTile: done compute normals\n";
}

//////////////////////////////////////////////////
void OceanTilePrivate::ComputeTangentSpace()
{
  // ignmsg << "OceanTile: compute tangent space\n";

  ComputeTBN(mVertices, mTexCoords, mFaces, mTangents, mBitangents, mNormals);

#if DEBUG
  for (size_t i=0; i<std::min(static_cast<size_t>(20), mVertices.size()) ; ++i)
  {
    ignmsg << "V["  << i << "]:  "  << mVertices[i]   << "\n";
    ignmsg << "UV[" << i << "]: "   << mTexCoords[i]  << "\n"
    ignmsg << "T["  << i << "]:  "  << mTangents[i]   << "\n";
    ignmsg << "B["  << i << "]:  "  << mBitangents[i] << "\n";
    ignmsg << "N["  << i << "]:  "  << mNormals[i]    << "\n";
  }
#endif

  // ignmsg << "OceanTile: done compute tangent space\n";
}

//////////////////////////////////////////////////
// Compute the tangent space vectors (Tanget, Bitangent, Normal) for one face
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
void OceanTilePrivate::ComputeTBN(
    const math::Vector3d& _p0, 
    const math::Vector3d& _p1, 
    const math::Vector3d& _p2, 
    const math::Vector2d& _uv0, 
    const math::Vector2d& _uv1, 
    const math::Vector2d& _uv2, 
    math::Vector3d& _tangent, 
    math::Vector3d& _bitangent, 
    math::Vector3d& _normal)
{
  // Correction to the TBN calculation when the v texture coordinate
  // is 0 at the top of a texture and 1 at the bottom. 
  double vsgn = -1.0;
  auto edge1 = _p1 - _p0;
  auto edge2 = _p2 - _p0;
  auto duv1 = _uv1 - _uv0;
  auto duv2 = _uv2 - _uv0;

  double f = 1.0f / (duv1.X() * duv2.Y() - duv2.X() * duv1.Y()) * vsgn;

  _tangent.X() = f * (duv2.Y() * edge1.X() - duv1.Y() * edge2.X()) * vsgn;
  _tangent.Y() = f * (duv2.Y() * edge1.Y() - duv1.Y() * edge2.Y()) * vsgn;
  _tangent.Z() = f * (duv2.Y() * edge1.Z() - duv1.Y() * edge2.Z()) * vsgn;
  _tangent.Normalize();

  _bitangent.X() = f * (-duv2.X() * edge1.X() + duv1.X() * edge2.X());
  _bitangent.Y() = f * (-duv2.X() * edge1.Y() + duv1.X() * edge2.Y());
  _bitangent.Z() = f * (-duv2.X() * edge1.Z() + duv1.X() * edge2.Z());
  _bitangent.Normalize();  

  _normal = _tangent.Cross(_bitangent);
  _normal.Normalize();  
}

//////////////////////////////////////////////////
// Compute the tangent space for the entire mesh.
void OceanTilePrivate::ComputeTBN(
    const std::vector<math::Vector3d>& _vertices,
    const std::vector<math::Vector2d>& _texCoords,
    const std::vector<math::Vector3i>& _faces, 
    std::vector<math::Vector3d>& _tangents,
    std::vector<math::Vector3d>& _bitangents,
    std::vector<math::Vector3d>& _normals)
{
  // 0. Resize and zero outputs.
  _tangents.assign(_vertices.size(), math::Vector3d::Zero);
  _bitangents.assign(_vertices.size(), math::Vector3d::Zero);
  _normals.assign(_vertices.size(), math::Vector3d::Zero);

  // 1. For each face calculate TBN and add to each vertex in the face.
  for (auto&& face : _faces)
  {
    // Face vertex indices.
    auto idx0 = face[0];
    auto idx1 = face[1];
    auto idx2 = face[2];

    // Face vertex points.
    auto&& p0 = _vertices[idx0];    
    auto&& p1 = _vertices[idx1];    
    auto&& p2 = _vertices[idx2];    

    // Face vertex texture coordinates.
    auto&& uv0 = _texCoords[idx0];
    auto&& uv1 = _texCoords[idx1];
    auto&& uv2 = _texCoords[idx2];

    // Compute tangent space.
    math::Vector3d T, B, N;
    ComputeTBN(p0, p1, p2, uv0, uv1, uv2, T, B, N);

    // Assign to vertices.
    for (int i=0; i<3; ++i)
    {
      auto idx = face[i];
      _tangents[idx]   += T;
      _bitangents[idx] += B;
      _normals[idx]    += N;
    }
  } 

  // 2. Normalise each vertex's tangent space basis.
  for (size_t i=0; i<_vertices.size(); ++i)
  {
    _tangents[i].Normalize();
    _bitangents[i].Normalize();
    _normals[i].Normalize();
  }
}

//////////////////////////////////////////////////
void OceanTilePrivate::Update(double _time)
{
  UpdateVertices(_time);

  if (mHasVisuals)
  {
    // Uncomment to calculate the tangent space using finite differences
    ComputeTangentSpace();
    // UpdateMesh(mAboveOceanSubMesh, 0.0, false);
    // UpdateMesh(mBelowOceanSubMesh, -0.05, false);
  }
}

//////////////////////////////////////////////////
void OceanTilePrivate::UpdateVertices(double _time)
{
  mWaveSim->SetTime(_time);

  if (mHasVisuals)
  {
    mWaveSim->ComputeDisplacementsAndDerivatives(
        mHeights, mDisplacementsX, mDisplacementsY,
        mDhdx, mDhdy, mDxdx, mDydy, mDxdy);
  }
  else
  {
    mWaveSim->ComputeHeights(mHeights);
    mWaveSim->ComputeDisplacements(mDisplacementsX, mDisplacementsY);
    mWaveSim->ComputeHeightDerivatives(mDhdx, mDhdy);
    mWaveSim->ComputeDisplacementDerivatives(mDxdx, mDydy, mDxdy);
  }

  const size_t N = mResolution;
  const size_t NPlus1 = N + 1;
  const size_t NMinus1 = N - 1;

  for (size_t iy=0; iy<N; ++iy)
  {
    for (size_t ix=0; ix<N; ++ix)
    {
      // 1. Update vertices
      size_t idx0 =  iy * NPlus1 + ix;
      size_t idx1 =  iy * N + ix;

      double h  = mHeights[idx1];
      double sx = mDisplacementsX[idx1];
      double sy = mDisplacementsY[idx1];

      auto&& v0 = mVertices0[idx0];
      auto&& v  = mVertices[idx0];
      v.X() = v0.X() + sx;
      v.Y() = v0.Y() + sy;
      v.Z() = v0.Z() + h;

      // 2. Update tangent and bitangent vectors (not normalised).
      // @TODO Check sign for displacement terms
      double dhdx  = mDhdx[idx1]; 
      double dhdy  = mDhdy[idx1]; 
      double dsxdx = mDxdx[idx1]; 
      double dsydy = mDydy[idx1]; 
      double dsxdy = mDxdy[idx1]; 

      auto&& t = mTangents[idx0];
      t.X() = dsxdx + 1.0;
      t.Y() = dsxdy;
      t.Z() = dhdx;
      
      auto&& b = mBitangents[idx0];
      b.X() = dsxdy;
      b.Y() = dsydy + 1.0;
      b.Z() = dhdy;
    }
  }

  // Set skirt values:
  // Apply shifts from row / column adjoining skirt to the skirt vertices.
  // Assume the tangent space vectors for the skirt match the adjoining row / column. 
  for (size_t i=0; i<N; ++i)
  {
    // Top row
    {
      size_t idx0 =  NMinus1 * N + i;
      size_t idx1 =  N * NPlus1 + i;

      double h  = mHeights[idx0];
      double sx = mDisplacementsX[idx0];
      double sy = mDisplacementsY[idx0];

      auto&& v0 = mVertices0[idx1];
      auto&& v  = mVertices[idx1];
      v.X() = v0.X() + sx;
      v.Y() = v0.Y() + sy;
      v.Z() = v0.Z() + h;

      mTangents[idx1] = mTangents[idx0];
      mBitangents[idx1] = mBitangents[idx0];

    }
    // Right column
    {
      size_t idx0 =  i * N + NMinus1;
      size_t idx1 =  i * NPlus1 + N;

      double h  = mHeights[idx0];
      double sx = mDisplacementsX[idx0];
      double sy = mDisplacementsY[idx0];

      auto&& v0 = mVertices0[idx1];
      auto&& v  = mVertices[idx1];
      v.X() = v0.X() + sx;
      v.Y() = v0.Y() + sy;
      v.Z() = v0.Z() + h;

      mTangents[idx1] = mTangents[idx0];
      mBitangents[idx1] = mBitangents[idx0];
    }
  }
  {
    // Top right corner.
    size_t idx0 =  NMinus1 * N + NMinus1;
    size_t idx1 =  N * NPlus1 + N;

    double h  = mHeights[idx0];
    double sx = mDisplacementsX[idx0];
    double sy = mDisplacementsY[idx0];

    auto&& v0 = mVertices0[idx1];
    auto&& v  = mVertices[idx1];
    v.X() = v0.X() + sx;
    v.Y() = v0.Y() + sy;
    v.Z() = v0.Z() + h;
    mTangents[idx1] = mTangents[idx0];
    mBitangents[idx1] = mBitangents[idx0];
  }
}

//////////////////////////////////////////////////
common::Mesh * OceanTilePrivate::CreateMesh(const std::string &_name, double _offsetZ,
    bool _reverseOrientation)
{
  // Logging
  ignmsg << "OceanTile: creating mesh\n";
  std::unique_ptr<common::Mesh> mesh = std::make_unique<common::Mesh>();
  mesh->SetName(_name);

  ignmsg << "OceanTile: create submesh\n";
  common::SubMesh *submesh = new common::SubMesh();

  // Add position vertices
  for (size_t i=0; i<mVertices.size(); ++i)
  {
    submesh->AddVertex(
        mVertices[i][0],
        mVertices[i][1],
        mVertices[i][2] + _offsetZ);

    submesh->AddNormal(
        mNormals[i][0],
        mNormals[i][1],
        mNormals[i][2]);

    // uv0
    submesh->AddTexCoord(
        mTexCoords[i][0],
        mTexCoords[i][1]);

    // uv6
    // *gpuTexVertices++ = mTangents[i][0];
    // *gpuTexVertices++ = mTangents[i][1];
    // *gpuTexVertices++ = mTangents[i][2];

    // uv7
    // *gpuTexVertices++ = mBitangents[i][0];
    // *gpuTexVertices++ = mBitangents[i][1];
    // *gpuTexVertices++ = mBitangents[i][2];
  }

  // Add indices
  for (size_t i=0; i<mFaces.size(); ++i)
  {
    // Reverse orientation on faces
    if (_reverseOrientation)
    {
      submesh->AddIndex(mFaces[i][0]);
      submesh->AddIndex(mFaces[i][2]);
      submesh->AddIndex(mFaces[i][1]);
    }
    else
    {
      submesh->AddIndex(mFaces[i][0]);
      submesh->AddIndex(mFaces[i][1]);
      submesh->AddIndex(mFaces[i][2]);
    }
  }

  mesh->AddSubMesh(*submesh);

  ignmsg << "OceanTile: mesh created." << std::endl;
  return mesh.release();
}

//////////////////////////////////////////////////
void OceanTilePrivate::UpdateMesh(double _time, common::Mesh *_mesh)
{
  this->Update(_time);

  // \todo: add checks
  // \todo: handle more than one submesh
  // Get the submesh
  auto subMesh = _mesh->SubMeshByIndex(0);

  // Update positions, normals, texture coords etc.
  for (size_t i=0; i<mVertices.size(); ++i)
  {
    subMesh.lock()->SetVertex(i, mVertices[i]);
    subMesh.lock()->SetNormal(i, mNormals[i]);
    subMesh.lock()->SetTexCoord(i, mTexCoords[i]);
  }
}

//////////////////////////////////////////////////
const std::vector<math::Vector3d>& OceanTilePrivate::Vertices() const
{
  return mVertices;
}

//////////////////////////////////////////////////
const std::vector<math::Vector3i>& OceanTilePrivate::Faces() const
{
  return mFaces;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
OceanTile::~OceanTile()
{
}

//////////////////////////////////////////////////
OceanTile::OceanTile(size_t _N, double _L, bool _hasVisuals) :
    dataPtr(std::make_unique<OceanTilePrivate>(_N, _L, _hasVisuals))
{
}

//////////////////////////////////////////////////
void OceanTile::SetWindVelocity(double _ux, double _uy)
{
  this->dataPtr->SetWindVelocity(_ux, _uy);
}

//////////////////////////////////////////////////
common::Mesh* OceanTile::CreateMesh()
{
  return this->dataPtr->CreateMesh();
}

//////////////////////////////////////////////////
void OceanTile::Update(double _time)
{
  this->dataPtr->Update(_time);
}

//////////////////////////////////////////////////
void OceanTile::UpdateMesh(double _time, common::Mesh *_mesh)
{
  this->dataPtr->UpdateMesh(_time, _mesh);
}

//////////////////////////////////////////////////
unsigned int OceanTile::VertexCount() const
{
  return this->dataPtr->mVertices.size();
}

//////////////////////////////////////////////////
math::Vector3d OceanTile::Vertex(unsigned int _index) const
{
  return this->dataPtr->mVertices[_index];
}

//////////////////////////////////////////////////
math::Vector2d OceanTile::UV0(unsigned int _index) const
{
  return this->dataPtr->mTexCoords[_index];
}

//////////////////////////////////////////////////
unsigned int OceanTile::FaceCount() const
{
  return this->dataPtr->mFaces.size();
}

//////////////////////////////////////////////////
math::Vector3i OceanTile::Face(unsigned int _index) const
{
  return this->dataPtr->mFaces[_index];
}

}
}
}
