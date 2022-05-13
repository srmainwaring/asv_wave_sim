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

#include "gz/marine/OceanTile.hh"

#include "gz/common/SubMeshWithTangents.hh"
#include "gz/marine/Geometry.hh"
#include "gz/marine/WaveSimulation.hh"
#include "gz/marine/WaveSimulationFFT2.hh"
#include "gz/marine/WaveSimulationSinusoid.hh"
#include "gz/marine/WaveSimulationTrochoid.hh"
#include "gz/marine/WaveParameters.hh"

#include <gz/common.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include <cmath>
#include <iostream>

namespace ignition
{
namespace marine
{

namespace vector
{
  /// \brief Zero vectors
  template <typename Vector3>
  Vector3 Zero;

  template <>
  math::Vector3d Zero<math::Vector3d> = math::Vector3d::Zero;

  template <>
  cgal::Point3 Zero<cgal::Point3> = cgal::Point3(0.0, 0.0, 0.0);
}

template <typename Vector3>
class OceanTilePrivate
{
public:
  ~OceanTilePrivate();

  OceanTilePrivate(unsigned int _N, double _L, bool _hasVisuals=true);

  OceanTilePrivate(WaveParametersPtr _params, bool _hasVisuals=true);

  void SetWindVelocity(double _ux, double _uy);

  bool                        mHasVisuals;
  size_t                      mResolution;      /// \brief FFT size (N = 2^n)
  size_t                      mRowLength;       /// \brief Number of vertices per row (N+1)
  size_t                      mNumVertices;     /// \brief Total number of vertices (N+1)^2
  size_t                      mNumFaces;        /// \brief Total number of faces 2 * N^2
  double                      mTileSize;        /// \brief Tile size in world units
  double                      mSpacing;         /// \brief Space between vertices

  std::vector<Vector3>        mVertices0;
  std::vector<Vector3>        mVertices;
  std::vector<math::Vector3i> mFaces;

  std::vector<Vector3>        mTangents;
  std::vector<math::Vector2d> mTexCoords;
  std::vector<Vector3>        mBitangents;
  std::vector<Vector3>        mNormals;

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

  void Create();

  /// \brief Create a new Mesh. The caller must take ownership.
  //
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
  common::Mesh * CreateMesh();

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
      const Vector3& _p0, 
      const Vector3& _p1, 
      const Vector3& _p2, 
      const math::Vector2d& _uv0, 
      const math::Vector2d& _uv1, 
      const math::Vector2d& _uv2, 
      Vector3& _tangent, 
      Vector3& _bitangent, 
      Vector3& _normal);

  static void ComputeTBN(
      const std::vector<Vector3>& _vertices,
      const std::vector<math::Vector2d>& _texCoords,
      const std::vector<math::Vector3i>& _faces, 
      std::vector<Vector3>& _tangents,
      std::vector<Vector3>& _bitangents,
      std::vector<Vector3>& _normals);

  void Update(double _time);

  /// \brief Update a vertex and it's tangent space.
  ///
  /// \param idx0 is the index for the (N + 1) x (N + 1) mesh vertices,
  ///             including skirt
  /// \param idx1 is the index for the N x N simulated vertices
  void UpdateVertex(size_t idx0, size_t idx1);
  void UpdateVertexAndTangents(size_t idx0, size_t idx1);
  void UpdateVertices(double _time);

  common::Mesh * CreateMesh(const std::string &_name, double _offsetZ,
      bool _reverseOrientation);

  void UpdateMesh(double _time, common::Mesh *_mesh);

};

//////////////////////////////////////////////////
template <typename Vector3>
OceanTilePrivate<Vector3>::~OceanTilePrivate()
{
}

//////////////////////////////////////////////////
template <typename Vector3>
OceanTilePrivate<Vector3>::OceanTilePrivate(
    unsigned int _N,
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
  // 0 - WaveSimulationSinusoid
  // 1 - WaveSimulationTrochoid
  // 2 - WaveSimulationFFT2

  const int wave_sim_type = 2;
  switch (wave_sim_type)
  {
    case 0:
    {
      // Simple
      double dir_x = 1.0;
      double dir_y = 0.0;
      double amplitude = 3.0;
      double period = 10.0;
      std::unique_ptr<WaveSimulationSinusoid> waveSim(
          new WaveSimulationSinusoid(_N, _L));
      waveSim->SetDirection(dir_x, dir_y);
      waveSim->SetAmplitude(amplitude);
      waveSim->SetPeriod(period);
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
      waveParams->SetDirection(math::Vector2d(1.0, 0.0));
      mWaveSim.reset(new WaveSimulationTrochoid(_N, _L, waveParams));
      break;
    }
    case 2:
    {
      // FFT2
      std::unique_ptr<WaveSimulationFFT2> waveSim(
          new WaveSimulationFFT2(_N, _L));
      waveSim->SetLambda(1.0);   // larger lambda => steeper waves.
      mWaveSim = std::move(waveSim);
      break;
    }
    default:
      break;
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
OceanTilePrivate<Vector3>::OceanTilePrivate(
    WaveParametersPtr _params,
    bool _hasVisuals) :
    mHasVisuals(_hasVisuals),
    mResolution(_params->CellCount()),
    mRowLength(_params->CellCount() + 1),
    mNumVertices((_params->CellCount() + 1) * (_params->CellCount() + 1)),
    mNumFaces(2 * _params->CellCount() * _params->CellCount()),
    mTileSize(_params->TileSize()),
    mSpacing(_params->TileSize() / static_cast<double>(_params->CellCount())),
    mHeights(_params->CellCount() * _params->CellCount(), 0.0),
    mDhdx(_params->CellCount() * _params->CellCount(), 0.0),
    mDhdy(_params->CellCount() * _params->CellCount(), 0.0),
    mDisplacementsX(_params->CellCount() * _params->CellCount(), 0.0),
    mDisplacementsY(_params->CellCount() * _params->CellCount(), 0.0),
    mDxdx(_params->CellCount() * _params->CellCount(), 0.0),
    mDydy(_params->CellCount() * _params->CellCount(), 0.0),
    mDxdy(_params->CellCount() * _params->CellCount(), 0.0)
{
  size_t _N = _params->CellCount();
  double _L = _params->TileSize();

  // Different types of wave simulator are supported...
  // 0 - WaveSimulationSinusoid
  // 1 - WaveSimulationTrochoid
  // 2 - WaveSimulationFFT2

  int wave_sim_type = 0;
  if (_params->Algorithm() == "sinusoid")
  {
    wave_sim_type = 0;
  }
  else if (_params->Algorithm() == "trochoid")
  {
    wave_sim_type = 1;
  }
  else if (_params->Algorithm() == "fft")
  {
    wave_sim_type = 2;
  }
  else
  {
    ignerr << "Invalid wave algorithm type: "
        << _params->Algorithm() << "\n";
  }

  switch (wave_sim_type)
  {
    case 0:
    {
      // Simple
      double dir_x = _params->Direction().X();
      double dir_y = _params->Direction().Y();
      double amplitude = _params->Amplitude();
      double period = _params->Period();
      std::unique_ptr<WaveSimulationSinusoid> waveSim(
          new WaveSimulationSinusoid(_N, _L));
      waveSim->SetDirection(dir_x, dir_y);
      waveSim->SetAmplitude(amplitude);
      waveSim->SetPeriod(period);
      mWaveSim = std::move(waveSim);
      break;
    }
    case 1:
    {
      // Trochoid
      mWaveSim.reset(new WaveSimulationTrochoid(_N, _L, _params));
      break;
    }
    case 2:
    {
      // FFT2
      std::unique_ptr<WaveSimulationFFT2> waveSim(
          new WaveSimulationFFT2(_N, _L));
      waveSim->SetLambda(_params->Steepness());  // larger lambda => steeper waves.
      mWaveSim = std::move(waveSim);
      break;
    }
    default:
      break;
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::SetWindVelocity(double _ux, double _uy)
{
  mWaveSim->SetWindVelocity(_ux, _uy);
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::Create()
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
  // Here we are actually mapping (u, v) to each quad in the tile
  // (not the entire tile). 
  // const double xTex = 1.0 * lx;
  // const double yTex = 1.0 * ly;
  /// \todo add param to tune bump map scaling
  double texScale = Lx;
  const double xTex = texScale / mResolution;
  const double yTex = texScale / mResolution;

  ignmsg << "OceanTile: calculating vertices\n";
  // Vertices - (N+1) vertices in each row / column
  for (size_t iy=0; iy<=ny; ++iy)
  {
    double py = iy * ly - Ly/2.0;
    for (size_t ix=0; ix<=nx; ++ix)
    {
      // Vertex position
      double px = ix * lx - Lx/2.0;

      Vector3 vertex(px, py, 0);
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
  mTangents.assign(mVertices.size(), vector::Zero<Vector3>);
  mBitangents.assign(mVertices.size(), vector::Zero<Vector3>);
  mNormals.assign(mVertices.size(), vector::Zero<Vector3>);

  if (mHasVisuals) 
  {
    /// \note: uncomment to calculate normals and tangents by finited difference
    // ComputeNormals();
    // ComputeTangentSpace();
#if 0
    CreateMesh(this->mAboveOceanMeshName, 0.0, false);
    CreateMesh(this->mBelowOceanMeshName, -0.05, true);
#endif
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
common::Mesh * OceanTilePrivate<Vector3>::CreateMesh()
{
  this->Create();
  return CreateMesh(this->mAboveOceanMeshName, 0.0, false);
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<math::Vector3d>::ComputeNormals()
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
template <>
void OceanTilePrivate<cgal::Point3>::ComputeNormals()
{
  // Not used
  ignerr << "No implementation"
      << " of OceanTilePrivate<cgal::Point3>::ComputeNormals\n";
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::ComputeTangentSpace()
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
template <>
void OceanTilePrivate<math::Vector3d>::ComputeTBN(
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
template <>
void OceanTilePrivate<cgal::Point3>::ComputeTBN(
    const cgal::Point3& _p0, 
    const cgal::Point3& _p1, 
    const cgal::Point3& _p2, 
    const math::Vector2d& _uv0, 
    const math::Vector2d& _uv1, 
    const math::Vector2d& _uv2, 
    cgal::Point3& _tangent, 
    cgal::Point3& _bitangent, 
    cgal::Point3& _normal)
{
  // Not used
  ignerr << "No implementation"
      << " of OceanTilePrivate<cgal::Point3>::ComputeTBN\n";
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<math::Vector3d>::ComputeTBN(
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
template <>
void OceanTilePrivate<cgal::Point3>::ComputeTBN(
    const std::vector<cgal::Point3>& _vertices,
    const std::vector<math::Vector2d>& _texCoords,
    const std::vector<math::Vector3i>& _faces, 
    std::vector<cgal::Point3>& _tangents,
    std::vector<cgal::Point3>& _bitangents,
    std::vector<cgal::Point3>& _normals)
{
  // Not used
  ignerr << "No implementation " 
      << " of OceanTilePrivate<cgal::Point3>::ComputeTBN\n";
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::Update(double _time)
{
  UpdateVertices(_time);

  if (mHasVisuals)
  {
    /// \note: Uncomment to calculate the tangent space using finite differences
    // ComputeTangentSpace();
    // UpdateMesh(mAboveOceanSubMesh, 0.0, false);
    // UpdateMesh(mBelowOceanSubMesh, -0.05, false);
  }
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<math::Vector3d>::UpdateVertex(
  size_t idx0, size_t idx1)
{
  // 1. Update vertex
  double h  = mHeights[idx1];
  double sx = mDisplacementsX[idx1];
  double sy = mDisplacementsY[idx1];

  auto&& v0 = mVertices0[idx0];
  auto&& v  = mVertices[idx0];
  v.X() = v0.X() + sx;
  v.Y() = v0.Y() + sy;
  v.Z() = v0.Z() + h;
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::UpdateVertex(size_t idx0, size_t idx1)
{
  // 1. Update vertex
  double h  = mHeights[idx1];
  double sx = mDisplacementsX[idx1];
  double sy = mDisplacementsY[idx1];

  auto&& v0 = mVertices0[idx0];
  mVertices[idx0] = cgal::Point3(
    v0.x() + sx,
    v0.y() + sy,
    v0.z() + h);
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<math::Vector3d>::UpdateVertexAndTangents(
    size_t idx0, size_t idx1)
{
  // 1. Update vertex
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

  auto&& n = mNormals[idx0];
  auto normal =  t.Cross(b);
  n.X() = normal.X();
  n.Y() = normal.Y();
  n.Z() = normal.Z();

  /// \todo add check if this is required for non-FFT models
  // 3. Normal must be reversed when using FTT waves. This is because
  // the coordinate change from matrix indexing to cartesian indexing
  // reflects the surface in the line x=y which flips the
  // surface orientation.
  n *= -1.0;
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::UpdateVertexAndTangents(
    size_t idx0, size_t idx1)
{
  // 1. Update vertex
  double h  = mHeights[idx1];
  double sx = mDisplacementsX[idx1];
  double sy = mDisplacementsY[idx1];

  auto&& v0 = mVertices0[idx0];
  auto&& v  = mVertices[idx0] = cgal::Point3(
    v0.x() + sx,
    v0.y() + sy,
    v0.z() + h);

  // 2. Update tangent and bitangent vectors (not normalised).
  // @TODO Check sign for displacement terms
  double dhdx  = mDhdx[idx1]; 
  double dhdy  = mDhdy[idx1]; 
  double dsxdx = mDxdx[idx1]; 
  double dsydy = mDydy[idx1]; 
  double dsxdy = mDxdy[idx1]; 

  mTangents[idx0] = cgal::Point3(
    dsxdx + 1.0,
    dsxdy,
    dhdx);
  
  mBitangents[idx0] = cgal::Point3(
    dsxdy,
    dsydy + 1.0,
    dhdy);
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::UpdateVertices(double _time)
{
  mWaveSim->SetTime(_time);

  if (mHasVisuals)
  {
    mWaveSim->ComputeDisplacementsAndDerivatives(
        mHeights, mDisplacementsX, mDisplacementsY,
        mDhdx, mDhdy, mDxdx, mDydy, mDxdy);
  
    const size_t N = mResolution;
    const size_t NPlus1  = N + 1;
    const size_t NMinus1 = N - 1;

    for (size_t iy=0; iy<N; ++iy)
    {
      for (size_t ix=0; ix<N; ++ix)
      {
        size_t idx0 = iy * NPlus1 + ix;
        size_t idx1 = iy * N + ix;
        UpdateVertexAndTangents(idx0, idx1);
      }
    }

    // Set skirt values assuming periodic boundary conditions:
    for (size_t i=0; i<N; ++i)
    {
      // Top row (iy = N) periodic with bottom row (iy = 0)
      {
        size_t idx0 = N * NPlus1 + i;
        size_t idx1 = i;
        UpdateVertexAndTangents(idx0, idx1);
      }
      // Right column (ix = N) periodic with left column (ix = 0)
      {
        size_t idx0 = i * NPlus1 + N;
        size_t idx1 = i * N;
        UpdateVertexAndTangents(idx0, idx1);
      }
    }
    {
      // Top right corner period with bottom right corner.
      size_t idx0 = NPlus1 * NPlus1 - 1;
      size_t idx1 = 0;
      UpdateVertexAndTangents(idx0, idx1);
    }  
  }
  else
  {
    mWaveSim->ComputeHeights(mHeights);
    mWaveSim->ComputeDisplacements(mDisplacementsX, mDisplacementsY);

    const size_t N = mResolution;
    const size_t NPlus1  = N + 1;
    const size_t NMinus1 = N - 1;

    for (size_t iy=0; iy<N; ++iy)
    {
      for (size_t ix=0; ix<N; ++ix)
      {
        size_t idx0 = iy * NPlus1 + ix;
        size_t idx1 = iy * N + ix;
        UpdateVertex(idx0, idx1);
      }
    }

    // Set skirt values assuming periodic boundary conditions:
    for (size_t i=0; i<N; ++i)
    {
      // Top row (iy = N) periodic with bottom row (iy = 0)
      {
        size_t idx0 = N * NPlus1 + i;
        size_t idx1 = i;
        UpdateVertex(idx0, idx1);
      }
      // Right column (ix = N) periodic with left column (ix = 0)
      {
        size_t idx0 = i * NPlus1 + N;
        size_t idx1 = i * N;
        UpdateVertex(idx0, idx1);
      }
    }
    {
      // Top right corner period with bottom right corner.
      size_t idx0 = NPlus1 * NPlus1 - 1;
      size_t idx1 = 0;
      UpdateVertex(idx0, idx1);
    }
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
common::Mesh * OceanTilePrivate<Vector3>::CreateMesh(
    const std::string &_name, double _offsetZ,
    bool _reverseOrientation)
{
  // Logging
  ignmsg << "OceanTile: creating mesh\n";
  std::unique_ptr<common::Mesh> mesh = std::make_unique<common::Mesh>();
  mesh->SetName(_name);

  ignmsg << "OceanTile: create submesh\n";
  std::unique_ptr<common::SubMeshWithTangents> submesh(
      new common::SubMeshWithTangents());

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

    /// using an extension that supports tangents
    submesh->AddTangent(
        mTangents[i][0],
        mTangents[i][1],
        mTangents[i][2]);

    // uv0
    submesh->AddTexCoord(
        mTexCoords[i][0],
        mTexCoords[i][1]);
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
 
  // move
  mesh->AddSubMesh(std::move(submesh));

  ignmsg << "OceanTile: mesh created." << std::endl;
  return mesh.release();
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<math::Vector3d>::UpdateMesh(
    double _time, common::Mesh *_mesh)
{
  this->Update(_time);

  // \todo: add checks
  // \todo: handle more than one submesh

  // Get the submesh
  auto baseSubMesh = _mesh->SubMeshByIndex(0).lock();
  auto subMesh = std::dynamic_pointer_cast<
      common::SubMeshWithTangents>(baseSubMesh);
  if (!subMesh)
  {
    ignwarn << "OceanTile: submesh does not support tangents\n";
    return;
  }

  // Update positions, normals, texture coords etc.
  for (size_t i=0; i<mVertices.size(); ++i)
  {
    subMesh->SetVertex(i, mVertices[i]);
    subMesh->SetNormal(i, mNormals[i]);
    subMesh->SetTangent(i, mTangents[i]);
    subMesh->SetTexCoord(i, mTexCoords[i]);
  }
}

//////////////////////////////////////////////////
// Specialisation for math::Vector3d
//////////////////////////////////////////////////
template <>
OceanTileT<math::Vector3d>::~OceanTileT()
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<math::Vector3d>::OceanTileT(
    unsigned int _N, double _L, bool _hasVisuals) :
    dataPtr(std::make_unique<OceanTilePrivate<math::Vector3d>>(
        _N, _L, _hasVisuals))
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<math::Vector3d>::OceanTileT(
    WaveParametersPtr _params, bool _hasVisuals) :
    dataPtr(std::make_unique<OceanTilePrivate<math::Vector3d>>(
        _params, _hasVisuals))
{
}

//////////////////////////////////////////////////
template <>
void OceanTileT<math::Vector3d>::SetWindVelocity(double _ux, double _uy)
{
  this->dataPtr->SetWindVelocity(_ux, _uy);
}

//////////////////////////////////////////////////
template <>
double OceanTileT<math::Vector3d>::TileSize() const
{
  return this->dataPtr->mTileSize;
}

//////////////////////////////////////////////////
template <>
unsigned int OceanTileT<math::Vector3d>::Resolution() const
{
  return this->dataPtr->mResolution;
}

//////////////////////////////////////////////////
template <>
void OceanTileT<math::Vector3d>::Create()
{
  return this->dataPtr->Create();
}

//////////////////////////////////////////////////
template <>
common::Mesh* OceanTileT<math::Vector3d>::CreateMesh()
{
  return this->dataPtr->CreateMesh();
}

//////////////////////////////////////////////////
template <>
void OceanTileT<math::Vector3d>::Update(double _time)
{
  this->dataPtr->Update(_time);
}

//////////////////////////////////////////////////
template <>
void OceanTileT<math::Vector3d>::UpdateMesh(double _time, common::Mesh *_mesh)
{
  this->dataPtr->UpdateMesh(_time, _mesh);
}

//////////////////////////////////////////////////
template <>
unsigned int OceanTileT<math::Vector3d>::VertexCount() const
{
  return this->dataPtr->mVertices.size();
}

//////////////////////////////////////////////////
template <>
math::Vector3d OceanTileT<math::Vector3d>::Vertex(unsigned int _index) const
{
  return this->dataPtr->mVertices[_index];
}

//////////////////////////////////////////////////
template <>
math::Vector2d OceanTileT<math::Vector3d>::UV0(unsigned int _index) const
{
  return this->dataPtr->mTexCoords[_index];
}

//////////////////////////////////////////////////
template <>
unsigned int OceanTileT<math::Vector3d>::FaceCount() const
{
  return this->dataPtr->mFaces.size();
}

//////////////////////////////////////////////////
template <>
math::Vector3i OceanTileT<math::Vector3d>::Face(unsigned int _index) const
{
  return this->dataPtr->mFaces[_index];
}

//////////////////////////////////////////////////
template <>
const std::vector<math::Vector3d>& OceanTileT<math::Vector3d>::Vertices() const
{
  return this->dataPtr->mVertices;
}

//////////////////////////////////////////////////
// Specialisation for cgal::Point3
//////////////////////////////////////////////////
template <>
OceanTileT<cgal::Point3>::~OceanTileT()
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<cgal::Point3>::OceanTileT(
    unsigned int _N, double _L, bool _hasVisuals) :
    dataPtr(std::make_unique<OceanTilePrivate<cgal::Point3>>(
        _N, _L, _hasVisuals))
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<cgal::Point3>::OceanTileT(
    WaveParametersPtr _params, bool _hasVisuals) :
    dataPtr(std::make_unique<OceanTilePrivate<cgal::Point3>>(
        _params, _hasVisuals))
{
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::SetWindVelocity(double _ux, double _uy)
{
  this->dataPtr->SetWindVelocity(_ux, _uy);
}

//////////////////////////////////////////////////
template <>
double OceanTileT<cgal::Point3>::TileSize() const
{
  return this->dataPtr->mTileSize;
}

//////////////////////////////////////////////////
template <>
unsigned int OceanTileT<cgal::Point3>::Resolution() const
{
  return this->dataPtr->mResolution;
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::Create()
{
  return this->dataPtr->Create();
}

//////////////////////////////////////////////////
template <>
common::Mesh* OceanTileT<cgal::Point3>::CreateMesh()
{
  return this->dataPtr->CreateMesh();
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::Update(double _time)
{
  this->dataPtr->Update(_time);
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::UpdateMesh(double _time, common::Mesh *_mesh)
{
  this->dataPtr->UpdateMesh(_time, _mesh);
}

//////////////////////////////////////////////////
template <>
unsigned int OceanTileT<cgal::Point3>::VertexCount() const
{
  return this->dataPtr->mVertices.size();
}

//////////////////////////////////////////////////
template <>
cgal::Point3 OceanTileT<cgal::Point3>::Vertex(unsigned int _index) const
{
  return this->dataPtr->mVertices[_index];
}

//////////////////////////////////////////////////
template <>
math::Vector2d OceanTileT<cgal::Point3>::UV0(unsigned int _index) const
{
  return this->dataPtr->mTexCoords[_index];
}

//////////////////////////////////////////////////
template <>
unsigned int OceanTileT<cgal::Point3>::FaceCount() const
{
  return this->dataPtr->mFaces.size();
}

//////////////////////////////////////////////////
template <>
math::Vector3i OceanTileT<cgal::Point3>::Face(unsigned int _index) const
{
  return this->dataPtr->mFaces[_index];
}

//////////////////////////////////////////////////
template <>
const std::vector<cgal::Point3>& OceanTileT<cgal::Point3>::Vertices() const
{
  return this->dataPtr->mVertices;
}
}
}
