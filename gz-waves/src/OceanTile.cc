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

#include "gz/waves/OceanTile.hh"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include "gz/common/SubMeshWithTangents.hh"
#include "gz/waves/Geometry.hh"
#include "gz/waves/LinearRandomFFTWaveSimulation.hh"
#include "gz/waves/LinearRandomWaveSimulation.hh"
#include "gz/waves/LinearRegularWaveSimulation.hh"
#include "gz/waves/TrochoidIrregularWaveSimulation.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/WaveParameters.hh"
#include "gz/waves/WaveSimulation.hh"

namespace gz
{
namespace waves
{

namespace vector
{
  /// \brief Zero vectors
  template <typename Vector3>
  Vector3 Zero;

  template <>
  gz::math::Vector3d Zero<gz::math::Vector3d> = gz::math::Vector3d::Zero;

  template <>
  cgal::Point3 Zero<cgal::Point3> = cgal::Point3(0.0, 0.0, 0.0);
}  // namespace vector

//////////////////////////////////////////////////
template <typename Vector3>
class OceanTilePrivate
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~OceanTilePrivate();

  explicit OceanTilePrivate(Index nx, Index ny, double lx, double ly,
      bool has_visuals = true);

  explicit OceanTilePrivate(WaveParametersPtr params, bool has_visuals = true);

  void SetWindVelocity(double ux, double uy);
  void SetSteepness(double value);

  bool                        has_visuals_;
  /// \brief FFT size (nx, ny must be a power of 2)
  Index                       nx_;
  Index                       ny_;

  /// \brief Total number of faces 2 * nx * ny
  Index                       num_faces_;

  /// \brief Tile size in world units (m)
  double                      lx_;
  double                      ly_;

  std::vector<Vector3>        vertices0_;
  std::vector<Vector3>        vertices_;
  std::vector<gz::math::Vector3i> faces_;

  std::vector<Vector3>        tangents_;
  std::vector<gz::math::Vector2d> tex_coords_;
  std::vector<Vector3>        bitangents_;
  std::vector<Vector3>        normals_;

  std::string                 above_ocean_mesh_name_ = "AboveOceanTileMesh";
  std::string                 below_ocean_mesh_name_ = "BelowOceanTileMesh";

  std::unique_ptr<IWaveSimulation> wave_sim_;

  Eigen::ArrayXd             heights_;  // height
  Eigen::ArrayXd             dhdx_;     // height deriv
  Eigen::ArrayXd             dhdy_;     // height deriv
  Eigen::ArrayXd             sx_;       // x displacement
  Eigen::ArrayXd             sy_;       // y displacement
  Eigen::ArrayXd             dsxdx_;
  Eigen::ArrayXd             dsydy_;
  Eigen::ArrayXd             dsxdy_;

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
  gz::common::Mesh* CreateMesh();

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
      const Vector3& p0,
      const Vector3& p1,
      const Vector3& p2,
      const gz::math::Vector2d& uv0,
      const gz::math::Vector2d& uv1,
      const gz::math::Vector2d& uv2,
      Vector3& tangent,
      Vector3& bitangent,
      Vector3& normal);

  static void ComputeTBN(
      const std::vector<Vector3>& vertices,
      const std::vector<gz::math::Vector2d>& tex_coords,
      const std::vector<gz::math::Vector3i>& faces,
      std::vector<Vector3>& tangents,
      std::vector<Vector3>& bitangents,
      std::vector<Vector3>& normals);

  void Update(double time);

  /// \brief Update a vertex and it's tangent space.
  ///
  /// \param v_idx  is the index for the (nx + 1) * (ny + 1) mesh vertices,
  ///               including skirt
  /// \param w_idx  is the index for the nx * ny simulated vertices
  void UpdateVertex(Index v_idx, Index w_idx);
  void UpdateVertexAndTangents(Index v_idx, Index w_idx);
  void UpdateVertices(double time);

  gz::common::Mesh * CreateMesh(const std::string& name, double offset_z,
      bool reverse_orientation);

  void UpdateMesh(double time, gz::common::Mesh *mesh);
};

//////////////////////////////////////////////////
template <typename Vector3>
OceanTilePrivate<Vector3>::~OceanTilePrivate()
{
}

//////////////////////////////////////////////////
template <typename Vector3>
OceanTilePrivate<Vector3>::OceanTilePrivate(
    Index nx,
    Index ny,
    double lx,
    double ly,
    bool has_visuals) :
    has_visuals_(has_visuals),
    nx_(nx),
    ny_(ny),
    num_faces_(2 * nx * ny),
    lx_(lx),
    ly_(ly)
{
  auto size = nx_ * ny_;
  heights_ = Eigen::ArrayXd::Zero(size);
  dhdx_ = Eigen::ArrayXd::Zero(size);
  dhdy_ = Eigen::ArrayXd::Zero(size);
  sx_ = Eigen::ArrayXd::Zero(size);
  sy_ = Eigen::ArrayXd::Zero(size);
  dsxdx_ = Eigen::ArrayXd::Zero(size);
  dsydy_ = Eigen::ArrayXd::Zero(size);
  dsxdy_ = Eigen::ArrayXd::Zero(size);

  // Different types of wave simulator are supported...
  // 0 - LinearRegularWaveSimulation
  // 1 - TrochoidIrregularWaveSimulation
  // 2 - LinearRandomFFTWaveSimulation

  Index wave_sim_type = 2;
  switch (wave_sim_type)
  {
    case 0:
    {
      auto wave_sim = std::make_unique<LinearRegularWaveSimulation>(
          lx_, ly_, nx_, ny_);

      double dir_x = 1.0;
      double dir_y = 0.0;
      double amplitude = 3.0;
      double period = 10.0;

      wave_sim->SetDirection(dir_x, dir_y);
      wave_sim->SetAmplitude(amplitude);
      wave_sim->SetPeriod(period);
      wave_sim_ = std::move(wave_sim);
      break;
    }
    case 1:
    {
      auto wave_sim = std::make_unique<TrochoidIrregularWaveSimulation>(
          lx_, ly_, nx_, ny_);

      std::shared_ptr<WaveParameters> wave_params(new WaveParameters());
      wave_params->SetNumber(3);
      wave_params->SetAngle(0.6);
      wave_params->SetScale(1.2);
      wave_params->SetSteepness(1.0);
      wave_params->SetAmplitude(3.0);
      wave_params->SetPeriod(7.0);
      wave_params->SetDirection(gz::math::Vector2d(1.0, 0.0));

      wave_sim_ = std::move(wave_sim);
      break;
    }
    case 2:
    {
      auto wave_sim = std::make_unique<LinearRandomFFTWaveSimulation>(
          lx_, ly_, nx_, ny_);

      wave_sim->SetLambda(1.0);
      wave_sim_ = std::move(wave_sim);
      break;
    }
    default:
      break;
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
OceanTilePrivate<Vector3>::OceanTilePrivate(
    WaveParametersPtr params,
    bool has_visuals) :
    has_visuals_(has_visuals),
    nx_(std::get<0>(params->CellCount())),
    ny_(std::get<1>(params->CellCount())),
    num_faces_(2 * nx_ * ny_),
    lx_(std::get<0>(params->TileSize())),
    ly_(std::get<1>(params->TileSize()))
{
  auto size = nx_ * ny_;
  heights_ = Eigen::ArrayXd::Zero(size);
  dhdx_ = Eigen::ArrayXd::Zero(size);
  dhdy_ = Eigen::ArrayXd::Zero(size);
  sx_ = Eigen::ArrayXd::Zero(size);
  sy_ = Eigen::ArrayXd::Zero(size);
  dsxdx_ = Eigen::ArrayXd::Zero(size);
  dsydy_ = Eigen::ArrayXd::Zero(size);
  dsxdy_ = Eigen::ArrayXd::Zero(size);

  // Different types of wave simulator are supported...
  // 0 - LinearRegularWaveSimulation
  // 1 - TrochoidIrregularWaveSimulation
  // 2 - LinearRandomFFTWaveSimulation
  // 3 - LinearRandomWaveSimulation

  Index wave_sim_type = -1;
  if (params->Algorithm() == "sinusoid" ||
      params->Algorithm() == "linear_regular")
  {
    wave_sim_type = 0;
  }
  if (params->Algorithm() == "trochoid" ||
      params->Algorithm() == "trochoid_irregular")
  {
    wave_sim_type = 1;
  }
  if (params->Algorithm() == "fft" ||
      params->Algorithm() == "linear_random_fft")
  {
    wave_sim_type = 2;
  }
  if (params->Algorithm() == "linear_random")
  {
    wave_sim_type = 3;
  }
  if (wave_sim_type < 0)
  {
    gzerr << "Invalid wave algorithm type: "
        << params->Algorithm() << "\n";
  }

  switch (wave_sim_type)
  {
    case 0:
    {
      auto wave_sim = std::make_unique<LinearRegularWaveSimulation>(
          lx_, ly_, nx_, ny_);

      wave_sim->SetAmplitude(params->Amplitude());
      wave_sim->SetPeriod(params->Period());
      wave_sim->SetDirection(params->Direction().X(), params->Direction().Y());

      wave_sim_ = std::move(wave_sim);
      break;
    }
    case 1:
    {
      auto wave_sim = std::make_unique<TrochoidIrregularWaveSimulation>(
          lx_, ly_, nx_, ny_);

      wave_sim->SetNumber(params->Number());
      wave_sim->SetAmplitude(params->Amplitude_V());
      wave_sim->SetWaveNumber(params->Wavenumber_V());
      wave_sim->SetOmega(params->AngularFrequency_V());
      wave_sim->SetPhase(params->Phase_V());
      wave_sim->SetSteepness(params->Steepness_V());
      wave_sim->SetDirection(params->Direction_V());

      wave_sim_ = std::move(wave_sim);
      break;
    }
    case 2:
    {
      auto wave_sim = std::make_unique<LinearRandomFFTWaveSimulation>(
          lx_, ly_, nx_, ny_);

      wave_sim->SetLambda(params->Steepness());

      wave_sim_ = std::move(wave_sim);
      break;
    }
    case 3:
    {
      auto wave_sim = std::make_unique<LinearRandomWaveSimulation>(
          lx_, ly_, nx_, ny_);

      wave_sim->SetNumWaves(params->Number());

      wave_sim_ = std::move(wave_sim);
      break;
    }
    default:
      break;
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::SetWindVelocity(double ux, double uy)
{
  wave_sim_->SetWindVelocity(ux, uy);
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::SetSteepness(double value)
{
  wave_sim_->SetSteepness(value);
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::Create()
{
  // Grid dimensions
  const double dx = lx_ / static_cast<double>(nx_);
  const double dy = ly_ / static_cast<double>(ny_);

  gzmsg << "OceanTile: create tile\n";
  gzmsg << "Resolution:    " << nx_  << ", " << ny_ << "\n";
  gzmsg << "NumFaces:      " << num_faces_ << "\n";
  gzmsg << "TileSize:      " << lx_ << ", " << ly_ << "\n";
  gzmsg << "Spacing:       " << dx << ", " << dy << "\n";

  // Here we are actually mapping (u, v) to each quad in the tile
  // (not the entire tile).
  // const double x_tex = 1.0 * lx;
  // const double y_tex = 1.0 * ly;
  /// \todo add param to tune bump map scaling
  double tex_scale = 0.1;
  const double x_tex = tex_scale * dx;
  const double y_tex = tex_scale * dy;

  gzmsg << "OceanTile: calculating vertices\n";
  // Vertices - (nx+1) / (ny+1) vertices in each row / column
  for (Index iy=0; iy <= ny_; ++iy)
  {
    double py = iy * dy - ly_ / 2.0;
    for (Index ix=0; ix <= nx_; ++ix)
    {
      // Vertex position
      double px = ix * dx - lx_ / 2.0;

      Vector3 vertex(px, py, 0);
      vertices0_.push_back(vertex);
      vertices_.push_back(vertex);
      // Texture coordinates (u, v): top left: (0, 0), bottom right: (1, 1)
      gz::math::Vector2d tex_coord(ix * x_tex, 1.0 - (iy * y_tex));
      tex_coords_.push_back(tex_coord);
    }
  }

  gzmsg << "OceanTile: calculating indices\n";
  // Indices
  for (Index iy=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix)
    {
      // Get the vertices in the cell coordinates
      const Index idx0 = iy * (nx_ + 1) + ix;
      const Index idx1 = iy * (nx_ + 1) + ix + 1;
      const Index idx2 = (iy + 1) * (nx_ + 1) + ix + 1;
      const Index idx3 = (iy + 1) * (nx_ + 1) + ix;

      // Indices
      faces_.push_back(gz::math::Vector3i(idx0, idx1, idx2));
      faces_.push_back(gz::math::Vector3i(idx0, idx2, idx3));
    }
  }

  gzmsg << "OceanTile: assigning texture coords\n";
  // Texture Coordinates
  tangents_.assign(vertices_.size(), vector::Zero<Vector3>);
  bitangents_.assign(vertices_.size(), vector::Zero<Vector3>);
  normals_.assign(vertices_.size(), vector::Zero<Vector3>);

  if (has_visuals_)
  {
    /// \note: uncomment to calculate normals and tangents by finited difference
    // ComputeNormals();
    // ComputeTangentSpace();
#if 0
    CreateMesh(above_ocean_mesh_name_, 0.0, false);
    CreateMesh(below_ocean_mesh_name_, -0.05, true);
#endif
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
gz::common::Mesh * OceanTilePrivate<Vector3>::CreateMesh()
{
  Create();
  return CreateMesh(above_ocean_mesh_name_, 0.0, false);
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<gz::math::Vector3d>::ComputeNormals()
{
  // gzmsg << "OceanTile: compute normals\n";

  // 0. Reset normals.
  normals_.assign(vertices_.size(), gz::math::Vector3d::Zero);

  // 1. For each face calculate the normal and add to each vertex in the face
  for (Index i=0; i < num_faces_; ++i)
  {
    // Vertices
    auto v0_idx = faces_[i][0];
    auto v1_idx = faces_[i][1];
    auto v2_idx = faces_[i][2];
    auto&& v0 = vertices_[v0_idx];
    auto&& v1 = vertices_[v1_idx];
    auto&& v2 = vertices_[v2_idx];

    // Normal
    gz::math::Vector3d normal(gz::math::Vector3d::Normal(v0, v1, v2));

    // Add to vertices
    normals_[v0_idx] += normal;
    normals_[v1_idx] += normal;
    normals_[v2_idx] += normal;
  }

  // 2. Normalise each vertex normal.
  for (auto&& normal : normals_)
  {
      normal.Normalize();
  }

  // gzmsg << "OceanTile: done compute normals\n";
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::ComputeNormals()
{
  // Not used
  gzerr << "No implementation"
      << " of OceanTilePrivate<cgal::Point3>::ComputeNormals\n";
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::ComputeTangentSpace()
{
  // gzmsg << "OceanTile: compute tangent space\n";

  ComputeTBN(vertices_, tex_coords_, faces_, tangents_, bitangents_, normals_);

#if DEBUG
  for (Index i=0; i<std::min(static_cast<Index>(20), vertices_.size()) ; ++i)
  {
    gzmsg << "V["  << i << "]:  "  << vertices_[i]   << "\n";
    gzmsg << "UV[" << i << "]: "   << tex_coords_[i] << "\n"
    gzmsg << "T["  << i << "]:  "  << tangents_[i]   << "\n";
    gzmsg << "B["  << i << "]:  "  << bitangents_[i] << "\n";
    gzmsg << "N["  << i << "]:  "  << normals_[i]    << "\n";
  }
#endif

  // gzmsg << "OceanTile: done compute tangent space\n";
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<gz::math::Vector3d>::ComputeTBN(
    const gz::math::Vector3d& p0,
    const gz::math::Vector3d& p1,
    const gz::math::Vector3d& p2,
    const gz::math::Vector2d& uv0,
    const gz::math::Vector2d& uv1,
    const gz::math::Vector2d& uv2,
    gz::math::Vector3d& tangent,
    gz::math::Vector3d& bitangent,
    gz::math::Vector3d& normal)
{
  // Correction to the TBN calculation when the v texture coordinate
  // is 0 at the top of a texture and 1 at the bottom.
  double vsgn = -1.0;
  auto edge1 = p1 - p0;
  auto edge2 = p2 - p0;
  auto duv1 = uv1 - uv0;
  auto duv2 = uv2 - uv0;

  double f = 1.0f / (duv1.X() * duv2.Y() - duv2.X() * duv1.Y()) * vsgn;

  tangent.X() = f * (duv2.Y() * edge1.X() - duv1.Y() * edge2.X()) * vsgn;
  tangent.Y() = f * (duv2.Y() * edge1.Y() - duv1.Y() * edge2.Y()) * vsgn;
  tangent.Z() = f * (duv2.Y() * edge1.Z() - duv1.Y() * edge2.Z()) * vsgn;
  tangent.Normalize();

  bitangent.X() = f * (-duv2.X() * edge1.X() + duv1.X() * edge2.X());
  bitangent.Y() = f * (-duv2.X() * edge1.Y() + duv1.X() * edge2.Y());
  bitangent.Z() = f * (-duv2.X() * edge1.Z() + duv1.X() * edge2.Z());
  bitangent.Normalize();

  normal = tangent.Cross(bitangent);
  normal.Normalize();
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::ComputeTBN(
    const cgal::Point3& /*p0*/,
    const cgal::Point3& /*p1*/,
    const cgal::Point3& /*p2*/,
    const gz::math::Vector2d& /*uv0*/,
    const gz::math::Vector2d& /*uv1*/,
    const gz::math::Vector2d& /*uv2*/,
    cgal::Point3& /*tangent*/,
    cgal::Point3& /*bitangent*/,
    cgal::Point3& /*normal*/)
{
  // Not used
  gzerr << "No implementation"
      << " of OceanTilePrivate<cgal::Point3>::ComputeTBN\n";
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<gz::math::Vector3d>::ComputeTBN(
    const std::vector<gz::math::Vector3d>& vertices,
    const std::vector<gz::math::Vector2d>& tex_coords,
    const std::vector<gz::math::Vector3i>& faces,
    std::vector<gz::math::Vector3d>& tangents,
    std::vector<gz::math::Vector3d>& bitangents,
    std::vector<gz::math::Vector3d>& normals)
{
  // 0. Resize and zero outputs.
  tangents.assign(vertices.size(), gz::math::Vector3d::Zero);
  bitangents.assign(vertices.size(), gz::math::Vector3d::Zero);
  normals.assign(vertices.size(), gz::math::Vector3d::Zero);

  // 1. For each face calculate TBN and add to each vertex in the face.
  for (auto&& face : faces)
  {
    // Face vertex indices.
    auto idx0 = face[0];
    auto idx1 = face[1];
    auto idx2 = face[2];

    // Face vertex points.
    auto&& p0 = vertices[idx0];
    auto&& p1 = vertices[idx1];
    auto&& p2 = vertices[idx2];

    // Face vertex texture coordinates.
    auto&& uv0 = tex_coords[idx0];
    auto&& uv1 = tex_coords[idx1];
    auto&& uv2 = tex_coords[idx2];

    // Compute tangent space.
    gz::math::Vector3d tangent, bitangent, normal;
    ComputeTBN(p0, p1, p2, uv0, uv1, uv2, tangent, bitangent, normal);

    // Assign to vertices.
    for (Index i=0; i < 3; ++i)
    {
      auto idx = face[i];
      tangents[idx]   += tangent;
      bitangents[idx] += bitangent;
      normals[idx]    += normal;
    }
  }

  // 2. Normalise each vertex's tangent space basis.
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    tangents[i].Normalize();
    bitangents[i].Normalize();
    normals[i].Normalize();
  }
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::ComputeTBN(
    const std::vector<cgal::Point3> & /*vertices*/,
    const std::vector<gz::math::Vector2d>& /*tex_coords*/,
    const std::vector<gz::math::Vector3i>& /*faces*/,
    std::vector<cgal::Point3>& /*tangents*/,
    std::vector<cgal::Point3>& /*bitangents*/,
    std::vector<cgal::Point3>& /*normals*/)
{
  // Not used
  gzerr << "No implementation "
      << " of OceanTilePrivate<cgal::Point3>::ComputeTBN\n";
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::Update(double time)
{
  UpdateVertices(time);

  if (has_visuals_)
  {
    /// \note: Uncomment to calculate the tangent space using finite differences
    // ComputeTangentSpace();
    // UpdateMesh(mAboveOceanSubMesh, 0.0, false);
    // UpdateMesh(mBelowOceanSubMesh, -0.05, false);
  }
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<gz::math::Vector3d>::UpdateVertex(
  Index v_idx, Index w_idx)
{
  // 1. Update vertex
  double h  = heights_[w_idx];
  double sx = sx_[w_idx];
  double sy = sy_[w_idx];

  auto&& v0 = vertices0_[v_idx];
  auto&& v  = vertices_[v_idx];
  v.X() = v0.X() + sy;
  v.Y() = v0.Y() + sx;
  v.Z() = v0.Z() + h;
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::UpdateVertex(Index v_idx, Index w_idx)
{
  // 1. Update vertex
  double h  = heights_[w_idx];
  double sx = sx_[w_idx];
  double sy = sy_[w_idx];

  auto&& v0 = vertices0_[v_idx];
  vertices_[v_idx] = cgal::Point3(
    v0.x() + sy,
    v0.y() + sx,
    v0.z() + h);
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<gz::math::Vector3d>::UpdateVertexAndTangents(
    Index v_idx, Index w_idx)
{
  // 1. Update vertex
  double h  = heights_[w_idx];
  double sx = sx_[w_idx];
  double sy = sy_[w_idx];

  auto&& v0 = vertices0_[v_idx];
  auto&& v  = vertices_[v_idx];
  v.X() = v0.X() + sy;
  v.Y() = v0.Y() + sx;
  v.Z() = v0.Z() + h;

  // 2. Update tangent and bitangent vectors (not normalised).
  // @TODO Check sign for displacement terms
  double dhdx  = dhdx_[w_idx];
  double dhdy  = dhdy_[w_idx];
  double dsxdx = dsxdx_[w_idx];
  double dsydy = dsydy_[w_idx];
  double dsxdy = dsxdy_[w_idx];

  auto&& t = tangents_[v_idx];
  t.X() = dsydy + 1.0;
  t.Y() = dsxdy;
  t.Z() = dhdy;

  auto&& b = bitangents_[v_idx];
  b.X() = dsxdy;
  b.Y() = dsxdx + 1.0;
  b.Z() = dhdx;

  auto&& n = normals_[v_idx];
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
    Index v_idx, Index w_idx)
{
  // 1. Update vertex
  // double h  = heights_[w_idx];
  // double sx = sx_[w_idx];
  // double sy = sy_[w_idx];

  // auto&& v0 = vertices0_[v_idx];

  // 2. Update tangent and bitangent vectors (not normalised).
  // @TODO Check sign for displacement terms
  double dhdx  = dhdx_[w_idx];
  double dhdy  = dhdy_[w_idx];
  double dsxdx = dsxdx_[w_idx];
  double dsydy = dsydy_[w_idx];
  double dsxdy = dsxdy_[w_idx];

  tangents_[v_idx] = cgal::Point3(
    dsydy + 1.0,
    dsxdy,
    dhdy);

  bitangents_[v_idx] = cgal::Point3(
    dsxdy,
    dsxdx + 1.0,
    dhdx);
}

//////////////////////////////////////////////////
template <typename Vector3>
void OceanTilePrivate<Vector3>::UpdateVertices(double time)
{
  wave_sim_->SetTime(time);

  if (has_visuals_)
  {
    wave_sim_->DisplacementAndDerivAt(
        heights_, sx_, sy_,
        dhdx_, dhdy_, dsxdx_, dsydy_, dsxdy_);

    // const Index nx = nx_;
    // const Index ny = ny_;
    const Index nx_plus1  = nx_ + 1;
    const Index ny_plus1  = ny_ + 1;

    for (Index iy=0; iy < ny_; ++iy)
    {
      for (Index ix=0; ix < nx_; ++ix)
      {
        Index v_idx_cm = iy * nx_plus1 + ix;
        Index w_idx_cm = iy * nx_ + ix;
        UpdateVertexAndTangents(v_idx_cm, w_idx_cm);
      }
    }

    // Set skirt values assuming periodic boundary conditions:
    for (Index ix=0; ix < nx_; ++ix)
    {
      // Top row (iy = nx) periodic with bottom row (iy = 0)
      {
        Index v_idx_cm = ny_ * nx_plus1 + ix;
        Index w_idx_cm = ix;
        UpdateVertexAndTangents(v_idx_cm, w_idx_cm);
      }
    }
    for (Index iy=0; iy < ny_; ++iy)
    {
      // Right column (ix = nx) periodic with left column (ix = 0)
      {
        Index v_idx_cm = iy * nx_plus1 + nx_;
        Index w_idx_cm = iy * nx_;
        UpdateVertexAndTangents(v_idx_cm, w_idx_cm);
      }
    }
    {
      // Top right corner period with bottom right corner.
      Index v_idx_cm = nx_plus1 * ny_plus1 - 1;
      Index w_idx_cm = 0;
      UpdateVertexAndTangents(v_idx_cm, w_idx_cm);
    }
  } else {
    wave_sim_->ElevationAt(heights_);
    wave_sim_->DisplacementAt(sx_, sy_);

    // const Index nx = nx_;
    // const Index ny = ny_;
    const Index nx_plus1  = nx_ + 1;
    const Index ny_plus1  = ny_ + 1;

    for (Index iy=0; iy < ny_; ++iy)
    {
      for (Index ix=0; ix < nx_; ++ix)
      {
        Index v_idx_cm = iy * nx_plus1 + ix;
        Index w_idx_cm = iy * nx_ + ix;
        UpdateVertex(v_idx_cm, w_idx_cm);
      }
    }

    // Set skirt values assuming periodic boundary conditions:
    for (Index ix=0; ix < nx_; ++ix)
    {
      // Top row (iy = nx) periodic with bottom row (iy = 0)
      {
        Index v_idx_cm = ny_ * nx_plus1 + ix;
        Index w_idx_cm = ix;
        UpdateVertex(v_idx_cm, w_idx_cm);
      }
    }
    for (Index iy=0; iy < ny_; ++iy)
    {
      // Right column (ix = nx) periodic with left column (ix = 0)
      {
        Index v_idx_cm = iy * nx_plus1 + nx_;
        Index w_idx_cm = iy * nx_;
        UpdateVertex(v_idx_cm, w_idx_cm);
      }
    }
    {
      // Top right corner period with bottom right corner.
      Index v_idx_cm = nx_plus1 * ny_plus1 - 1;
      Index w_idx_cm = 0;
      UpdateVertex(v_idx_cm, w_idx_cm);
    }
  }
}

//////////////////////////////////////////////////
template <typename Vector3>
gz::common::Mesh* OceanTilePrivate<Vector3>::CreateMesh(
    const std::string& name, double offset_z,
    bool reverse_orientation)
{
  // Logging
  gzmsg << "OceanTile: creating mesh\n";
  std::unique_ptr<gz::common::Mesh> mesh = std::make_unique<gz::common::Mesh>();
  mesh->SetName(name);

  gzmsg << "OceanTile: create submesh\n";
  std::unique_ptr<gz::common::SubMeshWithTangents> submesh(
      new gz::common::SubMeshWithTangents());

  // Add position vertices
  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    submesh->AddVertex(
        vertices_[i][0],
        vertices_[i][1],
        vertices_[i][2] + offset_z);

    submesh->AddNormal(
        normals_[i][0],
        normals_[i][1],
        normals_[i][2]);

    /// using an extension that supports tangents
    submesh->AddTangent(
        tangents_[i][0],
        tangents_[i][1],
        tangents_[i][2]);

    // uv0
    submesh->AddTexCoord(
        tex_coords_[i][0],
        tex_coords_[i][1]);
  }

  // Add indices
  for (size_t i = 0; i < faces_.size(); ++i)
  {
    // Reverse orientation on faces
    if (reverse_orientation)
    {
      submesh->AddIndex(faces_[i][0]);
      submesh->AddIndex(faces_[i][2]);
      submesh->AddIndex(faces_[i][1]);
    } else {
      submesh->AddIndex(faces_[i][0]);
      submesh->AddIndex(faces_[i][1]);
      submesh->AddIndex(faces_[i][2]);
    }
  }

  // move
  mesh->AddSubMesh(std::move(submesh));

  gzmsg << "OceanTile: mesh created." << std::endl;
  return mesh.release();
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<gz::math::Vector3d>::UpdateMesh(
    double time, gz::common::Mesh* mesh)
{
  Update(time);

  // \todo: add checks
  // \todo: handle more than one submesh

  // Get the submesh
  auto base_submesh = mesh->SubMeshByIndex(0).lock();
  auto submesh = std::dynamic_pointer_cast<
      gz::common::SubMeshWithTangents>(base_submesh);
  if (!submesh)
  {
    gzwarn << "OceanTile: submesh does not support tangents\n";
    return;
  }

  // Update positions, normals, texture coords etc.
  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    submesh->SetVertex(i, vertices_[i]);
    submesh->SetNormal(i, normals_[i]);
    submesh->SetTangent(i, tangents_[i]);
    submesh->SetTexCoord(i, tex_coords_[i]);
  }
}

//////////////////////////////////////////////////
template <>
void OceanTilePrivate<cgal::Point3>::UpdateMesh(
    double /*time*/, gz::common::Mesh* /*mesh*/)
{
  /// \note This template specialisation is supplied for compilation on
  ///       macOS M1 (arm64). It should never be called.
  GZ_ASSERT(false, "Should never reach here");
}

//////////////////////////////////////////////////
// Specialisation for gz::math::Vector3d
//////////////////////////////////////////////////
template <>
OceanTileT<gz::math::Vector3d>::~OceanTileT()
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<gz::math::Vector3d>::OceanTileT(
    Index nx, Index ny, double lx, double ly, bool has_visuals) :
    impl_(std::make_unique<OceanTilePrivate<gz::math::Vector3d>>(
        nx, ny, lx, ly, has_visuals))
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<gz::math::Vector3d>::OceanTileT(
    WaveParametersPtr params, bool has_visuals) :
    impl_(std::make_unique<OceanTilePrivate<gz::math::Vector3d>>(
        params, has_visuals))
{
}

//////////////////////////////////////////////////
template <>
void OceanTileT<gz::math::Vector3d>::SetWindVelocity(double ux, double uy)
{
  impl_->SetWindVelocity(ux, uy);
}

//////////////////////////////////////////////////
template <>
void OceanTileT<gz::math::Vector3d>::SetSteepness(double value)
{
  impl_->SetSteepness(value);
}

//////////////////////////////////////////////////
template <>
std::array<double, 2> OceanTileT<gz::math::Vector3d>::TileSize() const
{
  return {impl_->lx_, impl_->ly_};
}

//////////////////////////////////////////////////
template <>
std::array<Index, 2> OceanTileT<gz::math::Vector3d>::CellCount() const
{
  return {impl_->nx_, impl_->ny_};
}

//////////////////////////////////////////////////
template <>
void OceanTileT<gz::math::Vector3d>::Create()
{
  return impl_->Create();
}

//////////////////////////////////////////////////
template <>
gz::common::Mesh* OceanTileT<gz::math::Vector3d>::CreateMesh()
{
  return impl_->CreateMesh();
}

//////////////////////////////////////////////////
template <>
void OceanTileT<gz::math::Vector3d>::Update(double time)
{
  impl_->Update(time);
}

//////////////////////////////////////////////////
template <>
void OceanTileT<gz::math::Vector3d>::UpdateMesh(
    double time, gz::common::Mesh *mesh)
{
  impl_->UpdateMesh(time, mesh);
}

//////////////////////////////////////////////////
template <>
Index OceanTileT<gz::math::Vector3d>::VertexCount() const
{
  return impl_->vertices_.size();
}

//////////////////////////////////////////////////
template <>
gz::math::Vector3d OceanTileT<gz::math::Vector3d>::Vertex(
    Index index) const
{
  return impl_->vertices_[index];
}

//////////////////////////////////////////////////
template <>
gz::math::Vector2d OceanTileT<gz::math::Vector3d>::UV0(
    Index index) const
{
  return impl_->tex_coords_[index];
}

//////////////////////////////////////////////////
template <>
Index OceanTileT<gz::math::Vector3d>::FaceCount() const
{
  return impl_->faces_.size();
}

//////////////////////////////////////////////////
template <>
gz::math::Vector3i OceanTileT<gz::math::Vector3d>::Face(
    Index index) const
{
  return impl_->faces_[index];
}

//////////////////////////////////////////////////
template <>
const std::vector<gz::math::Vector3d>&
OceanTileT<gz::math::Vector3d>::Vertices() const
{
  return impl_->vertices_;
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
    Index nx, Index ny, double lx, double ly, bool has_visuals) :
    impl_(std::make_unique<OceanTilePrivate<cgal::Point3>>(
        nx, ny, lx, ly, has_visuals))
{
}

//////////////////////////////////////////////////
template <>
OceanTileT<cgal::Point3>::OceanTileT(
    WaveParametersPtr params, bool has_visuals) :
    impl_(std::make_unique<OceanTilePrivate<cgal::Point3>>(
        params, has_visuals))
{
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::SetWindVelocity(double ux, double uy)
{
  impl_->SetWindVelocity(ux, uy);
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::SetSteepness(double value)
{
  impl_->SetSteepness(value);
}

//////////////////////////////////////////////////
template <>
std::array<double, 2> OceanTileT<cgal::Point3>::TileSize() const
{
  return {impl_->lx_, impl_->ly_};
}

//////////////////////////////////////////////////
template <>
std::array<Index, 2> OceanTileT<cgal::Point3>::CellCount() const
{
  return {impl_->nx_, impl_->ny_};
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::Create()
{
  return impl_->Create();
}

//////////////////////////////////////////////////
template <>
gz::common::Mesh* OceanTileT<cgal::Point3>::CreateMesh()
{
  return impl_->CreateMesh();
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::Update(double time)
{
  impl_->Update(time);
}

//////////////////////////////////////////////////
template <>
void OceanTileT<cgal::Point3>::UpdateMesh(double time, gz::common::Mesh* mesh)
{
  impl_->UpdateMesh(time, mesh);
}

//////////////////////////////////////////////////
template <>
Index OceanTileT<cgal::Point3>::VertexCount() const
{
  return impl_->vertices_.size();
}

//////////////////////////////////////////////////
template <>
cgal::Point3 OceanTileT<cgal::Point3>::Vertex(Index index) const
{
  return impl_->vertices_[index];
}

//////////////////////////////////////////////////
template <>
gz::math::Vector2d OceanTileT<cgal::Point3>::UV0(Index index) const
{
  return impl_->tex_coords_[index];
}

//////////////////////////////////////////////////
template <>
Index OceanTileT<cgal::Point3>::FaceCount() const
{
  return impl_->faces_.size();
}

//////////////////////////////////////////////////
template <>
gz::math::Vector3i OceanTileT<cgal::Point3>::Face(Index index) const
{
  return impl_->faces_[index];
}

//////////////////////////////////////////////////
template <>
const std::vector<cgal::Point3>& OceanTileT<cgal::Point3>::Vertices() const
{
  return impl_->vertices_;
}

}  // namespace waves
}  // namespace gz
