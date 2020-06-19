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

#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/WaveParameters.hh"


#include "asv_wave_sim_gazebo_plugins/OceanTile.hh"
#include "asv_wave_sim_gazebo_plugins/TriangulatedGrid.hh"

#include <gazebo/gazebo.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <array>
#include <iostream>
#include <cmath>
#include <string>

namespace asv 
{

///////////////////////////////////////////////////////////////////////////////
// Wavefield

  Wavefield::~Wavefield()
  {    
  }

///////////////////////////////////////////////////////////////////////////////
// WavefieldGerstnerPrivate

  /// \internal
  /// \brief Private data for the Wavefield.
  class WavefieldGerstnerPrivate
  {
    /// \brief Constructor.
    public: WavefieldGerstnerPrivate() :
      params(new WaveParameters()),
      size({ 1000, 1000 }),
      cellCount({ 50, 50 })
    {
    }

    /// \brief Constructor.
    ///
    /// \brief param[in] _size      The dimensions of the wave field [m].
    /// \brief param[in] _cellCount The number of cells in each direction.
    public: WavefieldGerstnerPrivate(
      const std::array<double, 2>& _size,
      const std::array<size_t, 2>& _cellCount 
    ) :
      params(new WaveParameters()),
      size(_size),
      cellCount(_cellCount)
    {
    }

    /// \brief Wave parameters
    public: std::shared_ptr<WaveParameters> params;

    /// brief The dimensions of the wave field in each direction [m].    
    public: std::array<double, 2> size;

    /// \brief The number of grid cells in each direction.
    public: std::array<size_t, 2> cellCount;

    /// \brief The initial wave field mesh. This is a triangulated regular grid.
    public: std::shared_ptr<const Grid> initialGrid;

    /// \brief The current position of the wave field.
    public: std::shared_ptr<Grid> grid;

    /// \brief The current position of the wave field.
    public: std::unique_ptr<TriangulatedGrid> triangulatedGrid;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldGerstner

  WavefieldGerstner::~WavefieldGerstner()
  {
  }

  WavefieldGerstner::WavefieldGerstner(
    const std::string& _name) :
    data(new WavefieldGerstnerPrivate())
  {
    // Grid
    this->data->initialGrid.reset(new Grid(
      this->data->size, this->data->cellCount));
    this->data->grid.reset(new Grid(
      this->data->size, this->data->cellCount));
    
    // Point Locator
    int N = this->data->cellCount[0];
    double L = this->data->size[0];
    this->data->triangulatedGrid = std::move(TriangulatedGrid::Create(N, L));

    // Update
    this->Update(0.0);
  }

  WavefieldGerstner::WavefieldGerstner(
    const std::string& _name,
    const std::array<double, 2>& _size,
    const std::array<size_t, 2>& _cellCount) : 
    data(new WavefieldGerstnerPrivate(_size, _cellCount))
  {
    // Grid
    this->data->initialGrid.reset(new Grid(
      this->data->size, this->data->cellCount));
    this->data->grid.reset(new Grid(
      this->data->size, this->data->cellCount));
    
    // Point Locator
    int64_t N = this->data->cellCount[0];
    double L = this->data->size[0];
    this->data->triangulatedGrid = std::move(TriangulatedGrid::Create(N, L));
    // this->data->triangulatedGrid->DebugPrintTriangulation();

    // Update
    this->Update(0.0);
  }

  std::shared_ptr<const Mesh> WavefieldGerstner::GetMesh() const
  {
    return this->data->grid->GetMesh();
  }

  std::shared_ptr<const Grid> WavefieldGerstner::GetGrid() const
  {
    return this->data->grid;
  }

  bool WavefieldGerstner::Height(const Point3& point, double& height) const
  {
    return this->data->triangulatedGrid->Height(point, height);    
  }

  std::shared_ptr<const WaveParameters> WavefieldGerstner::GetParameters() const
  {
    return this->data->params;
  }

  void WavefieldGerstner::SetParameters(std::shared_ptr<WaveParameters> _params) const
  {
    GZ_ASSERT(_params != nullptr, "Invalid parameter _params");
    this->data->params = _params;    
  }

  void WavefieldGerstner::Update(double _time)
  {
    this->UpdateGerstnerWave(_time);

    // Update point locator
    auto& mesh = *this->data->grid->GetMesh();
    this->data->triangulatedGrid->UpdatePoints(mesh);
  }

  void WavefieldGerstner::UpdateGerstnerWave(double _time)
  {
    // Single wave params
    // auto amplitude  = this->data->params->Amplitude();
    // auto wavenumber = this->data->params->Wavenumber();
    // auto omega      = this->data->params->AngularFrequency();
    // auto phase      = this->data->params->Phase();
    // auto q          = this->data->params->Steepness();
    // auto direction  = this->data->params->Direction();

    // Multiple wave params
    const auto  number     = this->data->params->Number();
    const auto& amplitude  = this->data->params->Amplitude_V();
    const auto& wavenumber = this->data->params->Wavenumber_V();
    const auto& omega      = this->data->params->AngularFrequency_V();
    const auto& phase      = this->data->params->Phase_V();
    const auto& q          = this->data->params->Steepness_V();
    const auto& direction  = this->data->params->Direction_V();

    const auto& initMesh = *this->data->initialGrid->GetMesh();
    auto& mesh = *this->data->grid->GetMesh();

    // Reset points to original positions
    for (
      auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
      it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
      ++it.first, ++it.second)
    {
      auto& vtx0 = *it.first;
      auto& vtx1 = *it.second;
      mesh.point(vtx1) = initMesh.point(vtx0);
    }
    
    // Multiple wave update 
    for (size_t i=0; i<number; ++i)
    // size_t i = 2;
    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      const auto& q_i = q[i];

      for (
        auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
        it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
        ++it.first, ++it.second)
      {
        auto& vtx0 = *it.first;
        auto& vtx1 = *it.second;

        const Point3& p0 = initMesh.point(vtx0);
        Vector2 v0(p0.x(), p0.y()); // - CGAL::ORIGIN;        

        // Multiple waves
        const double angle  = CGAL::to_double(direction_i * v0) * wavenumber_i - omega_i * _time + phase_i;
        const double s = std::sin(angle);
        const double c = std::cos(angle);
        Vector3 v1(
          - direction_i.x() * q_i * amplitude_i * s,
          - direction_i.y() * q_i * amplitude_i * s,
          + amplitude_i * c
        );

        mesh.point(vtx1) += v1;
      }
    }

    // Single wave update 
    // for (
    //   auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
    //   it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
    //   ++it.first, ++it.second)
    // {
    //   auto& vtx0 = *it.first;
    //   auto& vtx1 = *it.second;

    //   const Point3& p0 = initMesh.point(vtx0);
    //   Vector3 v0 = p0 - CGAL::ORIGIN;
    //   Vector3 v1 = CGAL::NULL_VECTOR;

    //   auto prj    = direction * v0;
    //   double ang  = CGAL::to_double(prj) * wavenumber - omega * _time + phase;
    //   double sang = std::sin(ang);
    //   double cang = std::cos(ang);
    //   v1 += Vector3(
    //     - direction.x() * q * amplitude * sang,
    //     - direction.y() * q * amplitude * sang,
    //     + amplitude * cang
    //   );
      
    //   mesh.point(vtx1) = p0 + v1;
    // }
  
  }

///////////////////////////////////////////////////////////////////////////////
// WavefieldOceanTilePrivate

  /// \internal
  /// \brief Private data for the WavefieldOceanTile.
  class WavefieldOceanTilePrivate
  {
    /// \brief Wave parameters
    public: std::shared_ptr<WaveParameters> params;

    /// \brief Ocean tile
    public: std::unique_ptr<OceanTile> oceanTile;

    /// \brief The current position of the wave field.
    public: std::shared_ptr<Grid> grid;

    /// \brief The current position of the wave field.
    public: std::unique_ptr<TriangulatedGrid> triangulatedGrid;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldOceanTile

  WavefieldOceanTile::~WavefieldOceanTile()
  {
  }

  WavefieldOceanTile::WavefieldOceanTile(
    const std::string& _name) :
    data(new WavefieldOceanTilePrivate())
  {
    gzmsg << "Constructing WavefieldOceanTile..." <<  std::endl;

    int N = 128;
    int NPlus1 = N + 1;
    double L = 256.0;
    double u = 5.0;

    // Wave parameters
    gzmsg << "Creating WaveParameters." <<  std::endl;
    this->data->params.reset(new WaveParameters());

    // OceanTile
    gzmsg << "Creating OceanTile." <<  std::endl;
    this->data->oceanTile.reset(new OceanTile(N, L, false));
    this->data->oceanTile->SetWindVelocity(u, 0.0);
    this->data->oceanTile->Create();
    this->data->oceanTile->Update(0.0);

    // Grid
    gzmsg << "Creating grid." <<  std::endl;
    this->data->grid.reset(new Grid({ L, L }, { static_cast<size_t>(NPlus1), static_cast<size_t>(NPlus1) }));
    
    // Point Locator
    gzmsg << "Creating triangulated grid." <<  std::endl;
    this->data->triangulatedGrid = std::move(TriangulatedGrid::Create(N, L));
    // this->data->TriangulatedGrid->DebugPrintTriangulation();
    
    // Update
    this->Update(0.0);

    gzmsg << "Done constructing WavefieldOceanTile." <<  std::endl;
  }

  std::shared_ptr<const Mesh> WavefieldOceanTile::GetMesh() const
  {
    return this->data->grid->GetMesh();
  }

  std::shared_ptr<const Grid> WavefieldOceanTile::GetGrid() const
  {
    return this->data->grid;
  }

  bool WavefieldOceanTile::Height(const Point3& point, double& height) const
  {
    return this->data->triangulatedGrid->Height(point, height);    
  }

  std::shared_ptr<const WaveParameters> WavefieldOceanTile::GetParameters() const
  {
    return this->data->params;
  }

  void WavefieldOceanTile::SetParameters(std::shared_ptr<WaveParameters> _params) const
  {
    GZ_ASSERT(_params != nullptr, "Invalid parameter _params");
    this->data->params = _params;    
  }

  void WavefieldOceanTile::Update(double _time)
  {
    // Update the tile.
    this->data->oceanTile->Update(_time);

    // Update the grid.
    auto& vertices = this->data->oceanTile->Vertices();
    auto& mesh = *this->data->grid->GetMesh();

    for (
      auto&& it = std::make_pair(std::begin(vertices), std::begin(mesh.vertices()));
      it.first != std::end(vertices) && it.second != std::end(mesh.vertices());
      ++it.first, ++it.second)
    {
      auto& vtx0 = *it.first;
      auto& vtx1 = *it.second;
      // Visual and physics out of phase? x-y transposed?
      mesh.point(vtx1) = Point3(vtx0.x, vtx0.y, vtx0.z);
    }

    // Update the point locator.
    this->data->triangulatedGrid->UpdatePoints(vertices);

  }

///////////////////////////////////////////////////////////////////////////////  

} // namespace asv
