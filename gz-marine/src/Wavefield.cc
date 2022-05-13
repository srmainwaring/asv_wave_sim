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

#include "gz/marine/Wavefield.hh"
#include "gz/marine/CGALTypes.hh"
#include "gz/marine/WaveParameters.hh"

#include "gz/marine/OceanTile.hh"
#include "gz/marine/TriangulatedGrid.hh"

#include "gz/marine/Utilities.hh"
#include <thread>

#include <gz/transport.hh>

#include <array>
#include <cmath>
#include <iostream>
#include <string>

namespace ignition
{
namespace marine
{
  /////////////////////////////////////////////////
  /// \internal
  /// \brief Private data for the Wavefield.
  class WavefieldPrivate
  {
    /// \brief Wave parameters
    public: std::shared_ptr<WaveParameters> params;

    /// \brief Ocean tile
    public: std::unique_ptr<physics::OceanTile> oceanTile;

    /// \brief The current position of the wave field.
    public: std::unique_ptr<TriangulatedGrid> triangulatedGrid;

    /// \todo(srmainwaring): port to ignition
    public: std::recursive_mutex mutex;
    // public: ignition::transport::Node node;
    // public: ignition::transport::SubscriberPtr waveWindSub;
  };

  /////////////////////////////////////////////////
  Wavefield::~Wavefield()
  {
    /// \todo(srmainwaring): port to ignition
    // this->data->waveWindSub.reset();
    // this->data->node.reset();
  }

  /////////////////////////////////////////////////
  Wavefield::Wavefield() :
    data(new WavefieldPrivate())
  {
    ignmsg << "Constructing Wavefield..." <<  std::endl;

    /// \todo(srmainwaring): port to ignition
    // this->data->node;
    // this->data->waveWindSub = this->data->node.Subscribe(
    //   "~/wave/wind", &Wavefield::OnWaveWindMsg, this);

    // Wave parameters
    ignmsg << "Creating WaveParameters." <<  std::endl;
    auto params = std::make_shared<WaveParameters>();
    this->SetParameters(params);

    // Update
    this->Update(0.0);

    ignmsg << "Done constructing Wavefield." <<  std::endl;
  }

  /////////////////////////////////////////////////
  bool Wavefield::Height(const cgal::Point3& point, double& height) const
  {
    /// \todo(srmainwaring) the calculation assumes that the tile origin is at its center.
    const double L = this->data->oceanTile->TileSize();
    const double LOver2 = L/2.0;

    auto pmod = [&](double x)
    {
        if (x < 0.0)
          return std::fmod(x - LOver2, L) + LOver2;
        else
          return std::fmod(x + LOver2, L) - LOver2;
    };

    // Obtain the point modulo the tile dimensions
    cgal::Point3 moduloPoint(pmod(point.x()), pmod(point.y()), point.z());

    return this->data->triangulatedGrid->Height(moduloPoint, height);
  }

  /////////////////////////////////////////////////
  std::shared_ptr<const WaveParameters> Wavefield::GetParameters() const
  {
    return this->data->params;
  }

  /////////////////////////////////////////////////
  void Wavefield::SetParameters(std::shared_ptr<WaveParameters> _params) const
  {
    this->data->params = _params;

    // Force an update of the ocean tile and point locator
    size_t N = this->data->params->CellCount();
    double L = this->data->params->TileSize();
    double u = this->data->params->WindVelocity().X();
    double v = this->data->params->WindVelocity().Y();

    // OceanTile
    ignmsg << "Creating OceanTile." <<  std::endl;
    this->data->oceanTile.reset(new physics::OceanTile(
        this->data->params, false));
    this->data->oceanTile->SetWindVelocity(u, v);
    this->data->oceanTile->Create();

    // Point Locator
    ignmsg << "Creating triangulated grid." <<  std::endl;
    this->data->triangulatedGrid = std::move(TriangulatedGrid::Create(N, L));
  }

  /////////////////////////////////////////////////
  void Wavefield::Update(double _time)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // Update the tile.
    this->data->oceanTile->Update(_time);

    // Update the point locator.
    auto& vertices = this->data->oceanTile->Vertices();
    this->data->triangulatedGrid->UpdatePoints(vertices);
  }

#if 0
  /// \todo(srmainwaring): port to ignition
  void Wavefield::OnWaveWindMsg(ConstParam_VPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // Get parameters from message
    double wind_angle = 0.0;
    double wind_speed = 0.0;
    wind_angle = Utilities::MsgParamDouble(*_msg, "wind_angle", wind_angle);
    wind_speed = Utilities::MsgParamDouble(*_msg, "wind_speed", wind_speed);

    // Convert from polar to cartesian
    double wind_vel_x = wind_speed * std::cos(wind_angle);
    double wind_vel_y = wind_speed * std::sin(wind_angle);

    // @DEBUG_INFO
    gzmsg << "Wavefield received message on topic ["
      << this->data->waveWindSub->GetTopic() << "]" << std::endl;
    gzmsg << "wind_angle: " << wind_angle << std::endl;
    gzmsg << "wind_speed: " << wind_speed << std::endl;
    gzmsg << "wind_vel_x: " << wind_vel_x << std::endl;
    gzmsg << "wind_vel_y: " << wind_vel_y << std::endl;

    // Update simulation
    this->data->oceanTile->SetWindVelocity(wind_vel_x, wind_vel_y);
  }
#endif
  /////////////////////////////////////////////////

}
}
