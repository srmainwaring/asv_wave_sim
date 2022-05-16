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

#include "ignition/marine/Wavefield.hh"
#include "ignition/marine/CGALTypes.hh"
#include "ignition/marine/WaveParameters.hh"

#include "ignition/marine/OceanTile.hh"
#include "ignition/marine/TriangulatedGrid.hh"

#include "ignition/marine/Utilities.hh"
#include <thread>

#include <ignition/transport.hh>

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
    /// \brief Callback for topic "/world/<world>/waves".
    ///
    /// \param[in] _msg Wave parameters message.
    public: void OnWaveMsg(const ignition::msgs::Param &_msg);

    /// \brief Wave parameters
    public: std::shared_ptr<WaveParameters> params;

    /// \brief Ocean tile
    public: std::unique_ptr<physics::OceanTile> oceanTile;

    /// \brief The current position of the wave field.
    public: std::unique_ptr<TriangulatedGrid> triangulatedGrid;

    /// \brief Mutex to protect parameter updates.
    public: std::recursive_mutex mutex;

    /// \brief Transport node
    public: transport::Node node;
  };

  /////////////////////////////////////////////////
  Wavefield::~Wavefield()
  {
  }

  /////////////////////////////////////////////////
  Wavefield::Wavefield(const std::string &_worldName) :
    dataPtr(new WavefieldPrivate())
  {
    ignmsg << "Constructing Wavefield..." <<  std::endl;

    // Subscribe to wave parameter updates
    std::string topic("/world/" + _worldName + "/waves");
    this->dataPtr->node.Subscribe(
        topic, &WavefieldPrivate::OnWaveMsg, this->dataPtr.get());

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
    const double L = this->dataPtr->oceanTile->TileSize();
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

    return this->dataPtr->triangulatedGrid->Height(moduloPoint, height);
  }

  /////////////////////////////////////////////////
  std::shared_ptr<const WaveParameters> Wavefield::GetParameters() const
  {
    return this->dataPtr->params;
  }

  /////////////////////////////////////////////////
  void Wavefield::SetParameters(std::shared_ptr<WaveParameters> _params)
  {
    this->dataPtr->params = _params;

    // Force an update of the ocean tile and point locator
    size_t N = this->dataPtr->params->CellCount();
    double L = this->dataPtr->params->TileSize();
    double u = this->dataPtr->params->WindVelocity().X();
    double v = this->dataPtr->params->WindVelocity().Y();

    // OceanTile
    ignmsg << "Creating OceanTile." <<  std::endl;
    this->dataPtr->oceanTile.reset(new physics::OceanTile(
        this->dataPtr->params, false));
    this->dataPtr->oceanTile->SetWindVelocity(u, v);
    this->dataPtr->oceanTile->Create();

    // Point Locator
    ignmsg << "Creating triangulated grid." <<  std::endl;
    this->dataPtr->triangulatedGrid = std::move(TriangulatedGrid::Create(N, L));
  }

  /////////////////////////////////////////////////
  void Wavefield::Update(double _time)
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

    // Update the tile.
    this->dataPtr->oceanTile->Update(_time);

    // Update the point locator.
    auto& vertices = this->dataPtr->oceanTile->Vertices();
    this->dataPtr->triangulatedGrid->UpdatePoints(vertices);
  }

  /////////////////////////////////////////////////
  //////////////////////////////////////////////////
  void WavefieldPrivate::OnWaveMsg(const ignition::msgs::Param &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->mutex);

    // ignmsg << _msg.DebugString();

    // current wind speed and angle
    double windSpeed = this->params->WindSpeed();
    double windAngleRad = this->params->WindAngleRad();

    // extract parameters
    {
      auto it = _msg.params().find("wind_speed");
      if (it != _msg.params().end())
      {
        /// \todo: assert the type is double
        auto param = it->second;
        auto type = param.type();
        auto value = param.double_value();
        windSpeed = value;
      }
    }
    {
      auto it = _msg.params().find("wind_angle");
      if (it != _msg.params().end())
      {
        /// \todo: assert the type is double
        auto param = it->second;
        auto type = param.type();
        auto value = param.double_value();
        windAngleRad = M_PI/180.0*value;
      }
    }

    /// \todo: update params correctly - put logic in one place
    // update wind velocity
    double ux = windSpeed * cos(windAngleRad);
    double uy = windSpeed * sin(windAngleRad);
    
    // update parameters and wavefield
    this->params->SetWindVelocity(math::Vector2d(ux, uy));
    this->oceanTile->SetWindVelocity(ux, uy);
  }

  /////////////////////////////////////////////////
  // void Wavefield::OnWaveWindMsg(ConstParam_VPtr &_msg)
  // {
  //   std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  //   // Get parameters from message
  //   double wind_angle = 0.0;
  //   double wind_speed = 0.0;
  //   wind_angle = Utilities::MsgParamDouble(*_msg, "wind_angle", wind_angle);
  //   wind_speed = Utilities::MsgParamDouble(*_msg, "wind_speed", wind_speed);

  //   // Convert from polar to cartesian
  //   double wind_vel_x = wind_speed * std::cos(wind_angle);
  //   double wind_vel_y = wind_speed * std::sin(wind_angle);

  //   // @DEBUG_INFO
  //   gzmsg << "Wavefield received message on topic ["
  //     << this->dataPtr->waveWindSub->GetTopic() << "]" << std::endl;
  //   gzmsg << "wind_angle: " << wind_angle << std::endl;
  //   gzmsg << "wind_speed: " << wind_speed << std::endl;
  //   gzmsg << "wind_vel_x: " << wind_vel_x << std::endl;
  //   gzmsg << "wind_vel_y: " << wind_vel_y << std::endl;

  //   // Update simulation
  //   this->dataPtr->oceanTile->SetWindVelocity(wind_vel_x, wind_vel_y);
  // }

  /////////////////////////////////////////////////

}
}
