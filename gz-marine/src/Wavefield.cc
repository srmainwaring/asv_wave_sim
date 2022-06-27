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

namespace gz
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
    public: void OnWaveMsg(const gz::msgs::Param &_msg);

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
    gzmsg << "Constructing Wavefield..." <<  std::endl;

    // Subscribe to wave parameter updates
    std::string topic("/world/" + _worldName + "/waves");
    this->dataPtr->node.Subscribe(
        topic, &WavefieldPrivate::OnWaveMsg, this->dataPtr.get());

    // Wave parameters
    gzmsg << "Creating WaveParameters." <<  std::endl;
    auto params = std::make_shared<WaveParameters>();
    this->SetParameters(params);

    // Update
    this->Update(0.0);

    gzmsg << "Done constructing Wavefield." <<  std::endl;
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
    gzmsg << "Creating OceanTile." <<  std::endl;
    this->dataPtr->oceanTile.reset(new physics::OceanTile(
        this->dataPtr->params, false));
    this->dataPtr->oceanTile->SetWindVelocity(u, v);
    this->dataPtr->oceanTile->Create();

    // Point Locator
    gzmsg << "Creating triangulated grid." <<  std::endl;
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
  void WavefieldPrivate::OnWaveMsg(const gz::msgs::Param &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->mutex);

    // gzmsg << _msg.DebugString();

    // // Get parameters from message
    // double wind_angle = 0.0;
    // double wind_speed = 0.0;
    // wind_angle = Utilities::MsgParamDouble(*_msg, "wind_angle", wind_angle);
    // wind_speed = Utilities::MsgParamDouble(*_msg, "wind_speed", wind_speed);

    // current wind speed and angle
    double windSpeed = this->params->WindSpeed();
    double windAngleRad = this->params->WindAngleRad();
    double steepness = this->params->Steepness();

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
    {
      auto it = _msg.params().find("steepness");
      if (it != _msg.params().end())
      {
        /// \todo: assert the type is double
        auto param = it->second;
        auto type = param.type();
        auto value = param.double_value();
        steepness = value;
      }
    }

    // update parameters and wavefield
    this->params->SetWindSpeedAndAngle(windSpeed, windAngleRad);
    this->params->SetSteepness(steepness);

    this->oceanTile->SetWindVelocity(
        this->params->WindVelocity().X(),
        this->params->WindVelocity().Y());
  }

  /////////////////////////////////////////////////

}
}
