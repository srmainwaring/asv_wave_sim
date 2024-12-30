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

#include "gz/waves/Wavefield.hh"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>

#include "gz/waves/CGALTypes.hh"
#include "gz/waves/OceanTile.hh"
#include "gz/waves/TriangulatedGrid.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/Utilities.hh"
#include "gz/waves/WaveParameters.hh"

namespace gz
{
namespace waves
{
//////////////////////////////////////////////////
/// \internal Private implementation.
class WavefieldPrivate
{
 public:
  /// \brief Callback for topic "/world/<world>/waves".
  ///
  /// \param[in] msg Wave parameters message.
  void OnWaveMsg(const gz::msgs::Param& msg);

  /// \brief Wave parameters
  std::shared_ptr<WaveParameters> params_;

  /// \brief Ocean tile
  std::unique_ptr<physics::OceanTile> ocean_tile_;

  /// \brief The current position of the wave field.
  std::unique_ptr<TriangulatedGrid> tri_grid_;

  /// \brief Mutex to protect parameter updates.
  std::recursive_mutex mutex_;

  /// \brief Transport node
  transport::Node node_;
};

//////////////////////////////////////////////////
Wavefield::~Wavefield()
{
}

//////////////////////////////////////////////////
Wavefield::Wavefield(const std::string& world_name) :
  impl_(new WavefieldPrivate())
{
  gzmsg << "Constructing Wavefield..." <<  std::endl;

  // Subscribe to wave parameter updates
  std::string topic("/world/" + world_name + "/waves");
  impl_->node_.Subscribe(
      topic, &WavefieldPrivate::OnWaveMsg, impl_.get());

  // Wave parameters
  gzmsg << "Creating WaveParameters." <<  std::endl;
  auto params = std::make_shared<WaveParameters>();
  SetParameters(params);

  // Update
  Update(0.0);

  gzmsg << "Done constructing Wavefield." <<  std::endl;
}

//////////////////////////////////////////////////
bool Wavefield::Height(const Eigen::Vector3d& point, double& height) const
{
  /// \todo(srmainwaring) the calculation assumes that the tile origin
  /// is at its center.
  auto [lx, ly] = impl_->ocean_tile_->TileSize();

  // See this SO article covering the lambda capture of a structured binding.
  // https://stackoverflow.com/questions/46114214/lambda-implicit-capture-fails-with-variable-declared-from-structured-binding
  auto px_mod = [lx = lx](double x)
  {
      if (x < 0.0)
        return std::fmod(x - lx/2.0, lx) + lx/2.0;
      else
        return std::fmod(x + lx/2.0, lx) - lx/2.0;
  };

  auto py_mod = [ly = ly](double y)
  {
      if (y < 0.0)
        return std::fmod(y - ly/2.0, ly) + ly/2.0;
      else
        return std::fmod(y + ly/2.0, ly) - ly/2.0;
  };

  // Obtain the point modulo the tile dimensions
  cgal::Point3 modulo_point(px_mod(point.x()), py_mod(point.y()), point.z());

  return impl_->tri_grid_->Height(modulo_point, height);
}

//////////////////////////////////////////////////
std::shared_ptr<const WaveParameters> Wavefield::GetParameters() const
{
  return impl_->params_;
}

//////////////////////////////////////////////////
void Wavefield::SetParameters(std::shared_ptr<WaveParameters> params)
{
  impl_->params_ = params;

  // Force an update of the ocean tile and point locator
  auto [nx, ny] = impl_->params_->CellCount();
  auto [lx, ly] = impl_->params_->TileSize();
  double u = impl_->params_->WindVelocity().X();
  double v = impl_->params_->WindVelocity().Y();

  // OceanTile
  gzmsg << "Creating OceanTile.\n";
  impl_->ocean_tile_.reset(new physics::OceanTile(
      impl_->params_, false));
  impl_->ocean_tile_->SetWindVelocity(u, v);
  impl_->ocean_tile_->Create();

  // Point Locator
  gzmsg << "Creating triangulated grid.\n";
  auto grid = TriangulatedGrid::Create(nx, ny, lx, ly);
  impl_->tri_grid_ = std::move(grid);
}

//////////////////////////////////////////////////
void Wavefield::Update(double time)
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex_);

  // Update the tile.
  impl_->ocean_tile_->Update(time);

  // Update the point locator.
  auto& vertices = impl_->ocean_tile_->Vertices();
  impl_->tri_grid_->UpdatePoints(vertices);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
void WavefieldPrivate::OnWaveMsg(const gz::msgs::Param& msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // gzmsg << msg.DebugString();

  // current wind speed and angle
  double wind_speed = params_->WindSpeed();
  double wind_angle_rad = params_->WindAngleRad();
  double steepness = params_->Steepness();

  // extract parameters
  {
    auto it = msg.params().find("wind_speed");
    if (it != msg.params().end())
    {
      /// \todo: assert the type is double
      auto param = it->second;
      // auto type = param.type();
      auto value = param.double_value();
      wind_speed = value;
    }
  }
  {
    auto it = msg.params().find("wind_angle");
    if (it != msg.params().end())
    {
      /// \todo: assert the type is double
      auto param = it->second;
      // auto type = param.type();
      auto value = param.double_value();
      wind_angle_rad = M_PI/180.0*value;
    }
  }
  {
    auto it = msg.params().find("steepness");
    if (it != msg.params().end())
    {
      /// \todo: assert the type is double
      auto param = it->second;
      // auto type = param.type();
      auto value = param.double_value();
      steepness = value;
    }
  }

  // update parameters and wavefield
  params_->SetWindSpeedAndAngle(wind_speed, wind_angle_rad);
  params_->SetSteepness(steepness);

  ocean_tile_->SetWindVelocity(
      params_->WindVelocity().X(),
      params_->WindVelocity().Y());
  ocean_tile_->SetSteepness(
      params_->Steepness());
}

}  // namespace waves
}  // namespace gz
