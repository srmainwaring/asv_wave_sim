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

/// \file Wavefield.hh
/// \brief A class to interact with a wave field.

#ifndef GZ_WAVES_WAVEFIELD_HH_
#define GZ_WAVES_WAVEFIELD_HH_

#include <Eigen/Dense>

#include <memory>
#include <string>

namespace gz
{
namespace waves
{
class WaveParameters;
class WavefieldPrivate;

/// \brief A class to manage a wave field.
class Wavefield
{
 public:
  /// \brief Destructor.
  virtual ~Wavefield();

  /// \brief Constructor.
  explicit Wavefield(const std::string& world_name);

  /// \brief Compute the height at a point (vertical distance to surface).
  bool Height(const Eigen::Vector3d& point, double& height) const;

  /// \brief Get the wave parameters.
  std::shared_ptr<const WaveParameters> GetParameters() const;

  /// \brief Set the wave parameters.
  ///
  /// \param[in] params    The new wave parameters.
  void SetParameters(std::shared_ptr<WaveParameters> params);

  /// \brief Update (recalculate) the wave field for the given time.
  ///
  /// \param[in] time    The time parameter for the wave evolution.
  void Update(double time);

 private:
  /// \internal Private implementation.
  std::shared_ptr<WavefieldPrivate> impl_;
};

typedef std::shared_ptr<Wavefield> WavefieldPtr;
typedef std::shared_ptr<const Wavefield> WavefieldConstPtr;
typedef std::weak_ptr<Wavefield> WavefieldWeakPtr;
typedef std::weak_ptr<const Wavefield> WavefieldConstWeakPtr;
}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_WAVEFIELD_HH_
