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

#include <gz/math.hh>

#include <gz/waves/WaveSimulation.hh>

namespace gz
{
namespace waves
{

class WaveParameters;
class WavefieldPrivate;

/// \brief A class to manage a wave field.
class Wavefield : public IWaveField1
{
 public:
  /// \brief Destructor.
  virtual ~Wavefield();

  /// \brief Constructor.
  ///
  /// \param[in] world_name
  ///   The name of the world containing the wavefield. Used to construct
  ///   a topic string for subscribing to parameter updates.
  ///   \todo(srmainwaring) replace with the topic string.
  explicit Wavefield(const std::string& world_name);

  /// \copydoc IWaveField1::Height()
  bool Height(const Eigen::Vector3d& point, double& height) const override;

  /// \copydoc IWaveField1::GetParameters()
  const WaveParameters& GetParameters() const override;

  /// \copydoc IWaveField1::SetParameters()
  void SetParameters(const WaveParameters& params) override;

  /// \copydoc IWaveField1::Update()
  void Update(double time) override;

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
