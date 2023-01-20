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

/// \file WaveParameters.hh

#ifndef GZ_WAVES_WAVEPARAMETERS_HH_
#define GZ_WAVES_WAVEPARAMETERS_HH_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs.hh>
#include <sdf/sdf.hh>

#include "gz/waves/Types.hh"

namespace gz
{
namespace waves
{

/// \internal Private implementation.
class WaveParametersPrivate;

/// \brief Parameters for generating a wave in a wave field.
class WaveParameters
{
 public:
  /// \brief Destructor.
  ~WaveParameters();

  /// \brief Constructor.
  WaveParameters();

  /// \brief Populate the message with the wave parameters.
  ///
  /// \param[out] msg  The message to be populated (a vector of parameters).
  void FillMsg(msgs::Param_V& msg) const;

  /// \brief Set the parameters from a message.
  ///
  /// \param[in] msg   The message containing the wave parameters.
  void SetFromMsg(const msgs::Param_V& msg);

  /// \brief Set the parameters from an SDF Element tree.
  ///
  /// \param[in] sdf   The SDF Element tree containing the wave parameters.
  void SetFromSDF(sdf::Element& sdf);

  /// \brief The wave algorithm (options are: 'sinusoid', 'trochoid', 'fft').
  std::string Algorithm() const;

  /// \brief The size of the wave tile (m).
  std::array<double, 2> TileSize() const;

  /// \brief The number of cells in the wave tile in each direction.
  std::array<Index, 2> CellCount() const;

  /// \brief The number of wave components.
  Index Number() const;

  /// \brief The angle between the mean wave direction and the
  ///        largest / smallest component waves.
  double Angle() const;

  /// \brief The scale between the mean and largest / smallest
  ///        component waves.
  double Scale() const;

  /// \brief A parameter in [0, 1] controlling the wave steepness
  ///        with 1 being steepest.
  double Steepness() const;

  /// \brief The angular frequency (rad/s)
  double AngularFrequency() const;

  /// \brief The amplitude of the mean wave (m).
  double Amplitude() const;

  /// \brief The period of the mean wave (s).
  double Period() const;

  /// \brief The phase of the mean wave.
  double Phase() const;

  /// \brief The mean wavelength (m).
  double Wavelength() const;

  /// \brief The mean angular wavenumber (rad/m).
  double Wavenumber() const;

  /// \brief A two component vector specifiying the direction
  ///        of the mean wave.
  math::Vector2d Direction() const;

  /// \brief A two component vector specifiying the horizontal
  ///        wind velocity (m/s).
  math::Vector2d WindVelocity() const;

  /// \brief The scalar wind speed (m/s).
  double WindSpeed() const;

  /// \brief The wind angle (counter clockwise from
  ///        positive x-axis) (rad).
  double WindAngleRad() const;

  /// \brief Set the wave algorithm (options are: 'sinusoid',
  ///        'trochoid', 'fft').
  ///
  /// \param[in] value  The wave algorithm.
  void SetAlgorithm(const std::string& value);

  /// \brief Set the size of the wave tile (m).
  ///
  /// \param[in] value  The size of the wave tile in both directions (m).
  void SetTileSize(double value);

  /// \brief Set the size of the wave tile (m).
  ///
  /// \param[in] lx  The size of the wave tile in the x-direction (m).
  /// \param[in] ly  The size of the wave tile in the y-direction (m).
  void SetTileSize(double lx, double ly);

  /// \brief Set the number of cells in the wave tile
  ///        in each direction.
  ///
  /// \param[in] value  The number of cells in both directions.
  void SetCellCount(Index value);

  /// \brief Set the number of cells in the wave tile
  ///        in each direction.
  ///
  /// \param[in] nx  The number of cells in the x-direction.
  /// \param[in] ny  The number of cells in the y-direction.
  void SetCellCount(Index nx, Index ny);

  /// \brief Set the number of wave components (3 max).
  ///
  /// \param[in] value  The number of component waves.
  void SetNumber(Index value);

  /// \brief Set the angle parameter controlling
  /// the direction of the component waves.
  ///
  /// \param[in] value  The angle parameter.
  void SetAngle(double value);

  /// \brief Set the scale parameter controlling
  /// the range of amplitudes of the component waves.
  ///
  /// \param[in] value  The scale parameter.
  void SetScale(double value);

  /// \brief Set the steepness parameter controlling
  /// the steepness of the waves. In [0, 1].
  ///
  /// \param[in] value  The steepness parameter.
  void SetSteepness(double value);

  /// \brief Set the mean wave amplitude (m). Must be positive.
  ///
  /// \param[in] value  The amplitude parameter (m).
  void SetAmplitude(double value);

  /// \brief Set the mean wave period (s). Must be positive.
  ///
  /// \param[in] value  The period parameter (s).
  void SetPeriod(double value);

  /// \brief Set the mean wave phase.
  ///
  /// \param[in] value  The phase parameter.
  void SetPhase(double value);

  /// \brief Set the mean wave direction.
  ///
  /// \param[in] value  The direction parameter,
  ///            a two component vector.
  void SetDirection(const math::Vector2d& value);

  /// \brief Set the horizontal wind velocity.
  ///
  /// \param[in] value  The wind velocity, a two component vector.
  void SetWindVelocity(const math::Vector2d& value);

  /// \brief Set the scalar wind speed and downwind angle
  ///        at 10m above MSL (m/s).
  ///
  /// \param[in] wind_speed     The wind speed (m/s).
  /// \param[in] wind_angle_rad The downwind angle (rad).
  void SetWindSpeedAndAngle(double wind_speed, double wind_angle_rad);

  /// \brief Access the component angular frequencies (rad/s).
  const std::vector<double>& AngularFrequency_V() const;

  /// \brief Access the component amplitudes (m).
  const std::vector<double>& Amplitude_V() const;

  /// \brief Access the component phases.
  const std::vector<double>& Phase_V() const;

  /// \brief Access the steepness components.
  const std::vector<double>& Steepness_V() const;

  /// \brief Access the component wavenumbers (rad/m).
  const std::vector<double>& Wavenumber_V() const;

  /// \brief Access the component directions.
  const std::vector<math::Vector2d>& Direction_V() const;

  /// \brief Print a summary of the wave parameters to the msg stream.
  void DebugPrint() const;

 private:
  /// \internal Private implementation.
  std::shared_ptr<WaveParametersPrivate> impl_;
};

typedef std::shared_ptr<WaveParameters> WaveParametersPtr;

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_WAVEPARAMETERS_HH_
