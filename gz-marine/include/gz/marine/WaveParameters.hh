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

#ifndef GZ_MARINE_WAVEPARAMETERS_HH_
#define GZ_MARINE_WAVEPARAMETERS_HH_

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs.hh>
#include <sdf/sdf.hh>

#include <memory>
#include <string>

namespace ignition
{
namespace marine
{

///////////////////////////////////////////////////////////////////////////////
// WaveParameters

  /// \internal
  /// \brief Class to hold private data for WaveParameters.
  class WaveParametersPrivate;

  /// \brief A class to manage the parameters for generating a wave in a wave field.
  class WaveParameters
  {
    /// \brief Destructor.
    public: ~WaveParameters();

    /// \brief Constructor.
    public: WaveParameters();

    /// \brief Populate the message with the wave parameters.
    ///
    /// \param[out] _msg  The message to be populated (a vector of parameters).
    public: void FillMsg(msgs::Param_V& _msg) const;

    /// \brief Set the parameters from a message.
    ///
    /// \param[in] _msg   The message containing the wave parameters.
    public: void SetFromMsg(const msgs::Param_V& _msg);

    /// \brief Set the parameters from an SDF Element tree.
    ///
    /// \param[in] _sdf   The SDF Element tree containing the wave parameters.
    public: void SetFromSDF(sdf::Element& _sdf);

    /// \brief The wave algorithm (options are: 'sinusoid', 'trochoid', 'fft').
    public: std::string Algorithm() const;

    /// \brief The size of the wave tile in metres (L).
    public: double TileSize() const;

    /// \brief The number of cells in the wave tile in each direction (N). 
    public: size_t CellCount() const;

    /// \brief The number of wave components (3 max if visualisation required).
    public: size_t Number() const;

    /// \brief The angle between the mean wave direction and the
    ///        largest / smallest component waves.
    public: double Angle() const;

    /// \brief The scale between the mean and largest / smallest
    ///        component waves.
    public: double Scale() const;

    /// \brief A parameter in [0, 1] controlling the wave steepness
    ///        with 1 being steepest.
    public: double Steepness() const;

    /// \brief The angular frequency 
    public: double AngularFrequency() const;

    /// \brief The amplitude of the mean wave in [m].
    public: double Amplitude() const;

    /// \brief The period of the mean wave in [s].
    public: double Period() const;

    /// \brief The phase of the mean wave.
    public: double Phase() const;

    /// \brief The mean wavelength.
    public: double Wavelength() const;

    /// \brief The mean wavenumber.
    public: double Wavenumber() const;

    /// \brief A two component vector specifiying the direction of the mean wave.
    public: math::Vector2d Direction() const;

    /// \brief A two component vector specifiying the horizontal wind velocity.
    public: math::Vector2d WindVelocity() const;

    /// \brief The scalar wind speed [m/s].
    public: double WindSpeed() const;

    /// \brief The wind angle (counter clockwise from positive x-axis) [rad].
    public: double WindAngleRad() const;

    /// \brief Set the wave algorithm (options are: 'sinusoid', 'trochoid', 'fft').
    ///
    /// \param[in] _algorithm    The wave algorithm.
    public: void SetAlgorithm(const std::string &_algorithm);

    /// \brief Set the size of the wave tile in metres (L).
    ///
    /// \param[in] _L    The size of the wave tile.
    public: void SetTileSize(double _L);

    /// \brief Set the number of cells in the wave tile in each direction (N). 
    ///
    /// \param[in] _N    The number of cells.
    public: void SetCellCount(size_t _N);

    /// \brief Set the number of wave components (3 max).
    ///
    /// \param[in] _number    The number of component waves.
    public: void SetNumber(size_t _number);

    /// \brief Set the angle parameter controlling 
    /// the direction of the component waves.
    ///
    /// \param[in] _angle     The angle parameter.
    public: void SetAngle(double _angle);

    /// \brief Set the scale parameter controlling
    /// the range of amplitudes of the component waves.
    ///
    /// \param[in] _scale   The scale parameter.
    public: void SetScale(double _scale);

    /// \brief Set the steepness parameter controlling
    /// the steepness of the waves. In [0, 1].
    ///
    /// \param[in] _steepness The steepness parameter.
    public: void SetSteepness(double _steepness);

    /// \brief Set the mean wave amplitude. Must be positive.
    ///
    /// \param[in] _amplitude The amplitude parameter.
    public: void SetAmplitude(double _amplitude);

    /// \brief Set the mean wave period. Must be positive.
    ///
    /// \param[in] _period The period parameter.
    public: void SetPeriod(double _period);

    /// \brief Set the mean wave phase.
    ///
    /// \param[in] _phase The phase parameter.
    public: void SetPhase(double _phase);

    /// \brief Set the mean wave direction.
    ///
    /// \param[in] _direction The direction parameter, a two component vector.
    public: void SetDirection(const math::Vector2d& _direction);

    /// \brief Set the horizontal wind velocity.
    ///
    /// \param[in] _windVelocity The wind velocity, a two component vector.
    public: void SetWindVelocity(const math::Vector2d& _windVelocity);

    /// \brief Access the component angular frequencies.
    public: const std::vector<double>& AngularFrequency_V() const;

    /// \brief Access the component amplitudes.
    public: const std::vector<double>& Amplitude_V() const;

    /// \brief Access the component phases.
    public: const std::vector<double>& Phase_V() const;

    /// \brief Access the steepness components.
    public: const std::vector<double>& Steepness_V() const;

    /// \brief Access the component wavenumbers.
    public: const std::vector<double>& Wavenumber_V() const;

    /// \brief Access the component directions.
    public: const std::vector<math::Vector2d>& Direction_V() const;

    /// \brief Print a summary of the wave parameters to the ignmsg stream.
    public: void DebugPrint() const;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WaveParametersPrivate> data;
  };

  typedef std::shared_ptr<WaveParameters> WaveParametersPtr; 

///////////////////////////////////////////////////////////////////////////////

}
}

#endif
