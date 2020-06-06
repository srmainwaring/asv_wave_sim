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
/// \brief This file contains definitions for classes used to manage
/// a wave field. This includes wave parameters, wave generation, 
/// and sampling from the wave field. 

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>

#include <memory>

namespace asv
{
  class Grid;

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
    public: void FillMsg(gazebo::msgs::Param_V& _msg) const;

    /// \brief Set the parameters from a message.
    ///
    /// \param[in] _msg   The message containing the wave parameters.
    public: void SetFromMsg(const gazebo::msgs::Param_V& _msg);

    /// \brief Set the parameters from an SDF Element tree.
    ///
    /// \param[in] _sdf   The SDF Element tree containing the wave parameters.
    public: void SetFromSDF(sdf::Element& _sdf);

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
    public: Vector2 Direction() const;

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
    public: void SetDirection(const Vector2& _direction);

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
    public: const std::vector<Vector2>& Direction_V() const;

    /// \brief Print a summary of the wave parameters to the gzmsg stream.
    public: void DebugPrint() const;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WaveParametersPrivate> data;
  };

///////////////////////////////////////////////////////////////////////////////
// Wavefield

  /// \internal
  /// \brief Class to hold private data for Wavefield.
  class WavefieldPrivate;
  
  /// \brief A class to manage a wave field.
  class Wavefield
  {
    /// Destructor.
    public: ~Wavefield();

    /// Constructor.
    ///
    /// \param[in] _name    The name for the wave field mesh.
    public: Wavefield(const std::string& _name);

    /// Constructor.
    ///
    /// \param[in] _name      The name for the wave field mesh.
    /// \param[in] _size      A two component array defining the size of the wavefield [m].
    /// \param[in] _cellCount A two component array defining the number of cells in each direction.
    public: Wavefield(
      const std::string& _name,
      const std::array<double, 2>& _size,
      const std::array<size_t, 2>& _cellCount);

    /// \brief Access the wave field mesh.
    public: std::shared_ptr<const Mesh> GetMesh() const;

    /// \brief Access the wave field grid.
    public: std::shared_ptr<const Grid> GetGrid() const;

    /// \brief Access the wave field mesh as a Gazebo Mesh.
    // public: std::shared_ptr<const gazebo::common::Mesh> GetGzMesh() const;

    /// \brief Get the wave parameters.
    public: std::shared_ptr<const WaveParameters> GetParameters() const;

    /// \brief Set the wave parameters.
    ///
    /// \param[in] _params    The new wave parameters.
    public: void SetParameters(std::shared_ptr<WaveParameters> _params) const;

    /// \brief Update (recalculate) the wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    public: void Update(double _time);

    /// internal
    /// \brief Update (recalculate) a Gerstner wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    private: void UpdateGerstnerWave(double _time);

    /// internal
    /// \brief Initialise the Gazebo version of the wave field mesh.
    // private: void InitGzMesh();

    /// internal
    /// \brief Just-in-time update of the Gazebo version of the wave field mesh.
    // private: void LazyUpdateGzMesh() const;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldPrivate> data;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldSampler

  /// \internal
  /// \brief Class to hold private data for WavefieldSampler.
  class WavefieldSamplerPrivate;
  
  /// \brief A class to manage sampling depths from a wave field.
  class WavefieldSampler
  {
    /// \brief Destructor
    public: ~WavefieldSampler();

    /// \brief Constructor
    ///
    /// \param[in] _wavefield     The wave field being sampled.
    /// \param[in] _waterPatch    The area of the wave field being sampled.
    public: WavefieldSampler(
      std::shared_ptr<const Wavefield> _wavefield,
      std::shared_ptr<const Grid> _waterPatch);

    /// \brief Get the water patch (i.e. the area of the wave field sampled).
    public: std::shared_ptr<const Grid> GetWaterPatch() const;

    /// \brief Translate the initial water patch using the Pose X Y coordinates.
    ///
    /// \param[in] _pose    The pose of the rigid body the water patch supports.
    public: void ApplyPose(const ignition::math::Pose3d& _pose);

    /// \brief Update the water patch 
    public: void UpdatePatch();

    /// \brief Compute the depth at a point.
    ///
    /// \param[in] _point       The point at which we want the depth
    /// \return                 The depth 'h' at the point.
    public: double ComputeDepth(const Point3& _point) const;

    /// \brief Compute the depth at a point.
    ///
    /// \param[in] _patch       A water patch. 
    /// \param[in] _point       The point at which we want the depth
    /// \return                 The depth 'h' at the point.
    public: static double ComputeDepth(const Grid& _patch, const Point3& _point);

    /// \brief Compute the depth at a point directly (no sampling or interpolation).
    ///
    /// This method solves for (x, y) that when input into the Gerstner wave function
    /// gives the coordinates of the supplied parameter _point (_point.x(), _point.y()),
    /// and also computes the wave height pz at this point.
    /// The depth h = pz - point.z().  
    /// This is a numerical method that uses a multi-variate Newton solver to solve
    /// the two dimensional non-linear system. In general it is not as fast as
    /// sampling from a discretised wave field with an efficient line intersection
    /// algorithm.
    ///
    /// \param[in] _waveParams  Gerstner wave parameters. 
    /// \param[in] _point       The point at which we want the depth.
    /// \return                 The depth 'h' at the point.
    public: static double ComputeDepthDirectly(
      const WaveParameters& _waveParams,
      const Point3& _point,
      double time);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldSamplerPrivate> data;
  };

///////////////////////////////////////////////////////////////////////////////

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_
