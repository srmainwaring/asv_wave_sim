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

/// \file WavefieldSampler.hh

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_SAMPLER_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_SAMPLER_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <ignition/math/Pose3.hh>

#include <memory>

///////////////////////////////////////////////////////////////////////////////
// Forward Declarations

namespace asv
{
  class Grid;
  class Wavefield;
  class WaveParameters;

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

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_SAMPLER_HH_
