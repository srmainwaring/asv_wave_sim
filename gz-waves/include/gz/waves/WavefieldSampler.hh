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

#ifndef GZ_WAVES_WAVEFIELDSAMPLER_HH_
#define GZ_WAVES_WAVEFIELDSAMPLER_HH_

#include <memory>

#include <gz/math/Pose3.hh>

#include "gz/waves/CGALTypes.hh"


namespace gz
{
namespace waves
{
class Grid;
class Wavefield;
class WaveParameters;

/// \internal
/// \brief Class to hold private data for WavefieldSampler.
class WavefieldSamplerPrivate;

/// \brief A class to manage sampling depths from a wave field.
class WavefieldSampler
{
 public:
  /// \brief Destructor
  ~WavefieldSampler();

  /// \brief Constructor
  ///
  /// \param[in] wavefield  The wave field being sampled.
  /// \param[in] patch      The area of the wave field being sampled.
  WavefieldSampler(
    std::shared_ptr<const Wavefield> wavefield,
    std::shared_ptr<const Grid> patch);

  /// \brief Get the water patch (i.e. the area of the wave field sampled).
  std::shared_ptr<const Grid> GetWaterPatch() const;

  /// \brief Translate the initial water patch using the Pose X Y coordinates.
  ///
  /// \param[in] pose     The pose of the rigid body the water patch supports.
  void ApplyPose(const math::Pose3d& pose);

  /// \brief Update the water patch
  void UpdatePatch();

  /// \brief Compute the depth at a point.
  ///
  /// \param[in] point       The point at which we want the depth
  /// \return                The depth 'h' at the point.
  double ComputeDepth(const cgal::Point3& point) const;

  /// \brief Compute the depth at a point.
  ///
  /// \param[in] patch       A water patch.
  /// \param[in] point       The point at which we want the depth
  /// \return                The depth 'h' at the point.
  static double ComputeDepth(const Grid& patch, const cgal::Point3& point);

  /// \brief Compute the depth at a point directly (no sampling).
  ///
  /// This method solves for (x, y) that when input into the Gerstner
  /// wave function gives the coordinates of the supplied parameter point
  /// (point.x(), point.y()), and also computes the wave height pz at this
  /// point. The depth h = pz - point.z().
  /// This is a numerical method that uses a multi-variate Newton solver
  /// to solve the two dimensional non-linear system. In general it is not
  /// as fast as sampling from a discretised wave field with an efficient
  /// line intersection algorithm.
  ///
  /// \param[in] wave_params  Gerstner wave parameters.
  /// \param[in] point       The point at which we want the depth.
  /// \return                 The depth 'h' at the point.
  static double ComputeDepthDirectly(
    const WaveParameters& wave_params,
    const cgal::Point3& point,
    double time);

 private:
  /// \internal Private implementation.
  std::shared_ptr<WavefieldSamplerPrivate> impl_;
};

typedef std::shared_ptr<WavefieldSampler> WavefieldSamplerPtr;

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_WAVEFIELDSAMPLER_HH_
