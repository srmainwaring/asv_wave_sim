// Copyright (C) 2022  Rhys Mainwaring
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

#ifndef GZ_WAVES_LINEARRANDOMWAVESIMULATION_HH_
#define GZ_WAVES_LINEARRANDOMWAVESIMULATION_HH_

#include <Eigen/Dense>

#include <memory>

#include "gz/waves/WaveSimulation.hh"

namespace gz
{
namespace waves
{
/// \brief A non-FFT linear random wave simulation.
///
/// The model is provided for comparison with the LinearIncidentWave model
/// used in buoy_sim when the wave spectrum type is set to Pierson-Moskowitz.
///
/// Properties:
///   - Superposition of num_waves waves.
///   - Sampled at constant angular frequency: MaxOmega / NumWaves.
///   - Amplitudes determined by the Pierson-Moskowitz spectrum.
///   - Wave direction may be set.
///   - Waves do not spread, all waves propagate in the same direction.
///   - Waves are assigned random phases.
///
/// Performance estimates for various grid sizes:
///   num waves       nx x ny         RTF
///   100             128 x 128         6
///   300             128 x 128         2
///
class LinearRandomWaveSimulation :
    public IWaveSimulation,
    public IWaveField
{
 public:
  virtual ~LinearRandomWaveSimulation();

  LinearRandomWaveSimulation(double lx, double ly,
      Index nx, Index ny);

  LinearRandomWaveSimulation(double lx, double ly, double lz,
      Index nx, Index ny, Index nz);

  /// \brief The number of wave components.
  Index NumWaves() const;

  /// \brief Set the number of wave components (has default = 100).
  void SetNumWaves(Index value);

  /// \brief The maximum angular frequency rad/s).
  double MaxOmega() const;

  /// \brief Set the maximum angular frequency (has default = 6.0 (rad/s)).
  void SetMaxOmega(double value);

  void SetWindVelocity(double ux, double uy) override;

  void SetSteepness(double value) override;

  void SetTime(double value) override;

  Index SizeX() const override;

  Index SizeY() const override;

  Index SizeZ() const override;

  // IWaveField - interface.
  void Elevation(
      double x, double y,
      double &eta) const override;

  void Elevation(
      const Eigen::Ref<const Eigen::ArrayXd>& x,
      const Eigen::Ref<const Eigen::ArrayXd>& y,
      Eigen::Ref<Eigen::ArrayXd> eta) const override;

  void Pressure(
      double x, double y, double z,
      double& pressure) const override;

  void Pressure(
      const Eigen::Ref<const Eigen::ArrayXd>& x,
      const Eigen::Ref<const Eigen::ArrayXd>& y,
      const Eigen::Ref<const Eigen::ArrayXd>& z,
      Eigen::Ref<Eigen::ArrayXd> pressure) const override;

  // lookup interface - array
  void ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> h) const override;

  void ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy) const override;

  void DisplacementAt(
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy) const override;

  void DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy) const override;

  void DisplacementAndDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> h,
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy,
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy) const override;

  void PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure) const override;

  // lookup interface - scalar
  void ElevationAt(
      Index ix, Index iy,
      double& eta) const override;

  void DisplacementAt(
      Index ix, Index iy,
      double& sx, double& sy) const override;

  void PressureAt(
      Index ix, Index iy, Index iz,
      double& pressure) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_LINEARRANDOMWAVESIMULATION_HH_
