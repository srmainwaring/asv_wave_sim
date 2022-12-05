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

#include <memory>

#include "WaveSimulation.hh"

using Eigen::ArrayXXd;

namespace gz
{
namespace waves
{
  /// \brief A non-FFT linear random wave simulation.
  ///
  /// Model provided for comparison with the LinearIncidentWave model
  /// used in buoy_sim with the wave spectrum type set to Pierson-Moskowitz.
  ///
  /// Properties:
  ///   - superposition of num_waves waves.
  ///   - sampled at constant angular frequency steps delta omega.
  ///   - amplitude determined by Pierson-Moskowitz spectrum.
  ///   - wave direction may be set.
  ///   - waves do not spread, all waves propagate in the same direction.
  ///   - waves have random phases.
  ///
  /// Usage:
  ///   - performance is poor on grids with resolution greater than 64 x 64.
  ///
  /// \todo(srmainwaring)
  ///   - elevation derivatives
  ///   - pressure calculations
  ///   - orientation checks
  ///   - tests
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

    /// \brief Set the number of wave components (has default = 30).
    void SetNumWaves(Index value);

    /// \brief The maximum angular frequency rad/s).
    double MaxOmega() const;

    /// \brief Set the maximum angular frequency (has default = 6.0 (rad/s)).
    void SetMaxOmega(double value);

    virtual void SetWindVelocity(double ux, double uy) override;

    virtual void SetTime(double value) override;

    virtual Index SizeX() const override;

    virtual Index SizeY() const override;

    virtual Index SizeZ() const override;

    // IWaveField - interface.
    virtual void Elevation(
        double x, double y,
        double &eta) override;

    virtual void Elevation(
        const Eigen::Ref<const Eigen::ArrayXd> &x,
        const Eigen::Ref<const Eigen::ArrayXd> &y,
        Eigen::Ref<Eigen::ArrayXd> eta) override;

    virtual void Pressure(
        double x, double y, double z,
        double &pressure) override;

    virtual void Pressure(
        const Eigen::Ref<const Eigen::ArrayXd> &x,
        const Eigen::Ref<const Eigen::ArrayXd> &y,
        const Eigen::Ref<const Eigen::ArrayXd> &z,
        Eigen::Ref<Eigen::ArrayXd> pressure) override;

    // lookup interface - array
    virtual void ElevationAt(
        Eigen::Ref<Eigen::ArrayXXd> h) override;

    virtual void ElevationDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy) override;

    virtual void DisplacementAt(
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy) override;

    virtual void DisplacementDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy) override;

    virtual void DisplacementAndDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> h,
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy,
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy) override;

    virtual void PressureAt(
        Index iz,
        Eigen::Ref<Eigen::ArrayXXd> pressure) override;

    // lookup interface - scalar
    virtual void ElevationAt(
        Index ix, Index iy,
        double &eta) override;

    virtual void DisplacementAt(
        Index ix, Index iy,
        double &sx, double &sy) override;

    virtual void PressureAt(
        Index ix, Index iy, Index iz,
        double &pressure) override;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
  };
}
}

#endif
