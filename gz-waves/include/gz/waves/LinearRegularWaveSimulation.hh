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

#ifndef GZ_WAVES_LINEARREGULARWAVESIMULATION_HH_
#define GZ_WAVES_LINEARREGULARWAVESIMULATION_HH_

#include <memory>

#include <Eigen/Dense>

#include "WaveSimulation.hh"

using Eigen::ArrayXXd;

namespace gz
{
namespace waves
{
  /// The grid has sides with lengths lx and ly.
  ///
  /// There are nx, ny vertices in each direction.
  ///
  /// The simulation updates nx x ny vertices.
  ///
  /// The distance between vertices is dx = lx / nx and dy = ly / ny.
  ///
  /// All storage is assumed to be sized to nx x ny and the
  /// vertices are traversed in column major order:
  /// i.e. the innermost loop is over the x direction.
  ///
  class LinearRegularWaveSimulation :
      public IWaveSimulation,
      public IWaveField
  {
  public:
    ~LinearRegularWaveSimulation();

    LinearRegularWaveSimulation(double lx, double ly,
        Index nx, Index ny);

    LinearRegularWaveSimulation(double lx, double ly, double lz,
        Index nx, Index ny, Index nz);

    void SetUseVectorised(bool value);

    void SetDirection(double dir_x, double dir_y);

    void SetAmplitude(double value);

    void SetPeriod(double value);

    virtual void SetWindVelocity(double ux, double uy) override;

    virtual void SetTime(double value) override;

    // interpolation interface
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
