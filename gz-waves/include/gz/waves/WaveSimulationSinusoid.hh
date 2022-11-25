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

#ifndef GZ_WAVES_WAVESIMULATIONSINUSOID_HH_
#define GZ_WAVES_WAVESIMULATIONSINUSOID_HH_

#include <memory>

#include <Eigen/Dense>

#include "WaveSimulation.hh"

using Eigen::MatrixXd;

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
  class WaveSimulationSinusoid : public WaveSimulation
  {
  public:
    ~WaveSimulationSinusoid();

    WaveSimulationSinusoid(double lx, double ly,
        int nx, int ny);

    WaveSimulationSinusoid(double lx, double ly, double lz,
        int nx, int ny, int nz);

    void SetUseVectorised(bool value);

    void SetDirection(double dir_x, double dir_y);

    void SetAmplitude(double value);

    void SetPeriod(double value);

    Eigen::VectorXd X() const;

    Eigen::VectorXd Y() const;

    Eigen::VectorXd Z() const;

    virtual void SetWindVelocity(double ux, double uy) override;

    virtual void SetTime(double value) override;

    virtual void ComputeElevation(
        Eigen::Ref<Eigen::MatrixXd> h) override;

    virtual void ComputeElevationDerivatives(
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy) override;

    virtual void ComputeDisplacements(
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy) override;

    virtual void ComputeDisplacementsDerivatives(
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy) override;

    virtual void ComputeDisplacementsAndDerivatives(
        Eigen::Ref<Eigen::MatrixXd> h,
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy,
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy,
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy) override;

    virtual void ComputePressureAt(
        Eigen::Ref<Eigen::MatrixXd> pressure,
        int iz);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
  };
}
}

#endif
