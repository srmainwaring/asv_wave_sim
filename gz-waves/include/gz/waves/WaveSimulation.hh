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

#ifndef GZ_WAVES_WAVESIMULATION_HH_
#define GZ_WAVES_WAVESIMULATION_HH_

#include <Eigen/Dense>

using Eigen::MatrixXd;

namespace gz
{
namespace waves
{
  class WaveSimulation
  {
  public:
    virtual ~WaveSimulation();

    WaveSimulation();

    virtual void SetWindVelocity(double ux, double uy) = 0;

    virtual void SetTime(double value) = 0;

    virtual void ComputeElevation(
        Eigen::Ref<Eigen::MatrixXd> h) = 0;

    virtual void ComputeElevationDerivatives(
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy) = 0;

    virtual void ComputeDisplacements(
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy) = 0;

    virtual void ComputeDisplacementsDerivatives(
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy) = 0;

    virtual void ComputeDisplacementsAndDerivatives(
        Eigen::Ref<Eigen::MatrixXd> h,
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy,
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy,
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy) = 0;

    // virtual double ComputePressureAt(int ix, int iy, int iz) {}

    // virtual double ComputePressureAt(
    //     Eigen::Ref<Eigen::MatrixXd> pressure,
    //     int iz) {}
  };
}
}

#endif
