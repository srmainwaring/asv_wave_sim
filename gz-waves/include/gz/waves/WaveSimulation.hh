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

using Eigen::ArrayXXd;

namespace gz
{
namespace waves
{
  /// \todo(srmainwaring) make interface = either split out interpolation
  ///       interface (preferred) or ensure all subclasses have implmentation. 
  class WaveSimulation
  {
  public:
    virtual ~WaveSimulation();

    WaveSimulation();

    virtual void SetWindVelocity(double ux, double uy) = 0;

    virtual void SetTime(double value) = 0;

    ///// interpolation interface
    virtual void Elevation(
        double x, double y,
        double &eta);

    virtual void Elevation(
        const Eigen::Ref<const Eigen::ArrayXd> &x,
        const Eigen::Ref<const Eigen::ArrayXd> &y,
        Eigen::Ref<Eigen::ArrayXd> eta);

    virtual void Pressure(
        double x, double y, double z,
        double &pressure);

    virtual void Pressure(
        const Eigen::Ref<const Eigen::ArrayXd> &x,
        const Eigen::Ref<const Eigen::ArrayXd> &y,
        const Eigen::Ref<const Eigen::ArrayXd> &z,
        Eigen::Ref<Eigen::ArrayXd> pressure);

    ///// lookup interface - array
    virtual void ElevationAt(
        Eigen::Ref<Eigen::ArrayXXd> h) = 0;

    virtual void ElevationDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy) = 0;

    virtual void DisplacementAt(
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy) = 0;

    virtual void DisplacementDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy) = 0;

    virtual void DisplacementAndDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> h,
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy,
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy) = 0;

    virtual void PressureAt(
        int iz,
        Eigen::Ref<Eigen::ArrayXXd> pressure);

    ///// lookup interface - scalar
    virtual void ElevationAt(
        int ix, int iy,
        double &eta);

    virtual void DisplacementAt(
        int ix, int iy,
        double &sx, double &sy);

    virtual void PressureAt(
        int ix, int iy, int iz,
        double &pressure);

  };
}
}

#endif
