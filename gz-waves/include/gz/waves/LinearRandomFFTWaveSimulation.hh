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

#ifndef GZ_WAVES_LINEARRANDOMFFTWAVESIMULATION_HH_
#define GZ_WAVES_LINEARRANDOMFFTWAVESIMULATION_HH_

#include <memory>

#include "WaveSimulation.hh"

using Eigen::ArrayXXd;

namespace gz
{
namespace waves
{
  class LinearRandomFFTWaveSimulation : public WaveSimulation
  {
  public:
    virtual ~LinearRandomFFTWaveSimulation();

    LinearRandomFFTWaveSimulation(double lx, double ly,
        int nx, int ny);

    LinearRandomFFTWaveSimulation(double lx, double ly, double lz,
        int nx, int ny, int nz);

    /// \brief Set lambda which controls the horizontal wave displacement.
    void SetLambda(double lambda);

    virtual void SetWindVelocity(double ux, double uy) override;

    virtual void SetTime(double value) override;

    ///// interpolation interface - not yet directly supported for FFT.
    #if 0
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
    #endif

    ///// lookup interface - array
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
        int iz,
        Eigen::Ref<Eigen::ArrayXXd> pressure) override;

    ///// lookup interface - scalar
    virtual void ElevationAt(
        int ix, int iy,
        double &eta) override;

    virtual void DisplacementAt(
        int ix, int iy,
        double &sx, double &sy) override;

    virtual void PressureAt(
        int ix, int iy, int iz,
        double &pressure) override;

    // public class declaration - for testing
    class Impl;

  private:
    std::unique_ptr<Impl> impl_;
  };
}
}

#endif
