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


#include "gz/waves/LinearRandomWaveSimulation.hh"

#include <complex>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include <gz/common.hh>

#include "gz/waves/WaveSpectrum.hh"
#include "gz/waves/WaveSpreadingFunction.hh"

namespace gz
{
namespace waves
{

  //////////////////////////////////////////////////
  class LinearRandomWaveSimulation::Impl
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~Impl();

    Impl(double lx, double ly, Index nx, Index ny);

    Impl(double lx, double ly, double lz, Index nx, Index ny, Index nz);

    void InitGrid();

    void SetTime(double value);

    // interpolation interface
    void Elevation(
        double x, double y,
        double &eta);

    void Elevation(
        const Eigen::Ref<const Eigen::ArrayXd> &x,
        const Eigen::Ref<const Eigen::ArrayXd> &y,
        Eigen::Ref<Eigen::ArrayXd> eta);

    void Pressure(
        double x, double y, double z,
        double &pressure);

    void Pressure(
        const Eigen::Ref<const Eigen::ArrayXd> &x,
        const Eigen::Ref<const Eigen::ArrayXd> &y,
        const Eigen::Ref<const Eigen::ArrayXd> &z,
        Eigen::Ref<Eigen::ArrayXd> pressure);

    // lookup interface
    void ElevationAt(
        Eigen::Ref<Eigen::ArrayXXd> h);

    void ElevationDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy);

    void DisplacementAndDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> h,
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy,
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy);

    void ElevationAt(
        Index ix, Index iy,
        double &h);

    void PressureAt(
        Index ix, Index iy, Index iz,
        double &pressure);

    void PressureAt(
        Index iz,
        Eigen::Ref<Eigen::ArrayXXd> pressure);

    static inline void PreComputeCoeff(
      double period, double t, double wave_angle,
      double &w, double &wt, double &k, double &cos_angle, double &sin_angle)
    {
      w = 2.0 * M_PI / period;
      wt = w * t;
      k = Physics::DeepWaterDispersionToWavenumber(w);
      cos_angle = std::cos(wave_angle);
      sin_angle = std::sin(wave_angle);
    }

    bool use_vectorised_{true};

    // elevation and pressure grid params
    Index nx_{2};
    Index ny_{2};
    Index nz_{1};
    double lx_{1.0};
    double ly_{1.0};
    double lz_{0.0};

    // wave params
    // double wave_angle_{0.0};
    // double amplitude_{1.0};
    // double period_{1.0};
    // double time_{0.0};

    // derived
    double dx_{0.0};
    double dy_{0.0};
    double lx_max_{0.0};
    double lx_min_{0.0};
    double ly_max_{0.0};
    double ly_min_{0.0};

    // vectorised
    Eigen::ArrayXd x_;
    Eigen::ArrayXd y_;
    Eigen::ArrayXXd x_grid_;
    Eigen::ArrayXXd y_grid_;
    Eigen::ArrayXd z_;
  };
  
  //////////////////////////////////////////////////
  LinearRandomWaveSimulation::Impl::~Impl() = default;

  //////////////////////////////////////////////////
  LinearRandomWaveSimulation::Impl::Impl(
    double lx, double ly, Index nx, Index ny) :
    lx_(lx),
    ly_(ly),
    nx_(nx),
    ny_(ny)
  {
  }

  //////////////////////////////////////////////////
  LinearRandomWaveSimulation::Impl::Impl(
    double lx, double ly, double lz, Index nx, Index ny, Index nz) :
    lx_(lx),
    ly_(ly),
    lz_(lz),
    nx_(nx),
    ny_(ny),
    nz_(nz)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::SetTime(double time)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> h)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::DisplacementAt(
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::ElevationAt(
      Index ix, Index iy,
      double &eta)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::DisplacementAt(
      Index ix, Index iy,
      double &sx, double &sy)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::PressureAt(
      Index ix, Index iy, Index iz,
      double &pressure)
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::InitWaveNumbers()
  {
    kx_fft_  = Eigen::ArrayXd::Zero(nx_);
    ky_fft_  = Eigen::ArrayXd::Zero(ny_);

    // wavenumbers in fft and math ordering
    for(Index ikx = 0; ikx < nx_; ++ikx)
    {
      double kx = (ikx - nx_/2) * kx_f_;
      kx_fft_((ikx + nx_/2) % nx_) = kx;
    }

    for(Index iky = 0; iky < ny_; ++iky)
    {
      double ky = (iky - ny_/2) * ky_f_;
      ky_fft_((iky + ny_/2) % ny_) = ky;
    }
  }

 //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::InitPressureGrid()
  {
    // pressure sample points (z is below the free surface)
    Eigen::ArrayXd zr = Eigen::ArrayXd::Zero(nz_);
    if (nz_ > 1)
    {
      // first element is zero - fill nz - 1 remaining elements
      Eigen::ArrayXd ln_z = Eigen::ArrayXd::LinSpaced(
          nz_ - 1, -std::log(lz_), std::log(lz_));
      zr(Eigen::seq(1, nz_ - 1)) = -1 * Eigen::exp(ln_z);
    }
    z_ = zr.reverse();
  }

  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  LinearRandomWaveSimulation::~LinearRandomWaveSimulation() = default;

  //////////////////////////////////////////////////
  LinearRandomWaveSimulation::LinearRandomWaveSimulation(
      double lx, double ly, Index nx, Index ny) :
    impl_(new LinearRandomWaveSimulation::Impl(lx, ly, nx, ny))
  {
  }

  //////////////////////////////////////////////////
  LinearRandomWaveSimulation::LinearRandomWaveSimulation(
      double lx, double ly, double lz, Index nx, Index ny, Index nz) :
    impl_(new LinearRandomWaveSimulation::Impl(lx, ly, lz, nx, ny, nz))
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::SetTime(double value)
  {
    impl_->SetTime(value);
  }

  //////////////////////////////////////////////////
  Index LinearRandomWaveSimulation::SizeX() const
  {
    return impl_->nx_;
  }

  //////////////////////////////////////////////////
  Index LinearRandomWaveSimulation::SizeY() const
  {
    return impl_->ny_;
  }

  //////////////////////////////////////////////////
  Index LinearRandomWaveSimulation::SizeZ() const
  {
    return impl_->nz_;
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> h)
  {
    impl_->ElevationAt(h);
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy)
    {
      impl_->ElevationDerivAt(dhdx, dhdy);
    }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::DisplacementAt(
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy)
    {
      impl_->DisplacementAt(sx, sy);
    }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy)
    {
      impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
    }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::DisplacementAndDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> h,
      Eigen::Ref<Eigen::ArrayXXd> sx,
      Eigen::Ref<Eigen::ArrayXXd> sy,
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> dsydy,
      Eigen::Ref<Eigen::ArrayXXd> dsxdy)
    {
      impl_->ElevationAt(h);
      impl_->ElevationDerivAt(dhdx, dhdy);
      impl_->DisplacementAt(sx, sy);
      impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
    }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure)
  {
    impl_->PressureAt(iz, pressure);
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::ElevationAt(
      Index ix, Index iy,
      double &eta)
  {
    impl_->ElevationAt(ix, iy, eta);
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::DisplacementAt(
      Index ix, Index iy,
      double &sx, double &sy)
  {
    impl_->DisplacementAt(ix, iy, sx, sy);
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::PressureAt(
      Index ix, Index iy, Index iz,
      double &pressure)
  {
    impl_->PressureAt(ix, iy, iz, pressure);
  }

}
}
