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

#include "gz/waves/WaveSimulationSinusoid.hh"
#include "gz/waves/Physics.hh"

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace gz
{
namespace waves
{
  /// \todo Proposed changes
  ///
  /// 1. set wind velocity as magnitude v (m/s) and direction theta (rad).
  ///
  /// 2. set wave height H or amplitude A = H/2
  ///
  /// 4. allow the dispersion relation to be configured - or set depth
  ///

  //////////////////////////////////////////////////
  // WaveSimulationSinusoid::Impl

  /// model description
  ///
  /// Waves are defined on a lx x ly mesh with nx x ny samples.
  ///
  /// H       - wave height
  /// A       - wave amplitide = H/2
  /// T       - wave period
  /// w       - wave angular frequency = 2 pi / T
  /// k       - wave number = w^2 / g for infinite depth
  /// (x, y)  - position
  /// eta     - wave height
  ///
  /// eta(x, y, t) = A cos(k (x cos(theta) + y sin(theta)) - w t)
  ///
  /// cd = cos(theta) - projection of wave direction on x-axis
  /// sd = sin(theta) - projection of wave direction on y-axis
  ///
  /// a = k (x cd + y sd) - w t
  /// sa = sin(a)
  /// ca = cos(a)
  ///
  /// da/dx = k cd
  /// da/dy = k sd
  /// 
  /// h = eta(x, y, y) = A ca
  /// dh/dx = - da/dx A sa
  /// dh/dx = - da/dy A sa
  ///

  //////////////////////////////////////////////////
  class WaveSimulationSinusoid::Impl
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~Impl();

    Impl(double lx, double ly, int nx, int ny);

    Impl(double lx, double ly, double lz, int nx, int ny, int nz);

    void Init();

    void SetUseVectorised(bool value);

    void SetDirection(double dir_x, double dir_y);

    void SetAmplitude(double value);

    void SetPeriod(double value);

    void SetWindVelocity(double ux, double uy);

    void SetTime(double value);

    void ComputeElevation(
        Eigen::Ref<Eigen::MatrixXd> h);

    void ComputeElevationDerivatives(
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy);

    void ComputeDisplacements(
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy);

    void ComputeDisplacementsDerivatives(
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy);

    void ComputeDisplacementsAndDerivatives(
        Eigen::Ref<Eigen::MatrixXd> h,
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy,
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy,
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy);

    void ComputePressureAt(
        Eigen::Ref<Eigen::MatrixXd> pressure, int iz);


    bool use_vectorised_{true};

    // elevation
    int nx_{2};
    int ny_{2};
    double lx_{1.0};
    double ly_{1.0};
    double wave_angle_{0.0};
    double amplitude_{1.0};
    double period_{1.0};
    double time_{0.0};
    
    Eigen::VectorXd x_;
    Eigen::VectorXd y_;
    Eigen::MatrixXd x_grid_;
    Eigen::MatrixXd y_grid_;

    // pressure
    double lz_{10.0};
    int nz_{2};

    Eigen::VectorXd z_;

  };

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::Impl::~Impl()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::Impl::Impl(
      double lx, double ly, int nx, int ny) :
    nx_(nx),
    ny_(ny),
    lx_(lx),
    ly_(ly)
  {
    Init();
  }

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::Impl::Impl(
      double lx, double ly, double lz, int nx, int ny, int nz) :
    nx_(nx),
    ny_(ny),
    lx_(lx),
    ly_(ly),
    nz_(nz),
    lz_(lz)
  {
    Init();
  }

 //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::Init()
  {
    // grid spacing
    double dx = lx_ / nx_;
    double dy = ly_ / ny_;

    // x and y coordinates
    double lx_min = - lx_ / 2.0;
    double lx_max =   lx_ / 2.0;
    double ly_min = - ly_ / 2.0;
    double ly_max =   ly_ / 2.0;

    // linspaced is on closed interval (unlike Python which is open to right)
    x_ = Eigen::VectorXd::LinSpaced(nx_, lx_min, lx_max - dx);
    y_ = Eigen::VectorXd::LinSpaced(ny_, ly_min, ly_max - dy);

    // broadcast to matrices (aka meshgrid)
    x_grid_ = Eigen::MatrixXd::Zero(nx_, ny_);
    y_grid_ = Eigen::MatrixXd::Zero(nx_, ny_);
    x_grid_.colwise() += x_;
    y_grid_.rowwise() += y_.transpose();

    // pressure sample points (z is below the free surface)
    Eigen::VectorXd zr = Eigen::VectorXd::Zero(nz_);
    if (nz_ > 1)
    {
      // first element is zero - fill nz - 1 remaining elements
      Eigen::VectorXd ln_z = Eigen::VectorXd::LinSpaced(
          nz_ - 1, -std::log(lz_), std::log(lz_));
      zr(Eigen::seq(1, nz_ - 1)) = -1 * Eigen::exp(ln_z.array());
    }
    z_ = zr.reverse();
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetUseVectorised(bool value)
  {
    use_vectorised_ = value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetDirection(double dir_x, double dir_y)
  {
    wave_angle_ = std::atan2(dir_y, dir_x);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetAmplitude(double value)
  {
    amplitude_ = value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetPeriod(double value)
  {
    period_ = value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetWindVelocity(
      double /*_ux*/, double /*_uy*/)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetTime(double value)
  {
    time_ = value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    // derived wave properties
    double w = 2.0 * M_PI / period_;
    double wt = w * time_;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(wave_angle_);
    double sd = std::sin(wave_angle_);

    if (use_vectorised_)
    {
      Eigen::MatrixXd a = k * (x_grid_.array() * cd
          + y_grid_.array() * sd) - wt;
      Eigen::MatrixXd ca = Eigen::cos(a.array());
      Eigen::MatrixXd h1 = amplitude_ * ca.array();
      h = h1.reshaped();
    }
    else
    {
      double dx = lx_ / nx_;
      double dy = ly_ / ny_;
      double lx_min = - lx_ / 2.0;
      double ly_min = - ly_ / 2.0;
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy + ly_min;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double ca = std::cos(a);
          double h1 = amplitude_ * ca;
          h(idx, 0) = h1;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    // derived wave properties
    double w = 2.0 * M_PI / period_;
    double wt = w * time_;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(wave_angle_);
    double sd = std::sin(wave_angle_);
    double dadx = k * cd;
    double dady = k * sd;

    if (use_vectorised_)
    {
      Eigen::MatrixXd a = k * (x_grid_.array() * cd
          + y_grid_.array() * sd) - wt;
      Eigen::MatrixXd sa = Eigen::sin(a.array());
      Eigen::MatrixXd dhdx1 = - dadx * amplitude_ * sa.array();
      Eigen::MatrixXd dhdy1 = - dady * amplitude_ * sa.array();
      dhdx = dhdx1.reshaped();
      dhdy = dhdy1.reshaped();
    }
    else
    {
      double dx = lx_ / nx_;
      double dy = ly_ / ny_;
      double lx_min = - lx_ / 2.0;
      double ly_min = - ly_ / 2.0;
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy + ly_min;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double sa = std::sin(a);
          double dhdx1 = - dadx * amplitude_ * sa;
          double dhdy1 = - dady * amplitude_ * sa;
          dhdx(idx, 0) = dhdx1;
          dhdy(idx, 0) = dhdy1;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> /*sx*/,
    Eigen::Ref<Eigen::MatrixXd> /*sy*/)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> /*dsxdx*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsydy*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsxdy*/)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> h,
    Eigen::Ref<Eigen::MatrixXd> /*sx*/,
    Eigen::Ref<Eigen::MatrixXd> /*sy*/,
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy,
    Eigen::Ref<Eigen::MatrixXd> /*dsxdx*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsydy*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsxdy*/)
  {
    // derived wave properties
    double w = 2.0 * M_PI / period_;
    double wt = w * time_;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(wave_angle_);
    double sd = std::sin(wave_angle_);
    double dadx = k * cd;
    double dady = k * sd;

    if (use_vectorised_)
    {
      Eigen::MatrixXd a = k * (x_grid_.array() * cd
          + y_grid_.array() * sd) - wt;
      Eigen::MatrixXd ca = Eigen::cos(a.array());
      Eigen::MatrixXd sa = Eigen::sin(a.array());
      Eigen::MatrixXd h1 = amplitude_ * ca.array();
      Eigen::MatrixXd dhdx1 = - dadx * amplitude_ * sa.array();
      Eigen::MatrixXd dhdy1 = - dady * amplitude_ * sa.array();
      h = h1.reshaped();
      dhdx = dhdx1.reshaped();
      dhdy = dhdy1.reshaped();
    }
    else
    {
      double dx = lx_ / nx_;
      double dy = ly_ / ny_;
      double lx_min = - lx_ / 2.0;
      double ly_min = - ly_ / 2.0;
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy + ly_min;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double ca = std::cos(a);
          double sa = std::sin(a);
          double h1 = amplitude_ * ca;
          double dhdx1 = - dadx * amplitude_ * sa;
          double dhdy1 = - dady * amplitude_ * sa;
          h(idx, 0) = h1;
          dhdx(idx, 0) = dhdx1;
          dhdy(idx, 0) = dhdy1;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputePressureAt(
    Eigen::Ref<Eigen::MatrixXd> pressure, int iz)
  {
    // derived wave properties
    double w = 2.0 * M_PI / period_;
    double wt = w * time_;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(wave_angle_);
    double sd = std::sin(wave_angle_);

    // value of z at index ix
    double z = z_(iz);

    // linear deep water wave pressure scaling factor
    double e = std::exp(k * z);

    if (use_vectorised_)
    {
      Eigen::MatrixXd a = k * (x_grid_.array() * cd
          + y_grid_.array() * sd) - wt;
      Eigen::MatrixXd ca = Eigen::cos(a.array());
      Eigen::MatrixXd p  = e * amplitude_ * ca.array();
      pressure = p.reshaped();
    }
    else
    {
      double dx = lx_ / nx_;
      double dy = ly_ / ny_;
      double lx_min = - lx_ / 2.0;
      double ly_min = - ly_ / 2.0;
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy + ly_min;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double ca = std::cos(a);
          double h1 = amplitude_ * ca;
          double p = e * h1;

          pressure(idx, 0) = p;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  WaveSimulationSinusoid::~WaveSimulationSinusoid()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::WaveSimulationSinusoid(
      double lx, double ly, int nx, int ny) :
    impl_(new WaveSimulationSinusoid::Impl(lx, ly, nx, ny))
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::WaveSimulationSinusoid(
      double lx, double ly, double lz, int nx, int ny, int nz) :
    impl_(new WaveSimulationSinusoid::Impl(lx, ly, lz, nx, ny, nz))
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetUseVectorised(bool value)
  {
    impl_->SetUseVectorised(value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetDirection(double dir_x, double dir_y)
  {
    impl_->SetDirection(dir_x, dir_y);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetAmplitude(double value)
  {
    impl_->SetAmplitude(value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetPeriod(double value)
  {
    impl_->SetPeriod(value);
  }

  //////////////////////////////////////////////////
  Eigen::VectorXd WaveSimulationSinusoid::X() const
  {
    return impl_->x_;
  }

  //////////////////////////////////////////////////
  Eigen::VectorXd WaveSimulationSinusoid::Y() const
  {
    return impl_->y_;
  }

  //////////////////////////////////////////////////
  Eigen::VectorXd WaveSimulationSinusoid::Z() const
  {
    return impl_->z_;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetWindVelocity(double ux, double uy)
  {
    impl_->SetWindVelocity(ux, uy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetTime(double value)
  {
    impl_->SetTime(value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    impl_->ComputeElevation(h);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    impl_->ComputeElevationDerivatives(dhdx, dhdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy)
  {
    impl_->ComputeDisplacements(sx, sy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    impl_->ComputeDisplacementsDerivatives(dsxdx, dsydy, dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> h,
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy,
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy,
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    // impl_->ComputeDisplacementsAndDerivatives(
    //     h, sx, sy, dhdx, dhdy, dsxdx, dsydy, dsxdy);

    /// \todo undo flip of dhdx <---> dhdy once render plugin fixed
    impl_->ComputeDisplacementsAndDerivatives(
        h, sx, sy, dhdy, dhdx, dsxdx, dsydy, dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputePressureAt(
    Eigen::Ref<Eigen::MatrixXd> pressure,
    int iz)
  {
    impl_->ComputePressureAt(pressure, iz);
  }

}
}
