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

#include "gz/waves/LinearRegularWaveSimulation.hh"
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
  // LinearRegularWaveSimulation::Impl

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
  class LinearRegularWaveSimulation::Impl
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~Impl();

    Impl(double lx, double ly, int nx, int ny);

    Impl(double lx, double ly, double lz, int nx, int ny, int nz);

    void InitGrid();

    void SetUseVectorised(bool value);

    void SetDirection(double dir_x, double dir_y);

    void SetAmplitude(double value);

    void SetPeriod(double value);

    void SetWindVelocity(double ux, double uy);

    void SetTime(double value);

    ///// interpolation interface
    void Elevation(
        double x, double y,
        double &eta);

    void Elevation(
        const Eigen::Ref<const Eigen::VectorXd> &x,
        const Eigen::Ref<const Eigen::VectorXd> &y,
        Eigen::Ref<Eigen::VectorXd> eta);

    void Pressure(
        double x, double y, double z,
        double &pressure);

    void Pressure(
        const Eigen::Ref<const Eigen::VectorXd> &x,
        const Eigen::Ref<const Eigen::VectorXd> &y,
        const Eigen::Ref<const Eigen::VectorXd> &z,
        Eigen::Ref<Eigen::VectorXd> pressure);

    ///// lookup interface
    void ElevationAt(
        Eigen::Ref<Eigen::MatrixXd> h);

    void ElevationDerivAt(
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy);

    void DisplacementAndDerivAt(
        Eigen::Ref<Eigen::MatrixXd> h,
        Eigen::Ref<Eigen::MatrixXd> sx,
        Eigen::Ref<Eigen::MatrixXd> sy,
        Eigen::Ref<Eigen::MatrixXd> dhdx,
        Eigen::Ref<Eigen::MatrixXd> dhdy,
        Eigen::Ref<Eigen::MatrixXd> dsxdx,
        Eigen::Ref<Eigen::MatrixXd> dsydy,
        Eigen::Ref<Eigen::MatrixXd> dsxdy);

    void ElevationAt(
        int ix, int iy,
        double &h);

    void PressureAt(
        int ix, int iy, int iz,
        double &pressure);

    void PressureAt(
        int iz,
        Eigen::Ref<Eigen::MatrixXd> pressure);

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

    // elevation grid params
    int nx_{2};
    int ny_{2};
    double lx_{1.0};
    double ly_{1.0};
    
    // pressure grid params
    int nz_{1};
    double lz_{0.0};

    // wave params
    double wave_angle_{0.0};
    double amplitude_{1.0};
    double period_{1.0};
    double time_{0.0};

    // derived
    double dx_{0.0};
    double dy_{0.0};
    double lx_max_{0.0};
    double lx_min_{0.0};
    double ly_max_{0.0};
    double ly_min_{0.0};

    // vectorised
    Eigen::VectorXd x_;
    Eigen::VectorXd y_;
    Eigen::MatrixXd x_grid_;
    Eigen::MatrixXd y_grid_;
    Eigen::VectorXd z_;
  };

  //////////////////////////////////////////////////
  LinearRegularWaveSimulation::Impl::~Impl()
  {
  }

  //////////////////////////////////////////////////
  LinearRegularWaveSimulation::Impl::Impl(
      double lx, double ly, int nx, int ny) :
    nx_(nx),
    ny_(ny),
    lx_(lx),
    ly_(ly)
  {
    InitGrid();
  }

  //////////////////////////////////////////////////
  LinearRegularWaveSimulation::Impl::Impl(
      double lx, double ly, double lz, int nx, int ny, int nz) :
    nx_(nx),
    ny_(ny),
    lx_(lx),
    ly_(ly),
    nz_(nz),
    lz_(lz)
  {
    InitGrid();
  }

 //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::InitGrid()
  {
    // grid spacing
    dx_ = lx_ / nx_;
    dy_ = ly_ / ny_;

    // x and y coordinates
    lx_min_ = - lx_ / 2.0;
    lx_max_ =   lx_ / 2.0;
    ly_min_ = - ly_ / 2.0;
    ly_max_ =   ly_ / 2.0;

    // linspaced is on closed interval (unlike Python which is open to right)
    x_ = Eigen::VectorXd::LinSpaced(nx_, lx_min_, lx_max_ - dx_);
    y_ = Eigen::VectorXd::LinSpaced(ny_, ly_min_, ly_max_ - dy_);

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
  void LinearRegularWaveSimulation::Impl::SetUseVectorised(bool value)
  {
    use_vectorised_ = value;
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::SetDirection(double dir_x, double dir_y)
  {
    wave_angle_ = std::atan2(dir_y, dir_x);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::SetAmplitude(double value)
  {
    amplitude_ = value;
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::SetPeriod(double value)
  {
    period_ = value;
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::SetWindVelocity(
      double /*_ux*/, double /*_uy*/)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::SetTime(double value)
  {
    time_ = value;
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::Elevation(
      double x, double y,
      double &eta)
  {
    double w, wt, k, cd, sd;
    PreComputeCoeff(
      period_, time_, wave_angle_, w, wt, k, cd, sd);

    double a  = k * (x * cd + y * sd) - wt;
    double ca = std::cos(a);
    double h1 = amplitude_ * ca;
    eta = h1;
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::Elevation(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      const Eigen::Ref<const Eigen::VectorXd> &y,
      Eigen::Ref<Eigen::VectorXd> eta)
  {
    auto xit = x.cbegin();
    auto yit = y.cbegin();
    auto eit = eta.begin();
    for ( ; xit != x.cend() && yit != y.cend() && eit != eta.end();
      ++xit, ++yit, ++eit)
    {
      Elevation(*xit, *yit, *eit);
    }
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::Pressure(
      double x, double y, double z,
      double &pressure)
  {
    double w, wt, k, cd, sd;
    PreComputeCoeff(
      period_, time_, wave_angle_, w, wt, k, cd, sd);

    // linear wave deep water pressure scaling factor
    double e = std::exp(k * z);

    double a  = k * (x * cd + y * sd) - wt;
    double ca = std::cos(a);
    double h1 = amplitude_ * ca;
    double p1 = e * h1;
    pressure = p1;
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::Pressure(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y,
    const Eigen::Ref<const Eigen::VectorXd> &z,
    Eigen::Ref<Eigen::VectorXd> pressure)
  {
    auto xit = x.cbegin();
    auto yit = y.cbegin();
    auto zit = z.cbegin(); 
    auto pit = pressure.begin();
    for ( ; xit != x.cend() && yit != y.cend() && pit != pressure.end();
      ++xit, ++yit, ++zit, ++pit)
    {
      Pressure(*xit, *yit, *zit, *pit);
    }
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::ElevationAt(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    // derived wave properties
    double w, wt, k, cd, sd;
    PreComputeCoeff(
      period_, time_, wave_angle_, w, wt, k, cd, sd);

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
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy_ + ly_min_;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx_ + lx_min_;
          double a  = k * (x * cd + y * sd) - wt;
          double ca = std::cos(a);
          double h1 = amplitude_ * ca;
          h(idx, 0) = h1;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::ElevationDerivAt(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    // derived wave properties
    double w, wt, k, cd, sd;
    PreComputeCoeff(
      period_, time_, wave_angle_, w, wt, k, cd, sd);

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
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy_ + ly_min_;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx_ + lx_min_;
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
  void LinearRegularWaveSimulation::Impl::DisplacementAndDerivAt(
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
    double w, wt, k, cd, sd;
    PreComputeCoeff(
      period_, time_, wave_angle_, w, wt, k, cd, sd);

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
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy_ + ly_min_;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx_ + lx_min_;
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
  void LinearRegularWaveSimulation::Impl::ElevationAt(
      int ix, int iy, double &eta)
  {
    double y = iy * dy_ + ly_min_;
    double x = ix * dx_ + lx_min_;
    Elevation(x, y, eta);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::PressureAt(
    int ix, int iy, int iz,
    double &pressure)
  {
    double y = iy * dy_ + ly_min_;
    double x = ix * dx_ + lx_min_;
    double z = z_(iz);
    Pressure(x, y, z, pressure);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Impl::PressureAt(
    int iz,
    Eigen::Ref<Eigen::MatrixXd> pressure)
  {
    // derived wave properties
    double w, wt, k, cd, sd;
    PreComputeCoeff(
      period_, time_, wave_angle_, w, wt, k, cd, sd);

    // value of z at index iz
    double z = z_(iz);

    // linear wave deep water pressure scaling factor
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
      for (int iy=0, idx=0; iy<ny_; ++iy)
      {
        double y = iy * dy_ + ly_min_;
        for (int ix=0; ix<nx_; ++ix, ++idx)
        {
          double x = ix * dx_ + lx_min_;
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
  LinearRegularWaveSimulation::~LinearRegularWaveSimulation()
  {
  }

  //////////////////////////////////////////////////
  LinearRegularWaveSimulation::LinearRegularWaveSimulation(
      double lx, double ly, int nx, int ny) :
    impl_(new LinearRegularWaveSimulation::Impl(lx, ly, nx, ny))
  {
  }

  //////////////////////////////////////////////////
  LinearRegularWaveSimulation::LinearRegularWaveSimulation(
      double lx, double ly, double lz, int nx, int ny, int nz) :
    impl_(new LinearRegularWaveSimulation::Impl(lx, ly, lz, nx, ny, nz))
  {
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::SetUseVectorised(bool value)
  {
    impl_->SetUseVectorised(value);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::SetDirection(double dir_x, double dir_y)
  {
    impl_->SetDirection(dir_x, dir_y);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::SetAmplitude(double value)
  {
    impl_->SetAmplitude(value);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::SetPeriod(double value)
  {
    impl_->SetPeriod(value);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::SetWindVelocity(double ux, double uy)
  {
    impl_->SetWindVelocity(ux, uy);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::SetTime(double value)
  {
    impl_->SetTime(value);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Elevation(
    double x, double y,
    double &eta)
  {
    impl_->Elevation(x, y, eta);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Elevation(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y,
    Eigen::Ref<Eigen::VectorXd> eta)
  {
    impl_->Elevation(x, y, eta);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Pressure(
    double x, double y, double z,
    double &pressure)
  {
    impl_->Pressure(x, y, z, pressure);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::Pressure(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y,
    const Eigen::Ref<const Eigen::VectorXd> &z,
    Eigen::Ref<Eigen::VectorXd> pressure)
  {
    impl_->Pressure(x, y, z, pressure);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::ElevationAt(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    impl_->ElevationAt(h);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::ElevationDerivAt(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    impl_->ElevationDerivAt(dhdx, dhdy);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::DisplacementAt(
    Eigen::Ref<Eigen::MatrixXd> /*sx*/,
    Eigen::Ref<Eigen::MatrixXd> /*sy*/)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::DisplacementDerivAt(
    Eigen::Ref<Eigen::MatrixXd> /*dsxdx*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsydy*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsxdy*/)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::DisplacementAndDerivAt(
    Eigen::Ref<Eigen::MatrixXd> h,
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy,
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy,
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    // impl_->DisplacementAndDerivAt(
    //     h, sx, sy, dhdx, dhdy, dsxdx, dsydy, dsxdy);

    /// \todo undo flip of dhdx <---> dhdy once render plugin fixed
    impl_->DisplacementAndDerivAt(
        h, sx, sy, dhdy, dhdx, dsxdx, dsydy, dsxdy);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::ElevationAt(
    int ix, int iy,
    double &eta)
  {
    impl_->ElevationAt(ix, iy, eta);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::DisplacementAt(
    int /*ix*/, int /*iy*/,
    double &/*sx*/, double &/*sy*/)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::PressureAt(
    int ix, int iy, int iz,
    double &pressure)
  {
    impl_->PressureAt(ix, iy, iz, pressure);
  }

  //////////////////////////////////////////////////
  void LinearRegularWaveSimulation::PressureAt(
    int iz,
    Eigen::Ref<Eigen::MatrixXd> pressure)
  {
    impl_->PressureAt(iz, pressure);
  }
}
}
