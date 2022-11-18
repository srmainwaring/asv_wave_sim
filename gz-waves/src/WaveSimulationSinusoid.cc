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
    ~Impl();

    Impl(double _lx, double _ly, int _nx, int _ny);

    void SetUseVectorised(bool _value);

    void SetDirection(double _dir_x, double _dir_y);

    void SetAmplitude(double _value);

    void SetPeriod(double _value);

    void SetWindVelocity(double _ux, double _uy);

    void SetTime(double _value);

    void ComputeElevation(
        Eigen::Ref<Eigen::MatrixXd> _h);

    void ComputeElevationDerivatives(
        Eigen::Ref<Eigen::MatrixXd> _dhdx,
        Eigen::Ref<Eigen::MatrixXd> _dhdy);

    void ComputeDisplacements(
        Eigen::Ref<Eigen::MatrixXd> _sx,
        Eigen::Ref<Eigen::MatrixXd> _sy);

    void ComputeDisplacementsDerivatives(
        Eigen::Ref<Eigen::MatrixXd> _dsxdx,
        Eigen::Ref<Eigen::MatrixXd> _dsydy,
        Eigen::Ref<Eigen::MatrixXd> _dsxdy);

    void ComputeDisplacementsAndDerivatives(
        Eigen::Ref<Eigen::MatrixXd> _h,
        Eigen::Ref<Eigen::MatrixXd> _sx,
        Eigen::Ref<Eigen::MatrixXd> _sy,
        Eigen::Ref<Eigen::MatrixXd> _dhdx,
        Eigen::Ref<Eigen::MatrixXd> _dhdy,
        Eigen::Ref<Eigen::MatrixXd> _dsxdx,
        Eigen::Ref<Eigen::MatrixXd> _dsydy,
        Eigen::Ref<Eigen::MatrixXd> _dsxdy);
  
    bool use_vectorised{true};

    int nx{2};
    int ny{2};
    double lx{1.0};
    double ly{1.0};
    double wave_angle{0.0};
    double amplitude{1.0};
    double period{1.0};
    double time{0.0};
    
    Eigen::VectorXd x_v;
    Eigen::VectorXd y_v;
    Eigen::MatrixXd x_grid;
    Eigen::MatrixXd y_grid;
  };

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::Impl::~Impl()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationSinusoid::Impl::Impl(
      double _lx, double _ly, int _nx, int _ny) :
    nx(_nx),
    ny(_ny),
    lx(_lx),
    ly(_ly)
  {
    // grid spacing
    double dx = this->lx / this->nx;
    double dy = this->ly / this->ny;

    // x and y coordinates
    double lx_min = - this->lx / 2.0;
    double lx_max =   this->lx / 2.0;
    double ly_min = - this->ly / 2.0;
    double ly_max =   this->ly / 2.0;

    // linspaced is on closed interval (unlike Python which is open to right)
    this->x_v = Eigen::VectorXd::LinSpaced(nx, lx_min, lx_max - dx);
    this->y_v = Eigen::VectorXd::LinSpaced(ny, ly_min, ly_max - dy);

    // broadcast to matrices (aka meshgrid)
    this->x_grid = Eigen::MatrixXd::Zero(this->nx, this->ny);
    this->y_grid = Eigen::MatrixXd::Zero(this->nx, this->ny);
    this->x_grid.colwise() += this->x_v;
    this->y_grid.rowwise() += this->y_v.transpose();
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetUseVectorised(bool _value)
  {
    this->use_vectorised = _value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetDirection(double _dir_x, double _dir_y)
  {
    this->wave_angle = std::atan2(_dir_y, _dir_x);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetAmplitude(double _value)
  {
    this->amplitude = _value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetPeriod(double _value)
  {
    this->period = _value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetWindVelocity(double _ux, double _uy)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::SetTime(double _value)
  {
    this->time = _value;
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> _heights)
  {
    // derived wave properties
    double w = 2.0 * M_PI / this->period;
    double wt = w * this->time;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(this->wave_angle);
    double sd = std::sin(this->wave_angle);

    if (this->use_vectorised)
    {
      Eigen::MatrixXd a = k * (this->x_grid.array() * cd
          + this->y_grid.array() * sd) - wt;
      Eigen::MatrixXd ca = Eigen::cos(a.array());
      Eigen::MatrixXd h = this->amplitude * ca.array();
      _heights = h.reshaped();
    }
    else
    {
      double dx = this->lx / this->nx;
      double dy = this->ly / this->ny;
      double lx_min = - this->lx / 2.0;
      double ly_min = - this->ly / 2.0;
      for (size_t iy=0, idx=0; iy<this->ny; ++iy)
      {
        double y = iy * dy + ly_min;
        for (size_t ix=0; ix<this->nx; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double ca = std::cos(a);
          double h = this->amplitude * ca;
          _heights(idx, 0) = h;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    // derived wave properties
    double w = 2.0 * M_PI / this->period;
    double wt = w * this->time;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(this->wave_angle);
    double sd = std::sin(this->wave_angle);
    double dadx = k * cd;
    double dady = k * sd;

    if (this->use_vectorised)
    {
      Eigen::MatrixXd a = k * (this->x_grid.array() * cd
          + this->y_grid.array() * sd) - wt;
      Eigen::MatrixXd sa = Eigen::sin(a.array());
      Eigen::MatrixXd dhdx = - dadx * this->amplitude * sa.array();
      Eigen::MatrixXd dhdy = - dady * this->amplitude * sa.array();
      _dhdx = dhdx.reshaped();
      _dhdy = dhdy.reshaped();
    }
    else
    {
      double dx = this->lx / this->nx;
      double dy = this->ly / this->ny;
      double lx_min = - this->lx / 2.0;
      double ly_min = - this->ly / 2.0;
      for (size_t iy=0, idx=0; iy<this->ny; ++iy)
      {
        double y = iy * dy + ly_min;
        for (size_t ix=0; ix<this->nx; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double sa = std::sin(a);
          double dhdx = - dadx * this->amplitude * sa;
          double dhdy = - dady * this->amplitude * sa;
          _dhdx(idx, 0) = dhdx;
          _dhdy(idx, 0) = dhdy;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    // No xy-displacement
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::Impl::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _h,
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy,
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    // derived wave properties
    double w = 2.0 * M_PI / this->period;
    double wt = w * this->time;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(this->wave_angle);
    double sd = std::sin(this->wave_angle);
    double dadx = k * cd;
    double dady = k * sd;

    if (this->use_vectorised)
    {
      Eigen::MatrixXd a = k * (this->x_grid.array() * cd
          + this->y_grid.array() * sd) - wt;
      Eigen::MatrixXd ca = Eigen::cos(a.array());
      Eigen::MatrixXd sa = Eigen::sin(a.array());
      Eigen::MatrixXd h = this->amplitude * ca.array();
      Eigen::MatrixXd dhdx = - dadx * this->amplitude * sa.array();
      Eigen::MatrixXd dhdy = - dady * this->amplitude * sa.array();
      _h = h.reshaped();
      _dhdx = dhdx.reshaped();
      _dhdy = dhdy.reshaped();
    }
    else
    {
      double dx = this->lx / this->nx;
      double dy = this->ly / this->ny;
      double lx_min = - this->lx / 2.0;
      double ly_min = - this->ly / 2.0;
      for (size_t iy=0, idx=0; iy<this->ny; ++iy)
      {
        double y = iy * dy + ly_min;
        for (size_t ix=0; ix<this->nx; ++ix, ++idx)
        {
          double x = ix * dx + lx_min;
          double a  = k * (x * cd + y * sd) - wt;
          double ca = std::cos(a);
          double sa = std::sin(a);
          double h = this->amplitude * ca;
          double dhdx = - dadx * this->amplitude * sa;
          double dhdy = - dady * this->amplitude * sa;
          _h(idx, 0) = h;
          _dhdx(idx, 0) = dhdx;
          _dhdy(idx, 0) = dhdy;
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
      double _lx, double _ly, int _nx, int _ny) :
    impl(new WaveSimulationSinusoid::Impl(_lx, _ly, _nx, _ny))
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetUseVectorised(bool _value)
  {
    impl->SetUseVectorised(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetDirection(double _dir_x, double _dir_y)
  {
    impl->SetDirection(_dir_x, _dir_y);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetAmplitude(double _value)
  {
    impl->SetAmplitude(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetPeriod(double _value)
  {
    impl->SetPeriod(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::SetTime(double _value)
  {
    impl->SetTime(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> _h)
  {
    impl->ComputeElevation(_h);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    impl->ComputeElevationDerivatives(_dhdx, _dhdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    impl->ComputeDisplacementsDerivatives(_dsxdx, _dsydy, _dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationSinusoid::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _h,
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy,
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    impl->ComputeDisplacementsAndDerivatives(
        _h, _sx, _sy, _dhdx, _dhdy, _dsxdx, _dsydy, _dsxdy);
  }
}
}
