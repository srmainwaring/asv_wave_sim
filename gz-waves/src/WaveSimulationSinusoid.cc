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

#include <vector>

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

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationSinusoid::Impl

  /// model description
  ///
  /// waves are defined on a Lx x Ly mesh with Nx x Ny samples
  /// where Lx = Ly = L and Nx = Ny = N. 
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

  class WaveSimulationSinusoid::Impl
  {
    public: ~Impl();

    public: Impl(
      int _N,
      double _L);

    public: void SetWindVelocity(double _ux, double _uy);

    public: void SetDirection(double _dir_x, double _dir_y);

    public: void SetAmplitude(double _amplitude);

    public: void SetPeriod(double _period);

    public: void SetTime(double _time);

    public: void ComputeHeights(
      std::vector<double>& _heights);

    public: void ComputeHeightDerivatives(
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy);

    public: void ComputeDisplacements(
      std::vector<double>& _sx,
      std::vector<double>& _sy);

    public: void ComputeDisplacementDerivatives(
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy);

    public: void ComputeDisplacementsAndDerivatives(
      std::vector<double>& _h,
      std::vector<double>& _sx,
      std::vector<double>& _sy,
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy,
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy);

    private: void ComputeBaseAmplitudes();

    private: void ComputeCurrentAmplitudes(double _time);
    
    public:  int N{2};
    public:  int N2{2};
    private: int NOver2{1};
    private: double L{1.0};
    private: double wave_angle{0.0};
    private: double dir_x{1.0}, dir_y{0.0};
    private: double amplitude{1.0};
    private: double period{1.0};
    private: double time{0.0};
  };

  WaveSimulationSinusoid::Impl::~Impl()
  {
  }

  WaveSimulationSinusoid::Impl::Impl(
    int _N,
    double _L) :
    N(_N),
    N2(_N * _N),
    NOver2(_N / 2),
    L(_L)
  {
  }

  void WaveSimulationSinusoid::Impl::SetWindVelocity(double _ux, double _uy)
  {
    // @TODO NO IMPLEMENTATION
  }

  void WaveSimulationSinusoid::Impl::SetDirection(double _dir_x, double _dir_y)
  {
    this->dir_x = _dir_x;
    this->dir_y = _dir_y;
    this->wave_angle = std::atan2(_dir_y, _dir_x);
  }

  void WaveSimulationSinusoid::Impl::SetAmplitude(double _amplitude)
  {
    this->amplitude = _amplitude;
  }

  void WaveSimulationSinusoid::Impl::SetPeriod(double _period)
  {
    this->period = _period;
  }

  void WaveSimulationSinusoid::Impl::SetTime(double _time)
  {
    this->time = _time;
  }

  void WaveSimulationSinusoid::Impl::ComputeHeights(
    std::vector<double>& _heights)
  {
    // Derived wave properties
    double w = 2.0 * M_PI / this->period;
    double wt = w * this->time;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(this->wave_angle);
    double sd = std::sin(this->wave_angle);

    // Wave update
    double LOverN = this->L / this->N;
    double LOver2 = this->L / 2.0;
    for (size_t iy=0; iy<this->N; ++iy)
    {
      // Regular grid
      double y = iy * LOverN - LOver2;

      for (size_t ix=0; ix<this->N; ++ix)
      {
        // Row major index
        size_t idx = iy * this->N + ix;

        // Regular grid
        double x = ix * LOverN - LOver2;

        // Single wave
        double a  = k * (x * cd + y * sd) - wt;
        double ca = std::cos(a);
        double h = this->amplitude * ca;

        _heights[idx] = h;
      }
    }
  }

  void WaveSimulationSinusoid::Impl::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    // Derived wave properties
    double w = 2.0 * M_PI / this->period;
    double wt = w * this->time;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(this->wave_angle);
    double sd = std::sin(this->wave_angle);

    // Wave update
    double LOverN = this->L / this->N;
    double LOver2 = this->L / 2.0;
    for (size_t iy=0; iy<this->N; ++iy)
    {
      // Regular grid
      double y = iy * LOverN - LOver2;

      for (size_t ix=0; ix<this->N; ++ix)
      {
        // Row major index
        size_t idx = iy * this->N + ix;

        // Regular grid
        double x = ix * LOverN - LOver2;

        // Single wave
        double a  = k * (x * cd + y * sd) - wt;
        double dadx = k * cd;
        double dady = k * sd;
        double sa = std::sin(a);
        double dhdx = - dadx * this->amplitude * sa;
        double dhdy = - dady * this->amplitude * sa;

        _dhdx[idx] = dhdx;
        _dhdy[idx] = dhdy;
      }
    }
  }

  void WaveSimulationSinusoid::Impl::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    // No xy-displacement
  }

  void WaveSimulationSinusoid::Impl::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    // No xy-displacement
  }

  void WaveSimulationSinusoid::Impl::ComputeDisplacementsAndDerivatives(
    std::vector<double>& _h,
    std::vector<double>& _sx,
    std::vector<double>& _sy,
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy,
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    // Derived wave properties
    double w = 2.0 * M_PI / this->period;
    double wt = w * this->time;
    double k = Physics::DeepWaterDispersionToWavenumber(w);
    double cd = std::cos(this->wave_angle);
    double sd = std::sin(this->wave_angle);

    // Wave update
    double LOverN = this->L / this->N;
    double LOver2 = this->L / 2.0;
    for (size_t iy=0; iy<this->N; ++iy)
    {
      double y = iy * LOverN - LOver2;

      for (size_t ix=0; ix<this->N; ++ix)
      {
        // Row major index
        size_t idx = iy * this->N + ix;

        // Regular grid
        double x = ix * LOverN - LOver2;

        // Single wave
        double a  = k * (x * cd + y * sd) - wt;
        double dadx = k * cd;
        double dady = k * sd;
        double ca = std::cos(a);
        double sa = std::sin(a);
        double h = this->amplitude * ca;
        double dhdx = - dadx * this->amplitude * sa;
        double dhdy = - dady * this->amplitude * sa;

        _h[idx] = h;
        _dhdx[idx] = dhdx;
        _dhdy[idx] = dhdy;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationSinusoid

  WaveSimulationSinusoid::~WaveSimulationSinusoid()
  {
  }

  WaveSimulationSinusoid::WaveSimulationSinusoid(
    int _N,
    double _L) :
    impl(new WaveSimulationSinusoid::Impl(_N, _L))
  {
  }

  void WaveSimulationSinusoid::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  void WaveSimulationSinusoid::SetDirection(double _dir_x, double _dir_y)
  {
    impl->SetDirection(_dir_x, _dir_y);
  }

  void WaveSimulationSinusoid::SetAmplitude(double _amplitude)
  {
    impl->SetAmplitude(_amplitude);
  }

  void WaveSimulationSinusoid::SetPeriod(double _period)
  {
    impl->SetPeriod(_period);
  }

  void WaveSimulationSinusoid::SetTime(double _time)
  {
    impl->SetTime(_time);
  }

  void WaveSimulationSinusoid::ComputeHeights(
    std::vector<double>& _h)
  {
    impl->ComputeHeights(_h);
  }

  void WaveSimulationSinusoid::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);
  }

  void WaveSimulationSinusoid::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);
  }

  void WaveSimulationSinusoid::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);
  }

  void WaveSimulationSinusoid::ComputeDisplacementsAndDerivatives(
    std::vector<double>& _h,
    std::vector<double>& _sx,
    std::vector<double>& _sy,
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy,
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeDisplacementsAndDerivatives(
        _h, _dhdx, _dhdy, _sx, _sy, _dsxdx, _dsydy, _dsxdy);
  }

  void WaveSimulationSinusoid::ComputeHeights(
    Eigen::Ref<Eigen::MatrixXd> _h)
  {
    auto size = impl->N2;
    std::vector<double> h(size);
    this->ComputeHeights(h);
    _h = Eigen::Map<Eigen::VectorXd>(h.data(), h.size());
  }

  void WaveSimulationSinusoid::ComputeHeightDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    auto size = impl->N2;
    std::vector<double> dhdx(size);
    std::vector<double> dhdy(size);
    this->ComputeHeightDerivatives(dhdx, dhdy);
    _dhdx = Eigen::Map<Eigen::VectorXd>(dhdx.data(), dhdx.size());
    _dhdy = Eigen::Map<Eigen::VectorXd>(dhdy.data(), dhdy.size());
  }

  void WaveSimulationSinusoid::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    auto size = impl->N2;
    std::vector<double> sx(size);
    std::vector<double> sy(size);
    this->ComputeDisplacements(sx, sy);
    _sx = Eigen::Map<Eigen::VectorXd>(sx.data(), sx.size());
    _sy = Eigen::Map<Eigen::VectorXd>(sy.data(), sy.size());
  }

  void WaveSimulationSinusoid::ComputeDisplacementDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    auto size = impl->N2;
    std::vector<double> dsxdx(size);
    std::vector<double> dsydy(size);
    std::vector<double> dsxdy(size);
    this->ComputeDisplacementDerivatives(dsxdx, dsxdy, dsxdy);
    _dsxdx = Eigen::Map<Eigen::VectorXd>(dsxdx.data(), dsxdx.size());
    _dsydy = Eigen::Map<Eigen::VectorXd>(dsydy.data(), dsydy.size());
    _dsxdy = Eigen::Map<Eigen::VectorXd>(dsxdy.data(), dsxdy.size());
  }

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
    auto size = impl->N2;
    std::vector<double> h(size);
    std::vector<double> sx(size);
    std::vector<double> sy(size);
    std::vector<double> dhdx(size);
    std::vector<double> dhdy(size);
    std::vector<double> dsxdx(size);
    std::vector<double> dsydy(size);
    std::vector<double> dsxdy(size);
    this->ComputeDisplacementsAndDerivatives(
        h, sx, sy, dhdx, dhdy, dsxdx, dsydy, dsxdy);
    _h = Eigen::Map<Eigen::VectorXd>(h.data(), h.size());
    _sx = Eigen::Map<Eigen::VectorXd>(sx.data(), sx.size());
    _sy = Eigen::Map<Eigen::VectorXd>(sy.data(), sy.size());
    _dhdx = Eigen::Map<Eigen::VectorXd>(dhdx.data(), dhdx.size());
    _dhdy = Eigen::Map<Eigen::VectorXd>(dhdy.data(), dhdy.size());
    _dsxdx = Eigen::Map<Eigen::VectorXd>(dsxdx.data(), dsxdx.size());
    _dsydy = Eigen::Map<Eigen::VectorXd>(dsydy.data(), dsydy.size());
    _dsxdy = Eigen::Map<Eigen::VectorXd>(dsxdy.data(), dsxdy.size());
  }

}
}
