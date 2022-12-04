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

#include <random>
#include <vector>

#include <Eigen/Dense>

#include <gz/common.hh>

#include "gz/waves/WaveSpectrum.hh"


namespace gz
{
namespace waves
{

  constexpr Index DEFAULT_N_PHASES{300};
  constexpr double MAX_FREQ{0.3};  // Hz

  //////////////////////////////////////////////////
  class LinearRandomWaveSimulation::Impl
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~Impl();

    Impl(double lx, double ly, Index nx, Index ny);

    Impl(double lx, double ly, double lz, Index nx, Index ny, Index nz);

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

    // lookup interface - scalar
    void ElevationAt(
        Index ix, Index iy,
        double &h);

    void PressureAt(
        Index ix, Index iy, Index iz,
        double &pressure);

    // lookup interface - array
    void ElevationAt(
        Eigen::Ref<Eigen::ArrayXXd> h);

    void ElevationDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy);

    void PressureAt(
        Index iz,
        Eigen::Ref<Eigen::ArrayXXd> pressure);

    void InitGrid();

    void ComputeAmplitudes();

    // elevation and pressure grid params
    Index nx_{2};
    Index ny_{2};
    Index nz_{1};
    double lx_{1.0};
    double ly_{1.0};
    double lz_{0.0};

    // simulation parameters
    double gravity_{9.81};
    double fluid_rho_{1025.0};

    // parameters
    Index num_waves_{DEFAULT_N_PHASES};
    double u19_{5.0};
    double wave_angle_{0.0};

    bool needs_update_{true};

    Eigen::ArrayXd spectrum_;
    Eigen::ArrayXd amplitude_;
    Eigen::ArrayXd w_;
    Eigen::ArrayXd k_;
    Eigen::ArrayXd phase_;

    // update
    double time_{0.0};

    // derived
    double dx_{0.0};
    double dy_{0.0};
    double lx_max_{0.0};
    double lx_min_{0.0};
    double ly_max_{0.0};
    double ly_min_{0.0};

    // pressure sample points
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
    InitGrid();
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
    InitGrid();
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::SetTime(double value)
  {
    time_ = value;
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::Elevation(
      double x, double y,
      double &eta)
  {
    ComputeAmplitudes();

    double xx = x * std::cos(wave_angle_) + y * std::sin(wave_angle_);

    double h = 0;
    for (Index ik = 0; ik < num_waves_; ++ik)
    {
      double wt = w_(ik) * time_;
      h += amplitude_(ik) * std::cos(k_(ik) * xx - wt + phase_(ik));
    }
    eta = h;
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::Elevation(
      const Eigen::Ref<const Eigen::ArrayXd> &x,
      const Eigen::Ref<const Eigen::ArrayXd> &y,
      Eigen::Ref<Eigen::ArrayXd> eta)
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
  void LinearRandomWaveSimulation::Impl::Pressure(
      double /*x*/, double /*y*/, double /*z*/,
      double &pressure)
  {
    ComputeAmplitudes();

    /// \todo(srmainwaring) implement
    pressure = 0.0;
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::Pressure(
      const Eigen::Ref<const Eigen::ArrayXd> &x,
      const Eigen::Ref<const Eigen::ArrayXd> &y,
      const Eigen::Ref<const Eigen::ArrayXd> &z,
      Eigen::Ref<Eigen::ArrayXd> pressure)
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
  void LinearRandomWaveSimulation::Impl::ElevationAt(
      Index ix, Index iy,
      double &h)
  {
    double x = ix * dx_ + lx_min_;
    double y = iy * dy_ + ly_min_;
    Elevation(x, y, h);
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::PressureAt(
      Index ix, Index iy, Index iz,
      double &pressure)
  {
    double x = ix * dx_ + lx_min_;
    double y = iy * dy_ + ly_min_;
    double z = z_(iz);
    Pressure(x, y, z, pressure);
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> h)
  {
    for (Index ix = 0; ix < nx_; ++ix)
    {
      for (Index iy = 0; iy < ny_; ++iy)
      {
        double h1{0.0};
        ElevationAt(ix, iy, h1);
        h(ix, iy) = h1;
      }
    }
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> /*dhdx*/,
      Eigen::Ref<Eigen::ArrayXXd> /*dhdy*/)
  {
    ComputeAmplitudes();

    /// \todo(srmainwaring) implement
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure)
  {
    for (Index ix = 0; ix < nx_; ++ix)
    {
      for (Index iy = 0; iy < ny_; ++iy)
      {
        double p1{0.0};
        PressureAt(ix, iy, iz, p1);
        pressure(ix, iy) = p1;
      }
    }
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::Impl::InitGrid()
  {
    // grid spacing
    dx_ = lx_ / nx_;
    dy_ = ly_ / ny_;

    // x and y coordinates
    lx_min_ = - lx_ / 2.0;
    lx_max_ =   lx_ / 2.0;
    ly_min_ = - ly_ / 2.0;
    ly_max_ =   ly_ / 2.0;

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
  void LinearRandomWaveSimulation::Impl::ComputeAmplitudes()
  {
    if (!needs_update_)
      return;

    // resize workspace
    spectrum_.resize(num_waves_);
    amplitude_.resize(num_waves_);
    w_.resize(num_waves_);
    k_.resize(num_waves_);
    phase_.resize(num_waves_);

    // set spectrum and parameters
    PiersonMoskowitzWaveSpectrum spectrum;
    spectrum.SetU19(u19_);

    // angular frequency step size
    double dw = MAX_FREQ * 2 * M_PI / num_waves_;

    // random uniforms for phase
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> distribution(0.0, 2.0 * M_PI);

    /// \todo(srmainwaring) check spectrum
    double k_prev = 0.0;
    for (Index ik = 0; ik < num_waves_; ++ik)
    {
      // equally-spaced w => variably-spaced k
      w_(ik) = dw * (ik + 1);
      k_(ik) = w_(ik) * w_(ik) / gravity_;
      double dk = k_(ik) - k_prev;
      k_prev = k_(ik);

      // spectrum variable is k
      spectrum_(ik) = spectrum.Evaluate(k_(ik));
      amplitude_(ik) = std::sqrt(2.0 * dk * spectrum_(ik));
      phase_(ik) = distribution(generator);
    }

    needs_update_ = false;
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
  void LinearRandomWaveSimulation::SetNumWaves(Index value)
  {
    impl_->num_waves_ = value;
    impl_->needs_update_ = true;
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::SetWindVelocity(double ux, double uy)
  {
    /// \todo(srmainwaring) standardise reference level for wind
    impl_->u19_ = std::sqrt(ux * ux + uy * uy);
    impl_->wave_angle_ = std::atan2(uy, ux);
    impl_->needs_update_ = true;
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

  void LinearRandomWaveSimulation::Elevation(
      double x, double y,
      double &eta)
  {
    impl_->Elevation(x, y, eta);
  }

  void LinearRandomWaveSimulation::Elevation(
      const Eigen::Ref<const Eigen::ArrayXd> &x,
      const Eigen::Ref<const Eigen::ArrayXd> &y,
      Eigen::Ref<Eigen::ArrayXd> eta)
  {
    impl_->Elevation(x, y, eta);
  }

  void LinearRandomWaveSimulation::Pressure(
      double x, double y, double z,
      double &pressure)
  {
    impl_->Pressure(x, y, z, pressure);
  }

  void LinearRandomWaveSimulation::Pressure(
      const Eigen::Ref<const Eigen::ArrayXd> &x,
      const Eigen::Ref<const Eigen::ArrayXd> &y,
      const Eigen::Ref<const Eigen::ArrayXd> &z,
      Eigen::Ref<Eigen::ArrayXd> pressure)
  {
    impl_->Pressure(x, y, z, pressure);
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
      Eigen::Ref<Eigen::ArrayXXd> /*sx*/,
      Eigen::Ref<Eigen::ArrayXXd> /*sy*/)
  {
    // no xy-displacement
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> /*dsxdx*/,
      Eigen::Ref<Eigen::ArrayXXd> /*dsydy*/,
      Eigen::Ref<Eigen::ArrayXXd> /*dsxdy*/)
  {
    // no xy-displacement
  }

  //////////////////////////////////////////////////
  void LinearRandomWaveSimulation::DisplacementAndDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> h,
      Eigen::Ref<Eigen::ArrayXXd> /*sx*/,
      Eigen::Ref<Eigen::ArrayXXd> /*sy*/,
      Eigen::Ref<Eigen::ArrayXXd> dhdx,
      Eigen::Ref<Eigen::ArrayXXd> dhdy,
      Eigen::Ref<Eigen::ArrayXXd> /*dsxdx*/,
      Eigen::Ref<Eigen::ArrayXXd> /*dsydy*/,
      Eigen::Ref<Eigen::ArrayXXd> /*dsxdy*/)
  {
    impl_->ElevationAt(h);
    impl_->ElevationDerivAt(dhdx, dhdy);
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
      Index /*ix*/, Index /*iy*/,
      double &/*sx*/, double &/*sy*/)
  {
    // no xy-displacement
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