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

#include <Eigen/Dense>

#include <random>
#include <vector>

#include <gz/common/Console.hh>

#include "gz/waves/WaveSpectrum.hh"


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

  void SetTime(double value);

  // interpolation interface
  void Elevation(
      double x, double y,
      double& eta);

  void ElevationDeriv(
      double x, double y,
      double& deta_dx, double& deta_dy);

  void Elevation(
      const Eigen::Ref<const Eigen::ArrayXd>& x,
      const Eigen::Ref<const Eigen::ArrayXd>& y,
      Eigen::Ref<Eigen::ArrayXd> eta);

  void Pressure(
      double x, double y, double z,
      double& pressure);

  void Pressure(
      const Eigen::Ref<const Eigen::ArrayXd>& x,
      const Eigen::Ref<const Eigen::ArrayXd>& y,
      const Eigen::Ref<const Eigen::ArrayXd>& z,
      Eigen::Ref<Eigen::ArrayXd> pressure);

  // lookup interface - scalar
  void ElevationAt(
      Index ix, Index iy,
      double& h);

  void ElevationDerivAt(
      Index ix, Index iy,
      double& dhdx, double& dhdy);

  void PressureAt(
      Index ix, Index iy, Index iz,
      double& pressure);

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
  Index num_waves_{100};
  double max_w_{6.0};
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
  nx_(nx),
  ny_(ny),
  lx_(lx),
  ly_(ly)
{
  InitGrid();
}

//////////////////////////////////////////////////
LinearRandomWaveSimulation::Impl::Impl(
  double lx, double ly, double lz, Index nx, Index ny, Index nz) :
  nx_(nx),
  ny_(ny),
  nz_(nz),
  lx_(lx),
  ly_(ly),
  lz_(lz)
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
    double& eta)
{
  ComputeAmplitudes();

  double cd = std::cos(wave_angle_);
  double sd = std::sin(wave_angle_);
  double xd = x * cd + y * sd;

  double h = 0;
  for (Index ik = 0; ik < num_waves_; ++ik)
  {
    double wt = w_(ik) * time_;
    double a  = k_(ik) * xd - wt + phase_(ik);
    double ca = std::cos(a);
    h += amplitude_(ik) * ca;
  }
  eta = h;
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::Impl::ElevationDeriv(
    double x, double y,
    double& deta_dx, double& deta_dy)
{
  ComputeAmplitudes();

  double cd = std::cos(wave_angle_);
  double sd = std::sin(wave_angle_);
  double xd  = x * cd + y * sd;

  double dhdx = 0;
  double dhdy = 0;
  for (Index ik = 0; ik < num_waves_; ++ik)
  {
    double wt = w_(ik) * time_;
    double a  = k_(ik) * xd - wt + phase_(ik);
    double sa = std::sin(a);
    double dadx = k_(ik) * cd;
    double dady = k_(ik) * sd;

    dhdx += - dadx * amplitude_(ik) * sa;
    dhdy += - dady * amplitude_(ik) * sa;
  }
  deta_dx = dhdx;
  deta_dy = dhdy;
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::Impl::Elevation(
    const Eigen::Ref<const Eigen::ArrayXd>& x,
    const Eigen::Ref<const Eigen::ArrayXd>& y,
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
    double x, double y, double z,
    double& pressure)
{
  ComputeAmplitudes();

  double cd = std::cos(wave_angle_);
  double sd = std::sin(wave_angle_);
  double xd = x * cd + y * sd;

  double p = 0;
  for (Index ik = 0; ik < num_waves_; ++ik)
  {
    double wt = w_(ik) * time_;
    double a  = k_(ik) * xd - wt + phase_(ik);
    double ca = std::cos(a);
    double h1 = amplitude_(ik) * ca;
    double e  = std::exp(k_(ik) * z);
    p += e * h1;
  }
  pressure = p;
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::Impl::Pressure(
    const Eigen::Ref<const Eigen::ArrayXd>& x,
    const Eigen::Ref<const Eigen::ArrayXd>& y,
    const Eigen::Ref<const Eigen::ArrayXd>& z,
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
    double& h)
{
  double x = ix * dx_ + lx_min_;
  double y = iy * dy_ + ly_min_;
  Elevation(x, y, h);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::Impl::ElevationDerivAt(
    Index ix, Index iy,
    double& dhdx, double& dhdy)
{
  double x = ix * dx_ + lx_min_;
  double y = iy * dy_ + ly_min_;
  ElevationDeriv(x, y, dhdx, dhdy);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::Impl::PressureAt(
    Index ix, Index iy, Index iz,
    double& pressure)
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

      // column major
      Index idx = iy * nx_ + ix;
      h(idx, 0) = h1;
    }
  }
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::Impl::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy)
{
  for (Index ix = 0; ix < nx_; ++ix)
  {
    for (Index iy = 0; iy < ny_; ++iy)
    {
      double dhdx1{0.0};
      double dhdy1{0.0};
      ElevationDerivAt(ix, iy, dhdx1, dhdy1);

      // column major
      Index idx = iy * nx_ + ix;
      dhdx(idx, 0) = dhdx1;
      dhdy(idx, 0) = dhdy1;
    }
  }
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

      // column major
      Index idx = iy * nx_ + ix;
      pressure(idx, 0) = p1;
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
  double dw = max_w_ / num_waves_;

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
Index LinearRandomWaveSimulation::NumWaves() const
{
  return impl_->num_waves_;
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::SetNumWaves(Index value)
{
  impl_->num_waves_ = value;
  impl_->needs_update_ = true;
}

//////////////////////////////////////////////////
double LinearRandomWaveSimulation::MaxOmega() const
{
  return impl_->max_w_;
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::SetMaxOmega(double value)
{
  impl_->max_w_ = value;
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
void LinearRandomWaveSimulation::SetSteepness(double /*value*/) // NOLINT
{
  /// \todo(srmainwaring) IMPLEMENT
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
    double &eta) const
{
  impl_->Elevation(x, y, eta);
}

void LinearRandomWaveSimulation::Elevation(
    const Eigen::Ref<const Eigen::ArrayXd>& x,
    const Eigen::Ref<const Eigen::ArrayXd>& y,
    Eigen::Ref<Eigen::ArrayXd> eta) const
{
  impl_->Elevation(x, y, eta);
}

void LinearRandomWaveSimulation::Pressure(
    double x, double y, double z,
    double& pressure) const
{
  impl_->Pressure(x, y, z, pressure);
}

void LinearRandomWaveSimulation::Pressure(
    const Eigen::Ref<const Eigen::ArrayXd>& x,
    const Eigen::Ref<const Eigen::ArrayXd>& y,
    const Eigen::Ref<const Eigen::ArrayXd>& z,
    Eigen::Ref<Eigen::ArrayXd> pressure) const
{
  impl_->Pressure(x, y, z, pressure);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h) const
{
  impl_->ElevationAt(h);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy) const
{
  impl_->ElevationDerivAt(dhdx, dhdy);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> /*sx*/,
    Eigen::Ref<Eigen::ArrayXXd> /*sy*/) const
{
  // no xy-displacement
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> /*dsxdx*/,
    Eigen::Ref<Eigen::ArrayXXd> /*dsydy*/,
    Eigen::Ref<Eigen::ArrayXXd> /*dsxdy*/) const
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
    Eigen::Ref<Eigen::ArrayXXd> /*dsxdy*/) const
{
  impl_->ElevationAt(h);
  impl_->ElevationDerivAt(dhdx, dhdy);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::PressureAt(
    Index iz,
    Eigen::Ref<Eigen::ArrayXXd> pressure) const
{
  impl_->PressureAt(iz, pressure);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::ElevationAt(
    Index ix, Index iy,
    double& eta) const
{
  impl_->ElevationAt(ix, iy, eta);
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::DisplacementAt(
    Index /*ix*/, Index /*iy*/,
    double& /*sx*/, double& /*sy*/) const
{
  // no xy-displacement
}

//////////////////////////////////////////////////
void LinearRandomWaveSimulation::PressureAt(
    Index ix, Index iy, Index iz,
    double& pressure) const
{
  impl_->PressureAt(ix, iy, iz, pressure);
}

}  // namespace waves
}  // namespace gz
