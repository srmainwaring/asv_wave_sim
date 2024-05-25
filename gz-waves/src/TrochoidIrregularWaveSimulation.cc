// Copyright (C) 2019-2023  Rhys Mainwaring
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

#include "gz/waves/TrochoidIrregularWaveSimulation.hh"

#include <Eigen/Dense>

#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector2.hh>

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
class TrochoidIrregularWaveSimulation::Impl
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~Impl();

  Impl(double lx, double ly, Index nx, Index ny);

  void InitGrid();

  // interpolation interface
  void Elevation(
      double x, double y,
      double &eta);

  void Elevation(
      const Eigen::Ref<const Eigen::ArrayXd>& x,
      const Eigen::Ref<const Eigen::ArrayXd>& y,
      Eigen::Ref<Eigen::ArrayXd> eta);

  void Pressure(
      double x, double y, double z,
      double &pressure);

  void Pressure(
      const Eigen::Ref<const Eigen::ArrayXd>& x,
      const Eigen::Ref<const Eigen::ArrayXd>& y,
      const Eigen::Ref<const Eigen::ArrayXd>& z,
      Eigen::Ref<Eigen::ArrayXd> pressure);

  // lookup interface
  void ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> _heights);

  void ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> _dhdx,
      Eigen::Ref<Eigen::ArrayXXd> _dhdy);

  void DisplacementAt(
      Eigen::Ref<Eigen::ArrayXXd> _sx,
      Eigen::Ref<Eigen::ArrayXXd> _sy);

  void DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> _dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> _dsydy,
      Eigen::Ref<Eigen::ArrayXXd> _dsxdy);

  void ElevationAt(
      Index ix, Index iy,
      double &h);

  void DisplacementAt(
      Index ix, Index iy,
      double& sx, double& sy);

  void PressureAt(
      Index ix, Index iy, Index iz,
      double &pressure);

  void PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure);

  bool CheckValid();

  // elevation and pressure grid params
  Index nx_{2};
  Index ny_{2};
  Index nz_{1};
  double lx_{1.0};
  double ly_{1.0};
  double lz_{0.0};

  // wave params
  Index number_{1};
  std::vector<double> amplitude_ = {1.0};
  std::vector<double> wavenumber_ = {2.0 * M_PI};
  std::vector<double> omega_ = {0.0};
  std::vector<double> phase_ = {0.0};
  std::vector<double> q_ = {1.0};
  std::vector<math::Vector2d> direction_ = {math::Vector2d(1.0, 0.0)};

  // checks
  bool is_param_check_dirty_{false};
  bool is_valid_{true};
  double time_;

  // derived
  double dx_{0.0};
  double dy_{0.0};
  double lx_max_{0.0};
  double lx_min_{0.0};
  double ly_max_{0.0};
  double ly_min_{0.0};
};

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::Impl::~Impl() = default;

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::Impl::Impl(
    double lx, double ly, Index nx, Index ny) :
  nx_(nx),
  ny_(ny),
  lx_(lx),
  ly_(ly)
{
  InitGrid();
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::InitGrid()
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
  // x_ = Eigen::ArrayXd::LinSpaced(nx_, lx_min_, lx_max_ - dx_);
  // y_ = Eigen::ArrayXd::LinSpaced(ny_, ly_min_, ly_max_ - dy_);

  // broadcast to matrices (aka meshgrid)
  // x_grid_ = Eigen::ArrayXXd::Zero(nx_, ny_);
  // y_grid_ = Eigen::ArrayXXd::Zero(nx_, ny_);
  // x_grid_.colwise() += x_;
  // y_grid_.rowwise() += y_.transpose();
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::Elevation(
    double /*x*/, double /*y*/,
    double &/*eta*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "Elevation: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::Elevation(
    const Eigen::Ref<const Eigen::ArrayXd>& /*x*/,
    const Eigen::Ref<const Eigen::ArrayXd>& /*y*/,
    Eigen::Ref<Eigen::ArrayXd> /*eta*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "Elevation: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::Pressure(
    double /*x*/, double /*y*/, double /*z*/,
    double &/*pressure*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "Pressure: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::Pressure(
    const Eigen::Ref<const Eigen::ArrayXd>& /*x*/,
    const Eigen::Ref<const Eigen::ArrayXd>& /*y*/,
    const Eigen::Ref<const Eigen::ArrayXd>& /*z*/,
    Eigen::Ref<Eigen::ArrayXd> /*pressure*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "Pressure: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h)
{
  if (!CheckValid())
    return;

  h = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  for (Index iw = 0; iw < number_; ++iw)
  {
    double a_i = amplitude_[iw];
    double k_i = wavenumber_[iw];
    double w_i = omega_[iw];
    double phi_i = phase_[iw];
    auto dir_i = direction_[iw];
    double cd_i = dir_i.X();
    double sd_i = dir_i.Y();

    for (Index iy = 0; iy < ny_; ++iy)
    {
      double y = iy * dy_ + ly_min_;
      for (Index ix = 0; ix < nx_; ++ix)
      {
        // col major index
        Index idx = iy * nx_ + ix;

        double x = ix * dx_ + lx_min_;
        double ddotx = x * cd_i + y * sd_i;
        double angle  = ddotx * k_i - w_i * time_ + phi_i;
        double ca = std::cos(angle);
        double h1 = a_i * ca;
        h(idx, 0) += h1;
      }
    }
  }
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy)
{
  if (!CheckValid())
    return;

  dhdx = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  dhdy = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  for (Index iw = 0; iw < number_; ++iw)
  {
    double a_i = amplitude_[iw];
    double k_i = wavenumber_[iw];
    double w_i = omega_[iw];
    double phi_i = phase_[iw];
    auto dir_i = direction_[iw];
    double cd_i = dir_i.X();
    double sd_i = dir_i.Y();
    double dadx = cd_i * k_i;
    double dady = sd_i * k_i;

    for (Index iy = 0; iy < ny_; ++iy)
    {
      double y = iy * dy_ + ly_min_;
      for (Index ix = 0; ix < nx_; ++ix)
      {
        // col major index
        Index idx = iy * nx_ + ix;

        double x = ix * dx_ + lx_min_;
        double ddotx = x * cd_i + y * sd_i;
        double angle  = ddotx * k_i - w_i * time_ + phi_i;
        double sa = std::sin(angle);
        double dhdx1 = - dadx * a_i * sa;
        double dhdy1 = - dady * a_i * sa;
        dhdx(idx, 0) += dhdx1;
        dhdy(idx, 0) += dhdy1;
      }
    }
  }
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy)
{
  if (!CheckValid())
    return;

  sx = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  sy = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  for (Index iw = 0; iw < number_; ++iw)
  {
    double a_i = amplitude_[iw];
    double k_i = wavenumber_[iw];
    double w_i = omega_[iw];
    double phi_i = phase_[iw];
    double q_i = q_[iw];
    auto dir_i = direction_[iw];
    double cd_i = dir_i.X();
    double sd_i = dir_i.Y();

    for (Index iy = 0; iy < ny_; ++iy)
    {
      double y = iy * dy_ + ly_min_;
      for (Index ix = 0; ix < nx_; ++ix)
      {
        // col major index
        Index idx = iy * nx_ + ix;

        double x = ix * dx_ + lx_min_;
        double ddotx = x * cd_i + y * sd_i;
        double angle  = ddotx * k_i - w_i * time_ + phi_i;
        double sa = std::sin(angle);
        double sx1 = - cd_i * q_i * a_i * sa;
        double sy1 = - sd_i * q_i * a_i * sa;
        sx(idx, 0) += sx1;
        sy(idx, 0) += sy1;
      }
    }
  }
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy)
{
  if (!CheckValid())
    return;

  dsxdx = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  dsydy = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  dsxdy = Eigen::ArrayXXd::Zero(nx_ * ny_, 0);
  for (Index iw = 0; iw < number_; ++iw)
  {
    double a_i = amplitude_[iw];
    double k_i = wavenumber_[iw];
    double w_i = omega_[iw];
    double phi_i = phase_[iw];
    double q_i = q_[iw];
    auto dir_i = direction_[iw];
    double cd_i = dir_i.X();
    double sd_i = dir_i.Y();
    double dadx = cd_i * k_i;
    double dady = sd_i * k_i;

    for (Index iy = 0; iy < ny_; ++iy)
    {
      double y = iy * dy_ + ly_min_;
      for (Index ix = 0; ix < nx_; ++ix)
      {
        // col major index
        Index idx = iy * nx_ + ix;

        double x = ix * dx_ + lx_min_;
        double ddotx = x * cd_i + y * sd_i;
        double angle  = ddotx * k_i - w_i * time_ + phi_i;
        double ca = std::cos(angle);
        double dsxdx1 = - cd_i * q_i * a_i * dadx * ca;
        double dsydy1 = - sd_i * q_i * a_i * dady * ca;
        double dsxdy1 = - cd_i * q_i * a_i * dady * ca;
        dsxdx(idx, 0) += dsxdx1;
        dsydy(idx, 0) += dsydy1;
        dsxdy(idx, 0) += dsxdy1;
      }
    }
  }
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::ElevationAt(
    Index /*ix*/, Index /*iy*/,
    double &/*h*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "ElevationAt: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::DisplacementAt(
    Index /*ix*/, Index /*iy*/,
    double& /*sx*/, double& /*sy*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "DisplacementAt: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::PressureAt(
    Index /*ix*/, Index /*iy*/, Index /*iz*/,
    double &/*pressure*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "PressureAt: Not implemented!\n";
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::PressureAt(
    Index /*iz*/,
    Eigen::Ref<Eigen::ArrayXXd> /*pressure*/)
{
  /// \todo(srmainwaring) IMPLEMENT
  gzerr << "PressureAt: Not implemented!\n";
}

//////////////////////////////////////////////////
bool TrochoidIrregularWaveSimulation::Impl::CheckValid()
{
  if (!is_param_check_dirty_)
    return is_valid_;

  is_valid_ = true;

  if (amplitude_.size() != static_cast<size_t>(number_))
  {
    gzerr << "Amplitude array must have size = " << number_ << ".\n";
    is_valid_ &= false;
  }
  if (wavenumber_.size() != static_cast<size_t>(number_))
  {
    gzerr << "Wavenumber array must have size = " << number_ << ".\n";
    is_valid_ &= false;
  }
  if (omega_.size() != static_cast<size_t>(number_))
  {
    gzerr << "Angular frequency array must have size = " << number_ << ".\n";
    is_valid_ &= false;
  }
  if (phase_.size() != static_cast<size_t>(number_))
  {
    gzerr << "Phase array must have size = " << number_ << ".\n";
    is_valid_ &= false;
  }
  if (q_.size() != static_cast<size_t>(number_))
  {
    gzerr << "Steepness array must have size = " << number_ << ".\n";
    is_valid_ &= false;
  }
  if (direction_.size() != static_cast<size_t>(number_))
  {
    gzerr << "Direction array must have size = " << number_ << ".\n";
    is_valid_ &= false;
  }

  is_param_check_dirty_ = false;
  return is_valid_;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::~TrochoidIrregularWaveSimulation() = default;

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::TrochoidIrregularWaveSimulation(
    double lx, double ly, Index nx, Index ny) :
  impl_(new TrochoidIrregularWaveSimulation::Impl(lx, ly, nx, ny))
{
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetNumber(Index value)
{
  impl_->number_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetAmplitude(
    const std::vector<double>& value)
{
  impl_->amplitude_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetWaveNumber(
    const std::vector<double>& value)
{
  impl_->wavenumber_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetOmega(
    const std::vector<double>& value)
{
  impl_->omega_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetPhase(
    const std::vector<double>& value)
{
  impl_->phase_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetSteepness(
    const std::vector<double>& value)
{
  impl_->q_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetDirection(
    const std::vector<math::Vector2d>& value)
{
  impl_->direction_ = value;
  impl_->is_param_check_dirty_ = true;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetWindVelocity(
    double /*ux*/, double /*uy*/)
{
  /// \note NO-OP
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetSteepness(
    double /*value*/)
{
  /// \todo(srmainwaring) DEPRECATE - not used
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetTime(double time)
{
  impl_->time_ = time;
}

//////////////////////////////////////////////////
Index TrochoidIrregularWaveSimulation::SizeX() const
{
  return impl_->nx_;
}

//////////////////////////////////////////////////
Index TrochoidIrregularWaveSimulation::SizeY() const
{
  return impl_->ny_;
}

//////////////////////////////////////////////////
Index TrochoidIrregularWaveSimulation::SizeZ() const
{
  return impl_->nz_;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Elevation(
    double x, double y,
    double& eta) const
{
  impl_->Elevation(x, y, eta);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Elevation(
    const Eigen::Ref<const Eigen::ArrayXd>& x,
    const Eigen::Ref<const Eigen::ArrayXd>& y,
    Eigen::Ref<Eigen::ArrayXd> eta) const
{
  impl_->Elevation(x, y, eta);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Pressure(
    double x, double y, double z,
    double& pressure) const
{
  impl_->Pressure(x, y, z, pressure);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Pressure(
    const Eigen::Ref<const Eigen::ArrayXd>& x,
    const Eigen::Ref<const Eigen::ArrayXd>& y,
    const Eigen::Ref<const Eigen::ArrayXd>& z,
    Eigen::Ref<Eigen::ArrayXd> pressure) const
{
  impl_->Pressure(x, y, z, pressure);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h) const
{
  impl_->ElevationAt(h);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy) const
{
  impl_->ElevationDerivAt(dhdx, dhdy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy) const
{
  impl_->DisplacementAt(sx, sy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
{
  impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementAndDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> h,
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy,
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
{
  impl_->ElevationAt(h);
  /// \todo undo flip of dhdx <---> dhdy once render plugin fixed
  impl_->ElevationDerivAt(dhdy, dhdx);
  // impl_->ElevationDerivAt(dhdx, dhdy);
  impl_->DisplacementAt(sx, sy);
  impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::PressureAt(
    Index iz,
    Eigen::Ref<Eigen::ArrayXXd> pressure) const
{
  impl_->PressureAt(iz, pressure);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::ElevationAt(
    Index ix, Index iy,
    double& eta) const
{
  impl_->ElevationAt(ix, iy, eta);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementAt(
    Index ix, Index iy,
    double& sx, double& sy) const
{
  impl_->DisplacementAt(ix, iy, sx, sy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::PressureAt(
    Index ix, Index iy, Index iz,
    double& pressure) const
{
  impl_->PressureAt(ix, iy, iz, pressure);
}

}  // namespace waves
}  // namespace gz
