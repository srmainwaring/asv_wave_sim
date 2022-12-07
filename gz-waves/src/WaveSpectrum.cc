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

#include "gz/waves/WaveSpectrum.hh"

#include <algorithm>
#include <cmath>

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
OmniDirectionalWaveSpectrum::~OmniDirectionalWaveSpectrum()
{
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
PiersonMoskowitzWaveSpectrum::~PiersonMoskowitzWaveSpectrum()
{
}

//////////////////////////////////////////////////
PiersonMoskowitzWaveSpectrum::PiersonMoskowitzWaveSpectrum(
    double u19, double gravity) :
  OmniDirectionalWaveSpectrum(),
  gravity_(gravity),
  u19_(u19)
{
}

//////////////////////////////////////////////////
double PiersonMoskowitzWaveSpectrum::Evaluate(double k) const
{
  if (std::abs(k) < 1.0E-8 || std::abs(u19_) < 1.0E-8)
  {
    return 0.0;
  }

  // constants
  const double alpha = 0.0081;
  const double beta = 0.74;

  // intermediates
  double g2 = gravity_ * gravity_;
  double k2 = k * k;
  double k3 = k * k2;
  double u2 = u19_ * u19_;
  double u4 = u2 * u2;

  double cap_s = alpha / 2.0 / k3 * std::exp(-beta * g2 / k2 / u4);
  return cap_s;
}

//////////////////////////////////////////////////
void PiersonMoskowitzWaveSpectrum::Evaluate(
    Eigen::Ref<Eigen::ArrayXXd> spectrum,
    const Eigen::Ref<const Eigen::ArrayXXd>& k) const
{
  /// \note Eigen asserts cbegin and cend are from the same expression.
  auto k_view = k.reshaped();
  std::transform(
    k_view.cbegin(),
    k_view.cend(),
    spectrum.reshaped().begin(),
    [this] (double k_i) -> double
    {
      return this->Evaluate(k_i);
    });
}

//////////////////////////////////////////////////
double PiersonMoskowitzWaveSpectrum::Gravity() const
{
  return gravity_;
}

//////////////////////////////////////////////////
void PiersonMoskowitzWaveSpectrum::SetGravity(double value)
{
  gravity_ = value;
}

//////////////////////////////////////////////////
double PiersonMoskowitzWaveSpectrum::U19() const
{
  return u19_;
}

//////////////////////////////////////////////////
void PiersonMoskowitzWaveSpectrum::SetU19(double value)
{
  u19_ = value;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
ECKVWaveSpectrum::~ECKVWaveSpectrum()
{
}

//////////////////////////////////////////////////
ECKVWaveSpectrum::ECKVWaveSpectrum(
    double u10, double cap_omega_c, double gravity) :
  OmniDirectionalWaveSpectrum(),
  gravity_(gravity),
  u10_(u10),
  cap_omega_c_(cap_omega_c)
{
}

//////////////////////////////////////////////////
double ECKVWaveSpectrum::Evaluate(double k) const
{
  if (std::abs(k) < 1.0E-8 || std::abs(u10_) < 1.0E-8)
  {
    return 0.0;
  }

  // constants
  // const double alpha = 0.0081;
  // const double beta = 1.25;
  const double cd_10n = 0.00144;
  // const double ao = 0.1733;
  // const double ap = 4.0;
  const double km = 370.0;
  const double cm = 0.23;

  // intermediates
  double u_star = std::sqrt(cd_10n) * u10_;
  // double am = 0.13 * u_star / cm;

  double gamma = 1.7;
  if (cap_omega_c_ < 1.0)
  {
    gamma = 1.7;
  } else {
    gamma = 1.7 + 6.0 * std::log10(cap_omega_c_);
  }

  double sigma = 0.08 * (1.0 + 4.0 * std::pow(cap_omega_c_, -3.0));
  double alpha_p = 0.006 * std::pow(cap_omega_c_, 0.55);

  double alpha_m;
  if (u_star <= cm)
  {
    alpha_m = 0.01 * (1.0 + std::log(u_star / cm));
  } else {
    alpha_m = 0.01 * (1.0 + 3.0 * std::log(u_star / cm));
  }

  double ko = gravity_ / u10_ / u10_;
  double kp = ko * cap_omega_c_ * cap_omega_c_;

  double cp = std::sqrt(gravity_ / kp);
  double c  = std::sqrt((gravity_ / k) * (1.0 + std::pow(k / km, 2.0)));

  double l_pm = std::exp(-1.25 * std::pow(kp / k, 2.0));

  double cap_gamma = std::exp(
      -1.0/(2.0 * std::pow(sigma, 2.0))
      * std::pow(std::sqrt(k / kp) - 1.0, 2.0));

  double j_p = std::pow(gamma, cap_gamma);

  double f_p = l_pm * j_p * std::exp(
      -0.3162 * cap_omega_c_ * (std::sqrt(k / kp) - 1.0));

  double f_m = l_pm * j_p * std::exp(
      -0.25 * std::pow(k / km - 1.0, 2.0));

  double b_l = 0.5 * alpha_p * (cp / c) * f_p;
  double b_h = 0.5 * alpha_m * (cm / c) * f_m;

  double k3 = k * k * k;
  double cap_s = (b_l + b_h) / k3;

  return cap_s;
}

//////////////////////////////////////////////////
void ECKVWaveSpectrum::Evaluate(
    Eigen::Ref<Eigen::ArrayXXd> spectrum,
    const Eigen::Ref<const Eigen::ArrayXXd>& k) const
{
  auto k_view = k.reshaped();
  std::transform(
    k_view.cbegin(),
    k_view.cend(),
    spectrum.reshaped().begin(),
    [this] (double k_i) -> double
    {
      return this->Evaluate(k_i);
    });
}

//////////////////////////////////////////////////
double ECKVWaveSpectrum::Gravity() const
{
  return gravity_;
}

//////////////////////////////////////////////////
void ECKVWaveSpectrum::SetGravity(double value)
{
  gravity_ = value;
}

//////////////////////////////////////////////////
double ECKVWaveSpectrum::U10() const
{
  return u10_;
}

//////////////////////////////////////////////////
void ECKVWaveSpectrum::SetU10(double value)
{
  u10_ = value;
}

//////////////////////////////////////////////////
double ECKVWaveSpectrum::CapOmegaC() const
{
  return cap_omega_c_;
}

//////////////////////////////////////////////////
void ECKVWaveSpectrum::SetCapOmegaC(double value)
{
  cap_omega_c_ = value;
}

}  // namespace waves
}  // namespace gz
