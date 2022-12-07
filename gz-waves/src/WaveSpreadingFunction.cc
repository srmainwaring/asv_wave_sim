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

#include "gz/waves/WaveSpreadingFunction.hh"

#include <algorithm>
#include <cmath>

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
DirectionalSpreadingFunction::~DirectionalSpreadingFunction()
{
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
Cos2sSpreadingFunction::~Cos2sSpreadingFunction()
{
}

//////////////////////////////////////////////////
Cos2sSpreadingFunction::Cos2sSpreadingFunction(double spread) :
    DirectionalSpreadingFunction(),
    spread_(spread)
{
  RecalcCoeffs();
}

//////////////////////////////////////////////////
double Cos2sSpreadingFunction::Evaluate(
    double theta, double theta_mean, double /*k*/) const
{
  double phi = theta - theta_mean;
  double cp = std::cos(phi / 2.0);
  double p1 = std::pow(cp, 2.0 * spread_);
  double cap_phi = cap_c_s_ * p1;
  return cap_phi;
}

//////////////////////////////////////////////////
void Cos2sSpreadingFunction::Evaluate(
    Eigen::Ref<Eigen::ArrayXXd> phi,
    const Eigen::Ref<const Eigen::ArrayXXd>& theta,
    double theta_mean,
    const Eigen::Ref<const Eigen::ArrayXXd>& /*k*/) const
{
  auto theta_view = theta.reshaped();
  std::transform(
    theta_view.cbegin(),
    theta_view.cend(),
    phi.reshaped().begin(),
    [&, this] (double theta_i) -> double
    {
      return this->Evaluate(theta_i, theta_mean);
    });
}

//////////////////////////////////////////////////
double Cos2sSpreadingFunction::Spread() const
{
  return spread_;
}

//////////////////////////////////////////////////
void Cos2sSpreadingFunction::SetSpread(double value)
{
    spread_ = value;
    RecalcCoeffs();
}

//////////////////////////////////////////////////
void Cos2sSpreadingFunction::RecalcCoeffs()
{
  double g1 = std::tgamma(spread_ + 1.0);
  double g2 = std::tgamma(spread_ + 0.5);
  cap_c_s_ = g1 / g2 / 2.0 / std::sqrt(M_PI);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
ECKVSpreadingFunction::~ECKVSpreadingFunction()
{
}

//////////////////////////////////////////////////
ECKVSpreadingFunction::ECKVSpreadingFunction(
    double u10,
    double cap_omega_c,
    double gravity) :
  DirectionalSpreadingFunction(),
  gravity_(gravity),
  u10_(u10),
  cap_omega_c_(cap_omega_c)
{
}

//////////////////////////////////////////////////
double ECKVSpreadingFunction::Evaluate(
    double theta, double theta_mean, double k) const
{
  double angle = theta - theta_mean;

  const double cd_10n = 0.00144;
  const double ao = 0.1733;
  const double ap = 4.0;
  const double km = 370.0;
  const double cm = 0.23;
  double u_star = std::sqrt(cd_10n) * u10_;
  double am = 0.13 * u_star / cm;
  double ko = gravity_ / u10_ / u10_;
  double kp = ko * cap_omega_c_ * cap_omega_c_;
  double cp = std::sqrt(gravity_ / kp);
  double c = std::sqrt((gravity_ / k) * (1.0 + std::pow(k / km, 2.0)));
  double p1 = std::pow(c / cp, 2.5);
  double p2 = std::pow(cm / c, 2.5);
  double t1 = std::tanh(ao + ap * p1 + am * p2);
  double c2p = std::cos(2.0 * angle);
  double cap_phi = (1.0 + t1 * c2p) / 2.0 / M_PI;
  return cap_phi;
}

//////////////////////////////////////////////////
void ECKVSpreadingFunction::Evaluate(
    Eigen::Ref<Eigen::ArrayXXd> phi,
    const Eigen::Ref<const Eigen::ArrayXXd>& theta,
    double theta_mean,
    const Eigen::Ref<const Eigen::ArrayXXd>& k) const
{
  auto theta_view = theta.reshaped();
  std::transform(
    theta_view.cbegin(),
    theta_view.cend(),
    k.reshaped().cbegin(),
    phi.reshaped().begin(),
    [&, this] (double theta_i, double k_i) -> double
    {
      return this->Evaluate(theta_i, theta_mean, k_i);
    });
}

//////////////////////////////////////////////////
double ECKVSpreadingFunction::Gravity() const
{
  return gravity_;
}

//////////////////////////////////////////////////
void ECKVSpreadingFunction::SetGravity(double value)
{
  gravity_ = value;
}

//////////////////////////////////////////////////
double ECKVSpreadingFunction::U10() const
{
  return u10_;
}

//////////////////////////////////////////////////
void ECKVSpreadingFunction::SetU10(double value)
{
  u10_ = value;
}

//////////////////////////////////////////////////
double ECKVSpreadingFunction::CapOmegaC() const
{
  return cap_omega_c_;
}

//////////////////////////////////////////////////
void ECKVSpreadingFunction::SetCapOmegaC(double value)
{
  cap_omega_c_ = value;
}

}  // namespace waves
}  // namespace gz
