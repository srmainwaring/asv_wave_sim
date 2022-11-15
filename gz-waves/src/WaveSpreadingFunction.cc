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
#include <cmath>

/// \todo REMOVE
#include <iostream>

using namespace gz; 
using namespace waves; 

///////////////////////////////////////////////////////////////////////////////
DirectionalSpreadingFunction::~DirectionalSpreadingFunction()
{
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
Cos2sSpreadingFunction::~Cos2sSpreadingFunction()
{
}

///////////////////////////////////////////////////////////////////////////////
Cos2sSpreadingFunction::Cos2sSpreadingFunction(double _spread) :
    DirectionalSpreadingFunction(),
    _spread(_spread)
{
  this->_RecalcCoeffs();
}

///////////////////////////////////////////////////////////////////////////////
double Cos2sSpreadingFunction::Evaluate(
    double _theta, double _theta_mean, double _k) const
{
  const double s = this->_spread;
  const double phi = _theta - _theta_mean;
  const double cp = std::cos(phi / 2.0);
  const double p1 = std::pow(cp, 2.0 * s);
  const double cap_phi = this->_cap_c_s * p1;
  return cap_phi;
}

///////////////////////////////////////////////////////////////////////////////
void Cos2sSpreadingFunction::Evaluate(
    Eigen::Ref<Eigen::MatrixXd> _phi,
    const Eigen::Ref<const Eigen::MatrixXd> &_theta,
    double _theta_mean,
    const Eigen::Ref<const Eigen::MatrixXd> &_k) const
{
  const double s = this->_spread;
  const Eigen::MatrixXd phi = _theta.array() - _theta_mean;
  const Eigen::MatrixXd cp = Eigen::cos(phi.array() / 2.0);
  const Eigen::MatrixXd p1 = Eigen::pow(cp.array(), 2.0 * s);
  _phi = this->_cap_c_s * p1.array();
}

///////////////////////////////////////////////////////////////////////////////
double Cos2sSpreadingFunction::Spread() const
{
  return this->_spread;
}

///////////////////////////////////////////////////////////////////////////////
void Cos2sSpreadingFunction::SetSpread(double _value)
{
    this->_spread = _value;
    this->_RecalcCoeffs();
}

///////////////////////////////////////////////////////////////////////////////
void Cos2sSpreadingFunction::_RecalcCoeffs()
{
    const double s = this->_spread;
    const double g1 = std::tgamma(s + 1.0);
    const double g2 = std::tgamma(s + 0.5);
    this->_cap_c_s = g1 / g2 / 2.0 / std::sqrt(M_PI);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
ECKVSpreadingFunction::~ECKVSpreadingFunction()
{
}

///////////////////////////////////////////////////////////////////////////////
ECKVSpreadingFunction::ECKVSpreadingFunction(
    double _u10,
    double _cap_omega_c,
    double _gravity) :
    DirectionalSpreadingFunction(),
    _u10(_u10),
    _cap_omega_c(_cap_omega_c),
    _gravity(_gravity)
{
}

///////////////////////////////////////////////////////////////////////////////
double ECKVSpreadingFunction::Evaluate(
    double _theta, double _theta_mean, double _k) const
{
  const double phi = _theta - _theta_mean;
  const double g = this->_gravity;
  const double u10 = this->_u10;
  const double cap_omega_c = this->_cap_omega_c;
  
  constexpr double cd_10n = 0.00144;
  const double u_star = std::sqrt(cd_10n) * u10;
  constexpr double ao = 0.1733;
  constexpr double ap = 4.0;
  constexpr double km = 370.0;
  constexpr double cm = 0.23;
  const double am = 0.13 * u_star / cm;
  const double ko = g / u10 / u10;
  const double kp = ko * cap_omega_c * cap_omega_c;
  const double cp = std::sqrt(g / kp);
  const double c = std::sqrt((g / _k) * (1.0 + std::pow(_k / km, 2.0)));
  const double p1 = std::pow(c / cp, 2.5);
  const double p2 = std::pow(cm / c, 2.5);
  const double t1 = std::tanh(ao + ap * p1 + am * p2);
  const double c2p = std::cos(2.0 * phi);
  const double cap_phi = (1.0 + t1 * c2p) / 2.0 / M_PI;
  return cap_phi;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVSpreadingFunction::Evaluate(
    Eigen::Ref<Eigen::MatrixXd> _phi,
    const Eigen::Ref<const Eigen::MatrixXd> &_theta,
    double _theta_mean,
    const Eigen::Ref<const Eigen::MatrixXd> &_k) const
{
  /// \todo check the size of _phi,  _theta and _k match

  const double g = this->_gravity;
  const double u10 = this->_u10;
  const double cap_omega_c = this->_cap_omega_c;
  
  constexpr double cd_10n = 0.00144;
  const double u_star = std::sqrt(cd_10n) * u10;
  constexpr double ao = 0.1733;
  constexpr double ap = 4.0;
  constexpr double km = 370.0;
  constexpr double cm = 0.23;
  const double am = 0.13 * u_star / cm;
  const double ko = g / u10 / u10;
  const double kp = ko * cap_omega_c * cap_omega_c;
  const double cp = std::sqrt(g / kp);

  const Eigen::MatrixXd phi = _theta.array() - _theta_mean;
  const Eigen::MatrixXd c = Eigen::sqrt((g / _k.array())
                    * (1.0 + Eigen::pow(_k.array() / km, 2.0)));
  const Eigen::MatrixXd p1 = Eigen::pow(c.array() / cp, 2.5);
  const Eigen::MatrixXd p2 = Eigen::pow(cm / c.array(), 2.5);
  const Eigen::MatrixXd t1 = Eigen::tanh(ao + ap * p1.array() + am * p2.array());
  const Eigen::MatrixXd c2p = Eigen::cos(2.0 * phi.array());
  _phi = (1.0 + t1.array() * c2p.array()) / 2.0 / M_PI;
}

///////////////////////////////////////////////////////////////////////////////
double ECKVSpreadingFunction::Gravity() const
{
  return this->_gravity;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVSpreadingFunction::SetGravity(double _value)
{
  this->_gravity = _value;
}

///////////////////////////////////////////////////////////////////////////////
double ECKVSpreadingFunction::U10() const
{
  return this->_u10;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVSpreadingFunction::SetU10(double _value)
{
  this->_u10 = _value;
}

///////////////////////////////////////////////////////////////////////////////
double ECKVSpreadingFunction::CapOmegaC() const
{
  return this->_cap_omega_c;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVSpreadingFunction::SetCapOmegaC(double _value)
{
  this->_cap_omega_c = _value;
}

///////////////////////////////////////////////////////////////////////////////
