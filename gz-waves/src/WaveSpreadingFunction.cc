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
  double s = this->_spread;
  double phi = _theta - _theta_mean;
  double cp = std::cos(phi / 2.0);
  double p1 = std::pow(cp, 2.0 * s);
  double cap_phi = this->_cap_c_s * p1;
  return cap_phi;
}

///////////////////////////////////////////////////////////////////////////////
void Cos2sSpreadingFunction::Evaluate(
    Eigen::Ref<Eigen::VectorXd> _phi,
    const Eigen::Ref<const Eigen::VectorXd> &_theta,
    double _theta_mean,
    const Eigen::Ref<const Eigen::VectorXd> &_k) const
{
  double s = this->_spread;
  Eigen::VectorXd phi = _theta.array() - _theta_mean;
  Eigen::VectorXd cp = Eigen::cos(phi.array() / 2.0);
  Eigen::VectorXd p1 = Eigen::pow(cp.array(), 2.0 * s);
  _phi = this->_cap_c_s * p1;
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
    double s = this->_spread;
    double g1 = std::tgamma(s + 1.0);
    double g2 = std::tgamma(s + 0.5);
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
  double phi = _theta - _theta_mean;
  double g = this->_gravity;
  double u10 = this->_u10;
  double cap_omega_c = this->_cap_omega_c;
  
  double cd_10n = 0.00144;
  double u_star = std::sqrt(cd_10n) * u10;
  double ao = 0.1733;
  double ap = 4.0;
  double km = 370.0;
  double cm = 0.23;
  double am = 0.13 * u_star / cm;
  double ko = g / u10 / u10;
  double kp = ko * cap_omega_c * cap_omega_c;
  double cp = std::sqrt(g / kp);
  double c = std::sqrt((g / _k) * (1.0 + std::pow(_k / km, 2.0)));
  double p1 = std::pow(c / cp, 2.5);
  double p2 = std::pow(cm / c, 2.5);
  double t1 = std::tanh(ao + ap * p1 + am * p2);
  double c2p = std::cos(2.0 * phi);
  double cap_phi = (1.0 + t1 * c2p) / 2.0 / M_PI;
  return cap_phi;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVSpreadingFunction::Evaluate(
    Eigen::Ref<Eigen::VectorXd> _phi,
    const Eigen::Ref<const Eigen::VectorXd> &_theta,
    double _theta_mean,
    const Eigen::Ref<const Eigen::VectorXd> &_k) const
{
  /// \todo check the size of _phi,  _theta and _k match

  double g = this->_gravity;
  double u10 = this->_u10;
  double cap_omega_c = this->_cap_omega_c;
  
  double cd_10n = 0.00144;
  double u_star = std::sqrt(cd_10n) * u10;
  double ao = 0.1733;
  double ap = 4.0;
  double km = 370.0;
  double cm = 0.23;
  double am = 0.13 * u_star / cm;
  double ko = g / u10 / u10;
  double kp = ko * cap_omega_c * cap_omega_c;
  double cp = std::sqrt(g / kp);

  Eigen::VectorXd phi = _theta.array() - _theta_mean;
  Eigen::VectorXd c = Eigen::sqrt((g / _k.array())
                    * (1.0 + Eigen::pow(_k.array() / km, 2.0)));
  Eigen::VectorXd p1 = Eigen::pow(c.array() / cp, 2.5);
  Eigen::VectorXd p2 = Eigen::pow(cm / c.array(), 2.5);
  Eigen::VectorXd t1 = Eigen::tanh(ao + ap * p1.array() + am * p2.array());
  Eigen::VectorXd c2p = Eigen::cos(2.0 * phi.array());
  _phi = (1.0 + t1.array() * c2p.array()) / 2.0 / M_PI;
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
