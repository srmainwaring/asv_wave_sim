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

#include <cmath>

using namespace gz;
using namespace waves;

///////////////////////////////////////////////////////////////////////////////
const double GRAVITY = 9.8;
const double OMEGA_0 = 10.0;

const double PIERSON_MOSKOWITZ_ALPHA = 0.0081;
const double PIERSON_MOSKOWITZ_BETA  = 0.74;

///////////////////////////////////////////////////////////////////////////////
WaveSpectrum::~WaveSpectrum()
{
}

///////////////////////////////////////////////////////////////////////////////
WaveSpectrum::WaveSpectrum() :
  ux(0.0),
  uy(0.0),
  u(0.0)
{
}

///////////////////////////////////////////////////////////////////////////////
void WaveSpectrum::SetWindVelocity(double _ux, double _uy)
{
  this->ux = _ux;
  this->uy = _uy;
  this->u = std::sqrt(_ux*_ux + _uy*_uy);
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::Dispersion(double _k)
{ 
  double omega = std::sqrt(GRAVITY * std::abs(_k));
  return omega;
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::InvDispersion(double _omega)
{ 
  double k = _omega * _omega / GRAVITY;
  return k;
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::QuantisedDispersion(double _k)
{ 
  double omega = std::floor(Dispersion(_k) / OMEGA_0) * OMEGA_0;
  return omega;
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::SignificantWaveHeight(double _u)
{
  double hs = 0.21 * _u * _u / GRAVITY;
  return hs;
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::PiersonMoskowitzK0(double _u)
{
  double k0 = GRAVITY / _u / _u;
  return k0;
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::Spectrum(double _k, double _kx, double _ky, double _u, double _ux, double _uy)
{
  return PiersonMoskowitzSpectrum(_k, _kx, _ky, _u, _ux, _uy);
}

///////////////////////////////////////////////////////////////////////////////
double WaveSpectrum::PiersonMoskowitzSpectrum(double _k, double _kx, double _ky, double _u, double _ux, double _uy)
{
  if (std::abs(_k) < 1.0E-8 || std::abs(_u) < 1.0E-8)
  {
    return 0.0;
  }
  else
  {
    double g = GRAVITY;
    double alpha = PIERSON_MOSKOWITZ_ALPHA;
    double beta = PIERSON_MOSKOWITZ_BETA;
    double k0 = PiersonMoskowitzK0(_u);
    double k2 = _k * _k;
    double k4 = k2 * k2;
    double r = k0 / _k;
    double r2 = r * r;    
    double c = (_kx * _ux + _ky * _uy) / _k / _u;
    double c2 = c * c;
    double s = (alpha / k4 / M_PI) * exp(-beta * r2) * c2;
    return s;
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
OmniDirectionalWaveSpectrum::~OmniDirectionalWaveSpectrum()
{
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
PiersonMoskowitzWaveSpectrum::~PiersonMoskowitzWaveSpectrum()
{
}

PiersonMoskowitzWaveSpectrum::PiersonMoskowitzWaveSpectrum(
    double _u19, double _gravity) :
  OmniDirectionalWaveSpectrum(),
  _u19(_u19),
  _gravity(_gravity)
{
}

///////////////////////////////////////////////////////////////////////////////
double PiersonMoskowitzWaveSpectrum::Evaluate(double _k) const
{
  if (std::abs(_k) < 1.0E-8 || std::abs(this->_u19) < 1.0E-8)
  {
    return 0.0;
  }

  // aliases
  const double g = this->_gravity;
  const double u = this->_u19;

  // constants
  constexpr double alpha = 0.0081;
  constexpr double beta = 0.74;

  // intermediates
  const double g2 = g * g;
  const double k2 = _k * _k;
  const double k3 = _k * k2;
  const double u2 = u * u;
  const double u4 = u2 * u2;

  const double cap_s = alpha / 2.0 / k3 * std::exp(-beta * g2 / k2 / u4);
  return cap_s;
}

///////////////////////////////////////////////////////////////////////////////
void PiersonMoskowitzWaveSpectrum::Evaluate(
    Eigen::Ref<Eigen::MatrixXd> _spectrum,
    const Eigen::Ref<const Eigen::MatrixXd> &_k) const
{
  auto rows =  _spectrum.rows();
  auto cols =  _spectrum.cols();

  if (std::abs(this->_u19) < 1.0E-8)
  {
    _spectrum.setZero();
  }

  // array k has no zero elements
  Eigen::MatrixXd k = (_k.array() == 0).select(
      Eigen::MatrixXd::Ones(rows, cols), _k);

  // aliases
  const double g = this->_gravity;
  const double u = this->_u19;

  // constants
  constexpr double alpha = 0.0081;
  constexpr double beta = 0.74;

  // intermediates
  const double g2 = g * g;
  const double u2 = u * u;
  const double u4 = u2 * u2;
  const Eigen::MatrixXd k2 = Eigen::pow(k.array(), 2.0);
  const Eigen::MatrixXd k3 = Eigen::pow(k.array(), 3.0);

  // evaluate for k
  const Eigen::MatrixXd cap_s = alpha / 2.0 / k3.array()
                              * Eigen::exp(-beta * g2 / k2.array() / u4);

  // apply filter
  _spectrum = (_k.array() == 0).select(
      Eigen::MatrixXd::Zero(rows, cols), cap_s);
}

///////////////////////////////////////////////////////////////////////////////
double PiersonMoskowitzWaveSpectrum::Gravity() const
{
  return this->_gravity;
}

///////////////////////////////////////////////////////////////////////////////
void PiersonMoskowitzWaveSpectrum::SetGravity(double _value)
{
  this->_gravity = _value;
}

///////////////////////////////////////////////////////////////////////////////
double PiersonMoskowitzWaveSpectrum::U19() const
{
  return this->_u19;
}

///////////////////////////////////////////////////////////////////////////////
void PiersonMoskowitzWaveSpectrum::SetU19(double _value)
{
  this->_u19 = _value;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
ECKVWaveSpectrum::~ECKVWaveSpectrum()
{
}

///////////////////////////////////////////////////////////////////////////////
ECKVWaveSpectrum::ECKVWaveSpectrum(
    double _u10, double _cap_omega_c, double _gravity) :
  OmniDirectionalWaveSpectrum(),
  _u10(_u10),
  _cap_omega_c(_cap_omega_c),
  _gravity(_gravity)
{
}

///////////////////////////////////////////////////////////////////////////////
double ECKVWaveSpectrum::Evaluate(double _k) const
{
  if (std::abs(_k) < 1.0E-8 || std::abs(this->_u10) < 1.0E-8)
  {
    return 0.0;
  }

  const double k = _k;

  // aliases
  const double g = this->_gravity;
  const double u10 = this->_u10;
  const double cap_omega_c = this->_cap_omega_c;

  // constants
  constexpr double alpha = 0.0081;
  constexpr double beta = 1.25;
  constexpr double cd_10n = 0.00144;
  constexpr double ao = 0.1733;
  constexpr double ap = 4.0;
  constexpr double km = 370.0;
  constexpr double cm = 0.23;

  // intermediates
  const double u_star = std::sqrt(cd_10n) * u10;
  const double am = 0.13 * u_star / cm;
  
  double gamma = 1.7;
  if (cap_omega_c < 1.0)
  {
    gamma = 1.7;
  }
  else
  {
    gamma = 1.7 + 6.0 * std::log10(cap_omega_c);
  }

  const double sigma = 0.08 * (1.0 + 4.0 * std::pow(cap_omega_c, -3.0));
  const double alpha_p = 0.006 * std::pow(cap_omega_c, 0.55);

  double alpha_m; 
  if (u_star <= cm)
  {
    alpha_m = 0.01 * (1.0 + std::log(u_star / cm));
  }
  else
  {
    alpha_m = 0.01 * (1.0 + 3.0 * std::log(u_star / cm));
  }

  const double ko = g / u10 / u10;
  const double kp = ko * cap_omega_c * cap_omega_c;

  const double cp = std::sqrt(g / kp);
  const double c  = std::sqrt((g / k) * (1.0 + std::pow(k / km, 2.0)));

  const double l_pm = std::exp(-1.25 * std::pow(kp / k, 2.0));
  
  const double cap_gamma = std::exp(
      -1.0/(2.0 * std::pow(sigma, 2.0))
      * std::pow(std::sqrt(k / kp) - 1.0, 2.0)
  );
  
  const double j_p = std::pow(gamma, cap_gamma);
  
  const double f_p = l_pm * j_p * std::exp(
      -0.3162 * cap_omega_c * (std::sqrt(k / kp) - 1.0)
  );

  const double f_m = l_pm * j_p * std::exp(
      -0.25 * std::pow(k / km - 1.0, 2.0)
  );

  const double b_l = 0.5 * alpha_p * (cp / c) * f_p;
  const double b_h = 0.5 * alpha_m * (cm / c) * f_m;
  
  const double k3 = k * k * k;
  const double cap_s = (b_l + b_h) / k3;
  
  return cap_s;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVWaveSpectrum::Evaluate(
    Eigen::Ref<Eigen::MatrixXd> _spectrum,
    const Eigen::Ref<const Eigen::MatrixXd> &_k) const
{
  auto rows =  _spectrum.rows();
  auto cols =  _spectrum.cols();

  if (std::abs(this->_u10) < 1.0E-8)
  {
    _spectrum.setZero();
  }

  // array k has no zero elements
  Eigen::MatrixXd k = (_k.array() == 0).select(
      Eigen::MatrixXd::Ones(rows, cols), _k);

  // aliases
  const double g = this->_gravity;
  const double u10 = this->_u10;
  const double cap_omega_c = this->_cap_omega_c;

  // constants
  constexpr double alpha = 0.0081;
  constexpr double beta = 1.25;
  constexpr double cd_10n = 0.00144;
  constexpr double ao = 0.1733;
  constexpr double ap = 4.0;
  constexpr double km = 370.0;
  constexpr double cm = 0.23;

  // intermediates
  const double u_star = std::sqrt(cd_10n) * u10;
  const double am = 0.13 * u_star / cm;
  
  double gamma = 1.7;
  if (cap_omega_c < 1.0)
  {
    gamma = 1.7;
  }
  else
  {
    gamma = 1.7 + 6.0 * std::log10(cap_omega_c);
  }

  const double sigma = 0.08 * (1.0 + 4.0 * std::pow(cap_omega_c, -3.0));
  const double alpha_p = 0.006 * std::pow(cap_omega_c, 0.55);

  double alpha_m; 
  if (u_star <= cm)
  {
    alpha_m = 0.01 * (1.0 + std::log(u_star / cm));
  }
  else
  {
    alpha_m = 0.01 * (1.0 + 3.0 * std::log(u_star / cm));
  }

  const double ko = g / u10 / u10;
  const double kp = ko * cap_omega_c * cap_omega_c;
  const double cp = std::sqrt(g / kp);

  const Eigen::MatrixXd c  = Eigen::sqrt(
      (g / k.array()) * (1.0 + Eigen::pow(k.array() / km, 2.0))
  );

  const Eigen::MatrixXd l_pm = Eigen::exp(
      -1.25 * Eigen::pow(kp / k.array(), 2.0)
  );
  
  const Eigen::MatrixXd cap_gamma = Eigen::exp(
      -1.0/(2.0 * std::pow(sigma, 2.0))
      * Eigen::pow(Eigen::sqrt(k.array() / kp) - 1.0, 2.0)
  );
  
  const Eigen::MatrixXd j_p = Eigen::pow(gamma, cap_gamma.array());
  
  const Eigen::MatrixXd f_p = l_pm.array() * j_p.array() * Eigen::exp(
      -0.3162 * cap_omega_c * (Eigen::sqrt(k.array() / kp) - 1.0)
  );

  const Eigen::MatrixXd f_m = l_pm.array() * j_p.array() * Eigen::exp(
      -0.25 * Eigen::pow(k.array() / km - 1.0, 2.0)
  );

  const Eigen::MatrixXd b_l = 0.5 * alpha_p * (cp / c.array()) * f_p.array();
  const Eigen::MatrixXd b_h = 0.5 * alpha_m * (cm / c.array()) * f_m.array();
  
  const Eigen::MatrixXd k3 = Eigen::pow(k.array(), 3.0);
  const Eigen::MatrixXd cap_s = (b_l.array() + b_h.array()) / k3.array();

   // apply filter
  _spectrum = (_k.array() == 0).select(
      Eigen::MatrixXd::Zero(rows, cols), cap_s);
}

///////////////////////////////////////////////////////////////////////////////
double ECKVWaveSpectrum::Gravity() const
{
  return this->_gravity;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVWaveSpectrum::SetGravity(double _value)
{
  this->_gravity = _value;
}

///////////////////////////////////////////////////////////////////////////////
double ECKVWaveSpectrum::U10() const
{
  return this->_u10;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVWaveSpectrum::SetU10(double _value)
{
  this->_u10 = _value;
}

///////////////////////////////////////////////////////////////////////////////
double ECKVWaveSpectrum::CapOmegaC() const
{
  return this->_cap_omega_c;
}

///////////////////////////////////////////////////////////////////////////////
void ECKVWaveSpectrum::SetCapOmegaC(double _value)
{
  this->_cap_omega_c = _value;
}

///////////////////////////////////////////////////////////////////////////////

