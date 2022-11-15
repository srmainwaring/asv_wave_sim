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
double PiersonMoskowitzWaveSpectrum::Evaluate(
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
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
double ECKVWaveSpectrum::Evaluate(
    Eigen::Ref<Eigen::MatrixXd> _spectrum,
    const Eigen::Ref<const Eigen::MatrixXd> &_k) const
{
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

