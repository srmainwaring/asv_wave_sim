#include "asv_wave_sim_gazebo_plugins/WaveSpectrum.hh"

#include <cmath>

namespace asv
{

  const double GRAVITY = 9.8;
  const double OMEGA_0 = 10.0;

  const double PIERSON_MOSKOWITZ_ALPHA = 0.0081;
  const double PIERSON_MOSKOWITZ_BETA  = 0.74;

  WaveSpectrum::~WaveSpectrum()
  {
  }

  WaveSpectrum::WaveSpectrum() :
    ux(0.0),
    uy(0.0),
    u(0.0)
  {
  }

  void WaveSpectrum::SetWindVelocity(double _ux, double _uy)
  {
    this->ux = _ux;
    this->uy = _uy;
    this->u = std::sqrt(_ux*_ux + _uy*_uy);
  }

  double WaveSpectrum::Dispersion(double _k)
  { 
    double omega = std::sqrt(GRAVITY * std::abs(_k));
    return omega;
  }

  double WaveSpectrum::InvDispersion(double _omega)
  { 
    double k = _omega * _omega / GRAVITY;
    return k;
  }

  double WaveSpectrum::QuantisedDispersion(double _k)
  { 
    double omega = std::floor(Dispersion(_k) / OMEGA_0) * OMEGA_0;
    return omega;
  }

  double WaveSpectrum::SignificantWaveHeight(double _u)
  {
    double hs = 0.21 * _u * _u / GRAVITY;
    return hs;
  }

  double WaveSpectrum::PiersonMoskowitzK0(double _u)
  {
    double k0 = GRAVITY / _u / _u;
    return k0;
  }

  double WaveSpectrum::Spectrum(double _k, double _kx, double _ky, double _u, double _ux, double _uy)
  {
    return PiersonMoskowitzSpectrum(_k, _kx, _ky, _u, _ux, _uy);
  }

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

}
