// Copyright (C) 2019  Rhys Mainwaring
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

#include "gz/waves/WaveSimulationTrochoid.hh"

#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"

#include <vector>

namespace gz
{
namespace waves
{

  //////////////////////////////////////////////////
  class WaveSimulationTrochoidImpl
  {
    public: ~WaveSimulationTrochoidImpl();

    public: WaveSimulationTrochoidImpl(
      int _N,
      double _L,
      std::shared_ptr<WaveParameters> _params);

    public: void SetWindVelocity(double _ux, double _uy);

    public: void SetTime(double _time);

    public: void ComputeHeights(
      Eigen::Ref<Eigen::MatrixXd> _heights);

    public: void ComputeHeightDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dhdx,
      Eigen::Ref<Eigen::MatrixXd> _dhdy);

    public: void ComputeDisplacements(
      Eigen::Ref<Eigen::MatrixXd> _sx,
      Eigen::Ref<Eigen::MatrixXd> _sy);

    public: void ComputeDisplacementDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dsxdx,
      Eigen::Ref<Eigen::MatrixXd> _dsydy,
      Eigen::Ref<Eigen::MatrixXd> _dsxdy);

    private: void ComputeBaseAmplitudes();

    private: void ComputeCurrentAmplitudes(double _time);
    
    public:  int N;
    public:  int N2;
    private: int NOver2;
    private: double L;
    private: double time;
    private: std::shared_ptr<WaveParameters> params;
  };

  //////////////////////////////////////////////////
  WaveSimulationTrochoidImpl::~WaveSimulationTrochoidImpl()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationTrochoidImpl::WaveSimulationTrochoidImpl(
    int _N,
    double _L,
    std::shared_ptr<WaveParameters> _params) :
    N(_N),
    N2(_N * _N),
    NOver2(_N / 2),
    L(_L),
    params(_params)
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::SetWindVelocity(double _ux, double _uy)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::SetTime(double _time)
  {
    // @TODO NO IMPLEMENTATION
    this->time = _time;
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::ComputeHeights(
    Eigen::Ref<Eigen::MatrixXd> _heights)
  {
    // Multiple wave params
    const auto  number     = this->params->Number();
    const auto& amplitude  = this->params->Amplitude_V();
    const auto& wavenumber = this->params->Wavenumber_V();
    const auto& omega      = this->params->AngularFrequency_V();
    const auto& phase      = this->params->Phase_V();
    const auto& q          = this->params->Steepness_V();
    const auto& direction  = this->params->Direction_V();

    // Multiple wave update 
    _heights = Eigen::MatrixXd::Zero(this->N2, 0);
    for (size_t i=0; i<number; ++i)
    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      const auto& q_i = q[i];

      for (size_t iy=0; iy<this->N; ++iy)
      {
        for (size_t ix=0; ix<this->N; ++ix)
        {
          // Col major index
          size_t idx = iy * this->N + ix;

          // Regular grid
          double vx = ix * this->L / this->N - this->L / 2.0;
          double vy = iy * this->L / this->N - this->L / 2.0;

          // Multiple waves
          double ddotx = direction_i.X() * vx + direction_i.Y() * vy;
          double angle  = ddotx * wavenumber_i - omega_i * time + phase_i;
          // double s = std::sin(angle);
          double c = std::cos(angle);
          // double sx = - direction_i.X() * q_i * amplitude_i * s;
          // double sy = - direction_i.Y() * q_i * amplitude_i * s;
          double h = amplitude_i * c;

          _heights(idx, 0) += h;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::ComputeHeightDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    // Multiple wave params
    const auto  number     = this->params->Number();
    const auto& amplitude  = this->params->Amplitude_V();
    const auto& wavenumber = this->params->Wavenumber_V();
    const auto& omega      = this->params->AngularFrequency_V();
    const auto& phase      = this->params->Phase_V();
    const auto& q          = this->params->Steepness_V();
    const auto& direction  = this->params->Direction_V();

    // Multiple wave update
    _sx = Eigen::MatrixXd::Zero(this->N2, 0);
    _sy = Eigen::MatrixXd::Zero(this->N2, 0);
    for (size_t i=0; i<number; ++i)
    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      const auto& q_i = q[i];

      for (size_t iy=0; iy<this->N; ++iy)
      {
        for (size_t ix=0; ix<this->N; ++ix)
        {
          // Col major index
          size_t idx = iy * this->N + ix;

          // Regular grid
          double vx = ix * this->L / this->N - this->L / 2.0;
          double vy = iy * this->L / this->N - this->L / 2.0;

          // Multiple waves
          double ddotx = direction_i.X() * vx + direction_i.Y() * vy;
          double angle  = ddotx * wavenumber_i - omega_i * time + phase_i;
          double s = std::sin(angle);
          // double c = std::cos(angle);
          double sx = - direction_i.X() * q_i * amplitude_i * s;
          double sy = - direction_i.Y() * q_i * amplitude_i * s;
          // double h = amplitude_i * c;

          _sx(idx, 0) += sx;
          _sy(idx, 0) += sy;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::ComputeDisplacementDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  WaveSimulationTrochoid::~WaveSimulationTrochoid()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationTrochoid::WaveSimulationTrochoid(
    int _N,
    double _L,
    std::shared_ptr<WaveParameters> _params) :
    impl(new WaveSimulationTrochoidImpl(_N, _L, _params))
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::SetTime(double _time)
  {
    impl->SetTime(_time);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeHeights(
    Eigen::Ref<Eigen::MatrixXd> _h)
  {
    impl->ComputeHeights(_h);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeHeightDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeDisplacementDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _h,
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy,
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    impl->ComputeHeights(_h);
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);
    impl->ComputeDisplacements(_sx, _sy);
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);
  }
}
}
