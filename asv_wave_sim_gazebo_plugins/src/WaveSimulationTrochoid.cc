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

#include "asv_wave_sim_gazebo_plugins/WaveSimulationTrochoid.hh"

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WaveParameters.hh"

#include <vector>

namespace asv
{

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationTrochoidImpl

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
      std::vector<double>& _heights);

    public: void ComputeHeightDerivatives(
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy);

    public: void ComputeDisplacements(
      std::vector<double>& _sx,
      std::vector<double>& _sy);

    public: void ComputeDisplacementDerivatives(
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy);

    private: void ComputeBaseAmplitudes();

    private: void ComputeCurrentAmplitudes(double _time);
    
    private: int N;
    private: int N2;
    private: int NOver2;
    private: double L;
    private: double time;
    private: std::shared_ptr<WaveParameters> params;
  };

  WaveSimulationTrochoidImpl::~WaveSimulationTrochoidImpl()
  {
  }

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

  void WaveSimulationTrochoidImpl::SetWindVelocity(double _ux, double _uy)
  {
    // @TODO NO IMPLEMENTATION
  }

  void WaveSimulationTrochoidImpl::SetTime(double _time)
  {
    // @TODO NO IMPLEMENTATION
    this->time = _time;
  }

  void WaveSimulationTrochoidImpl::ComputeHeights(
    std::vector<double>& _heights)
  {
    // Multiple wave params
    const auto  number     = this->params->Number();
    const auto& amplitude  = this->params->Amplitude_V();
    const auto& wavenumber = this->params->Wavenumber_V();
    const auto& omega      = this->params->AngularFrequency_V();
    const auto& phase      = this->params->Phase_V();
    const auto& q          = this->params->Steepness_V();
    const auto& direction  = this->params->Direction_V();

    // Resize output if necessary
    if (_heights.size() != this->N2)
    {
      _heights.resize(this->N2, 0.0);
    }

    // Multiple wave update 
    _heights.assign(this->N2, 0.0);
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
          // Row major index
          size_t idx = iy * this->N + ix;

          // Regular grid
          double vx = ix * this->L / this->N - this->L / 2.0;
          double vy = iy * this->L / this->N - this->L / 2.0;

          // Multiple waves
          double ddotx = direction_i.x() * vx + direction_i.y() * vy;
          double angle  = ddotx * wavenumber_i - omega_i * time + phase_i;
          // double s = std::sin(angle);
          double c = std::cos(angle);
          // double sx = - direction_i.x() * q_i * amplitude_i * s;
          // double sy = - direction_i.y() * q_i * amplitude_i * s;
          double h = amplitude_i * c;

          _heights[idx] += h;
        }
      }
    }
  }

  void WaveSimulationTrochoidImpl::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    // @TODO NO IMPLEMENTATION
  }

  void WaveSimulationTrochoidImpl::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    // Multiple wave params
    const auto  number     = this->params->Number();
    const auto& amplitude  = this->params->Amplitude_V();
    const auto& wavenumber = this->params->Wavenumber_V();
    const auto& omega      = this->params->AngularFrequency_V();
    const auto& phase      = this->params->Phase_V();
    const auto& q          = this->params->Steepness_V();
    const auto& direction  = this->params->Direction_V();

    // Resize output if necessary
    if (_sx.size() != this->N2)
    {
      _sx.resize(this->N2, 0.0);
    }
    if (_sy.size() != this->N2)
    {
      _sy.resize(this->N2, 0.0);
    }

    // Multiple wave update
    _sx.assign(this->N2, 0.0);
    _sy.assign(this->N2, 0.0);
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
          // Row major index
          size_t idx = iy * this->N + ix;

          // Regular grid
          double vx = ix * this->L / this->N - this->L / 2.0;
          double vy = iy * this->L / this->N - this->L / 2.0;

          // Multiple waves
          double ddotx = direction_i.x() * vx + direction_i.y() * vy;
          double angle  = ddotx * wavenumber_i - omega_i * time + phase_i;
          double s = std::sin(angle);
          // double c = std::cos(angle);
          double sx = - direction_i.x() * q_i * amplitude_i * s;
          double sy = - direction_i.y() * q_i * amplitude_i * s;
          // double h = amplitude_i * c;

          _sx[idx] += sx;
          _sy[idx] += sy;
        }
      }
    }
  }

  void WaveSimulationTrochoidImpl::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    // @TODO NO IMPLEMENTATION
  }

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationTrochoid

  WaveSimulationTrochoid::~WaveSimulationTrochoid()
  {
  }

  WaveSimulationTrochoid::WaveSimulationTrochoid(
    int _N,
    double _L,
    std::shared_ptr<WaveParameters> _params) :
    impl(new WaveSimulationTrochoidImpl(_N, _L, _params))
  {
  }

  void WaveSimulationTrochoid::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  void WaveSimulationTrochoid::SetTime(double _time)
  {
    impl->SetTime(_time);
  }

  void WaveSimulationTrochoid::ComputeHeights(
    std::vector<double>& _h)
  {
    impl->ComputeHeights(_h);    
  }

  void WaveSimulationTrochoid::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);  
  }

  void WaveSimulationTrochoid::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);    
  }

  void WaveSimulationTrochoid::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);      
  }

  void WaveSimulationTrochoid::ComputeDisplacementsAndDerivatives(
    std::vector<double>& _h,
    std::vector<double>& _sx,
    std::vector<double>& _sy,
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy,
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeHeights(_h);
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);
    impl->ComputeDisplacements(_sx, _sy);
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);    
  }

}
