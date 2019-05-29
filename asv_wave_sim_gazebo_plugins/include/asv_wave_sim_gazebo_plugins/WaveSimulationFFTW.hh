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

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SIMULATION_FFTW_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SIMULATION_FFTW_HH_

#include "WaveSimulation.hh"

#include <memory>
#include <vector>

namespace asv
{

  class WaveSimulationFFTWImpl;

  class WaveSimulationFFTW : public WaveSimulation
  {
    public: virtual ~WaveSimulationFFTW();

    public: WaveSimulationFFTW(int _N, double _L);

    public: virtual void SetWindVelocity(double _ux, double _uy) override;

    public: virtual void SetTime(double _time) override;

    public: virtual void ComputeHeights(
      std::vector<double>& _h) override;

    public: virtual void ComputeHeightDerivatives(
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy) override;

    public: virtual void ComputeDisplacements(
      std::vector<double>& _sx,
      std::vector<double>& _sy) override;

    public: virtual void ComputeDisplacementDerivatives(
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy) override;

    public: virtual void ComputeDisplacementsAndDerivatives(
      std::vector<double>& _h,
      std::vector<double>& _sx,
      std::vector<double>& _sy,
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy,
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy) override;

    private: std::unique_ptr<WaveSimulationFFTWImpl> impl;
  };

}

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SIMULATION_FFTW_HH_
