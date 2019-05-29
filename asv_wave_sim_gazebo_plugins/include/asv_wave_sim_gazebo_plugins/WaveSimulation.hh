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

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SIMULATION_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SIMULATION_HH_

#include <vector>

namespace asv
{

  class WaveSimulation
  {
    public: virtual ~WaveSimulation();

    public: WaveSimulation();

    public: virtual void SetWindVelocity(double _ux, double _uy) = 0;

    public: virtual void SetTime(double _time) = 0;

    public: virtual void ComputeHeights(
      std::vector<double>& _h) = 0;

    public: virtual void ComputeHeightDerivatives(
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy) = 0;

    public: virtual void ComputeDisplacements(
      std::vector<double>& _sx,
      std::vector<double>& _sy) = 0;

    public: virtual void ComputeDisplacementDerivatives(
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy) = 0;

    public: virtual void ComputeDisplacementsAndDerivatives(
      std::vector<double>& _h,
      std::vector<double>& _sx,
      std::vector<double>& _sy,
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy,
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy) = 0;
  };

}

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVE_SIMULATION_HH_
