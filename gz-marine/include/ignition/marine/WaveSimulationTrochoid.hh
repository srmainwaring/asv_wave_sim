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

#ifndef IGNITION_MARINE_WAVESIMULATIONTROCHOID_HH_
#define IGNITION_MARINE_WAVESIMULATIONTROCHOID_HH_

#include "WaveSimulation.hh"

#include <memory>
#include <vector>

namespace ignition
{
namespace marine
{

  class WaveParameters;
  class WaveSimulationTrochoidImpl;

  class WaveSimulationTrochoid : public WaveSimulation
  {
    public: virtual ~WaveSimulationTrochoid();

    public: WaveSimulationTrochoid(
      int _N,
      double _L,
      std::shared_ptr<WaveParameters> _params);

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

    private: std::unique_ptr<WaveSimulationTrochoidImpl> impl;
  };

}
}

#endif
