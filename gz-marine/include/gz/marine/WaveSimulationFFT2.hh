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

#ifndef GZ_MARINE_WAVESIMULATIONFFT2_HH_
#define GZ_MARINE_WAVESIMULATIONFFT2_HH_

#include "WaveSimulation.hh"

#include <memory>
#include <vector>

namespace ignition
{
namespace marine
{

  class WaveSimulationFFT2Impl;

  class WaveSimulationFFT2 : public WaveSimulation
  {
    public: virtual ~WaveSimulationFFT2();

    public: WaveSimulationFFT2(int _N, double _L);

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

    /// \brief Set lambda, a scaling factor controlling the horizontal wave displacement.
    public: void SetLambda(double _lambda);

    private: std::unique_ptr<WaveSimulationFFT2Impl> impl;
  };

}
}

#endif
