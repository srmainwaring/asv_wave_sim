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

#ifndef GZ_WAVES_WAVESIMULATIONFFT2_HH_
#define GZ_WAVES_WAVESIMULATIONFFT2_HH_

#include "WaveSimulation.hh"

using Eigen::MatrixXd;

#include <memory>

using Eigen::MatrixXd;

namespace gz
{
namespace waves
{
  class WaveSimulationFFT2Impl;

  class WaveSimulationFFT2 : public WaveSimulation
  {
    public: virtual ~WaveSimulationFFT2();

    public: WaveSimulationFFT2(int _N, double _L);

    public: virtual void SetWindVelocity(double _ux, double _uy) override;

    public: virtual void SetTime(double _time) override;

    public: virtual void ComputeElevation(
      Eigen::Ref<Eigen::MatrixXd> _h) override;

    public: virtual void ComputeElevationDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dhdx,
      Eigen::Ref<Eigen::MatrixXd> _dhdy) override;

    public: virtual void ComputeDisplacements(
      Eigen::Ref<Eigen::MatrixXd> _sx,
      Eigen::Ref<Eigen::MatrixXd> _sy) override;

    public: virtual void ComputeDisplacementsDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dsxdx,
      Eigen::Ref<Eigen::MatrixXd> _dsydy,
      Eigen::Ref<Eigen::MatrixXd> _dsxdy) override;

    public: virtual void ComputeDisplacementsAndDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _h,
      Eigen::Ref<Eigen::MatrixXd> _sx,
      Eigen::Ref<Eigen::MatrixXd> _sy,
      Eigen::Ref<Eigen::MatrixXd> _dhdx,
      Eigen::Ref<Eigen::MatrixXd> _dhdy,
      Eigen::Ref<Eigen::MatrixXd> _dsxdx,
      Eigen::Ref<Eigen::MatrixXd> _dsydy,
      Eigen::Ref<Eigen::MatrixXd> _dsxdy) override;

    /// \brief Set lambda, a scaling factor controlling the horizontal wave displacement.
    public: void SetLambda(double _lambda);

    private: std::unique_ptr<WaveSimulationFFT2Impl> impl;
  };
}
}

#endif
