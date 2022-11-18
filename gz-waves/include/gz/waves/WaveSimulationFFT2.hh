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
    public:
      virtual ~WaveSimulationFFT2();

      WaveSimulationFFT2(double _lx, double _ly, int _nx, int _ny);

      void SetUseVectorised(bool _value);

      /// \brief Set lambda which controls the horizontal wave displacement.
      void SetLambda(double _lambda);

      virtual void SetWindVelocity(double _ux, double _uy) override;

      virtual void SetTime(double _value) override;

      virtual void ComputeElevation(
          Eigen::Ref<Eigen::MatrixXd> _h) override;

      virtual void ComputeElevationDerivatives(
          Eigen::Ref<Eigen::MatrixXd> _dhdx,
          Eigen::Ref<Eigen::MatrixXd> _dhdy) override;

      virtual void ComputeDisplacements(
          Eigen::Ref<Eigen::MatrixXd> _sx,
          Eigen::Ref<Eigen::MatrixXd> _sy) override;

      virtual void ComputeDisplacementsDerivatives(
          Eigen::Ref<Eigen::MatrixXd> _dsxdx,
          Eigen::Ref<Eigen::MatrixXd> _dsydy,
          Eigen::Ref<Eigen::MatrixXd> _dsxdy) override;

      virtual void ComputeDisplacementsAndDerivatives(
          Eigen::Ref<Eigen::MatrixXd> _h,
          Eigen::Ref<Eigen::MatrixXd> _sx,
          Eigen::Ref<Eigen::MatrixXd> _sy,
          Eigen::Ref<Eigen::MatrixXd> _dhdx,
          Eigen::Ref<Eigen::MatrixXd> _dhdy,
          Eigen::Ref<Eigen::MatrixXd> _dsxdx,
          Eigen::Ref<Eigen::MatrixXd> _dsydy,
          Eigen::Ref<Eigen::MatrixXd> _dsxdy) override;

    private:
      std::unique_ptr<WaveSimulationFFT2Impl> impl;
  };
}
}

#endif
