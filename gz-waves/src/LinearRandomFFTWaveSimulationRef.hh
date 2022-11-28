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

#ifndef GZ_WAVES_WAVESIMULATIONFFT_HH_
#define GZ_WAVES_WAVESIMULATIONFFT_HH_

#include <memory>

#include "gz/waves/WaveSimulation.hh"

using Eigen::MatrixXd;

namespace gz
{
namespace waves
{
  class LinearRandomFFTWaveSimulationRef : public WaveSimulation
  {
    public:
      virtual ~LinearRandomFFTWaveSimulationRef();

      LinearRandomFFTWaveSimulationRef(double lx, double ly, int nx, int ny);

      void SetLambda(double lambda);

      virtual void SetWindVelocity(double ux, double uy) override;

      virtual void SetTime(double value) override;

      virtual void ElevationAt(
          Eigen::Ref<Eigen::MatrixXd> h) override;

      virtual void ElevationDerivAt(
          Eigen::Ref<Eigen::MatrixXd> dhdx,
          Eigen::Ref<Eigen::MatrixXd> dhdy) override;

      virtual void DisplacementAt(
          Eigen::Ref<Eigen::MatrixXd> sx,
          Eigen::Ref<Eigen::MatrixXd> sy) override;

      virtual void DisplacementDerivAt(
          Eigen::Ref<Eigen::MatrixXd> dsxdx,
          Eigen::Ref<Eigen::MatrixXd> dsydy,
          Eigen::Ref<Eigen::MatrixXd> dsxdy) override;

      virtual void DisplacementAndDerivAt(
          Eigen::Ref<Eigen::MatrixXd> h,
          Eigen::Ref<Eigen::MatrixXd> sx,
          Eigen::Ref<Eigen::MatrixXd> sy,
          Eigen::Ref<Eigen::MatrixXd> dhdx,
          Eigen::Ref<Eigen::MatrixXd> dhdy,
          Eigen::Ref<Eigen::MatrixXd> dsxdx,
          Eigen::Ref<Eigen::MatrixXd> dsydy,
          Eigen::Ref<Eigen::MatrixXd> dsxdy) override;

    // public class declaration - for testing
    class Impl;

    private:
      std::unique_ptr<Impl> impl_;
  };
}
}

#endif
