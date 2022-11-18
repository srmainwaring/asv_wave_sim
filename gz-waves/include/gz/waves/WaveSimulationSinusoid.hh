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

#ifndef GZ_WAVES_WAVESIMULATIONSINUSOID_HH_
#define GZ_WAVES_WAVESIMULATIONSINUSOID_HH_

#include "WaveSimulation.hh"

#include <Eigen/Dense>

#include <memory>

using Eigen::MatrixXd;

namespace gz
{
namespace waves
{
  /// The grid has sides with lengths lx and ly.
  ///
  /// There are nx, ny vertices in each direction.
  ///
  /// The simulation updates nx x ny vertices.
  ///
  /// The distance between vertices is dx = lx / nx and dy = ly / ny.
  ///
  /// All storage is assumed to be sized to nx x ny and the
  /// vertices are traversed in column major order:
  /// i.e. the innermost loop is over the x direction.
  ///
  class WaveSimulationSinusoid : public WaveSimulation
  {
    public:
      ~WaveSimulationSinusoid();

      WaveSimulationSinusoid(double _lx, double _ly, int _nx, int _ny);

      void SetUseVectorised(bool _value);

      void SetWindVelocity(double _ux, double _uy) override;

      void SetDirection(double _dir_x, double _dir_y);

      void SetAmplitude(double _value);

      void SetPeriod(double _value);

      void SetTime(double _value) override;

      virtual void ComputeHeights(
          Eigen::Ref<Eigen::MatrixXd> _h) override;

      virtual void ComputeHeightDerivatives(
          Eigen::Ref<Eigen::MatrixXd> _dhdx,
          Eigen::Ref<Eigen::MatrixXd> _dhdy) override;

      virtual void ComputeDisplacements(
          Eigen::Ref<Eigen::MatrixXd> _sx,
          Eigen::Ref<Eigen::MatrixXd> _sy) override;

      virtual void ComputeDisplacementDerivatives(
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
      class Impl;
      std::unique_ptr<Impl> impl;
  };
}
}

#endif
