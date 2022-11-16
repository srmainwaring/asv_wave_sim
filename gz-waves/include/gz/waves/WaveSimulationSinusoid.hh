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

#include <memory>
#include <vector>

namespace gz
{
namespace waves
{
  /// L is the length of each side of a square tile.
  ///
  /// There are N + 1 vertices in each direction, the additional vertex
  /// in each direction defines the tile skirt.
  ///
  /// The simulation updates N x N vertices
  ///
  /// The distance between vertices is N / L (because there
  /// are N+1 vertices in each direction including the skirt).
  ///
  /// All storage is assumed to be sized to N x N and the
  /// vertices are traversed in row major order:
  /// i.e. the innermost loop is over the x direction.
  ///
  class WaveSimulationSinusoid : public WaveSimulation
  {
    public: ~WaveSimulationSinusoid();

    public: WaveSimulationSinusoid(int _N, double _L);

    public: void SetWindVelocity(double _ux, double _uy) override;

    public: void SetDirection(double _dir_x, double _dir_y);

    public: void SetAmplitude(double _amplitude);

    public: void SetPeriod(double _period);

    public: void SetTime(double _time) override;

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

    public: virtual void ComputeFluidPotentialXY(
      std::vector<double>& _phiXY) override;

    public: virtual double ComputeFluidPotential(
      double _z, double _phiXY) override;

    public: virtual void ComputeHeights(
      Eigen::Ref<Eigen::MatrixXd> _h) override;

    public: virtual void ComputeHeightDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dhdx,
      Eigen::Ref<Eigen::MatrixXd> _dhdy) override;

    public: virtual void ComputeDisplacements(
      Eigen::Ref<Eigen::MatrixXd> _sx,
      Eigen::Ref<Eigen::MatrixXd> _sy) override;

    public: virtual void ComputeDisplacementDerivatives(
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

    class Impl;
    private: std::unique_ptr<Impl> impl;
  };

}
}

#endif
