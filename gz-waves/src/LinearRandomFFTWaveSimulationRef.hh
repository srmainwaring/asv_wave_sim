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

#ifndef GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONREF_HH_
#define GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONREF_HH_

#include <memory>

#include "gz/waves/WaveSimulation.hh"

namespace gz
{
namespace waves
{
class LinearRandomFFTWaveSimulationRef :
    public IWaveSimulation
{
 public:
    virtual ~LinearRandomFFTWaveSimulationRef();

    LinearRandomFFTWaveSimulationRef(double lx, double ly,
        Index nx, Index ny);

    void SetLambda(double lambda);

    void SetWindVelocity(double ux, double uy) override;

    void SetSteepness(double value) override;

    void SetTime(double value) override;

    Index SizeX() const override;

    Index SizeY() const override;

    Index SizeZ() const override;

    void ElevationAt(
        Eigen::Ref<Eigen::ArrayXXd> h) const override;

    void ElevationDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy) const override;

    void DisplacementAt(
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy) const override;

    void DisplacementDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy) const override;

    void DisplacementAndDerivAt(
        Eigen::Ref<Eigen::ArrayXXd> h,
        Eigen::Ref<Eigen::ArrayXXd> sx,
        Eigen::Ref<Eigen::ArrayXXd> sy,
        Eigen::Ref<Eigen::ArrayXXd> dhdx,
        Eigen::Ref<Eigen::ArrayXXd> dhdy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdx,
        Eigen::Ref<Eigen::ArrayXXd> dsydy,
        Eigen::Ref<Eigen::ArrayXXd> dsxdy) const override;

  // public class declaration - for testing
  class Impl;

 private:
    std::unique_ptr<Impl> impl_;
};
}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONREF_HH_
