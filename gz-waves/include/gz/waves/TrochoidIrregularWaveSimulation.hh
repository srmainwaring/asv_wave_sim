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

#ifndef GZ_WAVES_TROCHOIDIRREGULARWAVESIMULATION_HH_
#define GZ_WAVES_TROCHOIDIRREGULARWAVESIMULATION_HH_

#include <memory>

#include <Eigen/Dense> // NOLINT - cpplint false positive.

#include "gz/waves/WaveSimulation.hh"

using Eigen::ArrayXXd;

namespace gz
{
namespace waves
{
class WaveParameters;

class TrochoidIrregularWaveSimulation :
    public IWaveSimulation
{
 public:
  virtual ~TrochoidIrregularWaveSimulation();

  TrochoidIrregularWaveSimulation(
      Index nx,
      double lx,
      std::shared_ptr<WaveParameters> params);

  void SetWindVelocity(double ux, double uy) override;

  void SetTime(double time) override;

  Index SizeX() const override;

  Index SizeY() const override;

  Index SizeZ() const override;

  // lookup interface - array
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

  void PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure) const override;

  // lookup interface - scalar
  void ElevationAt(
      Index ix, Index iy,
      double &eta) const override;

  void DisplacementAt(
      Index ix, Index iy,
      double& sx, double& sy) const override;

  void PressureAt(
      Index ix, Index iy, Index iz,
      double& pressure) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_TROCHOIDIRREGULARWAVESIMULATION_HH_
