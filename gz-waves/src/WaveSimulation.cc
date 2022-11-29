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

#include "gz/waves/WaveSimulation.hh"

namespace gz
{
namespace waves
{

  //////////////////////////////////////////////////
  WaveSimulation::~WaveSimulation()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulation::WaveSimulation()
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulation::Elevation(
      double /*x*/, double /*y*/,
      double &/*eta*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::Elevation(
      const Eigen::Ref<const Eigen::ArrayXd> &/*x*/,
      const Eigen::Ref<const Eigen::ArrayXd> &/*y*/,
      Eigen::Ref<Eigen::ArrayXd> /*eta*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::Pressure(
      double /*x*/, double /*y*/, double /*z*/,
      double &/*pressure*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::Pressure(
      const Eigen::Ref<const Eigen::ArrayXd> &/*x*/,
      const Eigen::Ref<const Eigen::ArrayXd> &/*y*/,
      const Eigen::Ref<const Eigen::ArrayXd> &/*z*/,
      Eigen::Ref<Eigen::ArrayXd> /*pressure*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::PressureAt(
      int /*iz*/,
      Eigen::Ref<Eigen::ArrayXXd> /*pressure*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::ElevationAt(
      int /*ix*/, int /*iy*/,
      double &/*eta*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::DisplacementAt(
      int /*ix*/, int /*iy*/,
      double &/*sx*/, double &/*sy*/)
  {
    assert(0 && "Not implemented");
  }

  //////////////////////////////////////////////////
  void WaveSimulation::PressureAt(
      int /*ix*/, int /*iy*/, int /*iz*/,
      double &/*pressure*/)
  {
    assert(0 && "Not implemented");
  }


}
}
