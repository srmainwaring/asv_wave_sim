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

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <string>

#include "LinearRandomFFTWaveSimulationImpl.hh"

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

using gz::waves::LinearRandomFFTWaveSimulation;

//////////////////////////////////////////////////
// Define fixture
class LinearRandomFFTWaveSimDisplacementsTest: public ::testing::Test
{
 public:
  // number of evaluations
  int num_runs_ = 1000;

  // wave number grid (nx_, ny_)
  double lx_{200.0};
  double ly_{100.0};
  int    nx_{256};
  int    ny_{128};
};

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimDisplacementsTest, Elevation)
{
  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();

  Eigen::ArrayXd h(nx_ * ny_);

  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    model.ComputeCurrentAmplitudes(sim_time);
    model.ElevationAt(h);
    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimDisplacementsTest, DisplacementsAndDeriatives)
{
  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();

  Eigen::ArrayXd h(nx_ * ny_);
  Eigen::ArrayXd dhdx(nx_ * ny_);
  Eigen::ArrayXd dhdy(nx_ * ny_);
  Eigen::ArrayXd sx(nx_ * ny_);
  Eigen::ArrayXd sy(nx_ * ny_);
  Eigen::ArrayXd dsxdx(nx_ * ny_);
  Eigen::ArrayXd dsydy(nx_ * ny_);
  Eigen::ArrayXd dsxdy(nx_ * ny_);

  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    model.ComputeCurrentAmplitudes(sim_time);
    model.ElevationAt(h);
    model.ElevationDerivAt(dhdx, dhdy);
    model.DisplacementAt(sx, sy);
    model.DisplacementDerivAt(dsxdx, dsydy, dsxdy);
    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}
