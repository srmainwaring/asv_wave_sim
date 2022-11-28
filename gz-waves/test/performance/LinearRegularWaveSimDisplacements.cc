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

#include <chrono>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include "gz/waves/LinearRegularWaveSimulation.hh"

using Eigen::MatrixXd;

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

using namespace gz;
using namespace waves;

//////////////////////////////////////////////////
// Define fixture
class LinearRegularWaveSimTest: public ::testing::Test
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
TEST_F(LinearRegularWaveSimTest, Elevation)
{
  LinearRegularWaveSimulation model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(false);

  Eigen::VectorXd h(nx_ * ny_);

  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    model.ElevationAt(h);
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimTest, ElevationVectorised)
{
  LinearRegularWaveSimulation model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);

  Eigen::VectorXd h(nx_ * ny_);

  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    model.ElevationAt(h);
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimTest, DisplacementsAndDeriatives)
{
  LinearRegularWaveSimulation model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(false);

  Eigen::VectorXd h(nx_ * ny_);
  Eigen::VectorXd dhdx(nx_ * ny_);
  Eigen::VectorXd dhdy(nx_ * ny_);
  Eigen::VectorXd sx(nx_ * ny_);
  Eigen::VectorXd sy(nx_ * ny_);
  Eigen::VectorXd dsxdx(nx_ * ny_);
  Eigen::VectorXd dsydy(nx_ * ny_);
  Eigen::VectorXd dsxdy(nx_ * ny_);

  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    model.ElevationAt(h);
    model.ElevationDerivAt(dhdx, dhdy);
    model.DisplacementAt(sx, sy);
    model.DisplacementDerivAt(dsxdx, dsydy, dsxdy);
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimTest, DisplacementsAndDeriativesVectorised)
{
  LinearRegularWaveSimulation model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);

  Eigen::VectorXd h(nx_ * ny_);
  Eigen::VectorXd dhdx(nx_ * ny_);
  Eigen::VectorXd dhdy(nx_ * ny_);
  Eigen::VectorXd sx(nx_ * ny_);
  Eigen::VectorXd sy(nx_ * ny_);
  Eigen::VectorXd dsxdx(nx_ * ny_);
  Eigen::VectorXd dsydy(nx_ * ny_);
  Eigen::VectorXd dsxdy(nx_ * ny_);

  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    model.ElevationAt(h);
    model.ElevationDerivAt(dhdx, dhdy);
    model.DisplacementAt(sx, sy);
    model.DisplacementDerivAt(dsxdx, dsydy, dsxdy);
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}
