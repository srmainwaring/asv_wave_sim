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

#include "WaveSimulationFFTImpl.hh"

using Eigen::MatrixXd;

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

using namespace gz;
using namespace waves;

TEST(WaveSimulationFFTPerf, CurrentAmplitudes)
{
  double lx = 200.0;
  double ly = 100.0;
  int    nx = 256;
  int    ny = 126;

  WaveSimulationFFTImpl model(lx, ly, nx, ny);

  int num_runs = 1000;

#if 1
  {
    model.SetUseVectorised(false);
    model.ComputeBaseAmplitudes();
    double sim_time = 0.0;
    double sim_step = 0.001;
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      model.ComputeCurrentAmplitudes(sim_time);
      sim_time += sim_step;
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  {
    model.SetUseVectorised(true);
    model.ComputeBaseAmplitudes();
    double sim_time = 0.0;
    double sim_step = 0.001;
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      model.ComputeCurrentAmplitudes(sim_time);
      sim_time += sim_step;
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif
}
