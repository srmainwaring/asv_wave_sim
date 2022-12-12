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

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>

#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

using gz::waves::Index;
using gz::waves::Wavefield;
using gz::waves::WaveParameters;

//////////////////////////////////////////////////
// Define fixture
class WavefieldUpdateTest: public ::testing::Test
{
 public:

  void SetUp() override
  {
    // wave parameters
    wave_params_ = std::make_shared<WaveParameters>();
    wave_params_->SetAlgorithm("fft");
    wave_params_->SetTileSize(lx_);
    wave_params_->SetCellCount(nx_);
    wave_params_->SetWindSpeedAndAngle(5.0, 0.0);
    wave_params_->SetSteepness(2.0);

    // wave field
    wavefield_ = std::make_shared<Wavefield>("wave_world");
    wavefield_->SetParameters(wave_params_);
  }

  void TearDown() override
  {
     wave_params_.reset();
     wavefield_.reset();
  }

  // number of evaluations
  Index num_runs_ = 1000;

  // wave number grid (nx_, ny_)
  double lx_{200.0};
  double ly_{200.0};
  Index  nx_{128};
  Index  ny_{128};

  // wavefield params
  std::shared_ptr<WaveParameters> wave_params_;

  // wavefield params
  std::shared_ptr<Wavefield> wavefield_;
};

//////////////////////////////////////////////////
TEST_F(WavefieldUpdateTest, Update)
{
  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (Index i = 0; i < num_runs_; ++i)
  {
    wavefield_->Update(sim_time);

    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}
