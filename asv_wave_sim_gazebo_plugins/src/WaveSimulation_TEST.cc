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

#include "asv_wave_sim_gazebo_plugins/WaveSimulation.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationOpenCL.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationTrochoid.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSpectrum.hh"

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

using namespace asv;

///////////////////////////////////////////////////////////////////////////////
// Utilities

std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
{ 
  for (auto&& v : _vec )
    os << v << ", ";
  return os;
}

///////////////////////////////////////////////////////////////////////////////
// Define tests

TEST(WaveSimulation, WaveSimulationOpenCL)
{
  // Configure the wave spectrum.
  std::unique_ptr<WaveSpectrum> waveSpectrum(new WaveSpectrum());
  waveSpectrum->SetWindVelocity(20.0, 20.0);
  
  // Create the wave simulation.
  int N = 4;
  double L = 4.0;
  std::unique_ptr<WaveSimulation> waveSim(new WaveSimulationOpenCL(N, L));

  // Compute the initial height field.
  std::vector<double> h;
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be zero. 
  std::cout << h << std::endl;

  waveSim->SetWindVelocity(20.0, 20.0);
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be non-zero. 
  std::cout << h << std::endl;

  EXPECT_EQ(h.size(), N*N);
  // EXPECT_DOUBLE_EQ(1.0, 1.0);
}

TEST(WaveSimulation, WaveSimulationTrochoid)
{
  // Configure the wave parameters.
  std::shared_ptr<WaveParameters> waveParams(new WaveParameters());
  // waveParams->;
  
  // Create the wave simulation.
  int N = 4;
  double L = 4.0;
  std::unique_ptr<WaveSimulation> waveSim(new WaveSimulationTrochoid(N, L, waveParams));

  // Compute the initial height field.
  std::vector<double> h;
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be zero. 
  std::cout << h << std::endl;


  waveParams->SetNumber(3);
  waveParams->SetAngle(0.6);
  waveParams->SetScale(2.0);
  waveParams->SetSteepness(1.0);
  waveParams->SetAmplitude(1.0);
  waveParams->SetPeriod(8.0);
  waveParams->SetDirection(Vector2(1.0, 0.0));
  
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be non-zero. 
  std::cout << h << std::endl;

  EXPECT_EQ(h.size(), N*N);
}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

