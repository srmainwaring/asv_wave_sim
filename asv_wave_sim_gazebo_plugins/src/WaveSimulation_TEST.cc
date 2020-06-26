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

#include "asv_wave_sim_gazebo_plugins/OceanTile.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldSampler.hh"
#include "asv_wave_sim_gazebo_plugins/WaveParameters.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulation.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationFFTW.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationOpenCL.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSimulationSinusoidal.hh"
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

#if 0
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
  // std::cout << h << std::endl;

  waveSim->SetWindVelocity(20.0, 20.0);
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be non-zero. 
  // std::cout << h << std::endl;

  EXPECT_EQ(h.size(), N*N);
  // EXPECT_DOUBLE_EQ(1.0, 1.0);
}
#endif

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
  // std::cout << h << std::endl;


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
  // std::cout << h << std::endl;

  EXPECT_EQ(h.size(), N*N);
}

TEST(WaveSimulation, WaveSimulationSinusoidal)
{ 
  // Wave parameters
  int N = 4;
  int N2 = N*N;
  double L = 10.0;
  double amplitude = 1.0;
  double period = 10.0;
  double time = 0.0;

  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoidal> waveSim(new WaveSimulationSinusoidal(N, L));
  waveSim->SetParameters(amplitude, period);
  waveSim->SetTime(time);

  // Wave spectrum
  double omega = 2.0 * M_PI / period;
  double k = omega * omega / 9.8;

  // Grid spacing and offset
  double lm = - L / 2.0;
  double dl = L / N;

  // Verify heights
  std::vector<double> h(N2);
  waveSim->ComputeHeights(h);

  // @DEBUG_INFO Display heights. 
  // std::cout << h << std::endl;

  for (size_t iy=0, idx=0; iy<N; ++iy)
  {
    double vy = iy * dl + lm;
    for (size_t ix=0; ix<N; ++ix, ++idx)
    {
      double vx = ix * dl + lm;
      double theta = k * vx - omega * time;
      double c = std::cos(theta);
      double sz = amplitude * c;
      
      // size_t idx = iy * N + ix;
      EXPECT_DOUBLE_EQ(h[idx], sz);
    }
  }

  // Verify displacements
  std::vector<double> sx(N2);
  std::vector<double> sy(N2);
  waveSim->ComputeDisplacements(sx, sy);

  for (size_t iy=0, idx=0; iy<N; ++iy)
  {
    for (size_t ix=0; ix<N; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(sx[idx], 0.0);
      EXPECT_DOUBLE_EQ(sy[idx], 0.0);
    }
  }

}

TEST(OceanTile, WaveSimulationSinusoidal)
{
  // Wave parameters.
  int N = 4;
  int N2 = N*N;
  int NPlus1 = N+1;
  int NPlus12 = NPlus1*NPlus1;
  double L = 10.0;
  double amplitude = 1.0;
  double period = 10.0;
  double time = 0.0;

  // Ocean tile (in server mode)
  std::unique_ptr<OceanTile> oceanTile(new OceanTile(N, L, false));
  oceanTile->SetWindVelocity(25.0, 0.0);
  oceanTile->Create();

  // Grid spacing and offset
  double lm = - L / 2.0;
  double dl = L / N;

  // Check vertices before Update
  for (auto&& v : oceanTile->Vertices())
  {
    std::cout << v << std::endl;
  }

  EXPECT_EQ(oceanTile->Vertices().size(), NPlus12);
  for (size_t iy=0; iy<NPlus1; ++iy)
  {
    double vy = iy * dl + lm;
    for (size_t ix=0; ix<NPlus1; ++ix)
    {
      double vx = ix * dl + lm;
      size_t idx = iy * NPlus1 + ix;
      auto& v = oceanTile->Vertices()[idx];
      EXPECT_DOUBLE_EQ(v.x, vx);
      EXPECT_DOUBLE_EQ(v.y, vy);
    }
  }

  // Check vertices after Update
  oceanTile->Update(time);
  for (auto&& v : oceanTile->Vertices())
  {
    std::cout << v << std::endl;
  }

  EXPECT_EQ(oceanTile->Vertices().size(), NPlus12);
  for (size_t iy=0; iy<NPlus1; ++iy)
  {
    double vy = iy * dl + lm;
    for (size_t ix=0; ix<NPlus1; ++ix)
    {
      double vx = ix * dl + lm;
      size_t idx = iy * NPlus1 + ix;
      auto& v = oceanTile->Vertices()[idx];

      // @TODO: DISABLED - not working, requires investigation.
      // EXPECT_DOUBLE_EQ(v.x, vx);
      // EXPECT_DOUBLE_EQ(v.y, vy);
    }
  }

  // Check faces.
  for (auto&& face : oceanTile->Faces())
  {
    std::cout << face << std::endl;
  }

  // Grid
  // Grid grid({ L, L }, { NPlus1, NPlus1 });


}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

