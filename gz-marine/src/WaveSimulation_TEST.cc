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

#include "gz/marine/OceanTile.hh"
#include "gz/marine/Grid.hh"
#include "gz/marine/Wavefield.hh"
#include "gz/marine/WavefieldSampler.hh"
#include "gz/marine/WaveParameters.hh"
#include "gz/marine/WaveSimulation.hh"
#include "gz/marine/WaveSimulationFFTW.hh"
#include "gz/marine/WaveSimulationOpenCL.hh"
#include "gz/marine/WaveSimulationSinusoid.hh"
#include "gz/marine/WaveSimulationTrochoid.hh"
#include "gz/marine/WaveSpectrum.hh"

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

using namespace ignition;
using namespace marine;

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
  waveParams->SetDirection(ignition::math::Vector2d(1.0, 0.0));
  
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be non-zero. 
  // std::cout << h << std::endl;

  EXPECT_EQ(h.size(), N*N);
}


class WaveSimulationSinusoidTestSuite : public ::testing::Test
{
protected:
  void SetUp() override
  {        
    double time = 0.0;
  }

  void TearDown() override
  {        
  }

  double time = 0.0;

  // Grid dimensions
  const int N = 4;
  const int N2 = N*N;
  const double L = 10.0;
  const double dir_x = 1.0;
  const double dir_y = 0.0;

  // Parameters
  const double amplitude = 1.0;
  const double period = 10.0;

  // Wave spectrum
  const double omega = 2.0 * M_PI / period;
  const double k = omega * omega / 9.8;
};

TEST_F(WaveSimulationSinusoidTestSuite, testHeights)
{ 
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> waveSim(new WaveSimulationSinusoid(N, L));
  waveSim->SetDirection(dir_x, dir_y);
  waveSim->SetAmplitude(amplitude);
  waveSim->SetPeriod(period);
  waveSim->SetTime(time);

  // Grid spacing and offset
  double lm = - L / 2.0;
  double dl = L / N;

  // Verify heights
  std::vector<double> h(N2);
  waveSim->ComputeHeights(h);

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
}

TEST_F(WaveSimulationSinusoidTestSuite, testDisplacements)
{
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> waveSim(new WaveSimulationSinusoid(N, L));
  waveSim->SetDirection(dir_x, dir_y);
  waveSim->SetAmplitude(amplitude);
  waveSim->SetPeriod(period);
  waveSim->SetTime(time);

  // Grid spacing and offset
  double lm = - L / 2.0;
  double dl = L / N;

  // Verify displacements (expect zero for sinusoid waves)
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

// This test fails because it assumes the OceanTile is using 
// a WaveSimulationSinusoid (which it isnt)
#if 0
TEST(OceanTile, WaveSimulationSinusoid)
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
#endif

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

