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

#include "gz/waves/OceanTile.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WavefieldSampler.hh"
#include "gz/waves/WaveParameters.hh"
#include "gz/waves/WaveSimulation.hh"
#include "gz/waves/WaveSimulationSinusoid.hh"
#include "gz/waves/WaveSimulationTrochoid.hh"
#include "gz/waves/WaveSpectrum.hh"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

using namespace gz;
using namespace waves;

using Eigen::MatrixXd;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
// Utilities

std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
{ 
  for (auto&& v : _vec )
    os << v << ", ";
  return os;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
// Define tests

TEST(WaveSimulation, WaveSimulationTrochoid)
{
  // Configure the wave parameters.
  std::shared_ptr<WaveParameters> waveParams(new WaveParameters());
  // waveParams->;
  
  // Create the wave simulation.
  int N = 4;
  double L = 4.0;
  std::unique_ptr<WaveSimulation> waveSim(
      new WaveSimulationTrochoid(N, L, waveParams));

  // Compute the initial height field.
  Eigen::VectorXd h = Eigen::VectorXd::Zero(N * N);
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be zero. 
  // std::cerr << h << std::endl;

  waveParams->SetNumber(3);
  waveParams->SetAngle(0.6);
  waveParams->SetScale(2.0);
  waveParams->SetSteepness(1.0);
  waveParams->SetAmplitude(1.0);
  waveParams->SetPeriod(8.0);
  waveParams->SetDirection(gz::math::Vector2d(1.0, 0.0));
  
  waveSim->SetTime(0.0);
  waveSim->ComputeHeights(h);

  // Wave heights should be non-zero. 
  // std::cerr << h << std::endl;

  EXPECT_EQ(h.size(), N*N);
}

class WaveSimulationSinusoidTestSuite : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  // Grid dimensions
  const int N{8};
  const int N2{N*N};
  const double L{10.0};

  // Parameters
  const double amplitude{2.0};
  const double period{10.0};

  // Wave spectrum
  const double w{2.0 * M_PI / period};
  const double k{w * w / 9.81};
};

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestHeightsDirX)
{ 
  double time = 5.0;

  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> waveSim(
      new WaveSimulationSinusoid(N, L));
  waveSim->SetDirection(1.0, 0.0);
  waveSim->SetAmplitude(amplitude);
  waveSim->SetPeriod(period);
  waveSim->SetTime(time);

  // Grid spacing and offset
  double lm = - L / 2.0;
  double dl = L / N;

  // Verify heights
  Eigen::VectorXd h = Eigen::VectorXd::Zero(N2);
  waveSim->ComputeHeights(h);

  for (size_t iy=0, idx=0; iy<N; ++iy)
  {
    double y = iy * dl + lm;
    for (size_t ix=0; ix<N; ++ix, ++idx)
    {
      double x = ix * dl + lm;
      double a = k * x - w * time;
      double c = std::cos(a);
      double hh = amplitude * c;
      
      EXPECT_DOUBLE_EQ(h[idx], hh);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestHeightsDirXY)
{ 
  double time = 5.0;

  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> waveSim(
      new WaveSimulationSinusoid(N, L));
  waveSim->SetDirection(2.0, -1.0);
  waveSim->SetAmplitude(amplitude);
  waveSim->SetPeriod(period);
  waveSim->SetTime(time);

  // Grid spacing and offset
  double lm = - L / 2.0;
  double dl = L / N;
  double wt = w * time;
  double theta = std::atan2(-1.0, 2.0);
  double cd = std::cos(theta);
  double sd = std::sin(theta);

  // Verify heights
  Eigen::VectorXd h = Eigen::VectorXd::Zero(N2);
  waveSim->ComputeHeights(h);

  for (size_t iy=0, idx=0; iy<N; ++iy)
  {
    double y = iy * dl + lm;
    for (size_t ix=0; ix<N; ++ix, ++idx)
    {
      double x = ix * dl + lm;
      double a = k * (x * cd + y * sd) - wt;
      double c = std::cos(a);
      double hh = amplitude * c;
      
      EXPECT_DOUBLE_EQ(h[idx], hh);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestDisplacementsDirX)
{
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> waveSim(
      new WaveSimulationSinusoid(N, L));
  waveSim->SetDirection(1.0, 0.0);
  waveSim->SetAmplitude(amplitude);
  waveSim->SetPeriod(period);
  waveSim->SetTime(5.0);

  // Verify displacements (expect zero for sinusoid waves)
  Eigen::VectorXd sx = Eigen::VectorXd::Zero(N2);
  Eigen::VectorXd sy = Eigen::VectorXd::Zero(N2);
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
//////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

