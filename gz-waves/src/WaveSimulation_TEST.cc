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
  std::shared_ptr<WaveParameters> wave_params(new WaveParameters());
  
  // Create the wave simulation.
  int nx = 4;
  int ny = 4;
  double lx = 4.0;
  double ly = 4.0;
  std::unique_ptr<WaveSimulation> wave_sim(
      new WaveSimulationTrochoid(nx, lx, wave_params));

  // Compute the initial height field.
  Eigen::VectorXd h = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetTime(0.0);
  wave_sim->ComputeHeights(h);

  // Wave heights should be zero. 
  // std::cerr << h << std::endl;

  wave_params->SetNumber(3);
  wave_params->SetAngle(0.6);
  wave_params->SetScale(2.0);
  wave_params->SetSteepness(1.0);
  wave_params->SetAmplitude(1.0);
  wave_params->SetPeriod(8.0);
  wave_params->SetDirection(gz::math::Vector2d(1.0, 0.0));
  
  wave_sim->SetTime(0.0);
  wave_sim->ComputeHeights(h);

  // Wave heights should be non-zero. 
  // std::cerr << h << std::endl;

  EXPECT_EQ(h.size(), nx * ny);
}

//////////////////////////////////////////////////
class WaveSimulationSinusoidTestSuite : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  // simulation parameters
  const double gravity{9.81};

  // grid dimensions
  const int nx{8};
  const int ny{4};
  const double lx{10.0};
  const double ly{5.0};

  // wave parameters
  const double amplitude{2.0};
  const double period{10.0};

  // derived parameters
  const double w{2.0 * M_PI / period};
  const double k{w * w / gravity};
};

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestHeightsDirX)
{ 
  double time = 5.0;

  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(time);

  // Grid spacing and offset
  double lx_min = - lx / 2.0;
  double ly_min = - ly / 2.0;
  double dx = lx / nx;
  double dy = ly / ny;

  // Verify heights
  Eigen::VectorXd h = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->ComputeHeights(h);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    double y = iy * dy + ly_min;
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      double x = ix * dx + lx_min;
      double a = k * x - w * time;
      double c = std::cos(a);
      double h_test = amplitude * c;
      
      EXPECT_DOUBLE_EQ(h(idx), h_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestHeightsDirXY)
{ 
  double time = 5.0;

  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(time);

  // Grid spacing and offset
  double lx_min = - lx / 2.0;
  double ly_min = - ly / 2.0;
  double dx = lx / nx;
  double dy = ly / ny;
  double wt = w * time;
  double theta = std::atan2(-1.0, 2.0);
  double cd = std::cos(theta);
  double sd = std::sin(theta);

  // Verify heights
  Eigen::VectorXd h = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->ComputeHeights(h);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    double y = iy * dy + ly_min;
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      double x = ix * dx + lx_min;
      double a = k * (x * cd + y * sd) - wt;
      double c = std::cos(a);
      double h_test = amplitude * c;
      
      EXPECT_DOUBLE_EQ(h(idx), h_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestDisplacementsDirX)
{
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(5.0);

  // Verify displacements (expect zero for sinusoid waves)
  Eigen::VectorXd sx = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sy = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->ComputeDisplacements(sx, sy);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(sx(idx), 0.0);
      EXPECT_DOUBLE_EQ(sy(idx), 0.0);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestVectorisedGrid)
{
  // check the behaviour of Eigen meshgrid

  // grid spacing
  double dx = this->lx / this->nx;
  double dy = this->ly / this->ny;

  // create coordinate grids
  double lx_min = - lx / 2.0;
  double lx_max =   lx / 2.0;
  double ly_min = - ly / 2.0;
  double ly_max =   ly / 2.0;

  // linspaced is on closed interval (unlike Python which is open to right)
  Eigen::VectorXd x_v = Eigen::VectorXd::LinSpaced(nx, lx_min, lx_max - dx);
  Eigen::VectorXd y_v = Eigen::VectorXd::LinSpaced(ny, ly_min, ly_max - dy);

  // broadcast to matrices (aka meshgrid)
  Eigen::MatrixXd x_grid = Eigen::MatrixXd::Zero(this->nx, this->ny);
  Eigen::MatrixXd y_grid = Eigen::MatrixXd::Zero(this->nx, this->ny);
  x_grid.colwise() += x_v;
  y_grid.rowwise() += y_v.transpose();

  for (size_t ix=0; ix<nx; ++ix)
  {
    double x_test = ix * dx + lx_min;
    EXPECT_DOUBLE_EQ(x_v(ix), x_test);
  }

  for (size_t iy=0; iy<ny; ++iy)
  {
    double y_test = iy * dy + ly_min;
    EXPECT_DOUBLE_EQ(y_v(iy), y_test);
  }

  for (size_t ix=0; ix<nx; ++ix)
  {
    for (size_t iy=0; iy<ny; ++iy)
    {
      double x_test = ix * dx + lx_min;
      double y_test = iy * dy + ly_min;

      EXPECT_DOUBLE_EQ(x_grid(ix, iy), x_test);
      EXPECT_DOUBLE_EQ(y_grid(ix, iy), y_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestVectorisedHeightsDirX)
{ 
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(5.0);
  
  // vectorised
  Eigen::VectorXd h1 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(true);
  wave_sim->ComputeHeights(h1);
 
  // non-vectorised
  Eigen::VectorXd h2 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(false);
  wave_sim->ComputeHeights(h2);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestVectorisedHeightsDirXY)
{ 
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(5.0);
  
  // vectorised
  Eigen::VectorXd h1 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(true);
  wave_sim->ComputeHeights(h1);
 
  // non-vectorised
  Eigen::VectorXd h2 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(false);
  wave_sim->ComputeHeights(h2);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestVectorisedDisplacments)
{ 
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(5.0);
  
  // vectorised
  Eigen::VectorXd sx1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sy1 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(true);
  wave_sim->ComputeDisplacements(sx1, sy1);
 
  // non-vectorised
  Eigen::VectorXd sx2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sy2 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(false);
  wave_sim->ComputeDisplacements(sx2, sy2);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(sx1(idx), sx2(idx));
      EXPECT_DOUBLE_EQ(sy1(idx), sy2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite, TestVectorisedHeightDerivatives)
{ 
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(5.0);
  
  // vectorised
  Eigen::VectorXd dhdx1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dhdy1 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(true);
  wave_sim->ComputeHeightDerivatives(dhdx1, dhdy1);
 
  // non-vectorised
  Eigen::VectorXd dhdx2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dhdy2 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(false);
  wave_sim->ComputeHeightDerivatives(dhdx2, dhdy2);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(dhdx1(idx), dhdx2(idx));
      EXPECT_DOUBLE_EQ(dhdy1(idx), dhdy2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(WaveSimulationSinusoidTestSuite,
    TestVectorisedDisplacementsAndDerivatives)
{ 
  // Wave simulation
  std::unique_ptr<WaveSimulationSinusoid> wave_sim(
      new WaveSimulationSinusoid(lx, ly, nx, ny));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude);
  wave_sim->SetPeriod(period);
  wave_sim->SetTime(5.0);
  
  // vectorised
  Eigen::VectorXd h1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sx1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sy1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dhdx1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dhdy1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dsxdx1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dsydy1 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dsxdy1 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(true);
  wave_sim->ComputeDisplacementsAndDerivatives(
    h1, sx1, sy1, dhdx1, dhdy1, dsxdx1, dsydy1, dsxdy1);
 
  // non-vectorised
  Eigen::VectorXd h2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sx2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd sy2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dhdx2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dhdy2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dsxdx2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dsydy2 = Eigen::VectorXd::Zero(nx * ny);
  Eigen::VectorXd dsxdy2 = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetUseVectorised(false);
  wave_sim->ComputeDisplacementsAndDerivatives(
    h2, sx2, sy2, dhdx2, dhdy2, dsxdx2, dsydy2, dsxdy2);

  for (size_t iy=0, idx=0; iy<ny; ++iy)
  {
    for (size_t ix=0; ix<nx; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
      EXPECT_DOUBLE_EQ(sx1(idx), sx2(idx));
      EXPECT_DOUBLE_EQ(sy1(idx), sy2(idx));
      EXPECT_DOUBLE_EQ(dhdx1(idx), dhdx2(idx));
      EXPECT_DOUBLE_EQ(dhdy1(idx), dhdy2(idx));
      EXPECT_DOUBLE_EQ(dsxdx1(idx), dsxdx2(idx));
      EXPECT_DOUBLE_EQ(dsydy1(idx), dsydy2(idx));
      EXPECT_DOUBLE_EQ(dsxdy1(idx), dsxdy2(idx));
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

