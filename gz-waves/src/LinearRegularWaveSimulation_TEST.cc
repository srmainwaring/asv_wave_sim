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
#include "gz/waves/LinearRegularWaveSimulation.hh"
#include "gz/waves/TrochoidIrregularWaveSimulation.hh"
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
#if 0
TEST(WaveSimulation, TrochoidIrregularWaveSimulation)
{
  // Configure the wave parameters.
  std::shared_ptr<WaveParameters> wave_params(new WaveParameters());
  
  // Create the wave simulation.
  int nx = 4;
  int ny = 4;
  double lx = 4.0;
  double ly = 4.0;
  std::unique_ptr<WaveSimulation> wave_sim(
      new TrochoidIrregularWaveSimulation(nx, lx, wave_params));

  // Compute the initial height field.
  Eigen::VectorXd h = Eigen::VectorXd::Zero(nx * ny);
  wave_sim->SetTime(0.0);
  wave_sim->ElevationAt(h);

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
  wave_sim->ElevationAt(h);

  // Wave heights should be non-zero. 
  // std::cerr << h << std::endl;

  EXPECT_EQ(h.size(), nx * ny);
}
#endif
//////////////////////////////////////////////////
class LinearRegularWaveSimFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  // simulation parameters
  const double gravity_{9.81};

  // grid dimensions
  const int nx_{8};
  const int ny_{4};
  const double lx_{10.0};
  const double ly_{5.0};

  // grid depth dimension for pressure
  int nz_{5};
  double lz_{50.0};

  // wave parameters
  const double amplitude_{2.0};
  const double period_{10.0};

  // derived parameters
  const double w_{2.0 * M_PI / period_};
  const double k_{w_ * w_ / gravity_};
};

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightsDirX)
{ 
  double time = 5.0;

  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(time);

  // Grid spacing and offset
  double lx_min = - lx_ / 2.0;
  double dx = lx_ / nx_;

  // Verify heights
  Eigen::VectorXd h = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->ElevationAt(h);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    // double y = iy * dy + ly_min;
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      double x = ix * dx + lx_min;
      double a = k_ * x - w_ * time;
      double c = std::cos(a);
      double h_test = amplitude_ * c;
      
      EXPECT_DOUBLE_EQ(h(idx), h_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightsDirXY)
{ 
  double time = 5.0;

  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(time);

  // Grid spacing and offset
  double lx_min = - lx_ / 2.0;
  double ly_min = - ly_ / 2.0;
  double dx = lx_ / nx_;
  double dy = ly_ / ny_;
  double wt = w_ * time;
  double theta = std::atan2(-1.0, 2.0);
  double cd = std::cos(theta);
  double sd = std::sin(theta);

  // Verify heights
  Eigen::VectorXd h = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->ElevationAt(h);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    double y = iy * dy + ly_min;
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      double x = ix * dx + lx_min;
      double a = k_ * (x * cd + y * sd) - wt;
      double c = std::cos(a);
      double h_test = amplitude_ * c;
      
      EXPECT_DOUBLE_EQ(h(idx), h_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestDisplacementsDirX)
{
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);

  // Verify displacements (expect zero for sinusoid waves)
  Eigen::VectorXd sx = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sy = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->DisplacementAt(sx, sy);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(sx(idx), 0.0);
      EXPECT_DOUBLE_EQ(sy(idx), 0.0);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestEigenMeshGrid)
{
  // check the behaviour of Eigen meshgrid

  // grid spacing
  double dx = lx_ / nx_;
  double dy = ly_ / ny_;

  // create coordinate grids
  double lx_min = - lx_ / 2.0;
  double lx_max =   lx_ / 2.0;
  double ly_min = - ly_ / 2.0;
  double ly_max =   ly_ / 2.0;

  // linspaced is on closed interval (unlike Python which is open to right)
  Eigen::VectorXd x_v = Eigen::VectorXd::LinSpaced(nx_, lx_min, lx_max - dx);
  Eigen::VectorXd y_v = Eigen::VectorXd::LinSpaced(ny_, ly_min, ly_max - dy);

  // broadcast to matrices (aka meshgrid)
  Eigen::MatrixXd x_grid = Eigen::MatrixXd::Zero(nx_, ny_);
  Eigen::MatrixXd y_grid = Eigen::MatrixXd::Zero(nx_, ny_);
  x_grid.colwise() += x_v;
  y_grid.rowwise() += y_v.transpose();

  for (int ix=0; ix<nx_; ++ix)
  {
    double x_test = ix * dx + lx_min;
    EXPECT_DOUBLE_EQ(x_v(ix), x_test);
  }

  for (int iy=0; iy<ny_; ++iy)
  {
    double y_test = iy * dy + ly_min;
    EXPECT_DOUBLE_EQ(y_v(iy), y_test);
  }

  for (int ix=0; ix<nx_; ++ix)
  {
    for (int iy=0; iy<ny_; ++iy)
    {
      double x_test = ix * dx + lx_min;
      double y_test = iy * dy + ly_min;

      EXPECT_DOUBLE_EQ(x_grid(ix, iy), x_test);
      EXPECT_DOUBLE_EQ(y_grid(ix, iy), y_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightsDirXMatrixXd)
{ 
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);
  
  // array
  Eigen::VectorXd h1 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->ElevationAt(h1);
 
  // non-array
  Eigen::VectorXd h2 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->ElevationAt(h2);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightsDirXYMatrixXd)
{ 
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);
  
  // array
  Eigen::VectorXd h1 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->ElevationAt(h1);
 
  // non-array
  Eigen::VectorXd h2 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->ElevationAt(h2);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestDisplacmentsMatrixXd)
{ 
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);
  
  // array
  Eigen::VectorXd sx1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sy1 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->DisplacementAt(sx1, sy1);
 
  // non-array
  Eigen::VectorXd sx2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sy2 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->DisplacementAt(sx2, sy2);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(sx1(idx), sx2(idx));
      EXPECT_DOUBLE_EQ(sy1(idx), sy2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightDerivativesMatrixXd)
{ 
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);
  
  // array
  Eigen::VectorXd dhdx1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dhdy1 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->ElevationDerivAt(dhdx1, dhdy1);
 
  // non-array
  Eigen::VectorXd dhdx2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dhdy2 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->ElevationDerivAt(dhdx2, dhdy2);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    for (int ix=0; ix<nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(dhdx1(idx), dhdx2(idx));
      EXPECT_DOUBLE_EQ(dhdy1(idx), dhdy2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestDisplacementsAndDerivativesMatrixXd)
{ 
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);
  
  // array
  Eigen::VectorXd h1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sx1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sy1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dhdx1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dhdy1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dsxdx1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dsydy1 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dsxdy1 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->DisplacementAndDerivAt(
    h1, sx1, sy1, dhdx1, dhdy1, dsxdx1, dsydy1, dsxdy1);
 
  // non-array
  Eigen::VectorXd h2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sx2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd sy2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dhdx2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dhdy2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dsxdx2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dsydy2 = Eigen::VectorXd::Zero(nx_ * ny_);
  Eigen::VectorXd dsxdy2 = Eigen::VectorXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->DisplacementAndDerivAt(
    h2, sx2, sy2, dhdx2, dhdy2, dsxdx2, dsydy2, dsxdy2);

  for (int iy=0, idx=0; iy<ny_; ++iy)
  {
    for (int ix=0; ix<nx_; ++ix, ++idx)
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
// a LinearRegularWaveSimulation (which it isnt)
#if 0
TEST(OceanTile, LinearRegularWaveSimulation)
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
  for (int iy=0; iy<NPlus1; ++iy)
  {
    double vy = iy * dl + lm;
    for (int ix=0; ix<NPlus1; ++ix)
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
  for (int iy=0; iy<NPlus1; ++iy)
  {
    double vy = iy * dl + lm;
    for (int ix=0; ix<NPlus1; ++ix)
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
// Fluid pressure interpolation
TEST_F(LinearRegularWaveSimFixture, PressureAt)
{ 
  // pressure sample points (z is below the free surface)
  Eigen::VectorXd zr = Eigen::VectorXd::Zero(nz_);
  Eigen::VectorXd ln_z;
  if (nz_ > 1)
  {
    // first element is zero - fill nz - 1 remaining elements
    ln_z = Eigen::VectorXd::LinSpaced(
        nz_ - 1, -std::log(lz_), std::log(lz_));
    zr(Eigen::seq(1, nz_ - 1)) = -1 * Eigen::exp(ln_z.array());
  }
  Eigen::VectorXd z = zr.reverse(); 

  // debug
  // std::cerr << "nz:     " << nz_ << "\n";
  // std::cerr << "lz:     " << lz_ << "\n";
  // std::cerr << "ln_z:\n " << ln_z.transpose() << "\n";
  // std::cerr << "z:\n"     << z.transpose() << "\n\n";

  // time
  double time = 5.0;

  // model
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, lz_, nx_, ny_, nz_));
  wave_sim->SetUseVectorised(false);
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(time);

  // normalised pressure at free surface is the free surface elevation. 
  Eigen::VectorXd eta(nx_ * ny_);
  wave_sim->ElevationAt(eta);

  // pressure at z = 0
  Eigen::VectorXd pressure(nx_ * ny_);
  wave_sim->PressureAt(nz_ - 1, pressure);
  for (int ix = 0, idx = 0; ix < nx_; ++ix)
  {
    for (int iy = 0; iy < ny_; ++iy, ++idx)
    {
      EXPECT_DOUBLE_EQ(pressure(idx), eta(idx));
    }
  }

  for (int iz=0; iz<nz_; ++iz)
  {
    // scale factor for depth z 
    double ez = exp(k_ * z(iz));

    // pressure at z(iz)
    wave_sim->PressureAt(iz, pressure);

    // check
    for (int ix=0, idx=0; ix<nx_; ++ix)
    {
      for (int iy=0; iy<ny_; ++iy, ++idx)
      {
        double p_test = ez * eta(idx);
        EXPECT_DOUBLE_EQ(pressure(idx), p_test);
      }
    }
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

