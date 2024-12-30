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

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gz/waves/OceanTile.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WavefieldSampler.hh"
#include "gz/waves/WaveParameters.hh"
#include "gz/waves/WaveSimulation.hh"
#include "gz/waves/LinearRegularWaveSimulation.hh"
#include "gz/waves/TrochoidIrregularWaveSimulation.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/WaveSpectrum.hh"

using gz::waves::Index;
using gz::waves::IWaveSimulation;
using gz::waves::LinearRegularWaveSimulation;
using gz::waves::WaveParameters;
using gz::waves::TrochoidIrregularWaveSimulation;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
// Utilities

std::ostream& operator<<(std::ostream& os, const std::vector<double>& vec)
{
  for (auto&& v : vec)
    os << v << ", ";
  return os;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
#if 0
TEST(WaveSimulation, TrochoidIrregularWaveSimulation)
{
  // Configure the wave parameters.
  std::shared_ptr<WaveParameters> wave_params(new WaveParameters());

  // Create the wave simulation.
  Index nx = 4;
  Index ny = 4;
  double lx = 4.0;
  double ly = 4.0;
  std::unique_ptr<IWaveSimulation> wave_sim(
      new TrochoidIrregularWaveSimulation(nx, lx, wave_params));

  // Compute the initial height field.
  Eigen::ArrayXd h = Eigen::ArrayXd::Zero(nx * ny);
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
  const Index nx_{8};
  const Index ny_{4};
  const double lx_{10.0};
  const double ly_{5.0};

  // grid depth dimension for pressure
  Index nz_{5};
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
  Eigen::ArrayXd h = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->ElevationAt(h);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    // double y = iy * dy + ly_min;
    for (Index ix=0; ix < nx_; ++ix, ++idx)
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
  Eigen::ArrayXd h = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->ElevationAt(h);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    double y = iy * dy + ly_min;
    for (Index ix=0; ix < nx_; ++ix, ++idx)
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
  Eigen::ArrayXd sx = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sy = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->DisplacementAt(sx, sy);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix, ++idx)
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
  Eigen::ArrayXd x_v = Eigen::ArrayXd::LinSpaced(nx_, lx_min, lx_max - dx);
  Eigen::ArrayXd y_v = Eigen::ArrayXd::LinSpaced(ny_, ly_min, ly_max - dy);

  // broadcast to matrices (aka meshgrid)
  Eigen::ArrayXXd x_grid = Eigen::ArrayXXd::Zero(nx_, ny_);
  Eigen::ArrayXXd y_grid = Eigen::ArrayXXd::Zero(nx_, ny_);
  x_grid.colwise() += x_v;
  y_grid.rowwise() += y_v.transpose();

  for (Index ix=0; ix < nx_; ++ix)
  {
    double x_test = ix * dx + lx_min;
    EXPECT_DOUBLE_EQ(x_v(ix), x_test);
  }

  for (Index iy=0; iy < ny_; ++iy)
  {
    double y_test = iy * dy + ly_min;
    EXPECT_DOUBLE_EQ(y_v(iy), y_test);
  }

  for (Index ix=0; ix < nx_; ++ix)
  {
    for (Index iy=0; iy < ny_; ++iy)
    {
      double x_test = ix * dx + lx_min;
      double y_test = iy * dy + ly_min;

      EXPECT_DOUBLE_EQ(x_grid(ix, iy), x_test);
      EXPECT_DOUBLE_EQ(y_grid(ix, iy), y_test);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightsDirXArrayXXd)
{
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(1.0, 0.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);

  // array
  Eigen::ArrayXd h1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->ElevationAt(h1);

  // non-array
  Eigen::ArrayXd h2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->ElevationAt(h2);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightsDirXYArrayXXd)
{
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);

  // array
  Eigen::ArrayXd h1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->ElevationAt(h1);

  // non-array
  Eigen::ArrayXd h2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->ElevationAt(h2);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(h1(idx), h2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestDisplacmentsArrayXXd)
{
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);

  // array
  Eigen::ArrayXd sx1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sy1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->DisplacementAt(sx1, sy1);

  // non-array
  Eigen::ArrayXd sx2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sy2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->DisplacementAt(sx2, sy2);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(sx1(idx), sx2(idx));
      EXPECT_DOUBLE_EQ(sy1(idx), sy2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestHeightDerivativesArrayXXd)
{
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);

  // array
  Eigen::ArrayXd dhdx1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dhdy1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->ElevationDerivAt(dhdx1, dhdy1);

  // non-array
  Eigen::ArrayXd dhdx2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dhdy2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->ElevationDerivAt(dhdx2, dhdy2);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix, ++idx)
    {
      EXPECT_DOUBLE_EQ(dhdx1(idx), dhdx2(idx));
      EXPECT_DOUBLE_EQ(dhdy1(idx), dhdy2(idx));
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRegularWaveSimFixture, TestDisplacementsAndDerivativesArrayXXd)
{
  // Wave simulation
  std::unique_ptr<LinearRegularWaveSimulation> wave_sim(
      new LinearRegularWaveSimulation(lx_, ly_, nx_, ny_));
  wave_sim->SetDirection(2.0, -1.0);
  wave_sim->SetAmplitude(amplitude_);
  wave_sim->SetPeriod(period_);
  wave_sim->SetTime(5.0);

  // array
  Eigen::ArrayXd h1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sx1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sy1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dhdx1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dhdy1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dsxdx1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dsydy1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dsxdy1 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(true);
  wave_sim->DisplacementAndDerivAt(
    h1, sx1, sy1, dhdx1, dhdy1, dsxdx1, dsydy1, dsxdy1);

  // non-array
  Eigen::ArrayXd h2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sx2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd sy2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dhdx2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dhdy2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dsxdx2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dsydy2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  Eigen::ArrayXd dsxdy2 = Eigen::ArrayXd::Zero(nx_ * ny_);
  wave_sim->SetUseVectorised(false);
  wave_sim->DisplacementAndDerivAt(
    h2, sx2, sy2, dhdx2, dhdy2, dsxdx2, dsydy2, dsxdy2);

  for (Index iy=0, idx=0; iy < ny_; ++iy)
  {
    for (Index ix=0; ix < nx_; ++ix, ++idx)
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
  Index N = 4;
  Index N2 = N*N;
  Index NPlus1 = N+1;
  Index NPlus12 = NPlus1*NPlus1;
  double L = 10.0;
  double amplitude = 1.0;
  double period = 10.0;
  double time = 0.0;

  // Ocean tile (in server mode)
  std::unique_ptr<OceanTile> oceanTile(new OceanTile(N, N, L, L, false));
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
  for (Index iy=0; iy < NPlus1; ++iy)
  {
    double vy = iy * dl + lm;
    for (Index ix=0; ix < NPlus1; ++ix)
    {
      double vx = ix * dl + lm;
      Index idx = iy * NPlus1 + ix;
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
  for (Index iy=0; iy < NPlus1; ++iy)
  {
    double vy = iy * dl + lm;
    for (Index ix=0; ix < NPlus1; ++ix)
    {
      double vx = ix * dl + lm;
      Index idx = iy * NPlus1 + ix;
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
  Eigen::ArrayXd zr = Eigen::ArrayXd::Zero(nz_);
  Eigen::ArrayXd ln_z;
  if (nz_ > 1)
  {
    // first element is zero - fill nz - 1 remaining elements
    ln_z = Eigen::ArrayXd::LinSpaced(
        nz_ - 1, -std::log(lz_), std::log(lz_));
    zr(Eigen::seq(1, nz_ - 1)) = -1 * Eigen::exp(ln_z);
  }
  Eigen::ArrayXd z = zr.reverse();

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
  Eigen::ArrayXd eta(nx_ * ny_);
  wave_sim->ElevationAt(eta);

  // pressure at z = 0
  Eigen::ArrayXd pressure(nx_ * ny_);
  wave_sim->PressureAt(nz_ - 1, pressure);
  for (Index ix=0, idx = 0; ix < nx_; ++ix)
  {
    for (Index iy=0; iy < ny_; ++iy, ++idx)
    {
      EXPECT_DOUBLE_EQ(pressure(idx), eta(idx));
    }
  }

  for (Index iz=0; iz < nz_; ++iz)
  {
    // scale factor for depth z
    double ez = exp(k_ * z(iz));

    // pressure at z(iz)
    wave_sim->PressureAt(iz, pressure);

    // check
    for (Index ix=0, idx=0; ix < nx_; ++ix)
    {
      for (Index iy=0; iy < ny_; ++iy, ++idx)
      {
        double p_test = ez * eta(idx);
        EXPECT_DOUBLE_EQ(pressure(idx), p_test);
      }
    }
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
