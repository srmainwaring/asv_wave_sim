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

#include "gz/waves/WaveSimulationFFT2.hh"

#include "WaveSimulationFFT2Impl.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <string>

using Eigen::MatrixXd;

using namespace gz;
using namespace waves;

//////////////////////////////////////////////////
// Define fixture
class TestFixtureWaveSimulationFFT2: public ::testing::Test
{ 
public: 
  virtual ~TestFixtureWaveSimulationFFT2()
  {
    // cleanup any pending stuff, but no exceptions allowed
  }

  TestFixtureWaveSimulationFFT2()
  {
    // initialization code here
  } 

  virtual void SetUp() override
  { 
    // code here will execute just before the test ensues 
  }

  virtual void TearDown() override
  {
    // code here will be called just after the test completes
    // ok to through exceptions from here if need be
  }

  // put in any custom data members that you need 
  double lx_ = 200.0;
  double ly_ = 100.0;
  int    nx_ = 16;
  int    ny_ = 8;
};

//////////////////////////////////////////////////
// Define tests
TEST_F(TestFixtureWaveSimulationFFT2, AngularSpatialWavenumber)
{
  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);

  // check array dimensions
  EXPECT_EQ(model.kx_fft_.size(), nx_);
  EXPECT_EQ(model.ky_fft_.size(), ny_);
  EXPECT_EQ(model.kx_math_.size(), nx_);
  EXPECT_EQ(model.ky_math_.size(), ny_);

  std::vector<double> ikx_math = {
    -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0,
     0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0
  };
  std::vector<double> iky_math = {
    -4.0, -3.0, -2.0, -1.0,  0.0,  1.0,  2.0,  3.0
  };
  std::vector<double> ikx_fft = {
     0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,
    -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0
  };
  std::vector<double> iky_fft = {
    0.0,  1.0,  2.0,  3.0, -4.0, -3.0, -2.0, -1.0
  };

  // check kx math-ordering
  for (int i=0; i<nx_; ++i)
  {
    EXPECT_DOUBLE_EQ(model.kx_math_[i] / model.kx_f_, ikx_math[i]);
  }
  // check ky math-ordering
  for (int i=0; i<ny_; ++i)
  {
    EXPECT_DOUBLE_EQ(model.ky_math_[i] / model.ky_f_, iky_math[i]);
  }

  // check kx fft-ordering
  for (int i=0; i<nx_; ++i)
  {
    EXPECT_DOUBLE_EQ(model.kx_fft_[i] / model.kx_f_, ikx_fft[i]);
  }
  // check ky fft-ordering
  for (int i=0; i<ny_; ++i)
  {
    EXPECT_DOUBLE_EQ(model.ky_fft_[i] / model.ky_f_, iky_fft[i]);
  }
}

//////////////////////////////////////////////////
// Reference version checks
TEST_F(TestFixtureWaveSimulationFFT2, HermitianTimeZeroReference)
{
  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(0.0);

  for (int ikx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky)
    {
      // index for conjugate
      int ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      int cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      complex h  = model.fft_h_(ikx, iky);
      complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());
      
      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, HermitianTimeNonZeroReference)
{
  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(11.2);

  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      // index for conjugate
      int cdx = 0;
      int ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
        cdx += ckx * ny_;
      }
      int cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
        cdx += cky;
      }

      // look up amplitude and conjugate
      complex h  = model.fft_h_(ikx, iky);
      complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());
      
      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, ParsevalsIdentityTimeZeroReference)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(0.0);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }
  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, ParsevalsIdentityTimeNonZeroReference)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(25.3);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, HorizontalDisplacementsLambdaZeroReference)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);

  // displacements should be zero when lamda = 0
  model.SetLambda(0.0);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(10.0);

  Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd sy = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeDisplacements(sx, sy);

  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(sx(i, 0), 0.0);
    EXPECT_DOUBLE_EQ(sy(i, 0), 0.0);
  }
}

//////////////////////////////////////////////////
// Optimised version checks
TEST_F(TestFixtureWaveSimulationFFT2, HermitianTimeZero)
{
  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (int ikx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky)
    {
      // index for conjugate
      int ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
      }
      int cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
      }

      // look up amplitude and conjugate
      complex h  = model.fft_h_(ikx, iky);
      complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());
      
      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, HermitianTimeNonZero)
{
  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(11.2);

  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      // index for conjugate
      int cdx = 0;
      int ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
        cdx += ckx * ny_;
      }
      int cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
        cdx += cky;
      }

      // look up amplitude and conjugate
      complex h  = model.fft_h_(ikx, iky);
      complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());
      
      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, ParsevalsIdentityTimeZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, ParsevalsIdentityTimeNonZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(25.3);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, HorizontalDisplacementsLambdaZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);

  // displacements should be zero when lamda = 0
  model.SetLambda(0.0);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(10.0);

  Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd sy = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeDisplacements(sx, sy);

  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(sx(i, 0), 0.0);
    EXPECT_DOUBLE_EQ(sy(i, 0), 0.0);
  }
}

//////////////////////////////////////////////////
// Cross-check optimised version against reference 
TEST_F(TestFixtureWaveSimulationFFT2, ElevationTimeZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(0.0);

  Eigen::MatrixXd ref_z = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevation(ref_z);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(ref_z.size(), n2);
  EXPECT_EQ(z.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(z(i, 0), ref_z(i, 0));
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, ElevationTimeNonZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(31.7);

  Eigen::MatrixXd ref_z = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevation(ref_z);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(31.7);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(ref_z.size(), n2);
  EXPECT_EQ(z.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(z(i, 0), ref_z(i, 0));
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, Displacement)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(12.2);

  Eigen::MatrixXd ref_sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_sy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeDisplacements(ref_sx, ref_sy);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd sy = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeDisplacements(sx, sy);

  EXPECT_EQ(ref_sx.size(), n2);
  EXPECT_EQ(ref_sy.size(), n2);
  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(sx(i, 0), ref_sx(i, 0));
    EXPECT_DOUBLE_EQ(sy(i, 0), ref_sy(i, 0));
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, ElevationDerivatives)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(12.2);

  Eigen::MatrixXd ref_dhdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_dhdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevationDerivatives(ref_dhdx, ref_dhdy);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::MatrixXd dhdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd dhdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevationDerivatives(dhdx, dhdy);

  EXPECT_EQ(ref_dhdx.size(), n2);
  EXPECT_EQ(ref_dhdy.size(), n2);
  EXPECT_EQ(dhdx.size(), n2);
  EXPECT_EQ(dhdy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(dhdx(i, 0), ref_dhdx(i, 0));
    EXPECT_DOUBLE_EQ(dhdy(i, 0), ref_dhdy(i, 0));
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, DisplacementDerivatives)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(12.2);

  Eigen::MatrixXd ref_dsxdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_dsydy = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_dsxdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeDisplacementsDerivatives(ref_dsxdx, ref_dsydy, ref_dsxdy);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::MatrixXd dsxdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd dsydy = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd dsxdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeDisplacementsDerivatives(dsxdx, dsydy, dsxdy);

  EXPECT_EQ(ref_dsxdx.size(), n2);
  EXPECT_EQ(ref_dsydy.size(), n2);
  EXPECT_EQ(ref_dsxdy.size(), n2);
  EXPECT_EQ(dsxdx.size(), n2);
  EXPECT_EQ(dsydy.size(), n2);
  EXPECT_EQ(dsxdy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(dsxdx(i, 0), ref_dsxdx(i, 0));
    EXPECT_DOUBLE_EQ(dsydy(i, 0), ref_dsydy(i, 0));
    EXPECT_DOUBLE_EQ(dsxdy(i, 0), ref_dsxdy(i, 0));
  }
}

//////////////////////////////////////////////////
// check we're the indexing / stride rules used in the FFT routines
TEST_F(TestFixtureWaveSimulationFFT2, Indexing)
{
  int nxx = 4;
  int nyy = 3;

  // column major storage
  std::vector<std::vector<double>> a1(nyy, std::vector<double>(nxx, 0.0));
  
  for (int iky = 0, idx = 0; iky < nyy; ++iky)
  {
    for (int ikx = 0; ikx < nxx; ++ikx, ++idx)
    {
      a1[iky][ikx] = idx;
    }
  }

  // column major storage - fastest index over rows
  // 
  //  M = 0   4   8 
  //      1   5   9
  //      2   6  10
  //      3   7  11
  // 
  std::vector<std::vector<double>> a2 = {
    { 0, 1, 2, 3},
    { 4, 5, 6, 7},
    { 8, 9, 10, 11}
  };

  std::vector<double> a3 = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
  };

  EXPECT_EQ(a1.size(), nyy);
  EXPECT_EQ(a1[0].size(), nxx);

  EXPECT_EQ(a2.size(), nyy);
  EXPECT_EQ(a2[0].size(), nxx);

  EXPECT_EQ(a3.size(), nyy * nxx);

  for (int iky = 0; iky < nyy; ++iky)
  {
    for (int ikx = 0; ikx < nxx; ++ikx)
    {
      int idx = iky * nxx + ikx;
      EXPECT_EQ(a1[iky][ikx], a2[iky][ikx]);
      EXPECT_EQ(a3[idx], a2[iky][ikx]);
    }
  }

}

//////////////////////////////////////////////////
// Cross-check vectorised version 
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedHermitianTimeZero)
{
  WaveSimulationFFT2Impl model1(lx_, ly_, nx_, ny_);
  model1.SetUseVectorised(false);
  model1.ComputeBaseAmplitudes();
  model1.ComputeCurrentAmplitudes(0.0);

  WaveSimulationFFT2Impl model2(lx_, ly_, nx_, ny_);
  model2.SetUseVectorised(true);
  model2.ComputeBaseAmplitudes();
  model2.ComputeCurrentAmplitudes(0.0);

  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      // index for conjugate
      int cdx = 0;
      int ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
        cdx += ckx * ny_;
      }
      int cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
        cdx += cky;
      }

      // look up amplitude and conjugate
      complex h1  = model1.fft_h_(ikx, iky);
      complex hc1 = model1.fft_h_(ckx, cky);

      complex h2  = model2.fft_h_(ikx, iky);
      complex hc2 = model2.fft_h_(ckx, cky);

      // consistency: real part symmetric
      EXPECT_DOUBLE_EQ(h2.real(), hc2.real());
      
      // consistency: imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h2.imag(), -1.0 * hc2.imag());

      // cross-check: real part
      EXPECT_DOUBLE_EQ(h2.real(), h1.real());
      
      // cross-check: imaginary part
      EXPECT_NEAR(h2.imag(), h1.imag(), 1.0E-14);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedHermitianTimeNonZero)
{
  WaveSimulationFFT2Impl model1(lx_, ly_, nx_, ny_);
  model1.SetUseVectorised(false);
  model1.ComputeBaseAmplitudes();
  model1.ComputeCurrentAmplitudes(13.6);

  WaveSimulationFFT2Impl model2(lx_, ly_, nx_, ny_);
  model2.SetUseVectorised(true);
  model2.ComputeBaseAmplitudes();
  model2.ComputeCurrentAmplitudes(13.6);

  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      // index for conjugate
      int cdx = 0;
      int ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
        cdx += ckx * ny_;
      }
      int cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
        cdx += cky;
      }

      // look up amplitude and conjugate
      complex h1  = model1.fft_h_(ikx, iky);
      complex hc1 = model1.fft_h_(ckx, cky);

      complex h2  = model2.fft_h_(ikx, iky);
      complex hc2 = model2.fft_h_(ckx, cky);

      // consistency: real part symmetric
      EXPECT_DOUBLE_EQ(h2.real(), hc2.real());
      
      // consistency: imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h2.imag(), -1.0 * hc2.imag());

      // cross-check: real part
      // EXPECT_DOUBLE_EQ(h2.real(), h1.real());
      EXPECT_NEAR(h2.real(), h1.real(), 1.0E-14);
      
      // cross-check: imaginary part
      // EXPECT_DOUBLE_EQ(h2.imag(), h1.imag());
      EXPECT_NEAR(h2.imag(), h1.imag(), 1.0E-14);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedParsevalsIdentityTimeZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedParsevalsIdentityTimeNonZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(25.3);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int ikx=0, idx=0; ikx<nx_; ++ikx)
  {
    for (int iky=0; iky<ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_DOUBLE_EQ(sum_z2, sum_h2 * n2);
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedHorizontalDisplacementsLambdaZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);

  // displacements should be zero when lamda = 0
  model.SetUseVectorised(true);
  model.SetLambda(0.0);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(10.0);

  Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd sy = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeDisplacements(sx, sy);

  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(sx(i, 0), 0.0);
    EXPECT_DOUBLE_EQ(sy(i, 0), 0.0);
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedElevationTimeZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.SetUseVectorised(false);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(0.0);

  Eigen::MatrixXd ref_z = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevation(ref_z);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(ref_z.size(), n2);
  EXPECT_EQ(z.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    // EXPECT_DOUBLE_EQ(z(i, 0), ref_z(i, 0));
    EXPECT_NEAR(z(i, 0), ref_z(i, 0), 1.0E-14);
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedElevationTimeNonZero)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.SetUseVectorised(false);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(31.7);

  Eigen::MatrixXd ref_z = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevation(ref_z);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(31.7);

  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeElevation(z);

  EXPECT_EQ(ref_z.size(), n2);
  EXPECT_EQ(z.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    // EXPECT_DOUBLE_EQ(z(i, 0), ref_z(i, 0));
    EXPECT_NEAR(z(i, 0), ref_z(i, 0), 1.0E-14);
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedDisplacement)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.SetUseVectorised(false);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(12.2);

  Eigen::MatrixXd ref_sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_sy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeDisplacements(ref_sx, ref_sy);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd sy = Eigen::MatrixXd::Zero(n2, 1);
  model.ComputeDisplacements(sx, sy);

  EXPECT_EQ(ref_sx.size(), n2);
  EXPECT_EQ(ref_sy.size(), n2);
  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    // EXPECT_DOUBLE_EQ(sx(i, 0), ref_sx(i, 0));
    EXPECT_NEAR(sx(i, 0), ref_sx(i, 0), 1.0E-14);

    // EXPECT_DOUBLE_EQ(sy(i, 0), ref_sy(i, 0));
    EXPECT_NEAR(sy(i, 0), ref_sy(i, 0), 1.0E-14);
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedElevationDerivatives)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.SetUseVectorised(false);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(12.2);

  Eigen::MatrixXd ref_dhdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_dhdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevationDerivatives(ref_dhdx, ref_dhdy);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::MatrixXd dhdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd dhdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeElevationDerivatives(dhdx, dhdy);

  EXPECT_EQ(ref_dhdx.size(), n2);
  EXPECT_EQ(ref_dhdy.size(), n2);
  EXPECT_EQ(dhdx.size(), n2);
  EXPECT_EQ(dhdy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(dhdx(i, 0), ref_dhdx(i, 0));
    EXPECT_DOUBLE_EQ(dhdy(i, 0), ref_dhdy(i, 0));
  }
}

//////////////////////////////////////////////////
TEST_F(TestFixtureWaveSimulationFFT2, VectorisedDisplacementDerivatives)
{
  int n2 = nx_ * ny_;

  WaveSimulationFFT2Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.SetUseVectorised(false);
  ref_model.ComputeBaseAmplitudesReference();
  ref_model.ComputeCurrentAmplitudesReference(12.2);

  Eigen::MatrixXd ref_dsxdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_dsydy = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd ref_dsxdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeDisplacementsDerivatives(ref_dsxdx, ref_dsydy, ref_dsxdy);

  WaveSimulationFFT2Impl model(lx_, ly_, nx_, ny_);
  model.SetUseVectorised(true);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::MatrixXd dsxdx = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd dsydy = Eigen::MatrixXd::Zero(n2, 1);
  Eigen::MatrixXd dsxdy = Eigen::MatrixXd::Zero(n2, 1);
  ref_model.ComputeDisplacementsDerivatives(dsxdx, dsydy, dsxdy);

  EXPECT_EQ(ref_dsxdx.size(), n2);
  EXPECT_EQ(ref_dsydy.size(), n2);
  EXPECT_EQ(ref_dsxdy.size(), n2);
  EXPECT_EQ(dsxdx.size(), n2);
  EXPECT_EQ(dsydy.size(), n2);
  EXPECT_EQ(dsxdy.size(), n2);

  for (int i=0; i<n2; ++i)
  {
    EXPECT_DOUBLE_EQ(dsxdx(i, 0), ref_dsxdx(i, 0));
    EXPECT_DOUBLE_EQ(dsydy(i, 0), ref_dsydy(i, 0));
    EXPECT_DOUBLE_EQ(dsxdy(i, 0), ref_dsxdy(i, 0));
  }
}

//////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

