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

#include "ignition/marine/WaveSimulationFFT2.hh"

#include "WaveSimulationFFT2Impl.hh"

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

using namespace ignition;
using namespace marine;

///////////////////////////////////////////////////////////////////////////////
// Define tests

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
  double L = 100.0;
  int    N = 8;
};

///////////////////////////////////////////////////////////////////////////////
// Define tests
 
TEST_F(TestFixtureWaveSimulationFFT2, AngularSpatialWavenumber)
{
  WaveSimulationFFT2Impl model(this->N, this->L);

  // check array dimensions
  EXPECT_EQ(model.kx_fft.size(), this->N);
  EXPECT_EQ(model.ky_fft.size(), this->N);
  EXPECT_EQ(model.kx_math.size(), this->N);
  EXPECT_EQ(model.ky_math.size(), this->N);

  std::vector<double> ikx_math =
      {-4.0, -3.0, -2.0, -1.0,  0.0,  1.0,  2.0,  3.0};
  std::vector<double> iky_math = ikx_math;
  std::vector<double> ikx_fft =
      { 0.0,  1.0,  2.0,  3.0, -4.0, -3.0, -2.0, -1.0};
  std::vector<double> iky_fft = ikx_fft;

  // check kx math-ordering
  for (int i=0; i<this->N; ++i)
  {
    ASSERT_DOUBLE_EQ(model.kx_math[i] / model.kx_f, ikx_math[i]);
    ASSERT_DOUBLE_EQ(model.ky_math[i] / model.ky_f, iky_math[i]);
  }

  // check kx fft-ordering
  for (int i=0; i<this->N; ++i)
  {
    ASSERT_DOUBLE_EQ(model.kx_fft[i] / model.kx_f, ikx_fft[i]);
    ASSERT_DOUBLE_EQ(model.ky_fft[i] / model.ky_f, iky_fft[i]);
  }
}

TEST_F(TestFixtureWaveSimulationFFT2, HermitianTimeZeroReference)
{
  WaveSimulationFFT2Impl model(this->N, this->L);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(0.0);

  for (int ikx=0; ikx<this->N; ++ikx)
  {
    for (int iky=0; iky<this->N; ++iky)
    {
      // index for flattened array
      int idx = ikx * this->N + iky;

      // index for conjugate
      int cdx = 0;
      if (ikx == 0)
        cdx += ikx * this->N;
      else
        cdx += (this->N - ikx) * this->N;

      if (iky == 0)
        cdx += iky;
      else
        cdx += (this->N - iky);

      // look up amplitude and conjugate
      complex h  = model.mH[idx];
      complex hc = model.mH[cdx];

      // real part symmetric
      ASSERT_DOUBLE_EQ(h.real(), hc.real());
      
      // imaginary part anti-symmetric
      ASSERT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

TEST_F(TestFixtureWaveSimulationFFT2, HermitianTimeNonZeroReference)
{
  WaveSimulationFFT2Impl model(this->N, this->L);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(11.2);

  for (int ikx=0; ikx<this->N; ++ikx)
  {
    for (int iky=0; iky<this->N; ++iky)
    {
      // index for flattened array
      int idx = ikx * this->N + iky;

      // index for conjugate
      int cdx = 0;
      if (ikx == 0)
        cdx += ikx * this->N;
      else
        cdx += (this->N - ikx) * this->N;

      if (iky == 0)
        cdx += iky;
      else
        cdx += (this->N - iky);

      // look up amplitude and conjugate
      complex h  = model.mH[idx];
      complex hc = model.mH[cdx];

      // real part symmetric
      ASSERT_DOUBLE_EQ(h.real(), hc.real());
      
      // imaginary part anti-symmetric
      ASSERT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

TEST_F(TestFixtureWaveSimulationFFT2, ParsevalsIdentityTimeZeroReference)
{
  int N2 = this->N * this->N;

  WaveSimulationFFT2Impl model(this->N, this->L);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(0.0);

  std::vector<double> z;
  model.ComputeHeights(z);

  EXPECT_EQ(z.size(), N2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int i=0; i<N2; ++i)
  {
    sum_z2 += z[i]*z[i];
    sum_h2 += norm(model.mH[i]);
  }

  ASSERT_DOUBLE_EQ(sum_z2, sum_h2 * N2);
}

TEST_F(TestFixtureWaveSimulationFFT2, ParsevalsIdentityTimeNonZeroReference)
{
  int N2 = this->N * this->N;

  WaveSimulationFFT2Impl model(this->N, this->L);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(25.3);

  std::vector<double> z;
  model.ComputeHeights(z);

  EXPECT_EQ(z.size(), N2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (int i=0; i<N2; ++i)
  {
    sum_z2 += z[i]*z[i];
    sum_h2 += norm(model.mH[i]);
  }

  ASSERT_DOUBLE_EQ(sum_z2, sum_h2 * N2);
}

TEST_F(TestFixtureWaveSimulationFFT2, HorizontalDisplacementsLambdaZeroReference)
{
  int N2 = this->N * this->N;

  WaveSimulationFFT2Impl model(this->N, this->L);

  // displacements should be zero when lamda = 0
  model.SetLambda(0.0);
  model.ComputeBaseAmplitudesReference();
  model.ComputeCurrentAmplitudesReference(10.0);

  std::vector<double> sx, sy;
  model.ComputeDisplacements(sx, sy);

  EXPECT_EQ(sx.size(), N2);
  EXPECT_EQ(sy.size(), N2);

  for (int i=0; i<N2; ++i)
  {
    ASSERT_DOUBLE_EQ(sx[i], 0.0);
    ASSERT_DOUBLE_EQ(sy[i], 0.0);
  }
}


// check we're the indexing / stride rules used in the FFT routines
TEST_F(TestFixtureWaveSimulationFFT2, Indexing)
{
  int Nx = 3;
  int Ny = 4;

  std::vector<std::vector<double>> a1(Nx, std::vector<double>(Ny, 0.0));
  
  for (int ikx = 0, idx = 0; ikx < Nx; ++ikx)
  {
    for (int iky = 0; iky < Ny; ++iky, ++idx)
    {
      a1[ikx][iky] = idx;
    }
  }

  std::vector<std::vector<double>> a2 = {
    { 0, 1, 2, 3},
    { 4, 5, 6, 7},
    { 8, 9, 10, 11}
  };

  std::vector<double> a3 = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
  };

  EXPECT_EQ(a1.size(), Nx);
  EXPECT_EQ(a1[0].size(), Ny);

  EXPECT_EQ(a2.size(), Nx);
  EXPECT_EQ(a2[0].size(), Ny);

  EXPECT_EQ(a3.size(), Nx * Ny);

  for (int ikx = 0; ikx < Nx; ++ikx)
  {
    for (int iky = 0; iky < Ny; ++iky)
    {
      int idx = ikx * Ny + iky;
      EXPECT_EQ(a1[ikx][iky], a2[ikx][iky]);
      EXPECT_EQ(a3[idx], a2[ikx][iky]);
    }
  }

}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

