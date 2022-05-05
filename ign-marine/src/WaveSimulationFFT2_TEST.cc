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
  EXPECT_EQ(model.kx_math.size(), this->N);

  std::vector<double> ikx_math =
      {-4.0, -3.0, -2.0, -1.0,  0.0,  1.0,  2.0,  3.0};
  std::vector<double> ikx_fft =
      { 0.0,  1.0,  2.0,  3.0, -4.0, -3.0, -2.0, -1.0};

  // check kx math-ordering
  for (int i=0; i<this->N; ++i)
  {
    EXPECT_EQ(model.kx_math[i] / model.kx_f, ikx_math[i]);
  }

  // check kx fft-ordering
  for (int i=0; i<this->N; ++i)
  {
    EXPECT_EQ(model.kx_fft[i] / model.kx_f, ikx_fft[i]);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

