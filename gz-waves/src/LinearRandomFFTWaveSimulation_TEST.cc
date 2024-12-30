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

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <complex>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gz/waves/LinearRandomFFTWaveSimulation.hh"
#include "LinearRandomFFTWaveSimulationImpl.hh"
#include "LinearRandomFFTWaveSimulationRefImpl.hh"

using gz::waves::Index;
using gz::waves::LinearRandomFFTWaveSimulation;
using gz::waves::LinearRandomFFTWaveSimulationRef;

#define DISABLE_FOR_REAL_DFT 1

//////////////////////////////////////////////////
// Define fixture
class LinearRandomFFTWaveSimFixture: public ::testing::Test
{
 public:
  virtual ~LinearRandomFFTWaveSimFixture() = default;
  LinearRandomFFTWaveSimFixture() = default;

  // put in any custom data members that you need
  double lx_ = 200.0;
  double ly_ = 100.0;
  Index  nx_ = 16;
  Index  ny_ = 8;
};

//////////////////////////////////////////////////
// Define tests
TEST_F(LinearRandomFFTWaveSimFixture, AngularSpatialWavenumber)
{
  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);

  // check array dimensions
  EXPECT_EQ(model.kx_fft_.size(), nx_);
  EXPECT_EQ(model.ky_fft_.size(), ny_);

  std::vector<double> ikx_fft = {
     0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,
    -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0
  };
  std::vector<double> iky_fft = {
    0.0,  1.0,  2.0,  3.0, -4.0, -3.0, -2.0, -1.0
  };

  // check kx fft-ordering
  for (Index i=0; i < nx_; ++i)
  {
    EXPECT_DOUBLE_EQ(model.kx_fft_[i] / model.kx_f_, ikx_fft[i]);
  }
  // check ky fft-ordering
  for (Index i=0; i < ny_; ++i)
  {
    EXPECT_DOUBLE_EQ(model.ky_fft_[i] / model.ky_f_, iky_fft[i]);
  }
}

//////////////////////////////////////////////////
// Reference version checks
TEST_F(LinearRandomFFTWaveSimFixture, HermitianHTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_(ikx, iky);
      std::complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianDhDxTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_ikx_(ikx, iky);
      std::complex hc = model.fft_h_ikx_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianDhDyTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_iky_(ikx, iky);
      std::complex hc = model.fft_h_iky_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianSxTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_sx_(ikx, iky);
      std::complex hc = model.fft_sx_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianSyTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_sy_(ikx, iky);
      std::complex hc = model.fft_sy_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianDsxDxTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_kxkx_(ikx, iky);
      std::complex hc = model.fft_h_kxkx_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianDsyDyTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_kyky_(ikx, iky);
      std::complex hc = model.fft_h_kyky_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianDsxDyTimeZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
        ckx = nx_ - ikx;

      Index cky = 0;
      if (iky != 0)
        cky = ny_ - iky;

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_kxky_(ikx, iky);
      std::complex hc = model.fft_h_kxky_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HermitianTimeNonZeroReference)
{
  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(11.2);

  for (Index ikx=0, idx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky, ++idx)
    {
      // index for conjugate
      // Index cdx = 0;
      Index ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
        // cdx += ckx * ny_;
      }
      Index cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
        // cdx += cky;
      }

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_(ikx, iky);
      std::complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, ParsevalsIdentityTimeZeroReference)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::ArrayXXd z = Eigen::ArrayXXd::Zero(n2, 1);
  model.ElevationAt(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (Index ikx=0, idx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }
  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, ParsevalsIdentityTimeNonZeroReference)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(25.3);

  Eigen::ArrayXXd z = Eigen::ArrayXXd::Zero(n2, 1);
  model.ElevationAt(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (Index ikx=0, idx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture,
    HorizontalDisplacementsLambdaZeroReference)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl model(lx_, ly_, nx_, ny_);

  // displacements should be zero when lamda = 0
  model.SetLambda(0.0);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(10.0);

  Eigen::ArrayXXd sx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd sy = Eigen::ArrayXXd::Zero(n2, 1);
  model.DisplacementAt(sx, sy);

  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_DOUBLE_EQ(sx(i, 0), 0.0);
    EXPECT_DOUBLE_EQ(sy(i, 0), 0.0);
  }
}

//////////////////////////////////////////////////
// Optimised version checks
#if !DISABLE_FOR_REAL_DFT
/// \note test disabled - optimised version does not store HC
TEST_F(LinearRandomFFTWaveSimFixture, HermitianTimeZero)
{
  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  for (Index ikx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky)
    {
      // index for conjugate
      Index ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
      }
      Index cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
      }

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_(ikx, iky);
      std::complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}
#endif

//////////////////////////////////////////////////
#if !DISABLE_FOR_REAL_DFT
/// \note test disabled - optimised version does not store HC
TEST_F(LinearRandomFFTWaveSimFixture, HermitianTimeNonZero)
{
  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(11.2);

  for (Index ikx=0, idx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky, ++idx)
    {
      // index for conjugate
      Index cdx = 0;
      Index ckx = 0;
      if (ikx != 0)
      {
        ckx = nx_ - ikx;
        cdx += ckx * ny_;
      }
      Index cky = 0;
      if (iky != 0)
      {
        cky = ny_ - iky;
        cdx += cky;
      }

      // look up amplitude and conjugate
      std::complex h  = model.fft_h_(ikx, iky);
      std::complex hc = model.fft_h_(ckx, cky);

      // real part symmetric
      EXPECT_DOUBLE_EQ(h.real(), hc.real());

      // imaginary part anti-symmetric
      EXPECT_DOUBLE_EQ(h.imag(), -1.0 * hc.imag());
    }
  }
}
#endif

//////////////////////////////////////////////////
#if 0
TEST_F(LinearRandomFFTWaveSimFixture, ParsevalsIdentityTimeZero)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::ArrayXXd z = Eigen::ArrayXXd::Zero(n2, 1);
  model.ElevationAt(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (Index ikx=0, idx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}
#endif

//////////////////////////////////////////////////
#if 0
TEST_F(LinearRandomFFTWaveSimFixture, ParsevalsIdentityTimeNonZero)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(25.3);

  Eigen::ArrayXXd z = Eigen::ArrayXXd::Zero(n2, 1);
  model.ElevationAt(z);

  EXPECT_EQ(z.size(), n2);

  double sum_z2 = 0.0;
  double sum_h2 = 0.0;
  for (Index ikx=0, idx=0; ikx < nx_; ++ikx)
  {
    for (Index iky=0; iky < ny_; ++iky, ++idx)
    {
      sum_z2 += z(idx, 0) * z(idx, 0);
      sum_h2 += norm(model.fft_h_(ikx, iky));
    }
  }

  EXPECT_NEAR(sum_z2, sum_h2 * n2, 1.0E-14);
}
#endif

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, HorizontalDisplacementsLambdaZero)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);

  // displacements should be zero when lamda = 0
  model.SetLambda(0.0);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(10.0);

  Eigen::ArrayXXd sx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd sy = Eigen::ArrayXXd::Zero(n2, 1);
  model.DisplacementAt(sx, sy);

  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_DOUBLE_EQ(sx(i, 0), 0.0);
    EXPECT_DOUBLE_EQ(sy(i, 0), 0.0);
  }
}

//////////////////////////////////////////////////
// Cross-check optimised version against reference
TEST_F(LinearRandomFFTWaveSimFixture, ElevationTimeZero)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudes();
  ref_model.ComputeCurrentAmplitudes(0.0);

  Eigen::ArrayXXd ref_z = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.ElevationAt(ref_z);

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(0.0);

  Eigen::ArrayXXd z = Eigen::ArrayXXd::Zero(n2, 1);
  model.ElevationAt(z);

  EXPECT_EQ(ref_z.size(), n2);
  EXPECT_EQ(z.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_NEAR(z(i, 0), ref_z(i, 0), 1.0E-15);
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, ElevationTimeNonZero)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudes();
  ref_model.ComputeCurrentAmplitudes(31.7);

  Eigen::ArrayXXd ref_z = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.ElevationAt(ref_z);

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(31.7);

  Eigen::ArrayXXd z = Eigen::ArrayXXd::Zero(n2, 1);
  model.ElevationAt(z);

  EXPECT_EQ(ref_z.size(), n2);
  EXPECT_EQ(z.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_NEAR(z(i, 0), ref_z(i, 0), 1.0E-15);
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, Displacement)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudes();
  ref_model.ComputeCurrentAmplitudes(12.2);

  Eigen::ArrayXXd ref_sx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd ref_sy = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.DisplacementAt(ref_sx, ref_sy);

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::ArrayXXd sx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd sy = Eigen::ArrayXXd::Zero(n2, 1);
  model.DisplacementAt(sx, sy);

  EXPECT_EQ(ref_sx.size(), n2);
  EXPECT_EQ(ref_sy.size(), n2);
  EXPECT_EQ(sx.size(), n2);
  EXPECT_EQ(sy.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_NEAR(sx(i, 0), ref_sx(i, 0), 1.0E-15);
    EXPECT_NEAR(sy(i, 0), ref_sy(i, 0), 1.0E-15);
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, ElevationDerivatives)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudes();
  ref_model.ComputeCurrentAmplitudes(12.2);

  Eigen::ArrayXXd ref_dhdx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd ref_dhdy = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.ElevationDerivAt(ref_dhdx, ref_dhdy);

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::ArrayXXd dhdx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd dhdy = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.ElevationDerivAt(dhdx, dhdy);

  EXPECT_EQ(ref_dhdx.size(), n2);
  EXPECT_EQ(ref_dhdy.size(), n2);
  EXPECT_EQ(dhdx.size(), n2);
  EXPECT_EQ(dhdy.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_DOUBLE_EQ(dhdx(i, 0), ref_dhdx(i, 0));
    EXPECT_DOUBLE_EQ(dhdy(i, 0), ref_dhdy(i, 0));
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, DisplacementDerivatives)
{
  Index n2 = nx_ * ny_;

  LinearRandomFFTWaveSimulationRef::Impl ref_model(lx_, ly_, nx_, ny_);
  ref_model.ComputeBaseAmplitudes();
  ref_model.ComputeCurrentAmplitudes(12.2);

  Eigen::ArrayXXd ref_dsxdx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd ref_dsydy = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd ref_dsxdy = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.DisplacementDerivAt(ref_dsxdx, ref_dsydy, ref_dsxdy);

  LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
  model.ComputeBaseAmplitudes();
  model.ComputeCurrentAmplitudes(12.2);

  Eigen::ArrayXXd dsxdx = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd dsydy = Eigen::ArrayXXd::Zero(n2, 1);
  Eigen::ArrayXXd dsxdy = Eigen::ArrayXXd::Zero(n2, 1);
  ref_model.DisplacementDerivAt(dsxdx, dsydy, dsxdy);

  EXPECT_EQ(ref_dsxdx.size(), n2);
  EXPECT_EQ(ref_dsydy.size(), n2);
  EXPECT_EQ(ref_dsxdy.size(), n2);
  EXPECT_EQ(dsxdx.size(), n2);
  EXPECT_EQ(dsydy.size(), n2);
  EXPECT_EQ(dsxdy.size(), n2);

  for (Index i=0; i < n2; ++i)
  {
    EXPECT_DOUBLE_EQ(dsxdx(i, 0), ref_dsxdx(i, 0));
    EXPECT_DOUBLE_EQ(dsydy(i, 0), ref_dsydy(i, 0));
    EXPECT_DOUBLE_EQ(dsxdy(i, 0), ref_dsxdy(i, 0));
  }
}

//////////////////////////////////////////////////
// check we're the indexing / stride rules used in the FFT routines
TEST_F(LinearRandomFFTWaveSimFixture, Indexing)
{
  Index nxx = 4;
  Index nyy = 3;

  // column major storage
  std::vector<std::vector<double>> a1(nyy, std::vector<double>(nxx, 0.0));

  for (Index iky = 0, idx = 0; iky < nyy; ++iky)
  {
    for (Index ikx = 0; ikx < nxx; ++ikx, ++idx)
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

  for (Index iky = 0; iky < nyy; ++iky)
  {
    for (Index ikx = 0; ikx < nxx; ++ikx)
    {
      Index idx = iky * nxx + ikx;
      EXPECT_EQ(a1[iky][ikx], a2[iky][ikx]);
      EXPECT_EQ(a3[idx], a2[iky][ikx]);
    }
  }
}

//////////////////////////////////////////////////
TEST_F(LinearRandomFFTWaveSimFixture, DisplacementDerivativesWindDirection)
{
  int n2 = nx_ * ny_;
  double dt = 6.0;
  double du = 2.5;
  double da = 18.0;

  // test over a range of times, windspeeds and  directions
  for (int it=0; it < 10; ++it)
  {
    double time = it * dt;
    for (int iu=0; iu < 10; ++iu)
    {
      double u = iu * du;
      for (int ia=0; ia < 20; ++ia)
      {
        double angle_deg = ia * da;
        double angle_rad = angle_deg * M_PI / 180.0;

        double ux = u * std::cos(angle_rad);
        double uy = u * std::sin(angle_rad);

        // std::cerr << "angle: " << angle_rad << "\n";

        LinearRandomFFTWaveSimulationRef::Impl ref_model(lx_, ly_, nx_, ny_);
        ref_model.SetWindVelocity(ux, uy);
        ref_model.ComputeBaseAmplitudes();
        ref_model.ComputeCurrentAmplitudes(time);

        Eigen::MatrixXd ref_h = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd ref_sx = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd ref_sy = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd ref_dhdx = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd ref_dhdy = Eigen::MatrixXd::Zero(n2, 1);
        ref_model.ElevationAt(ref_h);
        ref_model.ElevationDerivAt(ref_dhdx, ref_dhdy);
        ref_model.DisplacementAt(ref_sx, ref_sy);

        LinearRandomFFTWaveSimulation::Impl model(lx_, ly_, nx_, ny_);
        model.SetWindVelocity(ux, uy);
        model.ComputeBaseAmplitudes();
        model.ComputeCurrentAmplitudes(time);

        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd sy = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd dhdx = Eigen::MatrixXd::Zero(n2, 1);
        Eigen::MatrixXd dhdy = Eigen::MatrixXd::Zero(n2, 1);
        model.ElevationAt(h);
        model.ElevationDerivAt(dhdx, dhdy);
        model.DisplacementAt(sx, sy);

        EXPECT_EQ(ref_h.size(), n2);
        EXPECT_EQ(h.size(), n2);

        for (int i=0; i < n2; ++i)
        {
          EXPECT_NEAR(h(i, 0), ref_h(i, 0), 1.0E-14);
          EXPECT_NEAR(sx(i, 0), ref_sx(i, 0), 1.0E-14);
          EXPECT_NEAR(sy(i, 0), ref_sy(i, 0), 1.0E-14);
          EXPECT_NEAR(dhdx(i, 0), ref_dhdx(i, 0), 1.0E-14);
          EXPECT_NEAR(dhdy(i, 0), ref_dhdy(i, 0), 1.0E-14);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

