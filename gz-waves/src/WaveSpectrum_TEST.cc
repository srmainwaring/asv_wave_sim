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

#include <memory>
#include <vector>

#include "gz/waves/Types.hh"
#include "gz/waves/WaveSpectrum.hh"

#include "LinearRandomFFTWaveSimulationRefImpl.hh"

using gz::waves::ECKVWaveSpectrum;
using gz::waves::Index;
using gz::waves::LinearRandomFFTWaveSimulationRef;
using gz::waves::PiersonMoskowitzWaveSpectrum;

//////////////////////////////////////////////////
TEST(WaveSpectrum, PiersonMoskowitzSpectrumRegression)
{
  double k[] {
    0.,         0.06283185, 0.12566371, 0.18849556,
    0.25132741, 0.31415927, 0.37699112, 0.43982297
  };

  { // Regress against values generate from Python reference version
    double tolerance = 1.0e-15;
    double u19 = 0.0;
    PiersonMoskowitzWaveSpectrum spectrum(u19);

    double cap_s[] = {
      0., 0., 0., 0.,
      0., 0., 0., 0.
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }

  {
    double tolerance = 1.0e-8;
    double u19 = 5.0;
    PiersonMoskowitzWaveSpectrum spectrum(u19);

    double cap_s[] = {
      0.00000000e+00, 4.76656355e-12, 1.50019573e-03, 2.44797002e-02,
      4.20064584e-02, 4.11734258e-02, 3.39059000e-02, 2.64125839e-02
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }

  {
    double tolerance = 1.0e-7;
    double u19 = 10.0;
    PiersonMoskowitzWaveSpectrum spectrum(u19);

    double cap_s[] = {
      0.,         2.68841334, 1.30008153, 0.49488587,
      0.22791438, 0.12152584, 0.07189522, 0.04588103
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }

  {
    double tolerance = 5.0e-6;
    double u19 = 15.0;
    PiersonMoskowitzWaveSpectrum spectrum(u19);

    double cap_s[] = {
      0.,         11.43315027,  1.86697307,  0.58124237,
      0.24949601,  0.12877022, 0.07484505,  0.04725667
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }
}

//////////////////////////////////////////////////
TEST(WaveSpectrum, PiersonMoskowitzSpectrumArrayXXd)
{
  { // Eigen array version
    double tolerance = 1.0e-16;

    double lx = 200.0;
    double ly = 100.0;
    Index nx = 32;
    Index ny = 16;

    double kx_nyquist = M_PI * nx / lx;
    double ky_nyquist = M_PI * ny / ly;

    // create wavenumber vectors
    Eigen::ArrayXd kx_v(nx);
    Eigen::ArrayXd ky_v(ny);

    for (Index i=0; i < nx; ++i)
    {
      kx_v(i) = (i * 2.0 / nx - 1.0) * kx_nyquist;
    }
    for (Index i=0; i < ny; ++i)
    {
      ky_v(i) = (i * 2.0 / ny - 1.0) * ky_nyquist;
    }

    // broadcast to matrices (aka meshgrid)
    Eigen::ArrayXXd kx = Eigen::ArrayXXd::Zero(nx, ny);
    kx.colwise() += kx_v;

    Eigen::ArrayXXd ky = Eigen::ArrayXXd::Zero(nx, ny);
    ky.rowwise() += ky_v.transpose();

    Eigen::ArrayXXd kx2 = Eigen::pow(kx, 2.0);
    Eigen::ArrayXXd ky2 = Eigen::pow(ky, 2.0);
    Eigen::ArrayXXd k = Eigen::sqrt(kx2 + ky2);

    PiersonMoskowitzWaveSpectrum spectrum;

    Eigen::ArrayXXd cap_s = Eigen::ArrayXXd::Zero(nx, ny);
    spectrum.Evaluate(cap_s, k);

    for (Index i=0; i < nx; ++i)
    {
      for (Index j=0; j < ny; ++j)
      {
        double cap_s_test = spectrum.Evaluate(k(i, j));
        EXPECT_NEAR(cap_s(i, j), cap_s_test, tolerance);
      }
    }
  }
}

//////////////////////////////////////////////////
TEST(WaveSpectrum, ECKVSpectrumRegression)
{
  double k[] {
    0.,         0.06283185, 0.12566371, 0.18849556,
    0.25132741, 0.31415927, 0.37699112, 0.43982297
  };

  { // Regress against values generate from Python reference version
    double tolerance = 1.0e-15;
    double u19 = 0.0;
    ECKVWaveSpectrum spectrum(u19);

    double cap_s[] = {
      0., 0., 0., 0.,
      0., 0., 0., 0.
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }

  {
    double tolerance = 1.0e-8;
    double u10 = 5.0;
    ECKVWaveSpectrum spectrum(u10);

    double cap_s[] = {
      0.0,            2.60276926e-10, 3.85980085e-03, 4.11638703e-02,
      6.43908578e-02, 6.17616606e-02, 5.05207166e-02, 3.91055100e-02
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }

  {
    double tolerance = 1.5e-7;
    double u10 = 10.0;
    ECKVWaveSpectrum spectrum(u10);

    double cap_s[] = {
      0.0,        4.21641049, 1.95535142, 0.70164121,
      0.30549674, 0.15782445, 0.0924573,  0.05921628
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }

  {
    double tolerance = 5.0e-6;
    double u10 = 15.0;
    ECKVWaveSpectrum spectrum(u10);

    double cap_s[] = {
      0.0,         16.9746576,  2.45350994,  0.74821723,
      0.32892719,  0.17409368,  0.10300532,  0.06576126
    };

    for (Index i=0; i < 8; ++i)
    {
      double cap_s_test = spectrum.Evaluate(k[i]);
      EXPECT_NEAR(cap_s[i], cap_s_test, tolerance);
    }
  }
}

//////////////////////////////////////////////////
TEST(WaveSpectrum, ECKVSpectrumArrayXXd)
{
  { // Eigen array version
    double tolerance = 1.0e-16;

    double lx = 200.0;
    double ly = 100.0;
    Index nx = 32;
    Index ny = 16;

    double kx_nyquist = M_PI * nx / lx;
    double ky_nyquist = M_PI * ny / ly;

    // create wavenumber vectors
    Eigen::ArrayXd kx_v(nx);
    Eigen::ArrayXd ky_v(ny);

    for (Index i=0; i < nx; ++i)
    {
      kx_v(i) = (i * 2.0 / nx - 1.0) * kx_nyquist;
    }
    for (Index i=0; i < ny; ++i)
    {
      ky_v(i) = (i * 2.0 / ny - 1.0) * ky_nyquist;
    }

    // broadcast to matrices (aka meshgrid)
    Eigen::ArrayXXd kx = Eigen::ArrayXXd::Zero(nx, ny);
    kx.colwise() += kx_v;

    Eigen::ArrayXXd ky = Eigen::ArrayXXd::Zero(nx, ny);
    ky.rowwise() += ky_v.transpose();

    Eigen::ArrayXXd kx2 = Eigen::pow(kx, 2.0);
    Eigen::ArrayXXd ky2 = Eigen::pow(ky, 2.0);
    Eigen::ArrayXXd k = Eigen::sqrt(kx2 + ky2);

    ECKVWaveSpectrum spectrum;

    Eigen::ArrayXXd cap_s = Eigen::ArrayXXd::Zero(nx, ny);
    spectrum.Evaluate(cap_s, k);

    for (Index i=0; i < nx; ++i)
    {
      for (Index j=0; j < ny; ++j)
      {
        double cap_s_test = spectrum.Evaluate(k(i, j));
        EXPECT_NEAR(cap_s(i, j), cap_s_test, tolerance);
      }
    }
  }
}

//////////////////////////////////////////////////
TEST(WaveSpectrum, ECKVSpectrumFFT2ImplRegression)
{
  const double u10 = 5.0;
  const double cap_omega_c = 0.84;

  { // Eigen array version
    double tolerance = 1.0e-16;

    double lx = 200.0;
    double ly = 100.0;
    Index nx = 32;
    Index ny = 16;

    double kx_nyquist = M_PI * nx / lx;
    double ky_nyquist = M_PI * ny / ly;

    // create wavenumber vectors
    Eigen::ArrayXd kx_v(nx);
    Eigen::ArrayXd ky_v(ny);

    for (Index i=0; i < nx; ++i)
    {
      kx_v(i) = (i * 2.0 / nx - 1.0) * kx_nyquist;
    }
    for (Index i=0; i < ny; ++i)
    {
      ky_v(i) = (i * 2.0 / ny - 1.0) * ky_nyquist;
    }

    // broadcast to matrices (aka meshgrid)
    Eigen::ArrayXXd kx = Eigen::ArrayXXd::Zero(nx, ny);
    kx.colwise() += kx_v;

    Eigen::ArrayXXd ky = Eigen::ArrayXXd::Zero(nx, ny);
    ky.rowwise() += ky_v.transpose();

    Eigen::ArrayXXd kx2 = Eigen::pow(kx, 2.0);
    Eigen::ArrayXXd ky2 = Eigen::pow(ky, 2.0);
    Eigen::ArrayXXd k = Eigen::sqrt(kx2 + ky2);

    ECKVWaveSpectrum spectrum(u10, cap_omega_c);

    Eigen::ArrayXXd cap_s = Eigen::ArrayXXd::Zero(nx, ny);
    spectrum.Evaluate(cap_s, k);

    for (Index i=0; i < nx; ++i)
    {
      for (Index j=0; j < ny; ++j)
      {
        double cap_s_test =
            LinearRandomFFTWaveSimulationRef::Impl::ECKVOmniDirectionalSpectrum(
                k(i, j), u10, cap_omega_c);
        EXPECT_NEAR(cap_s(i, j), cap_s_test, tolerance);
      }
    }
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

