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

#include "gz/waves/WaveSpreadingFunction.hh"
#include "WaveSimulationFFT2Impl.hh"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

using namespace gz;
using namespace waves;

///////////////////////////////////////////////////////////////////////////////
// Define tests

TEST(WaveSpreadingFunction, Cos2sRegression)
{
  { // Regress against values generate from Python reference version
    double tolerance = 1.0e-8;
    Cos2sSpreadingFunction spreadingFn;

    double theta_mean = 0.0;

    double theta[] {
      0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531
    };

    double phi[] = {
      9.03278127e-01, 7.05050192e-01, 3.31091480e-01, 8.98336768e-02,
      1.30308992e-02, 8.82107546e-04, 2.18874516e-05, 1.24955907e-07,
      5.69467280e-11, 6.95777405e-17, 0.00000000e+00, 6.95777405e-17,
      5.69467280e-11, 1.24955907e-07, 2.18874516e-05, 8.82107546e-04,
      1.30308992e-02, 8.98336768e-02, 3.31091480e-01, 7.05050192e-01,
      9.03278127e-01
    };

    for (int i=0; i<21; ++i)
    {
      double phi_test = spreadingFn.Evaluate(theta[i], theta_mean);
      EXPECT_NEAR(phi[i], phi_test, tolerance);
    }
  }
}

TEST(WaveSpreadingFunction, Cos2sVectorised)
{
  { // Eigen vectorised version
    Cos2sSpreadingFunction spreadingFn;

    double theta_mean = 0.0;

    Eigen::VectorXd theta(21);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;

    EXPECT_EQ(theta.size(), 21);

    Eigen::VectorXd phi(21);
    spreadingFn.Evaluate(phi, theta, theta_mean);

    for (int i=0; i<21; ++i)
    {
      double phi_test = spreadingFn.Evaluate(theta(i), theta_mean);
      EXPECT_DOUBLE_EQ(phi(i), phi_test);
    }
  }
}

TEST(WaveSpreadingFunction, Cos2sVectorisedNonZeroMean)
{
  { // Eigen vectorised version - non-zero mean
    Cos2sSpreadingFunction spreadingFn;

    double theta_mean = 1.5;

    Eigen::VectorXd theta(21);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;

    EXPECT_EQ(theta.size(), 21);

    Eigen::VectorXd phi(21);
    spreadingFn.Evaluate(phi, theta, theta_mean);

    for (int i=0; i<21; ++i)
    {
      double phi_test = spreadingFn.Evaluate(theta(i), theta_mean);
      EXPECT_DOUBLE_EQ(phi(i), phi_test);
    }
  }
}

TEST(WaveSpreadingFunction, Cos2sAccessors)
{
  { // accessor returns set value
    Cos2sSpreadingFunction spreadingFn;

    // default
    EXPECT_DOUBLE_EQ(spreadingFn.Spread(), 10.0);

    // setter
    spreadingFn.SetSpread(5.5);
    EXPECT_DOUBLE_EQ(spreadingFn.Spread(), 5.5);
  }

  { // accessor recalculates coeffs
    Cos2sSpreadingFunction spreadingFn1(5.0);
    Cos2sSpreadingFunction spreadingFn2(10.0);

    spreadingFn2.SetSpread(5.0);

    double theta_mean = 1.5;

    Eigen::VectorXd theta(21);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;

    Eigen::VectorXd phi1(21);
    Eigen::VectorXd phi2(21);

    spreadingFn1.Evaluate(phi1, theta, theta_mean);
    spreadingFn2.Evaluate(phi2, theta, theta_mean);

    for (int i=0; i<21; ++i)
    {
      EXPECT_DOUBLE_EQ(phi1(i), phi2(i));
    }
  }
}

TEST(WaveSpreadingFunction, Cos2sVectorisedVirtual)
{
  { // Call virtually from base class ptr.
    auto derivedSpreadingFn = std::make_unique<Cos2sSpreadingFunction>();
    std::unique_ptr<DirectionalSpreadingFunction> spreadingFn =
        std::move(derivedSpreadingFn);

    double theta_mean = 1.5;

    Eigen::MatrixXd theta(21, 1);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;

    EXPECT_EQ(theta.rows(), 21);
    EXPECT_EQ(theta.cols(), 1);

    Eigen::MatrixXd phi(21, 1);
    spreadingFn->Evaluate(phi, theta, theta_mean);

    EXPECT_EQ(phi.rows(), 21);
    EXPECT_EQ(phi.cols(), 1);

    for (int i=0; i<21; ++i)
    {
      double phi_test = spreadingFn->Evaluate(theta(i, 0), theta_mean);
      EXPECT_DOUBLE_EQ(phi(i, 0), phi_test);
    }
  }
}

TEST(WaveSpreadingFunction, Cos2sFFT2ImplRegression)
{
  constexpr double spread = 10.0;

  // not used in calc, required for regression function interface only.
  constexpr double u10 = 5.0;
  constexpr double cap_omega_c = 0.84;

  { // Eigen vectorised version
    Cos2sSpreadingFunction spreadingFn(spread);

    double theta_mean = 0.0;

    Eigen::VectorXd theta(21);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;

    EXPECT_EQ(theta.size(), 21);

    Eigen::VectorXd phi(21);
    spreadingFn.Evaluate(phi, theta, theta_mean);

    for (int i=0; i<21; ++i)
    {
      double dtheta = theta(i) - theta_mean;
      double phi_test = WaveSimulationFFT2Impl::Cos2SSpreadingFunction(
          spread, dtheta, u10, cap_omega_c);
      EXPECT_DOUBLE_EQ(phi(i), phi_test);
    }
  }
}

TEST(WaveSpreadingFunction, WaveNumberMatrix)
{
  { // componentwise operations
    double lx = 200.0;
    double ly = 100.0;
    size_t nx = 32;
    size_t ny = 16;

    double kx_nyquist = M_PI * nx / lx;
    double ky_nyquist = M_PI * ny / ly;

    // create wavenumber vectors
    Eigen::VectorXd kx_v(nx);
    Eigen::VectorXd ky_v(ny);

    for (size_t i=0; i<nx; ++i)
    {
      kx_v(i) = (i * 2.0 / nx - 1.0) * kx_nyquist;
    }
    for (size_t i=0; i<ny; ++i)
    {
      ky_v(i) = (i * 2.0 / ny - 1.0) * ky_nyquist;
    }

    // check size
    EXPECT_EQ(kx_v.rows(), nx);
    EXPECT_EQ(kx_v.cols(), 1);
    EXPECT_EQ(ky_v.rows(), ny);
    EXPECT_EQ(ky_v.cols(), 1);

    // cross check Nyquist frequency
    double kx_f = 2.0 * M_PI / lx;
    double ky_f = 2.0 * M_PI / ly;
    EXPECT_DOUBLE_EQ(kx_nyquist, kx_f * nx / 2.0);
    EXPECT_DOUBLE_EQ(ky_nyquist, ky_f * ny / 2.0);

    // check first elements of kx_v, ky_v
    EXPECT_DOUBLE_EQ(kx_v(0), -1 * kx_nyquist);
    EXPECT_DOUBLE_EQ(ky_v(0), -1 * ky_nyquist);

    // check zero wave number
    EXPECT_DOUBLE_EQ(kx_v(nx/2), 0.0);
    EXPECT_DOUBLE_EQ(ky_v(ny/2), 0.0);

    // check last element
    EXPECT_DOUBLE_EQ(kx_v(nx - 1), kx_nyquist - kx_f);
    EXPECT_DOUBLE_EQ(ky_v(ny - 1), ky_nyquist - ky_f);

    // broadcast to matrices (aka meshgrid)
    Eigen::MatrixXd kx = Eigen::MatrixXd::Zero(nx, ny);
    kx.colwise() += kx_v;
    
    Eigen::MatrixXd ky = Eigen::MatrixXd::Zero(nx, ny);
    ky.rowwise() += ky_v.transpose();

    // check size
    EXPECT_EQ(kx.rows(), nx);
    EXPECT_EQ(kx.cols(), ny);
    EXPECT_EQ(ky.rows(), nx);
    EXPECT_EQ(ky.cols(), ny);

    // check contents
    for (size_t i=0; i<nx; ++i)
    {
      for (size_t j=0; j<ny; ++j)
      {
        EXPECT_DOUBLE_EQ(kx(i, j), kx_v(i));
        EXPECT_DOUBLE_EQ(ky(i, j), ky_v(j));
      }
    }

    Eigen::MatrixXd kx2 = Eigen::pow(kx.array(), 2.0);
    Eigen::MatrixXd ky2 = Eigen::pow(ky.array(), 2.0);
    Eigen::MatrixXd k = Eigen::sqrt(kx2.array() + ky2.array());
    Eigen::MatrixXd theta = ky.binaryExpr(
        kx, [] (double y, double x) { return std::atan2(y, x);}
    );

    // check the wave number matrix and angle matrices
    for (size_t i=0; i<nx; ++i)
    { 
      for (size_t j=0; j<ny; ++j)
      {
        double kxij = kx(i, j);
        double kx2ij = kxij * kxij;
        EXPECT_DOUBLE_EQ(kx2(i, j), kx2ij);

        double kyij = ky(i, j);
        double ky2ij = kyij * kyij;
        EXPECT_DOUBLE_EQ(ky2(i, j), ky2ij);

        double kij = std::sqrt(kx2ij + ky2ij);
        EXPECT_DOUBLE_EQ(k(i, j), kij);

        double thetaij = std::atan2(kyij, kxij);
        EXPECT_DOUBLE_EQ(theta(i, j), thetaij);
      }
    }

  }
}

TEST(WaveSpreadingFunction, ECKVRegression)
{
  { // Regress against values generate from Python reference version
    double tolerance = 1.0e-8;
    ECKVSpreadingFunction spreadingFn;

    double theta_mean = 0.0;
    double k = 1.24;

    double theta[] {
      0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531
    };

    double phi[] = {
      0.26371613, 0.24374672, 0.19146613, 0.12684376, 0.07456316, 0.05459375,
      0.07456316, 0.12684376, 0.19146613, 0.24374672, 0.26371613, 0.24374672,
      0.19146613, 0.12684376, 0.07456316, 0.05459375, 0.07456316, 0.12684376,
      0.19146613, 0.24374672, 0.26371613
    };

    for (int i=0; i<21; ++i)
    {
      double phi_test = spreadingFn.Evaluate(theta[i], theta_mean, k);
      EXPECT_NEAR(phi[i], phi_test, tolerance);
    }
  }
}

TEST(WaveSpreadingFunction, ECKVVectorisedColVector)
{
  { // Eigen vectorised version
    ECKVSpreadingFunction spreadingFn;

    double theta_mean = 0.0;
    Eigen::MatrixXd k = Eigen::MatrixXd::Zero(21, 1);
    k.array() += 1.24;

    Eigen::MatrixXd theta(21, 1);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;


    EXPECT_EQ(theta.rows(), 21);
    EXPECT_EQ(theta.cols(), 1);

    Eigen::MatrixXd phi(21, 1);
    spreadingFn.Evaluate(phi, theta, theta_mean, k);
    EXPECT_EQ(phi.rows(), 21);
    EXPECT_EQ(phi.cols(), 1);

    for (int i=0; i<21; ++i)
    {
      double phi_test = spreadingFn.Evaluate(theta(i, 0), theta_mean, k(i, 0));
      EXPECT_DOUBLE_EQ(phi(i, 0), phi_test);
    }
  }
}

TEST(WaveSpreadingFunction, ECKVVectorisedMatrix)
{
  { // Eigen vectorised version
    double lx = 200.0;
    double ly = 100.0;
    size_t nx = 32;
    size_t ny = 16;

    double kx_nyquist = M_PI * nx / lx;
    double ky_nyquist = M_PI * ny / ly;

    // create wavenumber vectors
    Eigen::VectorXd kx_v(nx);
    Eigen::VectorXd ky_v(ny);

    for (size_t i=0; i<nx; ++i)
    {
      kx_v(i) = (i * 2.0 / nx - 1.0) * kx_nyquist;
    }
    for (size_t i=0; i<ny; ++i)
    {
      ky_v(i) = (i * 2.0 / ny - 1.0) * ky_nyquist;
    }

    // broadcast to matrices (aka meshgrid)
    Eigen::MatrixXd kx = Eigen::MatrixXd::Zero(nx, ny);
    kx.colwise() += kx_v;
    
    Eigen::MatrixXd ky = Eigen::MatrixXd::Zero(nx, ny);
    ky.rowwise() += ky_v.transpose();

    Eigen::MatrixXd kx2 = Eigen::pow(kx.array(), 2.0);
    Eigen::MatrixXd ky2 = Eigen::pow(ky.array(), 2.0);
    Eigen::MatrixXd k = Eigen::sqrt(kx2.array() + ky2.array());
    Eigen::MatrixXd theta = ky.binaryExpr(
        kx, [] (double y, double x) { return std::atan2(y, x);}
    );

    ECKVSpreadingFunction spreadingFn;

    double theta_mean = 0.0;

    Eigen::MatrixXd phi(nx, ny);
    spreadingFn.Evaluate(phi, theta, theta_mean, k);

    for (int i=0; i<nx; ++i)
    {
      for (int j=0; j<ny; ++j)
      {
        double phi_test =
            spreadingFn.Evaluate(theta(i, j), theta_mean, k(i, j));
        EXPECT_DOUBLE_EQ(phi(i, j), phi_test);
      }
    }
  }
}

TEST(WaveSpreadingFunction, ECKVFFT2ImplRegression)
{
  // not used in calc, required for regression function interface only.
  constexpr double spread = 10.0;

  // coefficients
  constexpr double u10 = 5.0;
  constexpr double cap_omega_c = 0.84;

  { // Eigen vectorised version
    ECKVSpreadingFunction spreadingFn(u10, cap_omega_c);

    double theta_mean = 0.0;

    Eigen::VectorXd theta(21);
    theta << 0.0, 0.31415927, 0.62831853, 0.9424778, 1.25663706, 1.57079633,
      1.88495559, 2.19911486, 2.51327412, 2.82743339, 3.14159265, 3.45575192,
      3.76991118, 4.08407045, 4.39822972, 4.71238898, 5.02654825, 5.34070751,
      5.65486678, 5.96902604, 6.28318531;

    EXPECT_EQ(theta.size(), 21);

    Eigen::VectorXd phi(21);
    spreadingFn.Evaluate(phi, theta, theta_mean);

    for (int i=0; i<21; ++i)
    {
      double dtheta = theta(i) - theta_mean;
      double phi_test = WaveSimulationFFT2Impl::ECKVSpreadingFunction(
          spread, dtheta, u10, cap_omega_c);
      EXPECT_DOUBLE_EQ(phi(i), phi_test);
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

