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

#include <fftw3.h>

#include <complex>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace Eigen
{
  typedef Eigen::Array<
    double,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > ArrayXXdRowMajor;
}  // namespace Eigen

//////////////////////////////////////////////////
TEST(EigenFFWT, DFT_C2C_1D)
{
  // Python code to generate test data
  //
  // x = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]
  // xhat = fft.fft(x, 8, norm="forward")
  // xx = fft.ifft(xhat, 8, norm="forward")
  //
  // with np.printoptions(precision=16, suppress=True):
  //     print(f"x:\n{x}")
  //     print(f"xhat:\n{xhat.real}\n{xhat.imag}")
  //     print(f"xx:\n{xx.real}\n{xx.imag}")
  //
  int n = 8;

  // expected inputs and outputs
  Eigen::ArrayXcd x = Eigen::ArrayXcd::Zero(n);
  x.real() << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0;

  Eigen::ArrayXcd xhat = Eigen::ArrayXcd::Zero(n);
  xhat.real() <<  0.25,   -0.2133883476483184,  0.125,  -0.0366116523516816,
                  0.,     -0.0366116523516816,  0.125,  -0.2133883476483184;
  xhat.imag() <<  0.,     -0.0883883476483184,  0.125,  -0.0883883476483184,
                  0.,      0.0883883476483184, -0.125,   0.0883883476483184;

  { // using fftw_complex
    fftw_complex* in  =
        reinterpret_cast<fftw_complex*>(fftw_malloc(n * sizeof(fftw_complex)));
    fftw_complex* out =
        reinterpret_cast<fftw_complex*>(fftw_malloc(n * sizeof(fftw_complex)));

    // create plan
    fftw_plan plan = fftw_plan_dft_1d(n, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

    // populate input
    for (int i=0; i < n; ++i)
    {
      in[i][0] = xhat(i).real();
      in[i][1] = xhat(i).imag();
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i < n; ++i)
    {
      // std::cerr << "[" << i << "] "
      //   << out[i][0] << " + " << out[i][1]
      //   << "\n";
      EXPECT_NEAR(out[i][0], x(i).real(), 1.0E-15);
      EXPECT_NEAR(out[i][1], 0.0, 1.0E-15);
    }

    // cleanup
    fftw_destroy_plan(plan);
    fftw_free(out);
    fftw_free(in);
  }

  { // using std::vector<std::complex>
    std::vector<std::complex<double>> in(n, 0.0);
    std::vector<std::complex<double>> out(n, 0.0);

    // create plan
    // https://stackoverflow.com/questions/4214400/problem-casting-stl-complexdouble-to-fftw-complex
    fftw_plan plan = fftw_plan_dft_1d(
      n,
      reinterpret_cast<fftw_complex*>(in.data()),
      reinterpret_cast<fftw_complex*>(out.data()),
      FFTW_BACKWARD, FFTW_ESTIMATE);

    // populate input
    for (int i=0; i < n; ++i)
    {
      in[i] = xhat(i);
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i < n; ++i)
    {
      EXPECT_NEAR(out[i].real(), x(i).real(), 1.0E-15);
      EXPECT_NEAR(out[i].imag(), 0.0, 1.0E-15);
    }
  }

  { // Eigen::ArrayXcd
    Eigen::ArrayXcd in = Eigen::ArrayXcd::Zero(n);
    Eigen::ArrayXcd out = Eigen::ArrayXcd::Zero(n);

    // create plan
    fftw_plan plan = fftw_plan_dft_1d(
      n,
      reinterpret_cast<fftw_complex*>(in.data()),
      reinterpret_cast<fftw_complex*>(out.data()),
      FFTW_BACKWARD, FFTW_ESTIMATE);

    // populate input
    for (int i=0; i < n; ++i)
    {
      in(i) = xhat(i);
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i < n; ++i)
    {
      EXPECT_NEAR(out(i).real(), x(i).real(), 1.0E-15);
      EXPECT_NEAR(out(i).imag(), 0.0, 1.0E-15);
    }
  }
}

//////////////////////////////////////////////////
TEST(EigenFFWT, DFT_C2R_1D)
{
  // Python code to generate test data
  int n = 8;

  // expected inputs and outputs
  Eigen::ArrayXcd x = Eigen::ArrayXcd::Zero(n);
  x.real() << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0;

  Eigen::ArrayXcd xhat = Eigen::ArrayXcd::Zero(n/2+1);
  xhat.real() <<  0.25,   -0.2133883476483184,  0.125,  -0.0366116523516816,
                  0.;
  xhat.imag() <<  0.,     -0.0883883476483184,  0.125,  -0.0883883476483184,
                  0.;

  {
    std::vector<std::complex<double>> in(n/2+1, 0.0);
    std::vector<double> out(n, 0.0);

    // create plan
    fftw_plan plan = fftw_plan_dft_c2r_1d(
        n,
        reinterpret_cast<fftw_complex*>(in.data()),
        reinterpret_cast<double*>(out.data()),
        FFTW_ESTIMATE);

    // populate input
    for (int i=0; i < n/2+1; ++i)
    {
      in[i] = xhat(i);
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i < n; ++i)
    {
      EXPECT_NEAR(out[i], x(i).real(), 1.0E-15);
    }

    // cleanup
    fftw_destroy_plan(plan);
  }
}

//////////////////////////////////////////////////
/// This test demonstrates changing storage order and accessing
/// the flattened data in either row major or column major format.
///
#if 0
TEST(EigenFFWT, EigenStorageOrdering)
{
  {
    std::cerr << "ArrayXXd" << "\n";

    Eigen::ArrayXXd col_maj(2, 3);
    col_maj << 1, 2, 3,
               4, 5, 6;

    std::cerr << "col-major:" << "\n";
    std::cerr << col_maj << "\n";

    std::cerr << "in memory (col-major):" << "\n";
    for (int i = 0; i < col_maj.size(); i++)
      std::cerr << *(col_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << col_maj.reshaped().transpose() << "\n";

    Eigen::ArrayXXdRowMajor row_maj = col_maj;

    std::cerr << "in memory (row-major):" << "\n";
    for (int i = 0; i < row_maj.size(); i++)
      std::cerr << *(row_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << row_maj.reshaped<Eigen::RowMajor>().transpose() << "\n";
  }

  {
    std::cerr << "ArrayX2x3d" << "\n";

    Eigen::Array<double, 2, 3, Eigen::ColMajor> col_maj;
    col_maj << 1, 2, 3,
               4, 5, 6;

    std::cerr << "col-major:" << "\n";
    std::cerr << col_maj << "\n";

    std::cerr << "in memory (col-major):" << "\n";
    for (int i = 0; i < col_maj.size(); i++)
      std::cerr << *(col_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << col_maj.reshaped().transpose() << "\n";

    Eigen::Array<double, 2, 3, Eigen::RowMajor> row_maj = col_maj;

    std::cerr << "in memory (row-major):" << "\n";
    for (int i = 0; i < row_maj.size(); i++)
      std::cerr << *(row_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << row_maj.reshaped<Eigen::RowMajor>().transpose() << "\n";
  }
}
#endif

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

