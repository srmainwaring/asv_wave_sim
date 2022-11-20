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

#include <complex>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <fftw3.h>

using Eigen::MatrixXcd;
using Eigen::VectorXcd;

//////////////////////////////////////////////////
TEST(EigenFFWT, EigenFFT1D)
{
  // Python code to generate test data
  //
  // x = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]
  // xhat = fft.ifft(x, 8, norm="forward")
  // 
  // with np.printoptions(precision=16, suppress=True):
  //     print(f"x: {x}")
  //     print(f"xhat: {xhat.real}")
  //     print(f"xhat: {xhat.imag}")

  int n = 8;

  // expected inputs and outputs
  Eigen::VectorXcd x = Eigen::VectorXcd::Zero(n);
  x.real() << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0;

  Eigen::VectorXcd xhat = Eigen::VectorXcd::Zero(n);
  xhat.real() <<   2.,  -1.7071067811865475,    1.,   -0.2928932188134524,
                   0.,  -0.2928932188134524,    1.,   -1.7071067811865475;
  xhat.imag() <<   0.,   0.7071067811865475,   -1.,    0.7071067811865475,
                   0.,  -0.7071067811865475,    1.,   -0.7071067811865475;
  
  { // using fftw_complex
    fftw_complex* in = (fftw_complex*)fftw_malloc(n * sizeof(fftw_complex));
    fftw_complex* out = (fftw_complex*)fftw_malloc(n * sizeof(fftw_complex));

    // create plan
    fftw_plan plan = fftw_plan_dft_1d(n, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

    // populate input
    for (int i=0; i<n; ++i)
    {
      in[i][0] = x(i).real();
      in[i][1] = x(i).imag();
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i<n; ++i)
    {
      // std::cerr << "[" << i << "] "
      //   << out[i][0] << " + " << out[i][1]
      //   << "\n";  
      EXPECT_DOUBLE_EQ(out[i][0], xhat(i).real());
      EXPECT_DOUBLE_EQ(out[i][1], xhat(i).imag());
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
    for (int i=0; i<n; ++i)
    {
      in[i] = x(i);
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i<n; ++i)
    {
      EXPECT_DOUBLE_EQ(out[i].real(), xhat(i).real());
      EXPECT_DOUBLE_EQ(out[i].imag(), xhat(i).imag());
    }
  }

  { // using Eigen::VectorXcd
    Eigen::VectorXcd in = Eigen::VectorXcd::Zero(n, 0.0);
    Eigen::VectorXcd out = Eigen::VectorXcd::Zero(n, 0.0);

    // create plan
    fftw_plan plan = fftw_plan_dft_1d(
      n,
      reinterpret_cast<fftw_complex*>(in.data()),
      reinterpret_cast<fftw_complex*>(out.data()),
      FFTW_BACKWARD, FFTW_ESTIMATE);

    // populate input
    for (int i=0; i<n; ++i)
    {
      in(i) = x(i);
    }

    // run fft
    fftw_execute(plan);

    // check output
    for (int i=0; i<n; ++i)
    {
      EXPECT_DOUBLE_EQ(out(i).real(), xhat(i).real());
      EXPECT_DOUBLE_EQ(out(i).imag(), xhat(i).imag());
    }
  }
}

namespace Eigen
{ 
  typedef Eigen::Matrix<
    double,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > MatrixXdRowMajor;
}

//////////////////////////////////////////////////
/// This test demonstrates changing storage order and accessing
/// the flattened data in either row major or column major format.
///
TEST(EigenFFWT, EigenStorageOrdering)
{
  {
    std::cerr << "MatrixXd" << "\n";

    Eigen::MatrixXd col_maj(2, 3);
    col_maj << 1, 2, 3,
               4, 5, 6;

    std::cerr << "col-major:" << "\n";
    std::cerr << col_maj << "\n";

    std::cerr << "in memory (col-major):" << "\n";
    for (int i = 0; i < col_maj.size(); i++)
      std::cerr << *(col_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << col_maj.reshaped().transpose() << "\n";

    Eigen::MatrixXdRowMajor row_maj = col_maj;

    std::cerr << "in memory (row-major):" << "\n";
    for (int i = 0; i < row_maj.size(); i++)
      std::cerr << *(row_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << row_maj.reshaped<Eigen::RowMajor>().transpose() << "\n";
  }

  {
    std::cerr << "Matrix2x3d" << "\n";

    Eigen::Matrix<double, 2, 3, Eigen::ColMajor> col_maj;
    col_maj << 1, 2, 3,
               4, 5, 6;

    std::cerr << "col-major:" << "\n";
    std::cerr << col_maj << "\n";

    std::cerr << "in memory (col-major):" << "\n";
    for (int i = 0; i < col_maj.size(); i++)
      std::cerr << *(col_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << col_maj.reshaped().transpose() << "\n";
    
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> row_maj = col_maj;

    std::cerr << "in memory (row-major):" << "\n";
    for (int i = 0; i < row_maj.size(); i++)
      std::cerr << *(row_maj.data() + i) << " ";
    std::cerr << "\n";
    std::cerr << row_maj.reshaped<Eigen::RowMajor>().transpose() << "\n";
  }
}


//////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

