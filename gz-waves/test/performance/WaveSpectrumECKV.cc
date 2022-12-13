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

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "gz/waves/WaveSpectrum.hh"

namespace Eigen
{
  typedef Eigen::Array<
    double,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > ArrayXXdRowMajor;
}

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

using gz::waves::ECKVWaveSpectrum;

//////////////////////////////////////////////////
// Define fixture
class WaveSpectrumECKVTest: public ::testing::Test
{
 public:
  WaveSpectrumECKVTest()
  {
    double kx_nyquist = M_PI * nx_ / lx_;
    double ky_nyquist = M_PI * ny_ / ly_;

    Eigen::ArrayXd kx_v(nx_);
    Eigen::ArrayXd ky_v(ny_);

    for (int i=0; i < nx_; ++i)
    {
      kx_v(i) = (i * 2.0 / nx_ - 1.0) * kx_nyquist;
    }
    for (int i=0; i < ny_; ++i)
    {
      ky_v(i) = (i * 2.0 / ny_ - 1.0) * ky_nyquist;
    }

    // broadcast to matrices (aka meshgrid)
    Eigen::ArrayXXd kx = Eigen::ArrayXXd::Zero(nx_, ny_);
    kx.colwise() += kx_v;

    Eigen::ArrayXXd ky = Eigen::ArrayXXd::Zero(nx_, ny_);
    ky.rowwise() += ky_v.transpose();

    Eigen::ArrayXXd kx2 = Eigen::pow(kx, 2.0);
    Eigen::ArrayXXd ky2 = Eigen::pow(ky, 2.0);
    k_ = Eigen::sqrt(kx2 + ky2);

    // create spectrum
    spectrum_ = ECKVWaveSpectrum(u19_);
  }

  void SetUp() override
  {
    cap_s_ = Eigen::ArrayXXd::Zero(nx_, ny_);
  }

  // number of evaluations
  int num_runs_ = 10000;

  // wave number grid (nx_, ny_)
  double lx_{200.0};
  double ly_{100.0};
  int    nx_{256};
  int    ny_{128};
  Eigen::ArrayXXd k_;

  // spectrum
  double u19_{0.0};
  ECKVWaveSpectrum spectrum_;

  // workspace - reset each test
  Eigen::ArrayXXd cap_s_;
};

//////////////////////////////////////////////////
TEST_F(WaveSpectrumECKVTest, ArrayXXdDoubleLoopColMajor)
{
  // non-vector version
  std::cerr << "Eigen::ArrayXXd double loop\n";
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    for (int ikx = 0; ikx < nx_; ++ikx)
    {
      for (int iky = 0; iky < ny_; ++iky)
      {
        cap_s_(ikx, iky) = spectrum_.Evaluate(k_(ikx, iky));
      }
    }
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs_:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(WaveSpectrumECKVTest, ArrayXXdDoubleLoopRowMajor)
{
  // loop in 'wrong' order
  std::cerr << "Eigen::ArrayXXd double loop, 'wrong' order\n";
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    for (int iky = 0; iky < ny_; ++iky)
    {
      for (int ikx = 0; ikx < nx_; ++ikx)
      {
        cap_s_(ikx, iky) = spectrum_.Evaluate(k_(ikx, iky));
      }
    }
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_run_:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(WaveSpectrumECKVTest, ArrayXXdReshapedIterator)
{
  // reshaped
  std::cerr << "Eigen::ArrayXXd reshaped iterator\n";
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    Eigen::ArrayXd k_view = k_.reshaped();
    Eigen::ArrayXd cap_s_view = cap_s_.reshaped();
    for (
      auto it1 = cap_s_view.begin(), it2 = k_view.begin();
      it1 != cap_s_view.end() && it2 != k_view.end();
      ++it1, ++it2
    )
    {
      *it1 = spectrum_.Evaluate(*it2);
    }
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(WaveSpectrumECKVTest, StdVectorDoubleLoop)
{
  // using row major std::vector
  std::cerr << "std::vector double loop\n";
  std::vector<double> kv(nx_ * ny_);
  for (int iky = 0, idx = 0; iky < ny_; ++iky)
  {
    for (int ikx = 0; ikx < nx_; ++ikx, ++idx)
    {
      kv[idx] = k_(ikx, iky);
    }
  }

  auto cap_s_view = cap_s_.reshaped();
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    for (int iky = 0, idx = 0; iky < ny_; ++iky)
    {
      for (int ikx = 0; ikx < nx_; ++ikx, ++idx)
      {
        cap_s_view(idx) = spectrum_.Evaluate(kv[idx]);
      }
    }
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(WaveSpectrumECKVTest, StdVectorSingleLoop)
{
  // using row major std::vector
  std::cerr << "std::vector single loop\n";
  std::vector<double> kv(nx_ * ny_);
  for (int iky = 0, idx = 0; iky < ny_; ++iky)
  {
    for (int ikx = 0; ikx < nx_; ++ikx, ++idx)
    {
      kv[idx] = k_(ikx, iky);
    }
  }

  auto cap_s_view = cap_s_.reshaped();
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    for (int idx = 0; idx < nx_ * ny_; ++idx)
    {
      cap_s_view(idx) = spectrum_.Evaluate(kv[idx]);
    }
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}

//////////////////////////////////////////////////
TEST_F(WaveSpectrumECKVTest, ArrayXXdCWise)
{
  // Eigen cwise-array calc
  std::cerr << "Eigen cwise-array\n";
  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    spectrum_.Evaluate(cap_s_, k_);
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}
