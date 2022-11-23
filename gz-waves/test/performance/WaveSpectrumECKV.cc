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

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include "gz/waves/WaveSpectrum.hh"

using Eigen::MatrixXd;

namespace Eigen
{ 
  typedef Eigen::Matrix<
    double,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > MatrixXdRowMajor;
}

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

using namespace gz;
using namespace waves;

TEST(WaveSpectrumPerf, ECKV)
{
  double lx = 200.0;
  double ly = 100.0;
  int    nx = 256;
  int    ny = 126;

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

  // spectrum
  double u19 = 0.0;
  ECKVWaveSpectrum spectrum(u19);

  int num_runs = 1000;

#if 1
  { // non-vector version
    std::cerr << "Eigen::MatrixXd double loop\n";
    Eigen::MatrixXd cap_s(nx, ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      for (int ikx = 0; ikx < nx; ++ikx)
      {
        for (int iky = 0; iky < ny; ++iky)
        {
          cap_s(ikx, iky) = spectrum.Evaluate(k(ikx, iky));
        }
      }
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  { // loop in 'wrong' order
    std::cerr << "Eigen::MatrixXd double loop, 'wrong' order\n";
    Eigen::MatrixXd cap_s(nx, ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      for (int iky = 0; iky < ny; ++iky)
      {
        for (int ikx = 0; ikx < nx; ++ikx)
        {
          cap_s(ikx, iky) = spectrum.Evaluate(k(ikx, iky));
        }
      }
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  { // reshaped
    std::cerr << "Eigen::MatrixXd reshaped iterator\n";
    Eigen::MatrixXd cap_s(nx, ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      Eigen::VectorXd k_view = k.reshaped();
      Eigen::VectorXd cap_s_view = cap_s.reshaped();
      for (
        auto it1 = cap_s_view.begin(), it2 = k_view.begin();
        it1 != cap_s_view.end() && it2 != k_view.end();
        ++it1, ++it2
      )
      {
        *it1 = spectrum.Evaluate(*it2);
      }
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  { // using row major std::vector
    std::cerr << "std::vector double loop\n";
    std::vector<double> kv(nx * ny);
    for (int iky = 0, idx = 0; iky < ny; ++iky)
    {
      for (int ikx = 0; ikx < nx; ++ikx, ++idx)
      {
        kv[idx] = k(ikx, iky);
      }
    }

    std::vector<double> cap_s(nx * ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      for (int iky = 0, idx = 0; iky < ny; ++iky)
      {
        for (int ikx = 0; ikx < nx; ++ikx, ++idx)
        {
          cap_s[idx] = spectrum.Evaluate(kv[idx]);
        }
      }
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  { // using row major std::vector
    std::cerr << "std::vector single loop\n";
    std::vector<double> kv(nx * ny);
    for (int iky = 0, idx = 0; iky < ny; ++iky)
    {
      for (int ikx = 0; ikx < nx; ++ikx, ++idx)
      {
        kv[idx] = k(ikx, iky);
      }
    }

    std::vector<double> cap_s(nx * ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      for (int idx = 0; idx < nx * ny; ++idx)
      {
        cap_s[idx] = spectrum.Evaluate(kv[idx]);
      }
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  { // using row major std::vector iterators
    std::cerr << "std::vector iterators\n";
    std::vector<double> kv(nx * ny);
    for (int iky = 0, idx = 0; iky < ny; ++iky)
    {
      for (int ikx = 0; ikx < nx; ++ikx, ++idx)
      {
        kv[idx] = k(ikx, iky);
      }
    }

    std::vector<double> cap_s(nx * ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      for (
        auto it1 = cap_s.begin(), it2 = kv.begin();
        it1 != cap_s.end() && it2 != kv.end();
        ++it1, ++it2
      )
      {
        *it1 = spectrum.Evaluate(*it2);
      }
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif

#if 1
  { // Eigen cwise-array calc
    std::cerr << "Eigen cwise-array\n";
    Eigen::MatrixXd cap_s(nx, ny);
    auto start = steady_clock::now();
    for (int i = 0; i < num_runs; ++i)
    {
      spectrum.Evaluate(cap_s, k);
    }
    auto end = steady_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    std::cerr << "num_runs:         " << num_runs << "\n";
    std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
    std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  }
#endif
}
