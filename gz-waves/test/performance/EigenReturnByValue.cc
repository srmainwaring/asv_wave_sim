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

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

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
class Generator
{
 public:
  Generator()
  {
    m_.resize(256, 256);
  }

  void Update(double time)
  {
    for (int ix=0; ix < 256; ++ix)
      for (int iy=0; iy < 256; ++iy)
          m_(ix, iy) = ((ix + iy) - time) * 1.0E-8;
  }

  void EvaluatePassByRef(double time, Eigen::Ref<Eigen::ArrayXXd> m)
  {
    Update(time);
    m = m_;
  }


  Eigen::ArrayXXd EvaluateReturnByValue(double time)
  {
    Update(time);
    return m_;
  }

  Eigen::Ref<const Eigen::ArrayXXd> EvaluateReturnByRef(double time)
  {
    Update(time);
    Eigen::Ref<const Eigen::ArrayXXd> rm(m_);
    return rm;
  }

  Eigen::ArrayXXd m_;
};

#if 1
//////////////////////////////////////////////////
TEST(EigenReturnByValueTest, Update)
{
  int num_runs = 10000;

  Generator generator;

  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (int i=0; i<num_runs; ++i)
  {
    generator.Update(sim_time);
    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
}
#endif

#if 1
//////////////////////////////////////////////////
// function                   time %
// Update                     99.6
// Eigen::Ref                 0.0
// Eigen::...stl_iterator     0.0
TEST(EigenReturnByValueTest, EvaluatePassByRef)
{
  int num_runs = 10000;

  Generator generator;

  Eigen::ArrayXXd m(256, 256);

  double sum{0.0};

  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (int i=0; i<num_runs; ++i)
  {
    generator.EvaluatePassByRef(sim_time, m);

    auto vc = m.reshaped<Eigen::ColMajor>();

    for (auto value : m.reshaped())
    {
      sum += value;
    }

    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  std::cerr << "sum:              " << sum << "\n";
}
#endif

#if 1
//////////////////////////////////////////////////
// function                   time %
// Update                     63.0
// Eigen::Array               6.2
// Eigen::...stl_iterator     3.0
TEST(EigenReturnByValueTest, EvaluateReturnByValue)
{
  int num_runs = 10000;

  Generator generator;

  double sum{0.0};

  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (int i=0; i<num_runs; ++i)
  {
    Eigen::ArrayXXd m =
        generator.EvaluateReturnByValue(sim_time);

    auto vc = m.reshaped<Eigen::ColMajor>();

    for (auto value : vc)
    {
      sum += value;
    }

    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  std::cerr << "sum:              " << sum << "\n";
}
#endif

#if 1
//////////////////////////////////////////////////
// function                   time %
// Update                     99.5
// Eigen::Array               0.0
// Eigen::...stl_iterator     3.9
TEST(EigenReturnByValueTest, EvaluateReturnByRef)
{
  int num_runs = 10000;

  Generator generator;

  double sum{0.0};

  double sim_time = 0.0;
  double sim_step = 0.001;
  auto start = steady_clock::now();
  for (int i=0; i<num_runs; ++i)
  {
    // assign to reference - no copy
    Eigen::Ref<const Eigen::ArrayXXd> rm =
        generator.EvaluateReturnByRef(sim_time);

    // assign to array - copy
    // Eigen::ArrayXXd m = rm;

    // reshape (ColMajor) - no copy
    auto vc = rm.reshaped<Eigen::ColMajor>();

    // reshape (RowMajor) - no copy, slower iteration
    // auto vr = rm.reshaped<Eigen::RowMajor>();
    for (auto value : vc)
    {
      sum += value;
    }

    sim_time += sim_step;
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs << "\n";
  std::cerr << "sum:              " << sum << "\n";
}
#endif