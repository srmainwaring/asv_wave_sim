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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <string>

#include "gz/waves/TriangulatedGrid.hh"
#include "gz/waves/Types.hh"

using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

namespace cgal
{
using gz::cgal::Point3;
using gz::cgal::Vector3;
}  // namespace cgal

using gz::waves::Index;
using gz::waves::Point3Range;
using gz::waves::TriangulatedGrid;

TEST(TriangulatedGridTest, UpdatePoints)
{
  double lx = 100.0;
  double ly = 100.0;
  Index nx = 16;
  Index ny = 16;
  Index nx_plus1 = nx + 1;
  Index ny_plus1 = ny + 1;
  auto tri_grid = TriangulatedGrid::Create(nx, ny, lx, ly);

  std::cerr << "IsValid: " << tri_grid->IsValid() << "\n";

  // get the grid points
  const auto& points = tri_grid->Points(); 

  // points to interpolate
  std::vector<cgal::Point3> queries;

  // amount to shift
  double dx = lx/nx;
  double dy = lx/ny;

  for (auto& p : points)
  {
    cgal::Point3 p1 = p + cgal::Vector3(0.5*dx, 0.5*dy, 0.0);
    queries.push_back(p1);
  }

  // interpolation output
  std::vector<double> heights(queries.size());

  // number of evaluations
  int num_runs_ = 1000;

  auto start = steady_clock::now();
  for (int i = 0; i < num_runs_; ++i)
  {
    tri_grid->Height(queries, heights);
  }
  auto end = steady_clock::now();
  std::chrono::duration<double, std::milli> duration_ms = end - start;
  std::cerr << "num_runs:         " << num_runs_ << "\n";
  std::cerr << "total time (ms):  " << duration_ms.count() << "\n";
  std::cerr << "av per run (ms):  " << duration_ms.count() / num_runs_ << "\n";
}
