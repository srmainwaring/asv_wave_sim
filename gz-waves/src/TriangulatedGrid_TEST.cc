// Copyright (C) 2019  Rhys Mainwaring
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
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include <gz/math/Pose3.hh>

#include "gz/waves/TriangulatedGrid.hh"
#include "gz/waves/Types.hh"

namespace cgal
{
using gz::cgal::Point3;
}  // namespace cgal

using gz::waves::Index;
using gz::waves::Point3Range;
using gz::waves::TriangulatedGrid;

//////////////////////////////////////////////////
TEST(TriangulatedGrid, Create) {
  // Create
  Index nx = 16;
  Index ny = 16;
  Index lx = 100.0;
  Index ly = 100.0;
  auto grid = TriangulatedGrid::Create(nx, ny, lx, ly);
  std::unique_ptr<TriangulatedGrid> tri_grid = std::move(grid);
  // tri_grid->DebugPrintMesh();
  // tri_grid->DebugPrintTriangulation();
  EXPECT_TRUE(tri_grid->IsValid());
  EXPECT_EQ(tri_grid->Origin(), CGAL::ORIGIN);

  // ApplyPose
  gz::math::Pose3d pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  tri_grid->ApplyPose(pose);
  // tri_grid->DebugPrintMesh();
  // tri_grid->DebugPrintTriangulation();
  EXPECT_TRUE(tri_grid->IsValid());
  EXPECT_EQ(tri_grid->Origin(), cgal::Point3(10.0, 0.0, 0.0));
}

//////////////////////////////////////////////////
TEST(TriangulatedGrid, Height) {
  // Create
  Index nx = 16;
  Index ny = 16;
  Index lx = 100.0;
  Index ly = 100.0;
  Index nx_plus1 = nx + 1;
  Index ny_plus1 = ny + 1;
  auto grid = TriangulatedGrid::Create(nx, ny, lx, ly);
  std::unique_ptr<TriangulatedGrid> source = std::move(grid);
  EXPECT_TRUE(source->IsValid());

  cgal::Point3 query(-5.0, -5.0, 0.0);
  double height;
  bool found = source->Height(query, height);
  EXPECT_TRUE(found);
  EXPECT_DOUBLE_EQ(height, 0.0);

  // Set points
  Point3Range points = source->Points();
  for (Index iy=0; iy < ny_plus1; ++iy) {
    for (Index ix=0; ix < nx_plus1; ++ix) {
      int64_t idx = iy * nx_plus1 + ix;
      double value = ix + iy;
      const cgal::Point3& p = points[idx];
      points[idx] = cgal::Point3(p.x(), p.y(), value);
    }
  }
  source->UpdatePoints(points);
  source->DebugPrintMesh();
  found = source->Height(query, height);
  EXPECT_TRUE(found);
  EXPECT_DOUBLE_EQ(height, 14.4);
}

//////////////////////////////////////////////////
TEST(TriangulatedGrid, Interpolate) {
  // Create
  Index nx = 16;
  Index ny = 16;
  Index lx = 100.0;
  Index ly = 100.0;
  Index nx_plus1 = nx + 1;
  Index ny_plus1 = ny + 1;
  auto grid1 = TriangulatedGrid::Create(nx, ny, lx, ly);
  std::unique_ptr<TriangulatedGrid> source = std::move(grid1);
  EXPECT_TRUE(source->IsValid());

  // Set points
  Point3Range points = source->Points();
  for (Index iy=0; iy < ny_plus1; ++iy) {
    for (Index ix=0; ix < nx_plus1; ++ix) {
      int64_t idx = iy * nx_plus1 + ix;
      double value = ix + iy;
      const cgal::Point3& p = points[idx];
      points[idx] = cgal::Point3(p.x(), p.y(), value);
    }
  }
  source->UpdatePoints(points);
  // source->DebugPrintMesh();
  // source->DebugPrintTriangulation();

  // Create patch
  auto grid2 = TriangulatedGrid::Create(2, 2, 10.0, 10.0);
  std::unique_ptr<TriangulatedGrid> patch = std::move(grid2);
  // patch->DebugPrintMesh();
  // patch->DebugPrintTriangulation();
  EXPECT_TRUE(patch->IsValid());

  // Interpolate
  bool found = source->Interpolate(*patch);
  patch->DebugPrintMesh();
  // patch->DebugPrintTriangulation();
  EXPECT_TRUE(found);


  // ApplyPose
  gz::math::Pose3d pose(20.0, 10.0, 0.0, 0.0, 0.0, 0.0);
  patch->ApplyPose(pose);
  // patch->DebugPrintMesh();
  // patch->DebugPrintTriangulation();
  EXPECT_TRUE(patch->IsValid());

  // Interpolate
  found = source->Interpolate(*patch);
  patch->DebugPrintMesh();
  // patch->DebugPrintTriangulation();
  EXPECT_TRUE(found);
}

//////////////////////////////////////////////////
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

