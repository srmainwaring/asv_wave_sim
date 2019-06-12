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

#include "asv_wave_sim_gazebo_plugins/TriangulatedGrid.hh"

#include <gtest/gtest.h>

#include <ignition/math/Pose3.hh>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <string>

///////////////////////////////////////////////////////////////////////////////
// Define tests

using namespace asv;

TEST(TriangulatedGrid, Create) {
  // Create
  int n = 2;
  int length = 100.0;
  std::unique_ptr<TriangulatedGrid> tri_grid 
    = std::move(TriangulatedGrid::Create(n, length));
  // tri_grid->DebugPrintMesh();
  // tri_grid->DebugPrintTriangulation();
  EXPECT_TRUE(tri_grid->IsValid());
  EXPECT_EQ(tri_grid->Origin(), CGAL::ORIGIN);

  // ApplyPose
  ignition::math::Pose3d pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  tri_grid->ApplyPose(pose);
  // tri_grid->DebugPrintMesh();
  // tri_grid->DebugPrintTriangulation();
  EXPECT_TRUE(tri_grid->IsValid());
  EXPECT_EQ(tri_grid->Origin(), Point3(10.0, 0.0, 0.0));
}

TEST(TriangulatedGrid, Height) {
  // Create
  int n = 16;
  int length = 100.0;
  int nplus1 = n + 1;
  std::unique_ptr<TriangulatedGrid> source 
    = std::move(TriangulatedGrid::Create(n, length));
  EXPECT_TRUE(source->IsValid());

  Point3 query(-5.0, -5.0, 0.0);
  double height;
  bool found = source->Height(query, height);
  EXPECT_TRUE(found);
  EXPECT_DOUBLE_EQ(height, 0.0);

  // Set points
  Point3Range points = source->Points();
  for (int iy=0; iy<nplus1; ++iy) {
    for (int ix=0; ix<nplus1; ++ix) {
      int64_t idx = iy * nplus1 + ix;
      double value = ix + iy;
      const Point3& p = points[idx];
      points[idx] = Point3(p.x(), p.y(), value);
    }
  }
  source->UpdatePoints(points);
  source->DebugPrintMesh();
  found = source->Height(query, height);
  EXPECT_TRUE(found);
  EXPECT_DOUBLE_EQ(height, 14.4);
}

TEST(TriangulatedGrid, Interpolate) {
  // Create
  int n = 16;
  int length = 100.0;
  int nplus1 = n + 1;
  std::unique_ptr<TriangulatedGrid> source 
    = std::move(TriangulatedGrid::Create(n, length));
  EXPECT_TRUE(source->IsValid());

  // Set points
  Point3Range points = source->Points();
  for (int iy=0; iy<nplus1; ++iy) {
    for (int ix=0; ix<nplus1; ++ix) {
      int64_t idx = iy * nplus1 + ix;
      double value = ix + iy;
      const Point3& p = points[idx];
      points[idx] = Point3(p.x(), p.y(), value);
    }
  }
  source->UpdatePoints(points);
  // source->DebugPrintMesh();
  // source->DebugPrintTriangulation();

  // Create patch
  std::unique_ptr<TriangulatedGrid> patch 
    = std::move(TriangulatedGrid::Create(2, 10.0));
  // patch->DebugPrintMesh();
  // patch->DebugPrintTriangulation();
  EXPECT_TRUE(patch->IsValid());

  // Interpolate
  bool found = source->Interpolate(*patch);
  patch->DebugPrintMesh();
  // patch->DebugPrintTriangulation();
  EXPECT_TRUE(found);


  // ApplyPose
  ignition::math::Pose3d pose(20.0, 10.0, 0.0, 0.0, 0.0, 0.0);
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



///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

