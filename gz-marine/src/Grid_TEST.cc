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

#include "gz/marine/Grid.hh"
#include "gz/marine/Geometry.hh"
#include "gz/marine/CGALTypes.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <string>

using namespace gz;
using namespace marine;

///////////////////////////////////////////////////////////////////////////////
// Define tests

TEST(Grid, Constructor)
{
  std::array<double, 2> size = { 2, 2 };
  std::array<size_t, 2> cellCount = { 2, 2 };
  Grid grid(size, cellCount);

  double Lx = size[0];
  double Ly = size[1];
  size_t nx = cellCount[0];
  size_t ny = cellCount[1];

  // GetSize
  EXPECT_EQ(grid.GetSize()[0], Lx);
  EXPECT_EQ(grid.GetSize()[1], Ly);

  // GetCellCount
  EXPECT_EQ(grid.GetCellCount()[0], nx);
  EXPECT_EQ(grid.GetCellCount()[1], ny);

  // Vertices
  // auto mesh = grid.GetMesh(); 
  // std::cout << "Vertices " << std::endl;
  // for(auto&& vertex : mesh->vertices())
  // {
  //   std::cout << vertex << ": " << mesh->point(vertex) << std::endl;
  // }

  // Faces
  // std::cout << "Faces " << std::endl;
  // for(auto&& face : mesh->faces())
  // {
  //   Triangle tri = marine::Geometry::MakeTriangle(*mesh, face);
  //   std::cout << face << ": " << tri << std::endl;
  // }

  // VertexCount
  EXPECT_EQ(grid.GetVertexCount(), (cellCount[0]+1) * (cellCount[1]+1));

  // GetPoint
  // for (size_t i=0; i<grid.GetVertexCount(); ++i)
  // {
  //   std:: cout << i << ": " << grid.GetPoint(i) << std::endl;    
  // }

  // SetPoint
  // for (size_t i=0; i<grid.GetVertexCount(); ++i)
  // {
  //   cgal::Point3 p = grid.GetPoint(i);
  //   p += cgal::Vector3(0, 0, 10); 
  //   grid.SetPoint(i, p);
  // }
  // for(auto&& vertex : mesh->vertices())
  // {
  //   std::cout << vertex << ": " << mesh->point(vertex) << std::endl;
  // }

  // Triangles
  // for (size_t iy=0; iy<ny; ++iy)
  // {
  //   for (size_t ix=0; ix<nx; ++ix)
  //   {
  //     for (size_t k=0; k<2; ++k)
  //     {
  //       std::cout 
  //         << "[" << ix << ", " << iy << ", " << k << "]: " 
  //         << grid.GetTriangle(ix, iy, k) << std::endl;
  //     }
  //   }    
  // }
}

TEST(Grid, FindIntersectionIndex)
{
  std::array<double, 2> size = { 4, 2 };
  std::array<size_t, 2> cellCount = { 4, 2 };
  Grid grid(size, cellCount);

  double Lx = size[0];
  double Ly = size[1];
  size_t nx = cellCount[0];
  size_t ny = cellCount[1];

  // 16 cases to check...
  { // cell(0, 0)
    double x = -1.25;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(0, 0)
    double x = -1.75;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 1);
  }
  { // cell(1, 0)
    double x = -0.25;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 1);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(1, 0)
    double x = -0.75;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 1);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 1);
  }
  { // cell(2, 0)
    double x =  0.75;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 2);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(2, 0)
    double x =  0.25;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 2);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 1);
  }
  { // cell(3, 0)
    double x =  1.75;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 3);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(3, 0)
    double x =  1.25;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 3);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 1);
  }

  { // cell(0, 1)
    double x = -1.25;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(0, 1)
    double x = -1.75;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 1);
  }
  { // cell(1, 1)
    double x = -0.25;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 1);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(1, 1)
    double x = -0.75;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 1);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 1);
  }
  { // cell(2, 1)
    double x =  0.75;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 2);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(2, 1)
    double x =  0.25;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 2);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 1);
  }
  { // cell(3, 1)
    double x =  1.75;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 3);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);
  }
  { // cell(3, 1)
    double x =  1.25;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 3);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 1);
  }
}

TEST(Grid, FindIntersectionTriangle)
{
  std::array<double, 2> size = { 4, 2 };
  std::array<size_t, 2> cellCount = { 4, 2 };
  Grid grid(size, cellCount);

  double Lx = size[0];
  double Ly = size[1];
  size_t nx = cellCount[0];
  size_t ny = cellCount[1];

  std::cout << std::endl;

  { // cell(0, 0)
    double x = -1.25;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };

    // Initial guess
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 0);

    // Find intersection
    cgal::Point3 origin = cgal::Point3(x, y, 10);
    cgal::Direction3 direction = cgal::Direction3(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    isFound = GridTools::FindIntersectionTriangle(
      grid, origin, direction, index, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(x, y, 0));
  }

  { // cell(0, 0)
    double x = -1.75;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };

    // Initial guess
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 1);

    // Find intersection
    cgal::Point3 origin = cgal::Point3(x, y, 10);
    cgal::Direction3 direction = cgal::Direction3(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    isFound = GridTools::FindIntersectionTriangle(
      grid, origin, direction, index, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(x, y, 0));
  }

  { // cell(2, 1)
    double x =  0.75;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };

    // Initial guess
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 2);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);

    // Find intersection
    cgal::Point3 origin = cgal::Point3(x, y, 10);
    cgal::Direction3 direction = cgal::Direction3(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    isFound = GridTools::FindIntersectionTriangle(
      grid, origin, direction, index, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(x, y, 0));
  }
}

TEST(Grid, FindIntersectionCell)
{
  std::array<double, 2> size = { 4, 2 };
  std::array<size_t, 2> cellCount = { 4, 2 };
  Grid grid(size, cellCount);

  double Lx = size[0];
  double Ly = size[1];
  size_t nx = cellCount[0];
  size_t ny = cellCount[1];

  std::cout << std::endl;

  { // cell(0, 0)
    double x = -1.25;
    double y = -0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };

    // Initial guess
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 0);

    // Swap triangle index
    index[2] = (index[2] == 0) ? 1 : 0;
    EXPECT_EQ(index[0], 0);
    EXPECT_EQ(index[1], 0);
    EXPECT_EQ(index[2], 1);

    // Find intersection
    cgal::Point3 origin = cgal::Point3(x, y, 10);
    cgal::Direction3 direction = cgal::Direction3(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    isFound = GridTools::FindIntersectionCell(
      grid, origin, direction, index, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(x, y, 0));

    std::cout << std::endl;
  }
}

TEST(Grid, FindIntersectionGrid)
{
  std::array<double, 2> size = { 4, 2 };
  std::array<size_t, 2> cellCount = { 4, 2 };
  Grid grid(size, cellCount);

  double Lx = size[0];
  double Ly = size[1];
  size_t nx = cellCount[0];
  size_t ny = cellCount[1];

  std::cout << std::endl;

  { // cell(3, 1)
    double x =  1.75;
    double y =  0.50;
    std::array<size_t, 3> index = { 0, 0, 0 };

    // Initial guess
    bool isFound = GridTools::FindIntersectionIndex(grid, x, y, index);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(index[0], 3);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);

    // Reset index
    index = { 0, 0, 0 };
    
    // Find intersection
    cgal::Point3 origin = cgal::Point3(x, y, 10);
    cgal::Direction3 direction = cgal::Direction3(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    isFound = GridTools::FindIntersectionGrid(
      grid, origin, direction, index, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(x, y, 0));
 
    // Found index
    EXPECT_EQ(index[0], 3);
    EXPECT_EQ(index[1], 1);
    EXPECT_EQ(index[2], 0);

    std::cout << std::endl;
  }
}


///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

