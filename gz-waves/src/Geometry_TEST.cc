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

#include <array>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>

#include "gz/waves/Geometry.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/MeshTools.hh"

namespace cgal
{
using gz::cgal::AABBTree;
using gz::cgal::Direction3;
using gz::cgal::Mesh;
using gz::cgal::Point3;
using gz::cgal::Triangle;
using gz::cgal::Vector3;
}  // namespace cgal

namespace waves
{
using gz::waves::Grid;
}  // namespace waves

using gz::waves::Geometry;

//////////////////////////////////////////////////
TEST(Geometry, TriangleArea)
{
  { // Case 1 - b = 1, h = 1
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 1, 0));
    double area = Geometry::TriangleArea(t);
    EXPECT_EQ(area, 0.5);
  }

  { // Case 1 - b = sqrt(2), h = srqt(1^2 + (1/2)^2 + (1/2)^2) = sqrt(1.5)
    cgal::Triangle t(
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 1, 0),
      cgal::Point3(0, 0, 1));
    double area = Geometry::TriangleArea(t);
    EXPECT_EQ(area, 0.5*std::sqrt(3));
  }
}

//////////////////////////////////////////////////
TEST(Geometry, TriangleCentroid)
{
  { // Case 1: triangle in xy-plane
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 1, 0));
    cgal::Point3 c = Geometry::TriangleCentroid(t);
    EXPECT_EQ(c, cgal::Point3(1.0/3.0, 1.0/3.0, 0));
  }

  { // Case 2: triangle intersecting each axis at 1
    cgal::Triangle t(
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 1, 0),
      cgal::Point3(0, 0, 1));
    cgal::Point3 c = Geometry::TriangleCentroid(t);
    EXPECT_EQ(c, cgal::Point3(1.0/3.0, 1.0/3.0, 1.0/3.0));
  }
}

//////////////////////////////////////////////////
TEST(Geometry, TriangleNormal)
{
  { // xy - plane
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 1, 0));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(0, 0, 1));
  }

  { // xz - plane reversed
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(0, 1, 0),
      cgal::Point3(1, 0, 0));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(0, 0, -1));
  }

  { // xz - plane
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 0, 1));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(0, -1, 0));
  }

  { // xz - plane reversed
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(0, 0, 1),
      cgal::Point3(1, 0, 0));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(0, 1, 0));
  }

  { // xyz - plane
    cgal::Triangle t(
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 1, 0),
      cgal::Point3(0, 0, 1));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(1/sqrt(3), 1/sqrt(3), 1/sqrt(3)));
  }

  { // xyz - plane reversed
    cgal::Triangle t(
      cgal::Point3(1, 0, 0),
      cgal::Point3(0, 0, 1),
      cgal::Point3(0, 1, 0));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(-1/sqrt(3), -1/sqrt(3), -1/sqrt(3)));
  }

  { // colinear
    cgal::Triangle t(
      cgal::Point3(0, 0, 0),
      cgal::Point3(1, 0, 0),
      cgal::Point3(2, 0, 0));
    cgal::Vector3 n = Geometry::Normal(t);
    EXPECT_EQ(n, cgal::Vector3(0, 0, 0));
  }
}

//////////////////////////////////////////////////
TEST(Geometry, HorizontalIntercept)
{
  // Case - parallel to xz-plane
  {
    cgal::Point3 H(0, 0, -10);
    cgal::Point3 M(10, 0, -20);
    cgal::Point3 L(0, 0, -20);
    cgal::Point3 D = Geometry::HorizontalIntercept(H, M, L);
    EXPECT_EQ(D, cgal::Point3(0, 0, -20));
  }

  // Case - parallel to xy-plane
  {
    cgal::Point3 H(0, 0, -10);
    cgal::Point3 M(10, 0, -10);
    cgal::Point3 L(0, 10, -10);
    cgal::Point3 D = Geometry::HorizontalIntercept(H, M, L);
    EXPECT_EQ(D, L);
  }
}

//////////////////////////////////////////////////
TEST(Geometry, MidPoint)
{
  { // General vector
    cgal::Point3 v0(1, 2, 3);
    cgal::Point3 v1(2, 4, 6);
    cgal::Point3 v2 = Geometry::MidPoint(v0, v1);
    EXPECT_EQ(v2, cgal::Point3(1.5, 3.0, 4.5));
  }

  { // General xy-plane
    cgal::Point3 v0(1, 2, 0);
    cgal::Point3 v1(2, 4, 0);
    cgal::Point3 v2 = Geometry::MidPoint(v0, v1);
    EXPECT_EQ(v2, cgal::Point3(1.5, 3.0, 0));
  }

  { // General xz-plane
    cgal::Point3 v0(1, 0, 3);
    cgal::Point3 v1(2, 0, 6);
    cgal::Point3 v2 = Geometry::MidPoint(v0, v1);
    EXPECT_EQ(v2, cgal::Point3(1.5, 0, 4.5));
  }

  { // General yz-plane
    cgal::Point3 v0(0, 2, 3);
    cgal::Point3 v1(0, 4, 6);
    cgal::Point3 v2 = Geometry::MidPoint(v0, v1);
    EXPECT_EQ(v2, cgal::Point3(0, 3.0, 4.5));
  }
}

//////////////////////////////////////////////////
TEST(Geometry, LineIntersectsTriangle)
{
  { // Triangle in xy-plane. Line intersects
    cgal::Point3 p0(0, 0, 0);
    cgal::Point3 p1(1, 0, 0);
    cgal::Point3 p2(0, 1, 0);
    cgal::Mesh mesh;
    cgal::Mesh::Vertex_index v0 = mesh.add_vertex(p0);
    cgal::Mesh::Vertex_index v1 = mesh.add_vertex(p1);
    cgal::Mesh::Vertex_index v2 = mesh.add_vertex(p2);
    mesh.add_face(v0, v1, v2);

    cgal::Point3 origin(0.5, 0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    std::shared_ptr<cgal::AABBTree> tree = Geometry::MakeAABBTree(mesh);
    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(0.5, 0.5, 0));
  }

  { // Triangle in xy-plane. Line intersects, Ray in opposite direction
    cgal::Point3 p0(0, 0, 0);
    cgal::Point3 p1(1, 0, 0);
    cgal::Point3 p2(0, 1, 0);
    cgal::Mesh mesh;
    cgal::Mesh::Vertex_index v0 = mesh.add_vertex(p0);
    cgal::Mesh::Vertex_index v1 = mesh.add_vertex(p1);
    cgal::Mesh::Vertex_index v2 = mesh.add_vertex(p2);
    mesh.add_face(v0, v1, v2);

    cgal::Point3 origin(0.5, 0.5, 1);
    cgal::Direction3 direction(0, 0, -1);
    cgal::Point3 point = CGAL::ORIGIN;
    std::shared_ptr<cgal::AABBTree> tree = Geometry::MakeAABBTree(mesh);
    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(0.5, 0.5, 0));
  }

  { // Triangle in xy-plane. Line does not intersect
    cgal::Point3 p0(0, 0, 0);
    cgal::Point3 p1(1, 0, 0);
    cgal::Point3 p2(0, 1, 0);
    cgal::Mesh mesh;
    cgal::Mesh::Vertex_index v0 = mesh.add_vertex(p0);
    cgal::Mesh::Vertex_index v1 = mesh.add_vertex(p1);
    cgal::Mesh::Vertex_index v2 = mesh.add_vertex(p2);
    mesh.add_face(v0, v1, v2);

    cgal::Point3 origin(2, 2, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    std::shared_ptr<cgal::AABBTree> tree = Geometry::MakeAABBTree(mesh);
    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, false);
    EXPECT_EQ(point, cgal::Point3(0, 0, 0));
  }

  { // Triangle in xy-plane. Line co-planar, does not intersect
    cgal::Point3 p0(0, 0, 0);
    cgal::Point3 p1(1, 0, 0);
    cgal::Point3 p2(0, 1, 0);
    cgal::Mesh mesh;
    cgal::Mesh::Vertex_index v0 = mesh.add_vertex(p0);
    cgal::Mesh::Vertex_index v1 = mesh.add_vertex(p1);
    cgal::Mesh::Vertex_index v2 = mesh.add_vertex(p2);
    mesh.add_face(v0, v1, v2);

    cgal::Point3 origin(0.5, 0.5, 1);
    cgal::Direction3 direction(1, 0, 0);
    cgal::Point3 point = CGAL::ORIGIN;
    std::shared_ptr<cgal::AABBTree> tree = Geometry::MakeAABBTree(mesh);
    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, false);
    EXPECT_EQ(point, cgal::Point3(0, 0, 0));
  }
}

//////////////////////////////////////////////////
// @TODO_UPDATE - these tests are over a Mesh not a cell - update
TEST(Geometry, SearchCell)
{
  // 2x2 Grid in xy-plane.
  waves::Grid grid({ 2, 2 }, { 2, 2 });

  const cgal::Mesh& mesh = *grid.GetMesh();
  std::shared_ptr<cgal::AABBTree> tree = Geometry::MakeAABBTree(mesh);

  { // Line intersects Cell(1, 1).
    cgal::Point3 origin(0.5, 0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(0.5, 0.5, 0));
  }

  { // Line does not intersect Cell(0, 0).
    cgal::Point3 origin(0.5, 0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;
    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(0.5, 0.5, 0));
  }
}

//////////////////////////////////////////////////
TEST(Geometry, SearchGrid)
{
  // 2x2 Grid in xy-plane.
  std::string gridName("grid_2x2");
  waves::Grid grid({ 2, 2 }, { 2, 2 });

  const cgal::Mesh& mesh = *grid.GetMesh();
  std::shared_ptr<cgal::AABBTree> tree = Geometry::MakeAABBTree(mesh);

  { // Line intersects Cell(0, 0).
    cgal::Point3 origin(-0.5, -0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;

    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(-0.5, -0.5, 0));
  }

  { // Line intersects Cell(1, 0).
    cgal::Point3 origin(0.5, -0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;

    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(0.5, -0.5, 0));
  }

  { // Line intersects Cell(0, 1).
    cgal::Point3 origin(-0.5, 0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;

    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(-0.5, 0.5, 0));
  }

  { // Line intersects Cell(1, 1).
    cgal::Point3 origin(0.5, 0.5, 1);
    cgal::Direction3 direction(0, 0, 1);
    cgal::Point3 point = CGAL::ORIGIN;

    bool isFound = Geometry::SearchMesh(*tree, origin, direction, point);
    EXPECT_EQ(isFound, true);
    EXPECT_EQ(point, cgal::Point3(0.5, 0.5, 0));
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


