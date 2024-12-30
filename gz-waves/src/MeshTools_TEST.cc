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

#include <CGAL/Timer.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include "gz/waves/MeshTools.hh"
#include "gz/waves/Convert.hh"
#include "gz/waves/Geometry.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"
#include "gz/waves/CGALTypes.hh"

typedef CGAL::Timer Timer;

namespace cgal
{
using gz::cgal::Mesh;
using gz::cgal::Triangle;
using gz::cgal::Vector3;
}  // namespace cgal

using gz::waves::Geometry;
using gz::waves::Grid;
using gz::waves::MeshTools;
using gz::waves::ToGz;

//////////////////////////////////////////////////
void TestFillArraysUnitBox()
{
  std::cout << "TestFillArraysUnitBox..." << std::endl;

  // Mesh: 1 x 1 x 1 box
  std::string meshName("box_1x1x1");
  gz::common::MeshManager::Instance()->CreateBox(
    meshName,
    gz::math::Vector3d(1, 1, 1),
    gz::math::Vector2d(1, 1));

  cgal::Mesh mesh;
  std::vector<float> vertices;
  std::vector<int> indices;
  MeshTools::FillArrays(
    *gz::common::MeshManager::Instance()->MeshByName(meshName),
    vertices, indices);

  // std::cout << "Vertices..." << std::endl;
  // for (auto&& v : vertices)
  //   std::cout << v << std::endl;

  // std::cout << "Indices..." << std::endl;
  // for (auto&& i : indices)
  //   std::cout << i << std::endl;

  // Vertices: 6 sides, 4 points per side, 3 coordinates per point
  std::cout << "test: " << vertices.size() << std::endl;
  std::cout << "chck: " << 6*4*3 << std::endl;

  // Indices: 6 sides, 2 faces per side, 3 vertices per face
  std::cout << "test: " << indices.size() << std::endl;
  std::cout << "chck: " << 6*2*3 << std::endl;
}

void TestMakeSurfaceMeshUnitBox()
{
  std::cout << "TestMakeSurfaceMeshUnitBox..." << std::endl;

  // Mesh: 1 x 1 x 1 box
  std::string meshName("box_1x1x1");
  gz::common::MeshManager::Instance()->CreateBox(
    meshName,
    gz::math::Vector3d(1, 1, 1),
    gz::math::Vector2d(1, 1));

  cgal::Mesh mesh;
  MeshTools::MakeSurfaceMesh(
    *gz::common::MeshManager::Instance()->MeshByName(meshName),
    mesh);

  // std::cout << "Vertices " << std::endl;
  // for(auto&& vertex : mesh.vertices())
  // {
  //   std::cout << vertex << ": " << mesh.point(vertex) << std::endl;
  // }

  // std::cout << "Faces " << std::endl;
  // for(auto&& face : mesh.faces())
  // {
  //   cgal::Triangle tri = Geometry::MakeTriangle(mesh, face);
  //   std::cout << face << ": " << tri << std::endl;
  // }
}

//////////////////////////////////////////////////
#if 0  // DEPRECATED FEATURE
void TestExportWaveMesh()
{
  std::cout << "TestExportWaveMesh..." << std::endl;

  // Wave parameters
  std::shared_ptr<WaveParameters> waveParams =
      std::make_shared<WaveParameters>();
  waveParams->SetNumber(1);
  waveParams->SetAmplitude(2.0);
  waveParams->SetPeriod(10.0);
  waveParams->SetDirection(gz::math::Vector2d(1.0, 1.0));

  // Wavefield
  Wavefield wavefield;
  wavefield.SetParameters(waveParams);
  wavefield.Update(0);

  // Get Mesh
  const auto& mesh = *wavefield.GetMesh();

  // Create GzMesh
  Timer t;
  t.start();

  std::string name("wavefield");
  std::shared_ptr<gz::common::Mesh> gzMesh(new gz::common::Mesh());
  gzMesh->SetName(name);

  std::unique_ptr<gz::common::SubMesh> gzSubMesh(new gz::common::SubMesh());
  int64_t iv = 0;
  for (auto&& face : mesh.faces())
  {
    cgal::Triangle tri  = Geometry::MakeTriangle(mesh, face);
    cgal::Vector3 normal = Geometry::Normal(tri);

    gz::math::Vector3d gzP0(ToGz(tri[0]));
    gz::math::Vector3d gzP1(ToGz(tri[1]));
    gz::math::Vector3d gzP2(ToGz(tri[2]));
    gz::math::Vector3d gzNormal(ToGZ(normal));

    gzSubMesh->AddVertex(gzP0);
    gzSubMesh->AddVertex(gzP1);
    gzSubMesh->AddVertex(gzP2);
    gzSubMesh->AddNormal(gzNormal);
    gzSubMesh->AddNormal(gzNormal);
    gzSubMesh->AddNormal(gzNormal);

    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);

    // @TODO - calculate texture coordinates
  }

  gzMesh->AddSubMesh(*gzSubMesh);

  t.stop();
  std::cout << "MakeGzMesh: " << t.time() << " sec" << std::endl;

  auto& meshManager = *gz::common::MeshManager::Instance();

  meshManager.Export(
    gzMesh.get(),
    "/Users/rhys/Code/ros/asv_ws/tmp/wavefield",
    "dae");
}
#endif

//////////////////////////////////////////////////
void TestExportGridMesh()
{
  std::cout << "TestExportGridMesh..." << std::endl;

  // Wavefield
  Grid grid({500, 500}, {100, 100});

  // Get Mesh
  const auto& mesh = *grid.GetMesh();

  // Create GzMesh
  Timer t;
  t.start();

  std::string name("grid_500m_x_500m_5m_x_5m");
  std::shared_ptr<gz::common::Mesh> gzMesh(new gz::common::Mesh());
  gzMesh->SetName(name);

  std::unique_ptr<gz::common::SubMesh> gzSubMesh(new gz::common::SubMesh());
  int64_t iv = 0;
  for (auto&& face : mesh.faces())
  {
    cgal::Triangle tri  = Geometry::MakeTriangle(mesh, face);
    cgal::Vector3 normal = Geometry::Normal(tri);

    gz::math::Vector3d gzP0(ToGz(tri[0]));
    gz::math::Vector3d gzP1(ToGz(tri[1]));
    gz::math::Vector3d gzP2(ToGz(tri[2]));
    gz::math::Vector3d gzNormal(ToGz(normal));

    gzSubMesh->AddVertex(gzP0);
    gzSubMesh->AddVertex(gzP1);
    gzSubMesh->AddVertex(gzP2);
    gzSubMesh->AddNormal(gzNormal);
    gzSubMesh->AddNormal(gzNormal);
    gzSubMesh->AddNormal(gzNormal);

    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);

    // @TODO - calculate texture coordinates
  }

  gzMesh->AddSubMesh(*gzSubMesh);

  t.stop();
  std::cout << "MakeGzMesh: " << t.time() << " sec" << std::endl;

  auto& meshManager = *gz::common::MeshManager::Instance();

  meshManager.Export(
    gzMesh.get(),
    std::string("/Users/rhys/Code/ros/asv_ws/tmp/").append(name),
    "dae");
}

//////////////////////////////////////////////////
void RunMeshToolsTests()
{
  TestFillArraysUnitBox();
  TestMakeSurfaceMeshUnitBox();
  // TestExportWaveMesh();
  // TestExportGridMesh();
}
