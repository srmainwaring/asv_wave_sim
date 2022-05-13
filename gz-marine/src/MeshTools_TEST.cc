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

#include "gz/marine/MeshTools.hh"
#include "gz/marine/Convert.hh"
#include "gz/marine/Geometry.hh"
#include "gz/marine/Grid.hh"
#include "gz/marine/Wavefield.hh"
#include "gz/marine/WaveParameters.hh"
#include "gz/marine/CGALTypes.hh"

#include <CGAL/Timer.h>

#include <gz/common.hh>
#include <gz/common/Util.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include <iostream>
#include <string>

using namespace ignition;
using namespace marine;

typedef CGAL::Timer Timer;

///////////////////////////////////////////////////////////////////////////////
// Define tests

void TestFillArraysUnitBox()
{
  std::cout << "TestFillArraysUnitBox..." << std::endl;

  // Mesh: 1 x 1 x 1 box
  std::string meshName("box_1x1x1");
  common::MeshManager::Instance()->CreateBox(
    meshName,
    math::Vector3d(1, 1, 1),
    math::Vector2d(1, 1));
  
  cgal::Mesh mesh;
  std::vector<float> vertices;
  std::vector<int> indices;
  MeshTools::FillArrays(
    *common::MeshManager::Instance()->MeshByName(meshName),
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
  common::MeshManager::Instance()->CreateBox(
    meshName,
    math::Vector3d(1, 1, 1),
    math::Vector2d(1, 1));
  
  cgal::Mesh mesh;
  MeshTools::MakeSurfaceMesh(
    *common::MeshManager::Instance()->MeshByName(meshName),
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

#if 0 // DEPRECATED FEATURE
void TestExportWaveMesh()
{
  std::cout << "TestExportWaveMesh..." << std::endl;

  // Wave parameters
  std::shared_ptr<WaveParameters> waveParams = std::make_shared<WaveParameters>();
  waveParams->SetNumber(1);
  waveParams->SetAmplitude(2.0);
  waveParams->SetPeriod(10.0);
  waveParams->SetDirection(math::Vector2d(1.0, 1.0));

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
  std::shared_ptr<common::Mesh> gzMesh(new common::Mesh());
  gzMesh->SetName(name);

  std::unique_ptr<common::SubMesh> gzSubMesh(new common::SubMesh());  
  unsigned long iv = 0;
  for(auto&& face : mesh.faces())
  {
    cgal::Triangle tri  = Geometry::MakeTriangle(mesh, face);
    cgal::Vector3 normal = Geometry::Normal(tri);

    math::Vector3d ignP0(ToIgn(tri[0]));
    math::Vector3d ignP1(ToIgn(tri[1]));
    math::Vector3d ignP2(ToIgn(tri[2]));
    math::Vector3d ignNormal(ToIgn(normal));

    gzSubMesh->AddVertex(ignP0);
    gzSubMesh->AddVertex(ignP1);
    gzSubMesh->AddVertex(ignP2);
    gzSubMesh->AddNormal(ignNormal);
    gzSubMesh->AddNormal(ignNormal);
    gzSubMesh->AddNormal(ignNormal);

    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);

    // @TODO - calculate texture coordinates
  }

  gzMesh->AddSubMesh(*gzSubMesh);
  
  t.stop();
  std::cout << "MakeGzMesh: " << t.time() << " sec" << std::endl;

  auto& meshManager = *common::MeshManager::Instance();

  meshManager.Export(
    gzMesh.get(),
    "/Users/rhys/Code/ros/asv_ws/tmp/wavefield",
    "dae");

}
#endif

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
  std::shared_ptr<common::Mesh> gzMesh(new common::Mesh());
  gzMesh->SetName(name);

  std::unique_ptr<common::SubMesh> gzSubMesh(new common::SubMesh());  
  unsigned long iv = 0;
  for(auto&& face : mesh.faces())
  {
    cgal::Triangle tri  = Geometry::MakeTriangle(mesh, face);
    cgal::Vector3 normal = Geometry::Normal(tri);

    math::Vector3d ignP0(ToIgn(tri[0]));
    math::Vector3d ignP1(ToIgn(tri[1]));
    math::Vector3d ignP2(ToIgn(tri[2]));
    math::Vector3d ignNormal(ToIgn(normal));

    gzSubMesh->AddVertex(ignP0);
    gzSubMesh->AddVertex(ignP1);
    gzSubMesh->AddVertex(ignP2);
    gzSubMesh->AddNormal(ignNormal);
    gzSubMesh->AddNormal(ignNormal);
    gzSubMesh->AddNormal(ignNormal);

    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);
    gzSubMesh->AddIndex(iv++);

    // @TODO - calculate texture coordinates
  }

  gzMesh->AddSubMesh(*gzSubMesh);
  
  t.stop();
  std::cout << "MakeGzMesh: " << t.time() << " sec" << std::endl;

  auto& meshManager = *common::MeshManager::Instance();

  meshManager.Export(
    gzMesh.get(),
    std::string("/Users/rhys/Code/ros/asv_ws/tmp/").append(name),
    "dae");

}

///////////////////////////////////////////////////////////////////////////////
// Run tests
void RunMeshToolsTests()
{
  TestFillArraysUnitBox();
  TestMakeSurfaceMeshUnitBox();
  // TestExportWaveMesh();
  // TestExportGridMesh();
}

