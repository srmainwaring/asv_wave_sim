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

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Physics.hh"
#include "asv_wave_sim_gazebo_plugins/Geometry.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/MeshTools.hh"
#include "asv_wave_sim_gazebo_plugins/PhysicalConstants.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"

#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace asv;

///////////////////////////////////////////////////////////////////////////////
// Define tests

TEST(Physics, CenterOfForce)
{
  { // Limits
    double fU = 10;
    double fL = 0;    
    Point3 CU(10, 20, 30);
    Point3 CL(40, 50, 60);  
    Point3 CF = Physics::CenterOfForce(fU, fL, CU, CL);   
    EXPECT_EQ(CF, CU);
  }

  { // Equal forces
    double fU = 10;
    double fL = 10;    
    Point3 CU(0, 0, 0);
    Point3 CL(5, 5, 5);    
    Point3 CF = Physics::CenterOfForce(fU, fL, CU, CL);
    EXPECT_EQ(CF, Point3(2.5, 2.5, 2.5));
  }
 
}

TEST(Physics, BuoyancyForceAtCenterOfPressure)
{
  double eps = 1E-8;

  { // H on surface
    Point3 H( 5, 0, 0);
    Point3 M(-5, 0, -2);
    Point3 L(5, 0, -2);
    Point3 center;
    Vector3 force;
    Vector3 normal = Geometry::Normal(H, M, L);
    Point3 C = Geometry::TriangleCentroid(H, M, L);
    double depthC = - C.z();    
    Physics::BuoyancyForceAtCenterOfPressure(depthC, C, H, M, L, normal, center, force);    
    EXPECT_NEAR(center[0], 1.25, eps);
    EXPECT_NEAR(center[1], 0.0, eps);
    EXPECT_NEAR(center[2], -1.5, eps);
    EXPECT_NEAR(force[0], 0, eps);
    EXPECT_NEAR(force[1], 130483.73333333334, eps);
    EXPECT_NEAR(force[2], 0, eps);
  }  

  { // H below surface
    Point3 H( 5, 0, -9);
    Point3 M(-5, 0, -11);
    Point3 L(5, 0, -11);
    Point3 center;
    Vector3 force;
    Vector3 normal = Geometry::Normal(H, M, L);
    Point3 C = Geometry::TriangleCentroid(H, M, L);
    double depthC = - C.z();
    Physics::BuoyancyForceAtCenterOfPressure(depthC, C, H, M, L, normal, center, force);    
    EXPECT_NEAR(center[0], 1.6129032258, eps);
    EXPECT_NEAR(center[1], 0.0, eps);
    EXPECT_NEAR(center[2], -10.35483871, eps);
    EXPECT_NEAR(force[0], 0, eps);
    EXPECT_NEAR(force[1], 1011248.9333333335, eps);
    EXPECT_NEAR(force[2], 0, eps);
  }

  { // M on surface
    Point3 H( 5, 0, -0);
    Point3 M(-5, 0, -0);
    Point3 L(-5, 0, -2);
    Point3 center;
    Vector3 force;
    Vector3 normal = Geometry::Normal(H, M, L);
    Point3 C = Geometry::TriangleCentroid(H, M, L);
    double depthC = - C.z();    
    Physics::BuoyancyForceAtCenterOfPressure(depthC, C, H, M, L, normal, center, force);    
    EXPECT_NEAR(center[0], -2.5, eps);
    EXPECT_NEAR(center[1],  0.0, eps);
    EXPECT_NEAR(center[2], -1.0, eps);
    EXPECT_NEAR(force[0], 0, eps);
    EXPECT_NEAR(force[1], 65241.866666666, eps);
    EXPECT_NEAR(force[2], 0, eps);
  }

  { // M below surface
    Point3 H( 5, 0, -9);
    Point3 M(-5, 0, -9);
    Point3 L(-5, 0, -11);
    Point3 center;
    Vector3 force;
    Vector3 normal = Geometry::Normal(H, M, L);
    Point3 C = Geometry::TriangleCentroid(H, M, L);
    double depthC = - C.z();    
    Physics::BuoyancyForceAtCenterOfPressure(depthC, C, H, M, L, normal, center, force);    
    EXPECT_NEAR(center[0], -1.724137931, eps);
    EXPECT_NEAR(center[1], 0.0, eps);
    EXPECT_NEAR(center[2], -9.689655172, eps);
    EXPECT_NEAR(force[0], 0, eps);
    EXPECT_NEAR(force[1], 946007.0666666666, eps);
    EXPECT_NEAR(force[2], 0, eps);
  }
}

TEST(Hydrodynamics, BuoyancyUnitBox)
{
  double eps = 1E-8;

  // Mesh: 1 x 1 x 1 box
  ignition::math::Pose3d linkPose;
  std::string linkMeshName("box_1x1x1");
  gazebo::common::MeshManager::Instance()->CreateBox(
    linkMeshName,
    ignition::math::Vector3d(1, 1, 1),
    ignition::math::Vector2d(1, 1));
  std::shared_ptr<Mesh> linkMesh = std::make_shared<Mesh>();
  MeshTools::MakeSurfaceMesh(
    *gazebo::common::MeshManager::Instance()->GetMesh(linkMeshName),
    *linkMesh);

  // Wavefield and water patch 2x2  
  std::shared_ptr<const Wavefield> wavefield(
    new  Wavefield("wavefield", { 2, 2 }, { 2, 2 }));
  std::shared_ptr<Grid> patch(
    new  Grid({ 2, 2 }, { 2, 2 }));
  std::shared_ptr<const WavefieldSampler> wavefieldSampler(
    new WavefieldSampler(wavefield, patch));

  // Hydrodynamics
  std::shared_ptr<HydrodynamicsParameters> hydroParams = std::make_shared<HydrodynamicsParameters>();
  Hydrodynamics hydrodynamics(hydroParams, linkMesh, wavefieldSampler);
  hydrodynamics.Update(wavefieldSampler, linkPose, CGAL::NULL_VECTOR, CGAL::NULL_VECTOR);
  Vector3 force = hydrodynamics.Force();

  double h = 0.5;
  double A = 1.0;
  double f = - PhysicalConstants::WaterDensity() * PhysicalConstants::Gravity() * A * h;
  EXPECT_NEAR(force[0], 0, eps);
  EXPECT_NEAR(force[1], 0, eps);
  EXPECT_NEAR(force[2], f, eps);
}

TEST(Hydrodynamics, Buoyancy10x4x2Box)
{
  double eps = 1E-8;

  // Mesh: 10 x 4 x 2 box
  ignition::math::Pose3d linkPose;
  std::string linkMeshName("box_10x4x2");
  gazebo::common::MeshManager::Instance()->CreateBox(
    linkMeshName,
    ignition::math::Vector3d(10, 4, 2),
    ignition::math::Vector2d(1, 1));
  std::shared_ptr<Mesh> linkMesh = std::make_shared<Mesh>();
  MeshTools::MakeSurfaceMesh(
    *gazebo::common::MeshManager::Instance()->GetMesh(linkMeshName),
    *linkMesh);

  // Wavefield and water patch 4x4
  std::shared_ptr<const Wavefield> wavefield(
    new  Wavefield("wavefield", { 20, 20 }, { 4, 4 }));
  std::shared_ptr<Grid> patch(
    new  Grid({ 20, 20 }, { 4, 4 }));
  std::shared_ptr<const WavefieldSampler> wavefieldSampler(
    new WavefieldSampler(wavefield, patch));
  
  // Hydrodynamics
  std::shared_ptr<HydrodynamicsParameters> hydroParams = std::make_shared<HydrodynamicsParameters>();
  Hydrodynamics hydrodynamics(hydroParams, linkMesh, wavefieldSampler);
  hydrodynamics.Update(wavefieldSampler, linkPose, CGAL::NULL_VECTOR, CGAL::NULL_VECTOR);
  Vector3 force =  hydrodynamics.Force();
  double h = 0.5 * 2.0;
  double A = 10.0 * 4.0;
  double f = - PhysicalConstants::WaterDensity() * PhysicalConstants::Gravity() * A * h;
  EXPECT_NEAR(force[0], 0, eps);
  EXPECT_NEAR(force[1], 0, eps);
  EXPECT_NEAR(force[2], f, eps);

  // if (1) 
  // {
  //   // Set Pose - 45 deg rotation about z
  //   std::cout << "Pose: rotate about Z..." << std::endl;
  //   Pose3d linkPose(0, 0, 0, 0, 0, M_PI/4.0);
  //   for (size_t i=0; i<linkMesh->GetVertexCount(); ++i)
  //   {
  //     ignition::math::Vector3d v0 = initLinkMesh->GetVertex(i);
  //     ignition::math::Vector3d v1 = linkPose.Rot().RotateVector(v0) + linkPose.Pos();
  //     linkMesh->SetVertex(i, v1);
  //   }

  //   // Hydrodynamics
  //   hydrodynamics.UpdateSubmergedTriangles();
  //   hydrodynamics.ComputeBuoyancyForce();
  //   force =  hydrodynamics.BuoyancyForce();
  //   std::cout << "test: " << force << std::endl;    
  //   std::cout << "chck: " << Vector3d(0, 0, f) << std::endl;    
  // }
  // if (1)
  // {
  //   // Set Pose - 45 deg rotation about x
  //   std::cout << "Pose: rotate about X..." << std::endl;
  //   Pose3d linkPose(0, 0, 0, M_PI/4.0, 0, 0);
  //   for (size_t i=0; i<linkMesh->GetVertexCount(); ++i)
  //   {
  //     ignition::math::Vector3d v0 = initLinkMesh->GetVertex(i);
  //     ignition::math::Vector3d v1 = linkPose.Rot().RotateVector(v0) + linkPose.Pos();
  //     linkMesh->SetVertex(i, v1);
  //   }

  //   // Hydrodynamics
  //   hydrodynamics.UpdateSubmergedTriangles();
  //   hydrodynamics.ComputeBuoyancyForce();
  //   force =  hydrodynamics.BuoyancyForce();
  //   std::cout << "test: " << force << std::endl;    
  //   std::cout << "chck: " << Vector3d(0, 0, f) << std::endl;    
  // }
  // if (1)
  // {
  //   // Set Pose - overflow example
  //   std::cout << "Pose: overflow example..." << std::endl;
  //   Pose3d linkPose(0, 0, 0.973235, -0.737731, 0, 0);
  //   for (size_t i=0; i<linkMesh->GetVertexCount(); ++i)
  //   {
  //     ignition::math::Vector3d v0 = initLinkMesh->GetVertex(i);
  //     ignition::math::Vector3d v1 = linkPose.Rot().RotateVector(v0) + linkPose.Pos();
  //     linkMesh->SetVertex(i, v1);
  //   }

  //   // Hydrodynamics
  //   hydrodynamics.UpdateSubmergedTriangles();
  //   hydrodynamics.ComputeBuoyancyForce();
  //   force =  hydrodynamics.BuoyancyForce();
  //   std::cout << "test: " << force << std::endl;    
  //   std::cout << "chck: " << Vector3d(0, 0, f) << std::endl;    

  //   // Fail case
  //   // Link:         base_link
  //   // Position:     -0 -0 0.028833
  //   // Rotation:     2.76014 -0 0
  //   // Force:        -0 0 379290
  //   // Torque:       137138 0 0
  //   // Invalid force: is not a number
  //   // Invalid center of pressure: not finite
  //   // Link:         base_link
  //   // Position:     -0 -0 0.029357
  //   // Rotation:     2.76183 -0 0
  //   // Force:        nan nan nan
  //   // Torque:       nan nan nan
  //   // Link:         base_link
  //   // Position:     -0 -0 0.02987
  //   // Rotation:     2.76353 -0 0
  //   // Force:        0 -0 378870
  //   // Torque:       135681 0 0
  //   // Link:         base_link
  //   // Position:     -0 -0 0.030392
  //   // Rotation:     2.76523 -0 0
  //   // Force:        -0 -0 378659
  //   // Torque:       134954 0 0
  // }

}

TEST(Hydrodynamics, Rotation)
{
  // Body coordinates
  Vector3 x(1, 0, 0);
  Vector3 y(0, 1, 0);
  Vector3 z(0, 0, 1);
  // {
  //   // World pose
  //   Pose3d pose(0, 0, 0, 0, 0, M_PI/2.0);

  //   // World coordinates
  //   Vector3d xw = pose.Rot().RotateVectorReverse(x);
  //   Vector3d yw = pose.Rot().RotateVectorReverse(y);
  //   Vector3d zw = pose.Rot().RotateVectorReverse(z);
  //   std::cout << "rotZ: " << std::endl;
  //   std::cout << "xw:   " << xw << std::endl;
  //   std::cout << "yw:   " << yw << std::endl;
  //   std::cout << "zw:   " << zw << std::endl;
  // }
}

TEST(Hydrodynamics, FrameTransforms)
{
  // Body frame coordinates
  Vector3 x(1, 0, 0);
  Vector3 y(0, 1, 0);
  Vector3 z(0, 0, 1);
  // std::cout << "x:    " << x << std::endl;
  // std::cout << "y:    " << y << std::endl;
  // std::cout << "z:    " << z << std::endl;

  // World pose
  // Pose3d pose(10, 0, 0, 0, 0, M_PI/2.0);

  // {
  //   // World coordinates
  //   Vector3 xw = pose.Rot().RotateVector(x) + pose.Pos(); 
  //   std::cout << "xw:   " << xw << std::endl;
  //   std::cout << "xw:   " << Vector3(10, 1, 0) << std::endl;
  // }
  // {
  //   // World coordinates
  //   Matrix4d T(pose);
  //   Vector3d xw = T * x;
  //   std::cout << "xw:   " << xw << std::endl;
  //   std::cout << "xw:   " << Vector3d(10, 1, 0) << std::endl;
  // }

}

///////////////////////////////////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

