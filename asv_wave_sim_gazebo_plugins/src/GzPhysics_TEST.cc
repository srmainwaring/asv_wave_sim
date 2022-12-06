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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Triangle3.hh>
#include <ignition/math/Vector3.hh>

#include <iostream>
#include <string>
#include <chrono>
#include <thread>

using namespace ignition;
using namespace math;
using namespace gazebo;
using namespace physics;

///////////////////////////////////////////////////////////////////////////////
// Utiltiies

sdf::SDFPtr CreateASVWorldSDF()
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
  <<    "<world name='asv_world'>"
  <<      "<model name='asv_box'>"
  <<        "<link name='base_link'>"
  <<          "<visual name='base_visual'>"
  <<            "<geometry>"
  <<              "<box>"
  <<                "<size>10 4 2</size>"
  <<              "</box>"
  <<            "</geometry>"
  <<          "</visual>"
  <<          "<collision name='base_collision'>"
  <<            "<geometry>"
  <<              "<box>"
  <<                "<size>10 4 2</size>"
  <<              "</box>"
  <<            "</geometry>"
  <<          "</collision>"
  <<          "<inertial>"
  <<            "<mass>20000</mass>"
  <<            "<inertia>"
  <<              "<ixx>33333</ixx>"
  <<              "<ixy>0.0</ixy>"
  <<              "<ixz>0.0</ixz>"
  <<              "<iyy>173333</iyy>"
  <<              "<iyz>0.0</iyz>"
  <<              "<izz>193333</izz>"
  <<            "</inertia>"
  <<          "</inertial>"
  <<        "</link>"
  <<      "</model>"
  <<    "</world>"
  <<  "</sdf>";
  sdf::SDFPtr sdf(new sdf::SDF);
  sdf->SetFromString(sdfStr.str());
  return sdf;
}

sdf::SDFPtr CreateASVBoxSDF()
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version='" << SDF_VERSION << "'>"
    <<    "<model name='asv_box'>"
    <<      "<link name='base_link'>"
    <<        "<visual name='base_visual'>"
    <<          "<geometry>"
    <<            "<box>"
    <<              "<size>10 4 2</size>"
    <<            "</box>"
    <<          "</geometry>"
    <<        "</visual>"
    <<        "<collision name='base_collision'>"
    <<          "<geometry>"
    <<            "<box>"
    <<              "<size>10 4 2</size>"
    <<            "</box>"
    <<          "</geometry>"
    <<        "</collision>"
    <<        "<inertial>"
    <<          "<mass>20000</mass>"
    <<          "<inertia>"
    <<            "<ixx>33333</ixx>"
    <<            "<ixy>0.0</ixy>"
    <<            "<ixz>0.0</ixz>"
    <<            "<iyy>173333</iyy>"
    <<            "<iyz>0.0</iyz>"
    <<            "<izz>193333</izz>"
    <<          "</inertia>"
    <<        "</inertial>"
    <<      "</link>"
    <<    "</model>"
    <<  "</sdf>";
  sdf::SDFPtr sdf(new sdf::SDF);
  sdf->SetFromString(sdfStr.str());
  return sdf;
}

gazebo::physics::WorldPtr LoadASVWorld()
{
  sdf::SDFPtr sdfWorld = CreateASVWorldSDF();
  gazebo::physics::WorldPtr world = gazebo::physics::create_world();
  gazebo::physics::load_world(world, sdfWorld->Root()->GetElement("world"));
  gazebo::physics::init_world(world);
  return world;
}

///////////////////////////////////////////////////////////////////////////////
// Define tests

void TestLinkProperties(gazebo::physics::WorldPtr _world)
{
  std::cout << "TestLinkProperties..." << std::endl;

  // List Models
  for (auto model : _world->Models())
  {
    std::cout << "Model:              " << model->GetName() << std::endl;
  }

  // Model
  std::string modelName("asv_box");
  gazebo::physics::ModelPtr model = _world->ModelByName(modelName);

  // Links
  for (auto link : model->GetLinks())
  {
    std::cout << "Link:               " << link->GetName() << std::endl;
  }

  // Pose
  std::string linkName("base_link");
  gazebo::physics::LinkPtr link = model->GetLink(linkName);
  ignition::math::Pose3d linkPose = link->WorldPose();
  std::cout << "Link:               " << link->GetName() << std::endl;
  std::cout << "WorldPos:           " << linkPose.Pos() << std::endl;
  std::cout << "WorldLinearVel:     " << link->WorldLinearVel() << std::endl;
  std::cout << "WorldAngularVel:    " << link->WorldAngularVel() << std::endl;

  // Update Pose
  linkPose.Pos() = ignition::math::Vector3d(0, 0, 1.0);
  link->SetWorldPose(linkPose);
  linkPose = link->WorldPose();
  std::cout << "Link:               " << link->GetName() << std::endl;
  std::cout << "WorldPos:           " << linkPose.Pos() << std::endl;
  std::cout << "WorldLinearVel:     " << link->WorldLinearVel() << std::endl;
  std::cout << "WorldAngularVel:    " << link->WorldAngularVel() << std::endl;

  // Mock up the box object... 
  // Mesh: 10 x 4 x 2 box
  std::string linkMeshName(modelName + "::" + linkName);
  gazebo::common::MeshManager::Instance()->CreateBox(
    linkMeshName,
    ignition::math::Vector3d(10, 4, 2),
    ignition::math::Vector2d(1, 1));

}

///////////////////////////////////////////////////////////////////////////////
// Run tests
void GzRunPhysicsTests(gazebo::physics::WorldPtr _world)
{
  TestLinkProperties(_world);

}
