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

// GzTestRunner.cc 
//
// A test / example runner
// See gazebo/examples/stand_alone/custom_main/custom_main.cc
//
// Running:
// 
// Make sure the environment variable GAZEBO_RESOURCE_PATH includes the 
// paths of all world files you wish to load.
//
// Note GAZEBO_RESOURCE_PATH does not appear to be able to be set using 
// the gazebo_ros <export> </export> directive in package.xml.
//

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <chrono>
#include <thread>

gazebo::physics::WorldPtr LoadASVWorld();
void GzRunPhysicsTests(gazebo::physics::WorldPtr world);

///////////////////////////////////////////////////////////////////////////////

int main(int _argc, char **_argv)
{
  try
  {
    gzmsg << "GzTestRunner...starting" << std::endl;

    std::string  str = "worlds/empty.world";
    if (_argc > 1) 
    {
      str = _argv[1];
    }

    // Initialize gazebo.
    gazebo::setupServer(_argc, _argv);

    // Load a world from SDF (See gazebo/gazebo.cc gazebo::loadWorld)
    // gazebo::physics::WorldPtr world = gazebo::loadWorld(str);
    gazebo::physics::WorldPtr world = LoadASVWorld();
    if (!world) 
    {
      std::cerr << "Could not load world: " + str << std::endl;
      gazebo::shutdown();
      return -1;
    }
    gzmsg << "World:              " << world->Name() << std::endl;

    // Run tests
    GzRunPhysicsTests(world);

    // This is your custom main loop. In this example the main loop is just a
    // for loop with 2 iterations.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (size_t i = 0; i < 2; ++i)
    {
      // Run simulation for 100 steps.
      gazebo::runWorld(world, 100);
    }

    // Close everything.
    gazebo::shutdown();
    gzmsg << "GzTestRunner...shutdown." << std::endl;
  }
  catch(const gazebo::common::Exception &_e)
  {
    gzmsg << _e.GetErrorStr() << std::endl;
    return -1;
  }
  catch(const std::exception &_e)
  {
    gzmsg << _e.what() << std::endl;
    return -1;
  }
  catch(...)
  {
    gzmsg << "Unknown Error" << std::endl;
    return -1;
  }

  return 0;
}
