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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <chrono>
#include <thread>

void RunMeshToolsTests();

///////////////////////////////////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  try 
  {
    std::cout << "TestRunner:" << std::endl;
    
    RunMeshToolsTests();
  }
  catch(const gazebo::common::Exception &_e)
  {
    std::cout << _e.GetErrorStr() << std::endl;
    return -1;
  }
  catch(const std::exception &_e)
  {
    std::cout << _e.what() << std::endl;
    return -1;
  }
  catch(...)
  {
    std::cout << "Unknown Error" << std::endl;
    return -1;
  }

  return 0;
}
