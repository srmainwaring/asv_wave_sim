// Copyright (C) 2022  Rhys Mainwaring
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

#include "Ogre2RenderEngineExtension.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2RenderEngineExtensionPlugin::Ogre2RenderEngineExtensionPlugin()
{
}

//////////////////////////////////////////////////
Ogre2RenderEngineExtensionPlugin::~Ogre2RenderEngineExtensionPlugin() = default;

//////////////////////////////////////////////////
std::string Ogre2RenderEngineExtensionPlugin::Name() const
{
  return Ogre2RenderEngineExtension::Instance()->Name();
}

//////////////////////////////////////////////////
RenderEngineExtension *Ogre2RenderEngineExtensionPlugin::Extension() const
{
  return Ogre2RenderEngineExtension::Instance();
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
Ogre2RenderEngineExtension::Ogre2RenderEngineExtension()
{
}

//////////////////////////////////////////////////
Ogre2RenderEngineExtension::~Ogre2RenderEngineExtension()
{
}

//////////////////////////////////////////////////
void Ogre2RenderEngineExtension::Destroy()
{
  BaseRenderEngineExtension::Destroy();
}

//////////////////////////////////////////////////
std::string Ogre2RenderEngineExtension::Name() const
{
  return "ogre2";
}

//////////////////////////////////////////////////
bool Ogre2RenderEngineExtension::LoadImpl(
    const std::map<std::string, std::string> &_params)
{
  try
  {
    this->LoadAttempt();
    this->loaded = true;
    return true;
  }
  catch(...)
  {
    gzerr << "Failed to load render-engine extension" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
bool Ogre2RenderEngineExtension::InitImpl()
{
  try
  {
    this->InitAttempt();
    return true;
  }
  catch (...)
  {
    gzerr << "Failed to initialize render-engine extension" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void Ogre2RenderEngineExtension::LoadAttempt()
{
  gzdbg << "Attempting to load Ogre2RenderEngineExtension" << std::endl;
}

//////////////////////////////////////////////////
void Ogre2RenderEngineExtension::InitAttempt()
{
  gzdbg << "Attempting to initialise Ogre2RenderEngineExtension" << std::endl;
}

//////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::rendering::Ogre2RenderEngineExtensionPlugin,
              gz::rendering::RenderEngineExtensionPlugin)

