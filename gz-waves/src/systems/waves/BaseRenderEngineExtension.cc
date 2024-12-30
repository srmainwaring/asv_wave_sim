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

#include "BaseRenderEngineExtension.hh"

#include <map>
#include <string>

#include <gz/common/Console.hh>

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

//////////////////////////////////////////////////
BaseRenderEngineExtension::BaseRenderEngineExtension()
{
}

//////////////////////////////////////////////////
BaseRenderEngineExtension::~BaseRenderEngineExtension()
{
}

//////////////////////////////////////////////////
bool BaseRenderEngineExtension::Load(
    const std::map<std::string, std::string> &_params)
{
  if (this->loaded)
  {
    gzwarn << "Render-engine extension has already been loaded" << std::endl;
    return true;
  }

  this->loaded = this->LoadImpl(_params);
  return this->loaded;
}

//////////////////////////////////////////////////
bool BaseRenderEngineExtension::Init()
{
  if (!this->loaded)
  {
    gzerr << "Render-engine extension must be loaded first"
        << std::endl;
    return false;
  }

  if (this->initialized)
  {
    gzwarn << "Render-engine extension has already been initialized"
        << std::endl;
    return true;
  }

  this->initialized = this->InitImpl();
  return this->initialized;
}

//////////////////////////////////////////////////
bool BaseRenderEngineExtension::IsInitialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
void BaseRenderEngineExtension::Destroy()
{
}

}
}  // namespace rendering
}  // namespace gz
