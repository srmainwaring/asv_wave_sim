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

#include <map>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>

#include "SceneNodeFactory.hh"
#include "Ogre2SceneNodeFactory.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

//////////////////////////////////////////////////
class GZ_RENDERING_OGRE2_HIDDEN Ogre2RenderEngineExtensionPrivate
{
  public: SceneNodeFactoryPtr sceneNodeFactory;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
Ogre2RenderEngineExtensionPlugin::Ogre2RenderEngineExtensionPlugin()
{
}

//////////////////////////////////////////////////
Ogre2RenderEngineExtensionPlugin::~Ogre2RenderEngineExtensionPlugin()
    = default;

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
Ogre2RenderEngineExtension::Ogre2RenderEngineExtension() :
    dataPtr(std::make_unique<Ogre2RenderEngineExtensionPrivate>())
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
    const std::map<std::string, std::string> &/*_params*/)
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

  std::string engineName("ogre2");

  // Check the render engine is available and loaded
  if (!rendering::hasEngine(engineName))
  {
    gzerr << "Failed to load render-engine extension ["
        << this->Name() << "] as render-engine [ "
        << engineName << "] is not available.\n";
  }

  if (!rendering::isEngineLoaded(engineName))
  {
    gzerr << "Failed to load render-engine extension ["
        << this->Name() << "] as render-engine [ "
        << engineName << "] is not loaded.\n";
  }

  // Check which graphics API is being used
  auto engine = rendering::engine("ogre2");
  auto graphicsAPI = engine->GraphicsAPI();
  gzdbg << "Using graphicsAPI: "
      << GraphicsAPIUtils::Str(graphicsAPI) << "\n";

  // create the scene node factory
  this->dataPtr->sceneNodeFactory = std::make_shared<Ogre2SceneNodeFactory>();
}

//////////////////////////////////////////////////
SceneNodeFactoryPtr Ogre2RenderEngineExtension::SceneNodeFactory() const
{
  return this->dataPtr->sceneNodeFactory;
}

}
}  // namespace rendering
}  // namespace gz

//////////////////////////////////////////////////
// Register this plugin
GZ_ADD_PLUGIN(gz::rendering::Ogre2RenderEngineExtensionPlugin,
              gz::rendering::RenderEngineExtensionPlugin)

