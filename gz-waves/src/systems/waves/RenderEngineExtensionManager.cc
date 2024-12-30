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

// Copied from gz-rendering RenderEngineManager

/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <list>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/plugin/Loader.hh>
#include <gz/rendering/config.hh>
#include <gz/rendering/InstallationDirectories.hh>

#include "RenderEngineExtensionManager.hh"
#include "RenderEngineExtension.hh"
#include "RenderEngineExtensionPlugin.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

/// \brief Holds information about an extension
struct ExtensionInfo
{
  /// \brief The name of the extension's shared library, default extensions
  /// can also be specified with their regular name (ogre, optix, etc.).
  std::string name;

  /// \brief The pointer to the render extension.
  RenderEngineExtension *extension;
};

/// \brief Private implementation of the RenderEngineExtensionManager class.
class RenderEngineExtensionManagerPrivate
{
  /// \brief ExtensionMap that maps extension name to an extension pointer.
  typedef std::map<std::string, RenderEngineExtension *> ExtensionMap;

  /// \brief ExtensionMap iterator.
  typedef ExtensionMap::iterator ExtensionIter;

  /// \brief Get a pointer to the render extension from an ExtensionMap
  /// iterator.
  /// \param[in] _iter ExtensionMap iterator
  /// \param[in] _path Another search path for rendering extension plugin.
  public: RenderEngineExtension *Extension(ExtensionInfo _info,
      const std::map<std::string, std::string> &_params,
      const std::string &_path);

  /// \brief Unload the given render extension from an ExtensionMap iterator.
  /// The extension will remain registered and can be loaded again later.
  /// \param[in] _iter ExtensionMap iterator
  /// \return True if the extension is unloaded
  public: bool UnloadExtension(ExtensionIter _iter);

  /// \brief Register default extensions supplied by ign-rendering
  public: void RegisterDefaultExtensions();

  /// \brief Unregister an extension using an ExtensionMap iterator.
  /// Once an extension is unregistered, it can no longer be loaded. Use
  /// RenderEngineExtensionManagerPrivate::UnloadExtension to unload the
  /// extension without unregistering it if you wish to load the
  /// extension again
  /// \param[in] _iter ExtensionMap iterator
  public: void UnregisterExtension(ExtensionIter _iter);

  /// \brief Load a render extension plugin.
  /// \param[in] _filename Filename of plugin shared library
  /// \param[in] _path Another search path for rendering extension plugin.
  /// \return True if the plugin is loaded successfully
  public: bool LoadExtensionPlugin(const std::string &_filename,
              const std::string &_path);

  /// \brief Unload a render extension plugin.
  /// \param[in] _extensionName Name of extension associated with this plugin
  /// \return True if the plugin is unloaded successfully
  public: bool UnloadExtensionPlugin(const std::string &_extensionName);

  /// \brief Extensions that have been registered
  public: ExtensionMap extensions;

  /// \brief A map of default extension library names to their plugin names.
  public: std::map<std::string, std::string> defaultExtensions;

  /// \brief A map of loaded extension plugins to its plugin name.
  public: std::map<std::string, std::string> extensionPlugins;

  /// \brief Plugin loader for managing render extension plugin libraries.
  public: gz::plugin::Loader pluginLoader;

  /// \brief Environment variable which holds paths to look for plugins
  public: std::string pluginPathEnv = "GZ_RENDERING_PLUGIN_PATH";

  /// \brief List which holds paths to look for extension plugins.
  public: std::list<std::string> pluginPaths;

  /// \brief Mutex to protect the extensions map.
  public: std::recursive_mutex extensionsMutex;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
RenderEngineExtensionManager::RenderEngineExtensionManager() :
  dataPtr(new RenderEngineExtensionManagerPrivate)
{
  this->dataPtr->RegisterDefaultExtensions();
}

//////////////////////////////////////////////////
RenderEngineExtensionManager::~RenderEngineExtensionManager() = default;

//////////////////////////////////////////////////
unsigned int RenderEngineExtensionManager::ExtensionCount() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  return this->dataPtr->extensions.size();
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManager::HasExtension(const std::string &_name) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto iter = this->dataPtr->extensions.find(_name);

  if (iter == this->dataPtr->extensions.end())
  {
    // Check if the provided name is a name of a default extension, if so,
    // translate the name to the shared library name
    auto defaultIt = this->dataPtr->defaultExtensions.find(_name);
    if (defaultIt != this->dataPtr->defaultExtensions.end())
      iter = this->dataPtr->extensions.find(defaultIt->second);
  }

  return iter != this->dataPtr->extensions.end();
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManager::IsExtensionLoaded(
    const std::string &_name) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto iter = this->dataPtr->extensions.find(_name);

  if (iter == this->dataPtr->extensions.end())
  {
    // Check if the provided name is a name of a default extension, if so,
    // translate the name to the shared library name
    auto defaultIt = this->dataPtr->defaultExtensions.find(_name);
    if (defaultIt != this->dataPtr->defaultExtensions.end())
    {
      iter = this->dataPtr->extensions.find(defaultIt->second);
      if (iter == this->dataPtr->extensions.end())
        return false;
    }
    else
      return false;
  }

  return nullptr != iter->second;
}

//////////////////////////////////////////////////
std::vector<std::string> RenderEngineExtensionManager::LoadedExtensions() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  std::vector<std::string> extensions;
  for (auto [name, extension] :  // NOLINT(whitespace/braces)
      this->dataPtr->extensions)
  {
    std::string n = name;
    if (nullptr != extension)
    {
      // gz-rendering3 changed loaded extension names to the actual lib name.
      // For backward compatibility, return extension name if it is one of the
      // default extensions
      for (const auto &it : this->dataPtr->defaultExtensions)
      {
        if (it.second == name)
        {
          n = it.first;
          break;
        }
      }
      extensions.push_back(n);
    }
  }
  return extensions;
}

//////////////////////////////////////////////////
RenderEngineExtension *RenderEngineExtensionManager::Extension(
    const std::string &_name,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  ExtensionInfo info{_name, nullptr};
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  // check in the list of available extensions
  auto iter = this->dataPtr->extensions.find(_name);
  if (iter != this->dataPtr->extensions.end())
  {
    info.name = iter->first;
    info.extension = iter->second;
  }

  return this->dataPtr->Extension(info, _params, _path);
}

//////////////////////////////////////////////////
RenderEngineExtension *RenderEngineExtensionManager::ExtensionAt(
    unsigned int _index,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  if (_index >= this->ExtensionCount())
  {
    gzerr << "Invalid render-extension index: " << _index << std::endl;
    return nullptr;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto iter = this->dataPtr->extensions.begin();
  std::advance(iter, _index);
  return this->dataPtr->Extension({iter->first, iter->second}, _params, _path);
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManager::UnloadExtension(const std::string &_name)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  // check in the list of available extensions
  auto iter = this->dataPtr->extensions.find(_name);

  if (iter == this->dataPtr->extensions.end())
  {
    // Check if the provided name is a name of a default extension, if so,
    // translate the name to the shared library name
    auto defaultIt = this->dataPtr->defaultExtensions.find(_name);
    if (defaultIt != this->dataPtr->defaultExtensions.end())
      iter = this->dataPtr->extensions.find(defaultIt->second);

    if (iter == this->dataPtr->extensions.end())
    {
      gzerr << "No render-extension registered with name: "
          << _name << std::endl;
      return false;
    }
  }

  return this->dataPtr->UnloadExtension(iter);
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManager::UnloadExtensionAt(unsigned int _index)
{
  if (_index >= this->ExtensionCount())
  {
    gzerr << "Invalid render-extension index: " << _index << std::endl;
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto iter = this->dataPtr->extensions.begin();
  std::advance(iter, _index);
  return this->dataPtr->UnloadExtension(iter);
}

//////////////////////////////////////////////////
void RenderEngineExtensionManager::RegisterExtension(const std::string &_name,
    RenderEngineExtension *_extension)
{
  if (!_extension)
  {
    gzerr << "Render-extension cannot be null" << std::endl;
    return;
  }

  if (this->HasExtension(_name))
  {
    gzerr << "Render-extension already registered with name: "
          << _name << std::endl;

    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  this->dataPtr->extensions[_name] = _extension;
}

//////////////////////////////////////////////////
void RenderEngineExtensionManager::UnregisterExtension(const std::string &_name)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto iter = this->dataPtr->extensions.find(_name);

  if (iter != this->dataPtr->extensions.end())
  {
    this->dataPtr->UnregisterExtension(iter);
  }
}

//////////////////////////////////////////////////
void RenderEngineExtensionManager::UnregisterExtension(
    RenderEngineExtension *_extension)
{
  if (!_extension)
    return;

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto begin = this->dataPtr->extensions.begin();
  auto end = this->dataPtr->extensions.end();

  for (auto iter = begin; iter != end; ++iter)
  {
    if (iter->second == _extension)
    {
      this->dataPtr->UnregisterExtension(iter);
      return;
    }
  }
}

//////////////////////////////////////////////////
void RenderEngineExtensionManager::UnregisterExtensionAt(unsigned int _index)
{
  if (_index >= this->ExtensionCount())
  {
    gzerr << "Invalid render-extension index: " << _index << std::endl;
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->extensionsMutex);
  auto iter = this->dataPtr->extensions.begin();
  std::advance(iter, _index);
  this->dataPtr->UnregisterExtension(iter);
}

//////////////////////////////////////////////////
void RenderEngineExtensionManager::SetPluginPaths(
    const std::list<std::string> &_paths)
{
  this->dataPtr->pluginPaths = _paths;
}

//////////////////////////////////////////////////
// RenderEngineExtensionManagerPrivate
//////////////////////////////////////////////////
RenderEngineExtension *RenderEngineExtensionManagerPrivate::Extension(
    ExtensionInfo _info,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  RenderEngineExtension *extension = _info.extension;

  if (!extension)
  {
    std::string libName = _info.name;

    // Check if the provided name is a name of a default extension, if so,
    // translate the name to the shared library name
    auto defaultIt = this->defaultExtensions.find(_info.name);
    if (defaultIt != this->defaultExtensions.end())
      libName = defaultIt->second;

    // Load the extension plugin
    if (this->LoadExtensionPlugin(libName, _path))
    {
      std::lock_guard<std::recursive_mutex> lock(this->extensionsMutex);
      auto extensionIt = this->extensions.find(libName);
      if (extensionIt != this->extensions.end())
        extension = extensionIt->second;
    }
  }

  if (!extension)
    return nullptr;

  if (!extension->IsInitialized())
  {
    extension->Load(_params);
    extension->Init();
  }

  return extension;
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManagerPrivate::UnloadExtension(ExtensionIter _iter)
{
  RenderEngineExtension *extension = _iter->second;

  if (!extension)
    return false;

  extension->Destroy();

  return this->UnloadExtensionPlugin(_iter->first);
}

//////////////////////////////////////////////////
void RenderEngineExtensionManagerPrivate::RegisterDefaultExtensions()
{
  // TODO(anyone): Find a cleaner way to get the default extension library name

  // cppcheck-suppress unreadVariable
  // std::string libName = "gz-rendering-";
  std::string libName = "gz-waves1-rendering-";

  // cppcheck-suppress unusedVariable
  std::string extensionName;

  std::lock_guard<std::recursive_mutex> lock(this->extensionsMutex);
// #if HAVE_OGRE2
  extensionName = "ogre2";
  this->defaultExtensions[extensionName] = libName + extensionName;
  if (this->extensions.find(libName + extensionName) == this->extensions.end())
    this->extensions[libName + extensionName] = nullptr;
// #endif
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManagerPrivate::LoadExtensionPlugin(
    const std::string &_filename, const std::string &_path)
{
  gzmsg << "Loading plugin [" << _filename << "]" << std::endl;

  gz::common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv(this->pluginPathEnv);

  // Add default install folder.
  systemPaths.AddPluginPaths(std::string(GZ_RENDERING_PLUGIN_PATH));
  systemPaths.AddPluginPaths(gz::rendering::getEngineInstallDir());

  // Add any preset plugin paths.
  for (const auto &path : this->pluginPaths)
    systemPaths.AddPluginPaths(path);

  // Add extra search path.
  systemPaths.AddPluginPaths(_path);

  auto pathToLib = systemPaths.FindSharedLibrary(_filename);

  // Load plugin
  auto pluginNames = this->pluginLoader.LoadLib(pathToLib);
  if (pluginNames.empty())
  {
    gzerr << "Failed to load plugin [" << _filename <<
              "] : couldn't load library on path [" << pathToLib <<
              "]." << std::endl;
    return false;
  }

  auto extensionNames = pluginLoader.PluginsImplementing<
      gz::rendering::RenderEngineExtensionPlugin>();

  if (extensionNames.empty())
  {
    std::stringstream error;
    error << "Found no render extension plugins in ["
          << _filename << "], available interfaces are:"
          << std::endl;
    for (auto pluginName : pluginNames)
    {
      error << "- " << pluginName << std::endl;
    }
    gzerr << error.str();
    return false;
  }

  auto extensionName = *extensionNames.begin();
  if (extensionNames.size() > 1)
  {
    std::stringstream warn;
    warn << "Found multiple render extension plugins in ["
          << _filename << "]:"
          << std::endl;
    for (auto pluginName : extensionNames)
    {
      warn << "- " << pluginName << std::endl;
    }
    warn << "Loading [" << extensionName << "]." << std::endl;
    gzwarn << warn.str();
  }

  auto plugin = pluginLoader.Instantiate(extensionName);
  if (!plugin)
  {
    gzerr << "Failed to instantiate plugin [" << extensionName << "]"
           << std::endl;
    return false;
  }

  auto renderPlugin =
      plugin->QueryInterface<gz::rendering::RenderEngineExtensionPlugin>();

  if (!renderPlugin)
  {
    gzerr << "Failed to query interface from [" << extensionName << "]"
           << std::endl;
    return false;
  }

  // This triggers the extension to be instantiated
  {
    std::lock_guard<std::recursive_mutex> lock(this->extensionsMutex);
    this->extensions[_filename] = renderPlugin->Extension();
  }

  // store extension plugin data so plugin can be unloaded later
  this->extensionPlugins[_filename] = extensionName;

  return true;
}

//////////////////////////////////////////////////
bool RenderEngineExtensionManagerPrivate::UnloadExtensionPlugin(
    const std::string &_extensionName)
{
  auto it = this->extensionPlugins.find(_extensionName);
  if (it == this->extensionPlugins.end())
  {
    gzmsg << "Skip unloading extension plugin. [" << _extensionName << "] "
           << "not loaded from plugin." << std::endl;
    return false;
  }

  std::string pluginName = it->second;
  this->extensionPlugins.erase(it);

#ifndef _WIN32
  // Unloading the plugin on windows causes tests to crash on exit
  // see issue #45
  if (!this->pluginLoader.ForgetLibraryOfPlugin(pluginName))
  {
    gzerr << "Failed to unload plugin: " << pluginName << std::endl;
  }
#endif

  std::lock_guard<std::recursive_mutex> lock(this->extensionsMutex);
  auto extensionIt = this->extensions.find(_extensionName);
  if (extensionIt == this->extensions.end())
    return false;

  // set to null - this means extension is still registered but not loaded
  this->extensions[_extensionName] = nullptr;

  return true;
}

//////////////////////////////////////////////////
void RenderEngineExtensionManagerPrivate::UnregisterExtension(
    ExtensionIter _iter)
{
  _iter->second->Destroy();

  std::lock_guard<std::recursive_mutex> lock(this->extensionsMutex);
  this->extensions.erase(_iter);
}

}
}  // namespace rendering
}  // namespace gz
