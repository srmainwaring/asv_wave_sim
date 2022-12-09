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

#ifndef GZ_RENDERING_RENDERENGINEEXTENSIONMANAGER_HH_
#define GZ_RENDERING_RENDERENGINEEXTENSIONMANAGER_HH_

#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/SingletonT.hh>
#include <gz/utils/SuppressWarning.hh>
#include <gz/rendering/config.hh>
#include <gz/rendering/Export.hh>

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

// forward declarations.
class RenderEngineExtension;
class RenderEngineExtensionManagerPrivate;

/// \class RenderEngineExtensionManager RenderEngineExtensionManager.hh
/// gz/rendering/RenderEngineExtensionManager.hh
/// \brief Collection of render-engine extensions. This provides access to
/// all the extensions available at runtime.
/// RenderEngineExtension objects should not be access directly, but instead
/// via the RenderEngineExtensionManager to maintain a flexible
/// render-engine agnostic design.
class GZ_RENDERING_VISIBLE RenderEngineExtensionManager :
  public virtual common::SingletonT<RenderEngineExtensionManager>
{
  /// \brief Constructor
  public: RenderEngineExtensionManager();

  /// \brief Destructor
  public: ~RenderEngineExtensionManager();

  /// \brief Get the number of available extensions
  /// \return the number of available extensions
  public: unsigned int ExtensionCount() const;

  /// \brief Determine if an extension with the given name is avaiable.
  /// It also checks the list of default extensions supplied by
  /// gz-rendering.
  /// \param[in] _name Name of the desired extension
  /// \return True if the specified extension is available
  public: bool HasExtension(const std::string &_name) const;

  /// \brief Determine if an extension with the given name is already
  /// loaded.
  /// \param[in] _name Name of the desired extension
  /// \return True if the specified extension is loaded.
  public: bool IsExtensionLoaded(const std::string &_name) const;

  /// \brief Get the list of all extensions already loaded.
  /// \return Names of all loaded extensions.
  public: std::vector<std::string> LoadedExtensions() const;

  /// \brief Get the extension with the given name. If the no
  /// extension is registered under the given name, NULL will be
  /// returned.
  /// \param[in] _name Name of the desired extension
  /// \param[in] _params Parameters to be passed to the extension.
  /// \param[in] _path Another search path for extension plugin.
  /// \return The specified extension
  public: RenderEngineExtension *Extension(const std::string &_name,
              const std::map<std::string, std::string> &_params = {},
              const std::string &_path = "");

  /// \brief Get the extension at the given index. If no
  /// extension is exists at the given index, NULL will be returned.
  /// \param[in] _index Index of the desired extension
  /// \param[in] _params Parameters to be passed to the render engine.
  /// \param[in] _path Another search path for rendering engine plugin.
  /// \return The specified extension
  public: RenderEngineExtension *ExtensionAt(unsigned int _index,
              const std::map<std::string, std::string> &_params = {},
              const std::string &_path = "");

  /// \brief Unload the extension with the given name. If the no
  /// extension is registered under the given name, false will be
  /// returned.
  /// \param[in] _name Name of the desired extension
  /// \return  True if the engine is unloaded
  public: bool UnloadExtension(const std::string &_name);

  /// \brief Unload the extension at the given index. If the no
  /// extension is registered under the given name, false will be
  /// returned.
  /// \param[in] _index Index of the desired extension
  /// \return  True if the engine is unloaded
  public: bool UnloadExtensionAt(unsigned int _index);

  /// \brief Register a new extension under the given name. If the
  /// given name is already in use, the extension will not be
  /// registered.
  /// \param[in] _name Name the extension will be registered under
  /// \param[in] _engine Render-engine to be registered
  public: void RegisterExtension(const std::string &_name,
              RenderEngineExtension *_engine);

  /// \brief Unregister an extension registered under the given name.
  /// If no extension is registered under the given name no work
  /// will be done.
  /// \param[in] _name Name of the extension to unregister
  public: void UnregisterExtension(const std::string &_name);

  /// \brief Unregister the given extension. If the given extension
  /// is not currently registered, no work will be done.
  /// \param[in] _engine Render-engine to unregister
  public: void UnregisterExtension(RenderEngineExtension *_engine);

  /// \brief Unregister an extension at the given index. If the no
  /// extension is registered at the given index, no work will be done.
  /// \param[in] _index Index of the extension to unregister
  public: void UnregisterExtensionAt(unsigned int _index);

  /// \brief Set the plugin paths from which render engines can be loaded.
  /// \param[in] _paths The list of the plugin paths
  public: void SetPluginPaths(const std::list<std::string> &_paths);

  GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief private implementation details
  private: std::unique_ptr<RenderEngineExtensionManagerPrivate> dataPtr;
  GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING

  /// \brief required SingletonT friendship
  private: friend class
      gz::common::SingletonT<RenderEngineExtensionManager>;
};

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_RENDERENGINEEXTENSIONMANAGER_HH_
