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

// Copied from gz-rendering RenderEnginePlugin.hh

/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GZ_RENDERING_RENDERENGINEEXTENSIONPLUGIN_HH_
#define GZ_RENDERING_RENDERENGINEEXTENSIONPLUGIN_HH_

#include <memory>
#include <string>

#include <gz/utils/SuppressWarning.hh>
#include <gz/rendering/config.hh>
#include <gz/rendering/Export.hh>

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {

// Forward declarations
class RenderEngineExtension;
class RenderEngineExtensionPluginPrivate;

/// \brief Base plugin class for render engine extensions
class GZ_RENDERING_VISIBLE RenderEngineExtensionPlugin
{
  /// \brief Constructor
  public: RenderEngineExtensionPlugin();

  /// \brief Destructor
  public: virtual ~RenderEngineExtensionPlugin();

  /// \brief Get the name of extension
  /// \return Name of render engine extension
  public: virtual std::string Name() const = 0;

  /// \brief Get a pointer to the extension
  /// \return Render engine extension instance
  public: virtual RenderEngineExtension *Extension() const = 0;

  /// \brief Pointer to private data class
  GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  public: std::unique_ptr<RenderEngineExtensionPluginPrivate> dataPtr;
  GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_RENDERENGINEEXTENSIONPLUGIN_HH_
