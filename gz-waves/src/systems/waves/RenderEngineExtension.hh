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

// Copied from gz-rendering RenderEngine

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
#ifndef GZ_RENDERING_RENDERENGINEEXTENSION_HH_
#define GZ_RENDERING_RENDERENGINEEXTENSION_HH_

#include <map>
#include <string>

#include <gz/rendering/config.hh>
#include <gz/rendering/GraphicsAPI.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Export.hh>

#include "SceneNodeFactory.hh"

namespace gz
{
namespace rendering
{
inline namespace GZ_RENDERING_VERSION_NAMESPACE {
//
/// \brief An abstract interface to a concrete render-engine extension.
class GZ_RENDERING_VISIBLE RenderEngineExtension
{
  /// \brief Destructor
  public: virtual ~RenderEngineExtension();

  /// \brief Load any necessary resources to set up the extension. This
  /// should called before any other function.
  /// \param[in] _params Parameters to be passed to the underlying
  /// rendering engine extension.
  /// \return True if the extension was successfully loaded
  public: virtual bool Load(
      const std::map<std::string, std::string> &_params = {}) = 0;

  /// \brief Initialize the extension. This should be called immediately
  /// after a successful call to Load.
  /// \return True if the extension was successfully initialized
  public: virtual bool Init() = 0;

  /// \brief Destroys all scenes created by extension and releases all
  /// loaded resources. This should be called when the given extension
  /// will no longer be used during runtime.
  /// \return True if the extension was successfully destroyed
  public: virtual void Destroy() = 0;

  /// \brief Determines if the extension has been initialized.
  /// \return True if the extension is initialized
  public: virtual bool IsInitialized() const = 0;

  /// \brief Get name of the extension.
  /// \return The extension name
  public: virtual std::string Name() const = 0;

  /// \brief Get the SceneNodeFactory.
  public: virtual SceneNodeFactoryPtr SceneNodeFactory() const = 0;
};

}
}  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_RENDERENGINEEXTENSION_HH_
