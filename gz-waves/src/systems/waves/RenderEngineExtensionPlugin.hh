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
    //
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
  }
}
#endif
