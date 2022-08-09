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
#ifndef GZ_RENDERING_BASE_RENDERENGINEEXTENSION_HH_
#define GZ_RENDERING_BASE_RENDERENGINEEXTENSION_HH_

#include "RenderEngineExtension.hh"

#include <map>
#include <string>
#include <gz/rendering/config.hh>
#include <gz/rendering/GraphicsAPI.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Export.hh>

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    class GZ_RENDERING_VISIBLE BaseRenderEngineExtension :
        public virtual RenderEngineExtension
    {
      protected: BaseRenderEngineExtension();

      public: virtual ~BaseRenderEngineExtension();

      public: virtual bool Load(
          const std::map<std::string, std::string> &_params = {}) override;

      public: virtual bool Init() override;

      public: virtual void Destroy() override;

      // public: virtual bool Fini() = 0;

      // public: virtual bool IsLoaded() const = 0;

      public: virtual bool IsInitialized() const override;

      // public: virtual bool IsEnabled() const = 0;

      // public: virtual std::string Name() const = 0;

      // public: virtual unsigned int SceneCount() const = 0;

      // public: virtual bool HasScene(ConstScenePtr _scene) const = 0;

      // public: virtual bool HasSceneId(unsigned int _id) const = 0;

      // public: virtual bool HasSceneName(const std::string &_name) const = 0;

      // public: virtual ScenePtr SceneById(unsigned int _id) const = 0;

      // public: virtual ScenePtr SceneByName(
      //             const std::string &_name) const = 0;

      // public: virtual ScenePtr SceneByIndex(unsigned int _index) const = 0;

      // public: virtual void DestroyScene(ScenePtr _scene) = 0;

      // public: virtual void DestroySceneById(unsigned int _id) = 0;

      // public: virtual void DestroySceneByName(const std::string &_name) = 0;

      // public: virtual void DestroySceneByIndex(unsigned int _index) = 0;

      // public: virtual void DestroyScenes() = 0;

      // public: virtual ScenePtr CreateScene(const std::string &_name) = 0;

      // public: virtual ScenePtr CreateScene(unsigned int _id,
      //             const std::string &_name) = 0;

      // public: virtual rendering::GraphicsAPI GraphicsAPI() const = 0;

      // public: virtual void SetHeadless(bool _headless) = 0;

      // public: virtual bool Headless() const = 0;

      // public: virtual void AddResourcePath(const std::string &_path) = 0;

      // public: virtual RenderPassSystemPtr RenderPassSystem() const = 0;

      protected: virtual bool LoadImpl(
          const std::map<std::string, std::string> &_params) = 0;

      protected: virtual bool InitImpl() = 0;

      protected: bool loaded = false;

      protected: bool initialized = false;
    };
    }
  }
}
#endif
