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

#include "BaseRenderEngineExtension.hh"

#include <gz/common/Console.hh>

using namespace gz;
using namespace rendering;

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
// bool BaseRenderEngineExtension::Fini()
// {
//   this->Destroy();
//   return true;
// }

//////////////////////////////////////////////////
// bool BaseRenderEngineExtension::IsLoaded() const
// {
//   return this->loaded;
// }

//////////////////////////////////////////////////
bool BaseRenderEngineExtension::IsInitialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
// bool BaseRenderEngineExtension::IsEnabled() const
// {
//   return this->initialized;
// }

//////////////////////////////////////////////////
// unsigned int BaseRenderEngineExtension::SceneCount() const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->Size();
//   return 0u;
// }

//////////////////////////////////////////////////
// bool BaseRenderEngineExtension::HasScene(ConstScenePtr _scene) const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->Contains(_scene);
//   return false;
// }

//////////////////////////////////////////////////
// bool BaseRenderEngineExtension::HasSceneId(unsigned int _id) const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->ContainsId(_id);
//   return false;
// }

//////////////////////////////////////////////////
// bool BaseRenderEngineExtension::HasSceneName(const std::string &_name) const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->ContainsName(_name);
//   return false;
// }

//////////////////////////////////////////////////
// ScenePtr BaseRenderEngineExtension::SceneById(unsigned int _id) const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->GetById(_id);
//   return ScenePtr();
// }

//////////////////////////////////////////////////
// ScenePtr BaseRenderEngineExtension::SceneByName(const std::string &_name) const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->GetByName(_name);
//   return ScenePtr();
// }

//////////////////////////////////////////////////
// ScenePtr BaseRenderEngineExtension::SceneByIndex(unsigned int _index) const
// {
//   auto scenes = this->Scenes();
//   if (scenes)
//     return scenes->GetByIndex(_index);
//   return ScenePtr();
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::DestroyScene(ScenePtr _scene)
// {
//   auto scenes = this->Scenes();
//   if (!scenes)
//     return;
//   scenes->Destroy(_scene);
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::DestroySceneById(unsigned int _id)
// {
//   auto scenes = this->Scenes();
//   if (!scenes)
//     return;
//   scenes->DestroyById(_id);
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::DestroySceneByName(const std::string &_name)
// {
//   auto scenes = this->Scenes();
//   if (!scenes)
//     return;
//   scenes->DestroyByName(_name);
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::DestroySceneByIndex(unsigned int _index)
// {
//   auto scenes = this->Scenes();
//   if (!scenes)
//     return;
//   scenes->DestroyByIndex(_index);
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::DestroyScenes()
// {
//   auto scenes = this->Scenes();
//   if (!scenes)
//     return;
//   scenes->DestroyAll();
// }

//////////////////////////////////////////////////
// ScenePtr BaseRenderEngineExtension::CreateScene(const std::string &_name)
// {
//   unsigned int sceneId = this->NextSceneId();
//   return this->CreateScene(sceneId, _name);
// }

//////////////////////////////////////////////////
// ScenePtr BaseRenderEngineExtension::CreateScene(unsigned int _id,
//     const std::string &_name)
// {
//   if (!this->IsInitialized())
//   {
//     gzerr << "Render-engine has not been initialized" << std::endl;
//     return nullptr;
//   }

//   if (this->HasSceneId(_id))
//   {
//     gzerr << "Scene already exists with id: " << _id << std::endl;
//     return nullptr;
//   }

//   if (this->HasSceneName(_name))
//   {
//     gzerr << "Scene already exists with id: " << _id << std::endl;
//     return nullptr;
//   }

//   ScenePtr scene = this->CreateSceneImpl(_id, _name);
//   this->PrepareScene(scene);
//   return scene;
// }

//////////////////////////////////////////////////
void BaseRenderEngineExtension::Destroy()
{
  // this->DestroyScenes();
  // this->loaded = false;
  // this->initialized = false;
}

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::AddResourcePath(const std::string &_path)
// {
//   this->resourcePaths.push_back(_path);
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::SetHeadless(bool _headless)
// {
//   this->isHeadless = _headless;
// }

//////////////////////////////////////////////////
// bool BaseRenderEngineExtension::Headless() const
// {
//   return this->isHeadless;
// }

//////////////////////////////////////////////////
// void BaseRenderEngineExtension::PrepareScene(ScenePtr _scene)
// {
//   if (_scene)
//   {
//     _scene->Load();
//     _scene->Init();
//   }
// }

//////////////////////////////////////////////////
// unsigned int BaseRenderEngineExtension::NextSceneId()
// {
//   return this->nextSceneId--;
// }

//////////////////////////////////////////////////
// RenderPassSystemPtr BaseRenderEngineExtension::RenderPassSystem() const
// {
//   if (!this->renderPassSystem)
//   {
//     gzerr << "Render pass not supported by the requested render engine"
//         << std::endl;
//     return RenderPassSystemPtr();
//   }
//   return this->renderPassSystem;
// }
