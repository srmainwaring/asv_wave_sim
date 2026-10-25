// Copyright (C) 2026  Kwon Yeseol
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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMeshWithTangents.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>
#include "Ogre2SceneNodeFactory.hh"

int main(int argc, char **argv)
{
  const std::string kind = argc > 1 ? argv[1] : "visual";
  const std::string mode = argc > 2 ? argv[2] : "shared";
  auto *engine = gz::rendering::Ogre2RenderEngine::Instance();
  if (!engine || !engine->Load() || !engine->Init())
    return 10;
  auto scene = engine->CreateScene("ocean_lifetime_test");
  auto factory = std::make_shared<gz::rendering::Ogre2SceneNodeFactory>();
  auto sub = std::make_unique<gz::common::SubMeshWithTangents>();
  sub->AddVertex(0, 0, 0);
  sub->AddVertex(1, 0, 0);
  sub->AddVertex(0, 1, 0);
  for (unsigned int i = 0; i < 3; ++i)
  {
    sub->AddNormal(0, 0, 1);
    sub->AddTangent(1, 0, 0);
    sub->AddTexCoord(0, 0);
    sub->AddIndex(i);
  }
  auto mesh = std::make_shared<gz::common::Mesh>();
  mesh->AddSubMesh(std::move(sub));
  auto shared = scene->CreateMaterial("shared_ocean_material");
  shared->SetAmbient(0, 0.05, 0.8, 0.92);
  shared->SetDiffuse(0, 0.1, 0.7, 0.92);
  shared->SetSpecular(0.7, 0.7, 0.7, 0.92);
  shared->SetRoughness(0.3);
  shared->SetMetalness(0.1);

  if (kind == "scene")
  {
    // Simulate GUI teardown without the server's RenderTeardown event.
    auto primary = scene->CreateVisual();
    auto box = scene->CreateBox();
    primary->AddGeometry(box);
    box->SetMaterial(shared, true);
    scene->RootVisual()->AddChild(primary);
    auto original = box->Material();
    auto oceanMaterial = mode == "clone" ? original->Clone() : original;
    if (oceanMaterial->Ambient() != original->Ambient() ||
        oceanMaterial->Diffuse() != original->Diffuse() ||
        oceanMaterial->Specular() != original->Specular() ||
        oceanMaterial->Roughness() != original->Roughness() ||
        oceanMaterial->Metalness() != original->Metalness())
      return 30;
    auto ocean = factory->CreateOceanVisual(scene);
    ocean->LoadMesh(mesh);
    ocean->SetMaterial(oceanMaterial, false);
    scene->RootVisual()->AddChild(ocean);
    scene->DestroyVisual(primary);
    if (scene->MaterialRegistered(original->Name()))
      return 31;
    if (mode == "clone" && !scene->MaterialRegistered(oceanMaterial->Name()))
      return 32;
    std::cout << "primary removed; ocean material " << mode << std::endl;
    engine->DestroyScene(scene);
    ocean.reset();
    oceanMaterial.reset();
    original.reset();
    primary.reset();
    box.reset();
    shared.reset();
    scene.reset();
  }
  else if (kind == "visual")
  {
    auto a = factory->CreateOceanVisual(scene);
    auto b = factory->CreateOceanVisual(scene);
    a->LoadMesh(mesh);
    b->LoadMesh(mesh);
    a->SetMaterial(shared, mode == "unique");
    b->SetMaterial(shared, false);
    const auto name = a->Material()->Name();
    scene->RootVisual()->AddChild(a);
    scene->RootVisual()->AddChild(b);
    scene->RootVisual()->RemoveChild(a);
    a->Destroy();
    if (!scene->MaterialRegistered(shared->Name()))
      return 20;
    if (mode == "unique" && scene->MaterialRegistered(name))
      return 21;
    a->Destroy();
    scene->RootVisual()->RemoveChild(b);
    b->Destroy();
    b->Destroy();
    if (!scene->MaterialRegistered(shared->Name()))
      return 22;
    a.reset();
    b.reset();
    scene->DestroyMaterial(shared);
    shared.reset();
    engine->DestroyScene(scene);
    scene.reset();
  }
  else
  {
    auto first = scene->CreateVisual(), second = scene->CreateVisual();
    auto a = factory->CreateOceanGeometry(scene);
    auto b = factory->CreateOceanGeometry(scene);
    a->LoadMesh(mesh);
    b->LoadMesh(mesh);
    a->SetMaterial(shared, mode == "unique");
    b->SetMaterial(shared, false);
    const auto name = a->Material()->Name();
    first->AddGeometry(a);
    second->AddGeometry(b);
    scene->RootVisual()->AddChild(first);
    scene->RootVisual()->AddChild(second);
    scene->DestroyVisual(first);
    if (!scene->MaterialRegistered(shared->Name()))
      return 20;
    if (mode == "unique" && scene->MaterialRegistered(name))
      return 21;
    a->Destroy();
    scene->DestroyVisual(second);
    b->Destroy();
    if (!scene->MaterialRegistered(shared->Name()))
      return 22;
    a.reset();
    b.reset();
    first.reset();
    second.reset();
    scene->DestroyMaterial(shared);
    shared.reset();
    engine->DestroyScene(scene);
    scene.reset();
  }
  std::cout << "PASS " << kind << " " << mode << std::endl;
  return 0;
}
