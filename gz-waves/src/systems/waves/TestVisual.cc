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


// Code for retrieving shader params from gz-sim/src/systems/shader_param

/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "TestVisual.hh"

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

#include <sdf/Element.hh>

#include <memory>

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::TestVisualPrivate
{
};

/////////////////////////////////////////////////
TestVisual::TestVisual()
    : System()
    , dataPtr(std::make_unique<TestVisualPrivate>())
{
}

/////////////////////////////////////////////////
TestVisual::~TestVisual()
{
}

/////////////////////////////////////////////////
void TestVisual::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  GZ_PROFILE("TestVisual::Configure");
}

//////////////////////////////////////////////////
void TestVisual::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &)
{
  GZ_PROFILE("TestVisual::PreUpdate");
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(TestVisual,
              gz::sim::System,
              TestVisual::ISystemConfigure,
              TestVisual::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(TestVisual,
  "gz::sim::systems::TestVisual")
