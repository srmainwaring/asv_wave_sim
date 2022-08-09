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


#include "OceanScene.hh"

#include "OceanVisual.hh"
#include "BaseOceanVisual.hh"
#include "Ogre2OceanVisual.hh"

#include "OceanGeometry.hh"
#include "BaseOceanGeometry.hh"
#include "Ogre2OceanGeometry.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OceanScene::OceanScene()
{
}

//////////////////////////////////////////////////
OceanScene::~OceanScene()
{
}

//////////////////////////////////////////////////
OceanVisualPtr OceanScene::CreateOceanVisual(ScenePtr _scene)
{
  // create name and increment the object id
  std::stringstream ss;
  ss << "OceanVisual(" << objId++ << ")";
  std::string objName = ss.str();

  // create visual
  rendering::OceanVisualPtr visual =
      std::make_shared<rendering::Ogre2OceanVisual>(); 
  visual->InitObject(_scene, objId, objName);

  return visual;
}

//////////////////////////////////////////////////
OceanGeometryPtr OceanScene::CreateOceanGeometry(ScenePtr _scene)
{
  // create name and increment the object id
  std::stringstream ss;
  ss << "OceanGeometry(" << objId++ << ")";
  std::string objName = ss.str();

  // create geometry
  rendering::OceanGeometryPtr geometry =
      std::make_shared<rendering::Ogre2OceanGeometry>(); 
  geometry->InitObject(_scene, objId, objName);

  return geometry;
}
