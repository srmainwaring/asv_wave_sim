/*
 * Copyright (C) 2022  Rhys Mainwaring
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
*/

#include "Ogre2OceanGeometry.hh"

/// \brief Private implementation
class ignition::rendering::Ogre2OceanGeometryPrivate
{
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2OceanGeometry::Ogre2OceanGeometry(ScenePtr _scene) :
  Ogre2DynamicMesh(_scene),
  dataPtr(std::make_unique<Ogre2OceanGeometryPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2OceanGeometry::~Ogre2OceanGeometry() = default;

//////////////////////////////////////////////////

