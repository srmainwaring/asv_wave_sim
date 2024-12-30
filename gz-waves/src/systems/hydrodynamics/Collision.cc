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

#include "Collision.hh"

#include <string>

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

//////////////////////////////////////////////////
class gz::sim::CollisionPrivate
{
  /// \brief Id of link entity.
  public: Entity id{kNullEntity};
};

namespace gz
{
namespace sim
{

//////////////////////////////////////////////////
Collision::~Collision() = default;

//////////////////////////////////////////////////
Collision::Collision(sim::Entity _entity)
  : dataPtr(std::make_unique<CollisionPrivate>())
{
  this->dataPtr->id = _entity;
}

//////////////////////////////////////////////////
Collision::Collision(const Collision &_collision)
  : dataPtr(std::make_unique<CollisionPrivate>(*_collision.dataPtr))
{
}

//////////////////////////////////////////////////
Collision::Collision(Collision &&_collision) noexcept = default;

//////////////////////////////////////////////////
Collision &Collision::operator=(Collision &&_collision) noexcept = default;

//////////////////////////////////////////////////
Collision &Collision::operator=(const Collision &_collision)
{
  *this->dataPtr = (*_collision.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
sim::Entity Collision::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Collision::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Collision>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Collision::Name(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<Model> Collision::ParentLink(
    const EntityComponentManager &_ecm) const
{
  auto parent = _ecm.Component<components::ParentEntity>(this->dataPtr->id);

  if (!parent)
    return std::nullopt;

  return std::optional<Model>(parent->Data());
}

}  // namespace sim
}  // namespace gz
