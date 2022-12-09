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

#ifndef GZ_SIM_COLLISION_HH_
#define GZ_SIM_COLLISION_HH_

#include <memory>
#include <optional>
#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {

  // Forward declaration
  class GZ_SIM_HIDDEN CollisionPrivate;

  class GZ_SIM_VISIBLE Collision
  {
    /// \brief Destructor
    public: ~Collision();

    /// \brief Constructor
    /// \param[in] _entity Collision entity
    public: explicit Collision(sim::Entity _entity = kNullEntity);

    /// \brief Copy constructor
    /// \param[in] _collision Collision to copy.
    public: Collision(const Collision &_collision);

    /// \brief Move constructor
    /// \param[in] _collision Collision to move.
    public: Collision(Collision &&_collision) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _collision Collision component to move.
    /// \return Reference to this.
    public: Collision &operator=(Collision &&_collision) noexcept;

    /// \brief Copy assignment operator.
    /// \param[in] _collision Collision to copy.
    /// \return Reference to this.
    public: Collision &operator=(const Collision &_collision);

    /// \brief Get the entity which this Collision is related to.
    /// \return Collision entity.
    public: sim::Entity Entity() const;

    /// \brief Check whether this link correctly refers to an entity that
    /// has a components::Collision.
    /// \param[in] _ecm Entity-component manager.
    /// \return True if it's a valid link in the manager.
    public: bool Valid(const EntityComponentManager &_ecm) const;

    /// \brief Get the link's unscoped name.
    /// \param[in] _ecm Entity-component manager.
    /// \return Collision's name or nullopt if the entity does not have a
    /// components::Name component
    public: std::optional<std::string> Name(
        const EntityComponentManager &_ecm) const;

    /// \brief Get the parent link
    /// \param[in] _ecm Entity-component manager.
    /// \return Parent Model or nullopt if the entity does not have a
    /// components::ParentEntity component.
    public: std::optional<Model> ParentLink(
        const EntityComponentManager &_ecm) const;

    /// \brief Pointer to private data.
    private: std::unique_ptr<CollisionPrivate> dataPtr;
  };
}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_COLLISION_HH_
