
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

#ifndef GZ_SIM_SYSTEMS_DYNAMICGEOMETRY_HH_
#define GZ_SIM_SYSTEMS_DYNAMICGEOMETRY_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
// Forward declaration
struct CubeVertices;
class DynamicGeometryPrivate;

/// \brief A plugin to demonstrate using Ogre2 for dynamic geometry
class DynamicGeometry :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  /// \brief Constructor
  public: DynamicGeometry();

  /// \brief Destructor
  public: ~DynamicGeometry() override;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager &_eventMgr) final;

  /// Documentation inherited
  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Private data pointer
  private: std::unique_ptr<DynamicGeometryPrivate> dataPtr;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_DYNAMICGEOMETRY_HH_
