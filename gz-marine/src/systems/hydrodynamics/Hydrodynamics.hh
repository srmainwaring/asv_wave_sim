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

#ifndef IGNITION_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_
#define IGNITION_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class HydrodynamicsPrivate;

  /// \brief A plugin to manage buoyancy and hydrodynamic force calculations
  ///
  /// # Usage
  /// 
  /// Add the SDF for the plugin to the <model> element of your model. 
  /// 
  /// /code
  /// <plugin name="ignition::gazebo::systems::Hydrodynamics"
  ///         filename="ignition-marine1-hydrodynamics-system">
  ///
  ///   <!-- Hydrodynamics -->
  ///   <hydrodynamics>
  ///     <damping_on>1</damping_on>
  ///     <viscous_drag_on>1</viscous_drag_on>
  ///     <pressure_drag_on>1</pressure_drag_on>
  ///
  ///     <!-- Linear and Angular Damping -->
  ///     <cDampL1>1.0E-6</cDampL1>
  ///     <cDampL2>1.0E-6</cDampL2>
  ///     <cDampR1>1.0E-6</cDampR1>
  ///     <cDampR2>1.0E-6</cDampR2>
  ///
  ///     <!-- 'Pressure' Drag -->
  ///     <cPDrag1>1.0E+2</cPDrag1>
  ///     <cPDrag2>1.0E+2</cPDrag2>
  ///     <fPDrag>0.4</fPDrag>
  ///     <cSDrag1>1.0E+2</cSDrag1>
  ///     <cSDrag2>1.0E+2</cSDrag2>
  ///     <fSDrag>0.4</fSDrag>
  ///     <vRDrag>1.0</vRDrag>
  ///   </hydrodynamics>
  /// </plugin>
  /// \endcode
  ///
  class Hydrodynamics
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: Hydrodynamics();

    /// \brief Destructor
    public: ~Hydrodynamics() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrodynamicsPrivate> dataPtr;
  };
  }
}
}
}

#endif
