// Copyright (C) 2019  Rhys Mainwaring
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

/// \file HydrodynamicsPlugin.hh
/// \brief A Gazebo model plugin to manage hydrodynamics calculations
/// for a buoyant object.  

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_HYDRODYNAMICS_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_HYDRODYNAMICS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <memory>

namespace asv
{
  /// \internal
  /// \brief Class to hold private data for HydrodynamicsPlugin.
  class HydrodynamicsPluginPrivate;

  /// \brief A Gazebo model plugin to manage buoyancy and hydrodynamic force
  /// calculations for a buoyant object.
  ///
  /// # Usage
  /// 
  /// Add the SDF for the plugin to the <model> element of your model. 
  /// 
  /// /code
  /// <plugin name="hydrodynamics" filename="libHydrodynamicsPlugin.so">
  ///   <!-- Wave Model -->
  ///   <wave_model>ocean_waves</wave_model>
  ///
  ///   <!-- Hydrodynamics -->
  ///   <damping_on>true</damping_on>
  ///   <viscous_drag_on>true</viscous_drag_on>
  ///   <pressure_drag_on>true</pressure_drag_on>
  ///
  ///   <!-- Linear and Angular Damping -->
  ///   <cDampL1>1.0E-6</cDampL1>
  ///   <cDampL2>1.0E-6</cDampL2>
  ///   <cDampR1>1.0E-6</cDampR1>
  ///   <cDampR2>1.0E-6</cDampR2>
  ///
  ///   <!-- 'Pressure' Drag -->
  ///   <cPDrag1>1.0E+2</cPDrag1>
  ///   <cPDrag2>1.0E+2</cPDrag2>
  ///   <fPDrag>0.4</fPDrag>
  ///   <cSDrag1>1.0E+2</cSDrag1>
  ///   <cSDrag2>1.0E+2</cSDrag2>
  ///   <fSDrag>0.4</fSDrag>
  ///   <vRDrag>1.0</vRDrag>
  ///
  ///   <!-- Markers -->
  ///   <markers>
  ///     <update_rate>30</update_rate>
  ///     <water_patch>false</water_patch>
  ///     <waterline>false</waterline>
  ///     <underwater_surface>false</underwater_surface>
  ///   </markers>
  /// </plugin>
  /// \endcode
  ///
  /// # Subscribed Topics
  ///
  /// 1. ~/hydrodynamics (gazebo::msgs::Param_V)
  ///
  /// # Published Topics
  ///
  /// 1. /marker (ignition::msgs::Marker)
  ///
  /// # Parameters
  ///
  /// 1. <wave_model> (string, default: "")
  ///   Name of the wave model referencing the plugin WavefieldModelPlugin
  ///
  /// 2. <damping_on> (bool, default: true)
  ///   Set to false to disable damping forces.
  ///
  /// 3. <viscous_drag_on> (bool, default: true)
  ///   Set to false to disable viscous drag forces.
  ///
  /// 4. <pressure_drag_on> (bool, default: true)
  ///   Set to false to disable pressure drag forces.
  ///
  /// 5. <cDampL1> (double, default: 1.0E-6)
  ///   Linear damping coefficient for linear motion.
  ///
  /// 6. <cDampL2> (double, default: 1.0E-6)
  ///   Quadratic damping coefficient for linear motion.
  ///
  /// 7. <cDampR1> (double, default: 1.0E-6)
  ///   Linear damping coefficient for angular motion.
  ///
  /// 8. <cDampR2> (double, default: 1.0E-6)
  ///   Quadratic damping coefficient for angular motion.
  ///
  /// 9. <cPDrag1> (double, default: 1.0E+2)
  ///   Linear coecoefficientff for positive pressure drag.
  ///
  /// 10. <cPDrag2> (double, default: 1.0E+2)
  ///   Quadratic coefficient for positive pressure drag.
  ///
  /// 11. <fPDrag> (double, default: 0.4)
  ///   Exponential coefficient for positive pressure drag.
  ///
  /// 12. <cSDrag1> (double, default: 1.0E+2)
  ///   Linear coecoefficientff for negative pressure drag.
  ///
  /// 13. <cSDrag2> (double, default: 1.0E+2)
  ///   Quadratic coefficient for negative pressure drag.
  ///
  /// 14. <fSDrag> (double, default: 0.4)
  ///   Exponential coefficient for negative pressure drag.
  ///
  /// 15. <vRDrag> (double, default: 1.0)
  ///   Reference speed for pressure drag.
  ///
  /// 16. <update_rate> (double, default: 30.0)
  ///   Update rate for publishing visual markers.
  ///
  /// 17. <water_patch> (bool, default: false)
  ///   Set true to display water patch visual markers.
  ///
  /// 18. <waterline> (bool, default: false)
  ///   Set true to display water line visual markers.
  ///
  /// 19. <underwater_surface> (bool, default: false)
  ///   Set true to display underwater surface visual markers.
  ///
  class GAZEBO_VISIBLE HydrodynamicsPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor.
    public: HydrodynamicsPlugin();

    /// \brief Destructor.
    public: virtual ~HydrodynamicsPlugin();

    /// \brief Inherited.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Custom initialisation.
    public: void Init();
    private: void InitPhysics();
    private: void InitMarkers();
    private: void InitWaterPatchMarkers();
    private: void InitWaterlineMarkers();
    private: void InitUnderwaterSurfaceMarkers();

    /// \brief Custom plugin reset.
    public: void Reset();
    private: void ResetPhysics();
    private: void ResetMarkers();
    private: void ResetWaterPatchMarkers();
    private: void ResetWaterlineMarkers();
    private: void ResetUnderwaterSurfaceMarkers();

    /// \brief Callback for World Update events.
    private: void OnUpdate();
    private: void UpdatePhysics();
    private: void UpdateMarkers();
    private: void UpdateWaterPatchMarkers();
    private: void UpdateWaterlineMarkers();
    private: void UpdateUnderwaterSurfaceMarkers();

    /// \brief Custom plugin finalisation.
    private: void Fini();
    private: void FiniPhysics();
    private: void FiniMarkers();
    private: void FiniWaterPatchMarkers();
    private: void FiniWaterlineMarkers();
    private: void FiniUnderwaterSurfaceMarkers();

    /// internal
    /// \brief Callback for gztopic "~/hydrodynamics".
    ///
    /// \param[in] _msg Hydrodynamics parameters message.
    private: void OnHydrodynamicsMsg(ConstParam_VPtr &_msg);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<HydrodynamicsPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_HYDRODYNAMICS_PLUGIN_HH_
