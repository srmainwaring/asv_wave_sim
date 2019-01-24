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
/// \brief This file contains the definition of a Gazebo ModelPlugin
/// that manages hydrodynamics calculations for a buoyant object.  

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

  /// \brief Model plugin class to compute buoyancy and hydrodynamic drag forces
  ///
  /// SDF parameters: 
  ///
  /// <wave_model>: Name of the wave model referencing the plugin libWavefieldModelPlugin.so
  /// <damping_on>: Set to false to disable damping forces. Default is true. 
  /// <viscous_drag_on>: Set to false to disable viscous drag forces. Default is true. 
  /// <pressure_drag_on>: Set to false to disable pressure drag forces. Default is true. 
  /// <cDampL1>: Linear damping coeff for linear motion. 
  /// <cDampL2>: Quadratic damping coeff for linear motion.
  /// <cDampR1>: Linear damping coeff for angular motion.
  /// <cDampR2>: Quadratic damping coeff for angular motion.
  /// <cPDrag1>: Linear coeff for positive pressure drag.
  /// <cPDrag2>: Quadratic coeff for positive pressure drag.
  /// <fPDrag>: Exponent coeff for positive pressure drag.
  /// <cSDrag1>: Linear coeff for negative pressure drag.
  /// <cSDrag2>: Quadratic coeff for negative pressure drag.
  /// <fSDrag>: Exponent coeff for negative pressure drag.
  /// <vRDrag>: Reference speed for pressure drag.
  /// <markers>
  ///   <update_rate>: Update rate for markers. Default value is 30.
  ///   <water_patch>: Show water patch markers. Default valie is false.
  ///   <waterline>: Show waterline markers. Default valie is false.
  ///   <underwater_surface>: Show underwater surface markers. Default valie is false.
  /// </markers>
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
