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

/// \file WavefieldModelPlugin.hh
/// \brief This file defines a Gazebo ModelPlugin used to manage a wave field.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPlugin

  /// \internal
  /// \brief Class to hold private data for WavefieldModelPlugin.
  class WavefieldModelPluginPrivate;

  /// \brief Model plugin class to generate Gerstner waves
  ///
  /// SDF parameters: 
  ///
  /// <static>: Display a static wave field if set to true. Default value is false.
  /// <update_rate>: The rate at which the wavefield is updated. Default value is 30.
  /// <size>: A two component vector for the size of the wave field in each direction. Default value is [1000, 1000] 
  /// <cell_count>: A two component vector for the number of grid cells in each direction. Default value is [50, 50]
  /// <wave>
  ///   <number>: The number of commponent waves. 
  ///   <scale>: The scale between the mean and largest / smallest component waves.
  ///   <angle>: The angle between the mean wave direction and the largest / smallest component waves.
  ///   <steepness>: A parameter in [0, 1] controlling the wave steepness with 1 being steepest.
  ///   <amplitude>: The amplitude of the mean wave in [m].
  ///   <period>: The period of the mean wave in [s].
  ///   <phase>: The phase of the mean wave.
  ///   <direction>: A two component vector specifiying the direction of the mean wave.
  /// </wave> 
  /// <markers>
  ///   <wave_patch>: Display a wave marker if set to true. Default value is false.
  ///   <wave_patch_size>: A two component vector for the size of the wave marker (in units of the wave grid).
  /// </markers>
  ///
  class GAZEBO_VISIBLE WavefieldModelPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~WavefieldModelPlugin();

    /// \brief Constructor.
    public: WavefieldModelPlugin();

    /// \brief Inherited.
    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Custom initialisation.
    public: void Init();

    /// \brief Custom finalisation.
    public: void Fini();

    /// \brief Custom reset.
    public: void Reset();

    /// internal
    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// internal
    /// \brief Callback for gztopic "~/request" when the request is "wave_param".
    ///
    /// \param[in] _msg Request message.
    private: void OnRequest(ConstRequestPtr &_msg);

    /// internal
    /// \brief Callback for gztopic "~/wave".
    ///
    /// \param[in] _msg Wave message.
    private: void OnWaveMsg(ConstParam_VPtr &_msg);

    /// \internal
    /// \brief Private methods for managing markers.
    private: void InitMarker();
    private: void FiniMarker();
    private: void ResetMarker();
    private: void UpdateMarker();

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldModelPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_
