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

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_OCEAN_VISUAL_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_OCEAN_VISUAL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>

#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// OceanVisualPlugin

  class OceanVisualPluginPrivate;

  class GZ_RENDERING_VISIBLE OceanVisualPlugin : public gazebo::VisualPlugin
  {
    /// \brief Destructor.
    public: virtual ~OceanVisualPlugin();

    /// \brief Constructor
    public: OceanVisualPlugin();

    /// \brief Load the visual.
    public: virtual void Load(
      gazebo::rendering::VisualPtr _visual,
      sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// \internal
    /// \brief Pointer to private data.
    private: std::shared_ptr<OceanVisualPluginPrivate> data;
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_OCEAN_VISUAL_PLUGIN_HH_
