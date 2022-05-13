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

#ifndef GZ_MARINE_OCEANVISUAL_HH_
#define GZ_MARINE_OCEANVISUAL_HH_

#include <gz/rendering/Visual.hh>
#include <gz/msgs/msgs.hh>

#include <memory>

namespace ignition
{
namespace marine
{
  
///////////////////////////////////////////////////////////////////////////////
// OceanVisual

  class OceanVisualPrivate;

  class GZ_RENDERING_VISIBLE OceanVisual : public gazebo::rendering::Visual
  {
    /// \brief Destructor.
    public: virtual ~OceanVisual();

    /// \brief Constructor
    public: OceanVisual(
      const std::string& _name,
      gazebo::rendering::VisualPtr _parent);

    /// \brief Load the visual.
    public: void Load(sdf::ElementPtr _sdf);

    /// \brief Load the visual.
    public: void Load() override;

    /// internal
    /// \brief Called every PreRender event.
    private: void OnUpdate();

    /// internal
    /// \brief Callback for gztopic "~/wave/wind".
    ///
    /// \param[in] _msg Wave wind message.
    void OnWaveWindMsg(ConstParam_VPtr &_msg);

    /// internal
    /// \brief Callback for topic "~/world_stats".
    ///
    /// \param[in] _msg World statistics message.
    private: void OnStatsMsg(ConstWorldStatisticsPtr &_msg);

    /// \internal
    /// \brief Pointer to private data.
    private: std::shared_ptr<OceanVisualPrivate> data;
  };

}
}

#endif
