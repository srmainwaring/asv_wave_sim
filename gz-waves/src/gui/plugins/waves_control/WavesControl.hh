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

#ifndef GZ_WAVES_WAVESCONTROL_HH_
#define GZ_WAVES_WAVESCONTROL_HH_

#include <gz/gui/qt.h>
#include <gz/sim/gui/GuiSystem.hh>

#include <memory>

namespace gz
{
namespace sim
{

// Inline bracket to help doxygen filtering.
inline namespace GZ_WAVES_VERSION_NAMESPACE
{
  class WavesControlPrivate;

  /// \brief Edit parameters controlling the waves systems plugins.
  class WavesControl : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: WavesControl();

    /// \brief Destructor
    public: ~WavesControl() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Callback when water patch marker checkbox state is changed
    /// \param[in] _checked indicates show or hide contacts
    public slots: void OnShowWaterPatchMarkers(bool _checked);

    /// \brief Callback when waterline marker checkbox state is changed
    /// \param[in] _checked indicates show or hide contacts
    public slots: void OnShowWaterlineMarkers(bool _checked);

    /// \brief Callback when submerged triangle marker checkbox state is changed
    /// \param[in] _checked indicates show or hide contacts
    public slots: void OnShowSubmergedTriangleMarkers(bool _checked);

    /// \brief Update the wind speed
    /// \param[in] _windSpeed new wind speed
    public slots: void UpdateWindSpeed(double _windSpeed);

    /// \brief Update the wind angle
    /// \param[in] _windAngle new wind angle
    public slots: void UpdateWindAngle(double _windAngle);

    /// \brief Update the wave steepness
    /// \param[in] _steepness new steepness
    public slots: void UpdateSteepness(double _steepness);

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<WavesControlPrivate> dataPtr;
  };

}
}
}

#endif
