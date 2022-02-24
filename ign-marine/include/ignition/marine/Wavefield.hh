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

/// \file Wavefield.hh
/// \brief This file contains definitions for the class used to manage
/// a wave field.

#ifndef IGNITION_MARINE_WAVEFIELD_HH_
#define IGNITION_MARINE_WAVEFIELD_HH_

#include "ignition/marine/CGALTypes.hh"

#include <ignition/math.hh>

#include <memory>

namespace ignition
{
namespace marine
{
  /////////////////////////////////////////////////
  class WaveParameters;
  
  /// \brief A class to manage a wave field.
  class Wavefield
  {
    /// Destructor.
    public: virtual ~Wavefield();

    // Compute the height at a point.
    public: virtual bool Height(const cgal::Point3& point, double& height) const { return false; } //= 0;

    /// \brief Get the wave parameters.
    public: virtual std::shared_ptr<const WaveParameters> GetParameters() const { return nullptr; } //= 0;

    /// \brief Set the wave parameters.
    ///
    /// \param[in] _params    The new wave parameters.
    public: virtual void SetParameters(std::shared_ptr<WaveParameters> _params) const {} //= 0;

    /// \brief Update (recalculate) the wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    public: virtual void Update(double _time) {} //= 0;
  };

  typedef std::shared_ptr<Wavefield> WavefieldPtr; 
  typedef std::weak_ptr<Wavefield> WavefieldWeakPtr; 

  /////////////////////////////////////////////////
  /// \brief A class to manage a wave field.
  class WavefieldOceanTilePrivate;

  class WavefieldOceanTile : public Wavefield
  {
    /// Destructor.
    public: ~WavefieldOceanTile() override;

    /// Constructor.
    ///
    /// \param[in] _name    The name for the wave field mesh.
    public: WavefieldOceanTile(const std::string& _name);

    // Compute the height at a point.
    public: bool Height(const cgal::Point3& point, double& height) const override;

    /// \brief Get the wave parameters.
    public: std::shared_ptr<const WaveParameters> GetParameters() const override;

    /// \brief Set the wave parameters.
    ///
    /// \param[in] _params    The new wave parameters.
    public: void SetParameters(std::shared_ptr<WaveParameters> _params) const override;

    /// \brief Update (recalculate) the wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    public: void Update(double _time) override;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldOceanTilePrivate> data;
  
  
    // @TODO: relocate 
  #if 0
    /// \todo(srmainwaring): port to ignition 
    void OnWaveWindMsg(ConstParam_VPtr &_msg);
  #endif
  };
  /////////////////////////////////////////////////
}
}

#endif
