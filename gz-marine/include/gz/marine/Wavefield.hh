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

#ifndef GZ_MARINE_WAVEFIELD_HH_
#define GZ_MARINE_WAVEFIELD_HH_

#include "gz/marine/CGALTypes.hh"

#include <gz/math.hh>

#include <memory>

namespace ignition
{
namespace marine
{
  class WaveParameters;
  class WavefieldPrivate;

  /// \brief A class to manage a wave field.
  class Wavefield
  {
    /// Destructor.
    public: virtual ~Wavefield();

    /// Constructor.
    public: Wavefield();

    // Compute the height at a point.
    public: virtual bool Height(const cgal::Point3& point, double& height) const;

    /// \brief Get the wave parameters.
    public: virtual std::shared_ptr<const WaveParameters> GetParameters() const;

    /// \brief Set the wave parameters.
    ///
    /// \param[in] _params    The new wave parameters.
    public: virtual void SetParameters(std::shared_ptr<WaveParameters> _params) const;

    /// \brief Update (recalculate) the wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    public: virtual void Update(double _time);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldPrivate> data;

    // @TODO: relocate
  #if 0
    /// \todo(srmainwaring): port to ignition
    void OnWaveWindMsg(ConstParam_VPtr &_msg);
  #endif
  };

  typedef std::shared_ptr<Wavefield> WavefieldPtr;
  typedef std::weak_ptr<Wavefield> WavefieldWeakPtr;
}
}

#endif
