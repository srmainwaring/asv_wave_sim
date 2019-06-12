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

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_

#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>

#include <memory>

namespace asv
{
  class Grid;

///////////////////////////////////////////////////////////////////////////////
// Wavefield

  class WaveParameters;
  
  /// \brief A class to manage a wave field.
  class Wavefield
  {
    /// Destructor.
    public: virtual ~Wavefield();

    /// \brief Access the wave field mesh.
    public: virtual std::shared_ptr<const Mesh> GetMesh() const = 0;

    /// \brief Access the wave field grid.
    public: virtual std::shared_ptr<const Grid> GetGrid() const = 0;

    // Compute the height at a point.
    public: virtual bool Height(const Point3& point, double& height) const = 0;

    /// \brief Get the wave parameters.
    public: virtual std::shared_ptr<const WaveParameters> GetParameters() const = 0;

    /// \brief Set the wave parameters.
    ///
    /// \param[in] _params    The new wave parameters.
    public: virtual void SetParameters(std::shared_ptr<WaveParameters> _params) const = 0;

    /// \brief Update (recalculate) the wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    public: virtual void Update(double _time) = 0;
  };

///////////////////////////////////////////////////////////////////////////////
/// \brief A class to manage a wave field.
  class WavefieldGerstnerPrivate;

  class WavefieldGerstner : public Wavefield
  {
    /// Destructor.
    public: ~WavefieldGerstner() override;

    /// Constructor.
    ///
    /// \param[in] _name    The name for the wave field mesh.
    public: WavefieldGerstner(const std::string& _name);

    /// Constructor.
    ///
    /// \param[in] _name      The name for the wave field mesh.
    /// \param[in] _size      A two component array defining the size of the wavefield [m].
    /// \param[in] _cellCount A two component array defining the number of cells in each direction.
    public: WavefieldGerstner(
      const std::string& _name,
      const std::array<double, 2>& _size,
      const std::array<size_t, 2>& _cellCount);

    /// \brief Access the wave field mesh.
    public: std::shared_ptr<const Mesh> GetMesh() const override;

    /// \brief Access the wave field grid.
    public: std::shared_ptr<const Grid> GetGrid() const override;

    // Compute the height at a point.
    public: bool Height(const Point3& point, double& height) const override;

    /// \brief Access the wave field mesh as a Gazebo Mesh.
    // public: std::shared_ptr<const gazebo::common::Mesh> GetGzMesh() const;

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

    /// internal
    /// \brief Update (recalculate) a Gerstner wave field for the given time.
    ///
    /// \param[in] _time    The time parameter for the wave evolution.
    private: void UpdateGerstnerWave(double _time);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldGerstnerPrivate> data;
  };


///////////////////////////////////////////////////////////////////////////////
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

    /// \brief Access the wave field mesh.
    public: std::shared_ptr<const Mesh> GetMesh() const override;

    /// \brief Access the wave field grid.
    public: std::shared_ptr<const Grid> GetGrid() const override;

    // Compute the height at a point.
    public: bool Height(const Point3& point, double& height) const override;

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
  };

///////////////////////////////////////////////////////////////////////////////

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_
