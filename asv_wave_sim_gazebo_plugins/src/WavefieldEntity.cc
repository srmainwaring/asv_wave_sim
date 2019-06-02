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

#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WaveParameters.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>

#include <iostream>
#include <string>

using namespace gazebo;
using namespace common;

namespace asv 
{

///////////////////////////////////////////////////////////////////////////////    
// WavefieldEntity

  /// \internal
  /// \brief Private data for the WavefieldEntity
  class WavefieldEntityPrivate
  {
    /// \brief The size of the wavefield. Default value is [1000 1000].
    public: Vector2 size;

    /// \brief The number of grid cells in the wavefield. Default value is [50 50].
    public: Vector2 cellCount;

    /// \brief The wave parameters.
    public: std::shared_ptr<asv::WaveParameters> waveParams;

    /// \brief The wavefield.
    public: std::shared_ptr<Wavefield> wavefield;
  };

  WavefieldEntity::~WavefieldEntity()
  {
  }

  WavefieldEntity::WavefieldEntity(physics::BasePtr _parent) :
    Base(_parent),
    data(new WavefieldEntityPrivate())
  {
  }

  void WavefieldEntity::Load(sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    Base::Load(_sdf);

    // Wavefield Parameters
    this->data->size      = Utilities::SdfParamVector2(*_sdf, "size",       Vector2(1000, 1000));
    this->data->cellCount = Utilities::SdfParamVector2(*_sdf, "cell_count", Vector2(50, 50));

    // Wave Parameters
    this->data->waveParams.reset(new WaveParameters());
    if (_sdf->HasElement("wave"))
    {
      sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
      this->data->waveParams->SetFromSDF(*sdfWave);
    }

    // @DEBUG_INFO
    // gzmsg << "WavefieldEntity..." <<  std::endl;
    // this->data->waveParams->DebugPrint();
  }

  void WavefieldEntity::Fini()
  {
    Base::Fini();
  }

  void WavefieldEntity::Init()
  {
    // Wavefield  
    std::string meshName = "_WAVEFIELD";
    std::string meshPath = "";

    double simTime = this->GetWorld()->SimTime().Double();

// @TODO SWITCH WAVE SIMULATION TYPE
#if 0
    this->data->wavefield.reset(new WavefieldGerstner(
      meshName,
      { this->data->size[0], this->data->size[1] },
      { static_cast<size_t>(this->data->cellCount[0]), static_cast<size_t>(this->data->cellCount[1]) }
    ));
#else
    this->data->wavefield.reset(new WavefieldOceanTile(meshName));

    this->data->wavefield->SetParameters(this->data->waveParams);
    this->data->wavefield->Update(simTime);
  }
#endif

  void WavefieldEntity::Reset()
  {
    double simTime = this->GetWorld()->SimTime().Double();
    this->data->wavefield->Update(simTime);
  }

  void WavefieldEntity::Update()
  {
    // Update the mesh
    double simTime = this->GetWorld()->SimTime().Double();
    this->data->wavefield->Update(simTime);
  }

  std::shared_ptr<const Wavefield> WavefieldEntity::GetWavefield() const
  {
    return this->data->wavefield;
  }

  std::string WavefieldEntity::MakeName(const std::string& _parentName)
  {
    return std::string(_parentName + "::wavefield_entity");
  }

} // namespace asv

