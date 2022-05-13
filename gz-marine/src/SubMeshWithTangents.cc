/*
 * Copyright (C) 2022  Rhys Mainwaring
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
*/

#include "gz/common/SubMeshWithTangents.hh"

#include <gz/common/Console.hh>

#include <string>

using namespace ignition;
using namespace common;

/// \brief Private data for SubMeshWithTangents
class ignition::common::SubMeshWithTangents::Implementation
{
  /// \brief the tangents array
  public: std::vector<ignition::math::Vector3d> tangents;

};

//////////////////////////////////////////////////
SubMeshWithTangents::SubMeshWithTangents()
    : SubMesh(),
    dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
SubMeshWithTangents::SubMeshWithTangents(const std::string &_name)
    : SubMesh(_name),
    dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
SubMeshWithTangents::SubMeshWithTangents(const SubMeshWithTangents &_submesh)
  : SubMesh(_submesh),
  dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  // copy this
  *this->dataPtr = (*_submesh.dataPtr);
}

//////////////////////////////////////////////////
SubMeshWithTangents::SubMeshWithTangents(SubMeshWithTangents &&_submesh)
    noexcept = default;

//////////////////////////////////////////////////
SubMeshWithTangents &SubMeshWithTangents::operator=(const SubMeshWithTangents &_submesh)
{
  if (this != &_submesh)
  {
    // copy base
    (common::SubMesh&)(*this) = _submesh;

    // copy this
    *this->dataPtr = (*_submesh.dataPtr);
  }

  return *this;
}

//////////////////////////////////////////////////
SubMeshWithTangents &SubMeshWithTangents::operator=(SubMeshWithTangents &&_submesh)
    noexcept = default;

//////////////////////////////////////////////////
SubMeshWithTangents::~SubMeshWithTangents()
{
}

//////////////////////////////////////////////////
void SubMeshWithTangents::AddTangent(const ignition::math::Vector3d &_tanget)
{
  this->dataPtr->tangents.push_back(_tanget);
}

//////////////////////////////////////////////////
void SubMeshWithTangents::AddTangent(const double _x, const double _y, const double _z)
{
  this->AddTangent(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SubMeshWithTangents::Tangent(const unsigned int _index) const
{
  if (_index >= this->dataPtr->tangents.size())
  {
    ignerr << "Index too large" << std::endl;
    return math::Vector3d::Zero;
  }

  return this->dataPtr->tangents[_index];
}

//////////////////////////////////////////////////
bool SubMeshWithTangents::HasTangent(const unsigned int _index) const
{
  return _index < this->dataPtr->tangents.size();
}

//////////////////////////////////////////////////
void SubMeshWithTangents::SetTangent(const unsigned int _index,
    const ignition::math::Vector3d &_n)
{
  if (_index >= this->dataPtr->tangents.size())
  {
    ignerr << "Index too large" << std::endl;
    return;
  }

  this->dataPtr->tangents[_index] = _n;
}

//////////////////////////////////////////////////
unsigned int SubMeshWithTangents::TangentCount() const
{
  return this->dataPtr->tangents.size();
}
