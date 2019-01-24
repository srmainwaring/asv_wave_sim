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

#include "asv_wave_sim_gazebo_plugins/Utilities.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include <algorithm>
#include <iostream>
#include <string>

namespace asv 
{

///////////////////////////////////////////////////////////////////////////////
// Templates

// This code adapted vmrc/usv_gazebo_plugins/usv_gazebo_dynamics_plugin.cc
template <typename T>
T SdfParam(sdf::Element& _sdf, const std::string &_paramName, const T _defaultVal)
{
  if (!_sdf.HasElement(_paramName))
  {
    gzmsg << "Parameter <" << _paramName << "> not found: " 
      <<  "Using default value of <" << _defaultVal << ">." << std::endl;
    return _defaultVal;
  }

  T val = _sdf.Get<T>(_paramName);
  gzmsg << "Parameter found - setting <" << _paramName 
    << "> to <" << val << ">." << std::endl;
  return val;
}

/// \brief Template function for extracting a value from a parameter message.
template <typename T>
T MsgParamGetValue(const gazebo::msgs::Param& _msg)
{
  gzwarn << "Using default template for MsgParamGetValue" << std::endl;
  return T();  
}

/// \brief Template specialization for extracting a bool from a parameter message.
template <>
bool MsgParamGetValue<bool>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.bool_value();
}

/// \brief Template specialization for extracting an int from a parameter message.
template <>
int MsgParamGetValue<int>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.int_value();
}

/// \brief Template specialization for extracting a size_t from a parameter message.
template <>
size_t MsgParamGetValue<size_t>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.int_value();
}

/// \brief Template specialization for extracting a double from a parameter message.
template <>
double MsgParamGetValue<double>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.double_value();
}

/// \brief Template specialization for extracting a string from a parameter message.
template <>
std::string MsgParamGetValue<std::string>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.string_value();
}

/// \brief Template specialization for extracting a Vector2 from a parameter message.
template <>
Vector2 MsgParamGetValue<Vector2>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  auto vec = paramValue.vector3d_value();
  return Vector2(vec.x(), vec.y());
}

/// \brief Template specialization for extracting a Vector3 from a parameter message.
template <>
Vector3 MsgParamGetValue<Vector3>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  auto vec = paramValue.vector3d_value();
  return Vector3(vec.x(), vec.y(), vec.z());
}

/// \brief Template for extracting a named parameter from a parameter vector message.
template <typename T>
T MsgParam(const gazebo::msgs::Param_V& _msg, const std::string &_paramName, const T _defaultVal)
{
  // Custom compare for params
  auto compare = [=](auto& _param)
  {
    return _param.name() == _paramName;
  };

  auto it = std::find_if(std::begin(_msg.param()), std::end(_msg.param()), compare);
  
  // Not found
  if (it == std::end(_msg.param()))
  {
  // @DEBUG_INFO
    // gzmsg << "Parameter <" << _paramName << "> not found: " 
    //   <<  "Using default value of <" << _defaultVal << ">." << std::endl;
    return _defaultVal;
  }

  // Found
  auto& param = *it;
  T val = MsgParamGetValue<T>(param);

  // @DEBUG_INFO
  // gzmsg << "Parameter found - setting <" << _paramName 
  //   << "> to <" << val << ">." << std::endl;
  return val;
}


///////////////////////////////////////////////////////////////////////////////
// Utilities

bool Utilities::SdfParamBool(sdf::Element& _sdf,
  const std::string& _paramName, const bool _defaultVal)
{
  return SdfParam<bool>(_sdf, _paramName, _defaultVal);
}

size_t Utilities::SdfParamSizeT(sdf::Element& _sdf,
  const std::string& _paramName, const size_t _defaultVal)
{
  return SdfParam<double>(_sdf, _paramName, _defaultVal);
}

double Utilities::SdfParamDouble(sdf::Element& _sdf,
  const std::string& _paramName, const double _defaultVal)
{
  return SdfParam<double>(_sdf, _paramName, _defaultVal);
}

std::string Utilities::SdfParamString(sdf::Element& _sdf,
  const std::string& _paramName, const std::string _defaultVal)
{
  return SdfParam<std::string>(_sdf, _paramName, _defaultVal);
}

Vector2 Utilities::SdfParamVector2(sdf::Element& _sdf,
  const std::string& _paramName, const Vector2 _defaultVal)
{
  return ToVector2(SdfParam<ignition::math::Vector2d>(
    _sdf, _paramName, ToIgn(_defaultVal)));
}

Vector3 Utilities::SdfParamVector3(sdf::Element& _sdf,
  const std::string& _paramName, const Vector3 _defaultVal)
{
  return ToVector3(SdfParam<ignition::math::Vector3d>(
    _sdf, _paramName, ToIgn(_defaultVal)));
}

bool Utilities::MsgParamBool(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const bool _defaultVal)
{
  return MsgParam<bool>(_msg, _paramName, _defaultVal);
}

size_t Utilities::MsgParamSizeT(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const size_t _defaultVal)
{
  return MsgParam<size_t>(_msg, _paramName, _defaultVal);
}

double Utilities::MsgParamDouble(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const double _defaultVal)
{
  return MsgParam<double>(_msg, _paramName, _defaultVal);
}

std::string Utilities::MsgParamString(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const std::string _defaultVal)
{
  return MsgParam<std::string>(_msg, _paramName, _defaultVal);
}

Vector2 Utilities::MsgParamVector2(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const Vector2 _defaultVal)
{
  return MsgParam<Vector2>(_msg, _paramName, _defaultVal);
}

Vector3 Utilities::MsgParamVector3(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const Vector3 _defaultVal)
{
  return MsgParam<Vector3>(_msg, _paramName, _defaultVal);
}


///////////////////////////////////////////////////////////////////////////////

} // namespace asv
