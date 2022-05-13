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

#include "gz/marine/Utilities.hh"
#include "gz/marine/Convert.hh"

#include <gz/common.hh>
#include <gz/math.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <iostream>
#include <string>

namespace ignition
{
namespace marine
{

///////////////////////////////////////////////////////////////////////////////
// Templates

// This code adapted vmrc/usv_gazebo_plugins/usv_gazebo_dynamics_plugin.cc
template <typename T>
T SdfParam(const sdf::Element& _sdf, const std::string &_paramName, const T _defaultVal)
{
  if (!_sdf.HasElement(_paramName))
  {
    ignmsg << "Parameter <" << _paramName << "> not found: " 
      <<  "Using default value of <" << _defaultVal << ">." << std::endl;
    return _defaultVal;
  }

  T val = _sdf.Get<T>(_paramName);
  ignmsg << "Parameter found - setting <" << _paramName 
    << "> to <" << val << ">." << std::endl;
  return val;
}

/// \brief Template function for extracting a value from a parameter message.
template <typename T>
T MsgParamGetValue(const ignition::msgs::Param& _msg)
{
  ignwarn << "Using default template for MsgParamGetValue" << std::endl;
  return T();  
}

/// \brief Template specialization for extracting a bool from a parameter message.
template <>
bool MsgParamGetValue<bool>(const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  return paramValue.bool_value();
}

/// \brief Template specialization for extracting an int from a parameter message.
template <>
int MsgParamGetValue<int>(const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  return paramValue.int_value();
}

/// \brief Template specialization for extracting a size_t from a parameter message.
template <>
size_t MsgParamGetValue<size_t>(const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  return paramValue.int_value();
}

/// \brief Template specialization for extracting a double from a parameter message.
template <>
double MsgParamGetValue<double>(const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  return paramValue.double_value();
}

/// \brief Template specialization for extracting a string from a parameter message.
template <>
std::string MsgParamGetValue<std::string>(const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  return paramValue.string_value();
}

/// \brief Template specialization for extracting a Vector2i from a parameter message.
template <>
ignition::math::Vector2i MsgParamGetValue<ignition::math::Vector2i>(
    const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  auto vec = paramValue.vector3d_value();
  return ignition::math::Vector2i(vec.x(), vec.y());
}

/// \brief Template specialization for extracting a Vector3 from a parameter message.
template <>
ignition::math::Vector3d MsgParamGetValue<ignition::math::Vector3d>(
    const ignition::msgs::Param& _msg)
{ 
  auto paramValue = _msg.params().begin()->second;
  // auto paramValue = _msg.value();
  auto vec = paramValue.vector3d_value();
  return ignition::math::Vector3d(vec.x(), vec.y(), vec.z());
}

/// \brief Template for extracting a named parameter from a parameter vector message.
template <typename T>
T MsgParam(const ignition::msgs::Param_V& _msg, const std::string &_paramName, const T _defaultVal)
{
  // Custom compare for params
  auto compare = [=](auto& _param)
  {
    bool has_key = _param.params().find(_paramName) !=  _param.params().end();
    return has_key;
    // return _param.name() == _paramName;
  };

  auto it = std::find_if(std::begin(_msg.param()), std::end(_msg.param()), compare);
  
  // Not found
  if (it == std::end(_msg.param()))
  {
  // @DEBUG_INFO
    // ignmsg << "Parameter <" << _paramName << "> not found: " 
    //   <<  "Using default value of <" << _defaultVal << ">." << std::endl;
    return _defaultVal;
  }

  // Found
  auto& param = *it;
  T val = MsgParamGetValue<T>(param);

  // @DEBUG_INFO
  // ignmsg << "Parameter found - setting <" << _paramName 
  //   << "> to <" << val << ">." << std::endl;
  return val;
}


///////////////////////////////////////////////////////////////////////////////
// Utilities

bool Utilities::SdfParamBool(const sdf::Element& _sdf,
  const std::string& _paramName, const bool _defaultVal)
{
  return SdfParam<bool>(_sdf, _paramName, _defaultVal);
}

size_t Utilities::SdfParamSizeT(const sdf::Element& _sdf,
  const std::string& _paramName, const size_t _defaultVal)
{
  return SdfParam<double>(_sdf, _paramName, _defaultVal);
}

double Utilities::SdfParamDouble(const sdf::Element& _sdf,
  const std::string& _paramName, const double _defaultVal)
{
  return SdfParam<double>(_sdf, _paramName, _defaultVal);
}

std::string Utilities::SdfParamString(const sdf::Element& _sdf,
  const std::string& _paramName, const std::string _defaultVal)
{
  return SdfParam<std::string>(_sdf, _paramName, _defaultVal);
}

ignition::math::Vector2d Utilities::SdfParamVector2d(const sdf::Element& _sdf,
  const std::string& _paramName, const ignition::math::Vector2d _defaultVal)
{
  return SdfParam<ignition::math::Vector2d>(
    _sdf, _paramName, _defaultVal);
}

ignition::math::Vector2i Utilities::SdfParamVector2i(const sdf::Element& _sdf,
  const std::string& _paramName, const ignition::math::Vector2i _defaultVal)
{
  return SdfParam<ignition::math::Vector2i>(
    _sdf, _paramName, _defaultVal);
}

ignition::math::Vector3d Utilities::SdfParamVector3d(const sdf::Element& _sdf,
  const std::string& _paramName, const ignition::math::Vector3d _defaultVal)
{
  return SdfParam<ignition::math::Vector3d>(
    _sdf, _paramName, _defaultVal);
}

bool Utilities::MsgParamBool(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const bool _defaultVal)
{
  return MsgParam<bool>(_msg, _paramName, _defaultVal);
}

size_t Utilities::MsgParamSizeT(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const size_t _defaultVal)
{
  return MsgParam<size_t>(_msg, _paramName, _defaultVal);
}

double Utilities::MsgParamDouble(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const double _defaultVal)
{
  return MsgParam<double>(_msg, _paramName, _defaultVal);
}

std::string Utilities::MsgParamString(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const std::string _defaultVal)
{
  return MsgParam<std::string>(_msg, _paramName, _defaultVal);
}

ignition::math::Vector2i Utilities::MsgParamVector2i(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const ignition::math::Vector2i _defaultVal)
{
  return MsgParam<ignition::math::Vector2i>(_msg, _paramName, _defaultVal);
}

ignition::math::Vector2d Utilities::MsgParamVector2d(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const ignition::math::Vector2d _defaultVal)
{
  return MsgParam<ignition::math::Vector2d>(_msg, _paramName, _defaultVal);
}

ignition::math::Vector3d Utilities::MsgParamVector3d(const ignition::msgs::Param_V& _msg,
  const std::string &_paramName, const ignition::math::Vector3d _defaultVal)
{
  return MsgParam<ignition::math::Vector3d>(_msg, _paramName, _defaultVal);
}


///////////////////////////////////////////////////////////////////////////////

}
}
