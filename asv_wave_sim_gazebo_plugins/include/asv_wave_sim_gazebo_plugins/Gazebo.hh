/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/// \file Gazebo.hh
/// \brief Support for methods not available in legacy versions of Gazebo.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_GAZEBO_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_GAZEBO_HH_

#include <gazebo/rendering/Visual.hh>
#include "gazebo/rendering/ogre_gazebo.h"

#include <ignition/math/Vector3.hh>

#include <memory>
#include <string>
#include <vector>

namespace gazebo
{
  namespace rendering
  {
    class Visual;

    void ToOgreVector3(
      const std::vector<double>& _v,
      Ogre::Vector3& _vout);

    void ToOgreVector3(
      const ignition::math::Vector3d& _v,
      Ogre::Vector3& _vout);

    void ToOgreVector3(
      const std::vector<ignition::math::Vector3d>& _v,
      Ogre::Vector3& _vout0,
      Ogre::Vector3& _vout1,
      Ogre::Vector3& _vout2);

    /// \brief Set a shader program parameter associated to this visual's
    /// material
    /// \param[in] _visual Reference to a Visual
    /// \param[in] _paramName Name of shader parameter
    /// \param[in] _shaderType Type of shader. Supported types:
    /// vertex, fragment
    /// \param[in] _value Value to set the parameter to. The value string can
    /// be a number (int, float) or a space delimited array of numbers
    /// (floats). The value type must match the type defined in the shaders.
    /// Note: Setting vec2/float2 params is only supported in ogre1.9+
    void SetMaterialShaderParam(
      Visual& _visual,
      const std::string &_paramName,
      const std::string &_shaderType,
      const std::string &_value);

    /// \brief Override Visual::AttachMesh.
    /// Derived from gazebo/rendering/Visual.cc
    ///
    Ogre::MovableObject* AttachMesh(
      Visual& _visual,
      const std::string& _meshName,
      const std::string& _subMesh="",
      bool _centerSubmesh=false,
      const std::string& _objName="");

    /// \brief Override HasElement("material") in Visual::Load.
    /// Derived from gazebo/rendering/Visual.cc lines 454-497
    /// 
    void SetMaterial(
      Visual& _visual,
      sdf::ElementPtr _sdf);

    /// \brief Custom implementation to insert a mesh into Ogre.
    /// \param[in] _mesh Pointer to the mesh to insert.
    /// \param[in] _subMesh Name of the mesh within _meshName to insert.
    /// \param[in] _centerSubmesh True to center the submesh.
    void InsertMesh(
      const common::Mesh *_mesh,
      const std::string &_subMesh="",
      bool _centerSubmesh=false);



  }; // namespace rendering

} // namespace gazebo

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_GAZEBO_HH_
