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

#include "asv_wave_sim_gazebo_plugins/Gazebo.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>

#include <array>
#include <iostream>
#include <iterator>
#include <string>

namespace gazebo
{
  namespace rendering
  {
    void ToOgreVector3(
      const std::vector<double>& _v,
      Ogre::Vector3& _vout)
    {
      _vout = Ogre::Vector3::ZERO;
      if (_v.size() > 3)
      {
        gzerr << "Vector must have size 3 or less" << std::endl;
        return;
      }
      for (size_t i=0; i<_v.size(); ++i)
      {
        _vout[i] = _v[i];
      }
    }

    void ToOgreVector3(
      const ignition::math::Vector3d& _v,
      Ogre::Vector3& _vout)
    {
      _vout.x = _v.X();
      _vout.y = _v.Y();
      _vout.z = _v.Z();
    }

    void ToOgreVector3(
      const std::vector<ignition::math::Vector3d>& _v,
      Ogre::Vector3& _vout0,
      Ogre::Vector3& _vout1,
      Ogre::Vector3& _vout2)
    {
      _vout0 = Ogre::Vector3::ZERO;
      _vout1 = Ogre::Vector3::ZERO;
      _vout2 = Ogre::Vector3::ZERO;

      if (_v.size() > 3)
      {
        gzerr << "Vector must have size 3 or less" << std::endl;
        return;
      }
      if (_v.size() > 0)
        ToOgreVector3(_v[0], _vout0);
      if (_v.size() > 1)
        ToOgreVector3(_v[1], _vout1);
      if (_v.size() > 2)
        ToOgreVector3(_v[2], _vout2);
    }

    void SetMaterialShaderParam(
      Visual& _visual,
      const std::string &_paramName,
      const std::string &_shaderType,
      const std::string &_value)
    {
      // currently only vertex and fragment shaders are supported
      if (_shaderType != "vertex" && _shaderType != "fragment")
      {
        gzerr << "Shader type: '" << _shaderType << "' is not supported"
              << std::endl;
        return;
      }

      // set the parameter based name and type defined in material script
      // and shaders
      auto setNamedParam = [](Ogre::GpuProgramParametersSharedPtr _params,
          const std::string &_name, const std::string &_v)
      {
        auto paramDef = _params->_findNamedConstantDefinition(_name);
        if (!paramDef)
          return;

        switch (paramDef->constType)
        {
          case Ogre::GCT_INT1:
          {
            int value = Ogre::StringConverter::parseInt(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          case Ogre::GCT_FLOAT1:
          {
            Ogre::Real value = Ogre::StringConverter::parseReal(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          #if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
          case Ogre::GCT_INT2:
          case Ogre::GCT_FLOAT2:
          {
            Ogre::Vector2 value = Ogre::StringConverter::parseVector2(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          #endif
          case Ogre::GCT_INT3:
          case Ogre::GCT_FLOAT3:
          {
            Ogre::Vector3 value = Ogre::StringConverter::parseVector3(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          case Ogre::GCT_INT4:
          case Ogre::GCT_FLOAT4:
          {
            Ogre::Vector4 value = Ogre::StringConverter::parseVector4(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          case Ogre::GCT_MATRIX_4X4:
          {
            Ogre::Matrix4 value = Ogre::StringConverter::parseMatrix4(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          default:
            break;
        }
      };

      // loop through material techniques and passes to find the param
      // std::cout << "MaterialName: " << _vis->GetMaterialName() << std::endl;
      Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(
          _visual.GetMaterialName());
          // this->dataPtr->myMaterialName);
      if (mat.isNull())
      {
        gzerr << "Failed to find material: '" << _visual.GetMaterialName()
              << std::endl;
        // gzerr << "Failed to find material: '" << this->dataPtr->myMaterialName
        //       << std::endl;
        return;
      }
      for (unsigned int i = 0; i < mat->getNumTechniques(); ++i)
      {
        Ogre::Technique *technique = mat->getTechnique(i);
        if (!technique)
          continue;
        for (unsigned int j = 0; j < technique->getNumPasses(); ++j)
        {
          Ogre::Pass *pass = technique->getPass(j);
          if (!pass)
            continue;

          // check if pass is programmable, ie if they are using shaders
          if (!pass->isProgrammable())
          {
            continue;
          }

          if (_shaderType == "vertex" && pass->hasVertexProgram())
          {
            // std::cout << "Set vertex param: " << _paramName << ": " << _value << std::endl;
            setNamedParam(pass->getVertexProgramParameters(), _paramName, _value);
          }
          else if (_shaderType == "fragment" && pass->hasFragmentProgram())
          {
            // std::cout << "Set fragment param: " << _paramName << ": " << _value << std::endl;
            setNamedParam(pass->getFragmentProgramParameters(), _paramName, _value);
          }
          else
          {
            gzerr << "Failed to retrieve shaders for material: '"
                  << _visual.GetMaterialName() << "', technique: '"
                  << technique->getName() << "', pass: '" << pass->getName() << "'"
                  << std::endl;
            // gzerr << "Failed to retrieve shaders for material: '"
            //       << this->dataPtr->myMaterialName << "', technique: '"
            //       << technique->getName() << "', pass: '" << pass->getName() << "'"
            //       << std::endl;
            continue;
          }
        }
      }
    }

    Ogre::MovableObject* AttachMesh(
      Visual& _visual,
      const std::string& _meshName,
      const std::string& _subMesh,
      bool _centerSubmesh,
      const std::string& _objName)
    {
      if (_meshName.empty())
        return nullptr;

      Ogre::SceneNode* sceneNode = _visual.GetSceneNode();
      // this->dataPtr->meshName = _meshName;
      // this->dataPtr->subMeshName = _subMesh;

      Ogre::MovableObject *obj;
      std::string objName = _objName;
      std::string meshName = _meshName;
      meshName += _subMesh.empty() ? "" : "::" + _subMesh;

      if (objName.empty())
        objName = sceneNode->getName() + "_ENTITY_" + meshName;

      // this->InsertMesh(_meshName, _subMesh, _centerSubmesh);

      if (sceneNode->getCreator()->hasEntity(objName))
      {
        obj = (Ogre::MovableObject*)
          (sceneNode->getCreator()->getEntity(objName));
      }
      else
      {
        // build tangent vectors if normal mapping in tangent space is specified
        if (_visual.GetShaderType() == "normal_map_tangent_space")
        {
          Ogre::MeshPtr ogreMesh = Ogre::MeshManager::getSingleton().getByName(_meshName);
          if (!ogreMesh.isNull())
          {
            try
            {
              uint16_t src, dest;
              if (!ogreMesh->suggestTangentVectorBuildParams(Ogre::VES_TANGENT, src, dest))
              {
                ogreMesh->buildTangentVectors(Ogre::VES_TANGENT, src, dest);
              }
            }
            catch(Ogre::Exception &e)
            {
              gzwarn << "Problem generating tangent vectors for " << _meshName
                    << ". Normal map will not work: " << e.what() << std::endl;
            }
          }
        }

        obj = (Ogre::MovableObject*)(sceneNode->getCreator()->createEntity(objName, meshName));
      }

      _visual.AttachObject(obj);
      return obj;
    }

    void SetMaterial(
      Visual& _visual,
      sdf::ElementPtr _sdf)
    {
      if (_sdf == nullptr)
      {
        gzerr << "Cannot set material: SDF Element is NULL" << std::endl;
        return;
      }

      // Set the material of the mesh
      if (_sdf->HasElement("material"))
      {
        sdf::ElementPtr matElem =
            _sdf->GetElement("material");

        // clone the material sdf to preserve the new values to be set
        // as updating the material name via SetMaterial can affect the
        // ambient/diffuse/specular/emissive color sdf elements.
        sdf::ElementPtr matElemClone = matElem->Clone();

        if (matElem->HasElement("script"))
        {
          sdf::ElementPtr scriptElem = matElem->GetElement("script");
          sdf::ElementPtr uriElem = scriptElem->GetElement("uri");

          // Add all the URI paths to the render engine
          while (uriElem)
          {
            std::string matUri = uriElem->Get<std::string>();
            if (!matUri.empty())
              rendering::RenderEngine::Instance()->AddResourcePath(matUri);
            uriElem = uriElem->GetNextElement("uri");
          }

          std::string matName = scriptElem->Get<std::string>("name");

          if (!matName.empty())
            _visual.SetMaterial(matName);
        }

        if (matElemClone->HasElement("ambient"))
          _visual.SetAmbient(matElemClone->Get<ignition::math::Color>("ambient"));
        if (matElemClone->HasElement("diffuse"))
          _visual.SetDiffuse(matElemClone->Get<ignition::math::Color>("diffuse"));
        if (matElemClone->HasElement("specular"))
          _visual.SetSpecular(matElemClone->Get<ignition::math::Color>("specular"));
        if (matElemClone->HasElement("emissive"))
          _visual.SetEmissive(matElemClone->Get<ignition::math::Color>("emissive"));

        if (matElem->HasElement("lighting"))
        {
          _visual.SetLighting(matElem->Get<bool>("lighting"));
        }
      }
    }

    void InsertMesh(
      const common::Mesh *_mesh,
      const std::string &_subMesh,
      bool _centerSubmesh)
    {
      Ogre::MeshPtr ogreMesh;

      GZ_ASSERT(_mesh != nullptr, "Unable to insert a null mesh");

      rendering::RenderEngine::Instance()->AddResourcePath(_mesh->GetPath());

      if (_mesh->GetSubMeshCount() == 0)
      {
        gzerr << "Visual::InsertMesh no submeshes, this is an invalid mesh\n";
        return;
      }

      // Don't re-add existing meshes
      if (Ogre::MeshManager::getSingleton().resourceExists(_mesh->GetName()))
      {
        return;
      }

      try
      {
        // Create a new mesh specifically for manual definition.
        if (_subMesh.empty())
        {
          ogreMesh = Ogre::MeshManager::getSingleton().createManual(
              _mesh->GetName(),
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        }
        else
        {
          ogreMesh = Ogre::MeshManager::getSingleton().createManual(
              _mesh->GetName() + "::" + _subMesh,
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        }

        Ogre::SkeletonPtr ogreSkeleton;

        if (_mesh->HasSkeleton())
        {
          common::Skeleton *skel = _mesh->GetSkeleton();
          ogreSkeleton = Ogre::SkeletonManager::getSingleton().create(
            _mesh->GetName() + "_skeleton",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            true);

          for (unsigned int i = 0; i < skel->GetNumNodes(); i++)
          {
            common::SkeletonNode *node = skel->GetNodeByHandle(i);
            Ogre::Bone *bone = ogreSkeleton->createBone(node->GetName());

            if (node->GetParent())
              ogreSkeleton->getBone(node->GetParent()->GetName())->addChild(bone);

            ignition::math::Matrix4d trans = node->Transform();
            ignition::math::Vector3d pos = trans.Translation();
            ignition::math::Quaterniond q = trans.Rotation();
            bone->setPosition(Ogre::Vector3(pos.X(), pos.Y(), pos.Z()));
            bone->setOrientation(Ogre::Quaternion(q.W(), q.X(), q.Y(), q.Z()));
            bone->setInheritOrientation(true);
            bone->setManuallyControlled(true);
            bone->setInitialState();
          }
          ogreMesh->setSkeletonName(_mesh->GetName() + "_skeleton");
        }

        for (unsigned int i = 0; i < _mesh->GetSubMeshCount(); i++)
        {
          if (!_subMesh.empty() && _mesh->GetSubMesh(i)->GetName() != _subMesh)
            continue;

          Ogre::SubMesh *ogreSubMesh;
          Ogre::VertexData *vertexData;
          Ogre::VertexDeclaration* vertexDecl;
          Ogre::HardwareVertexBufferSharedPtr vBuf;
          Ogre::HardwareIndexBufferSharedPtr iBuf;
          float *vertices;
          uint32_t *indices;

          size_t currOffset = 0;

          // Copy the original submesh. We may need to modify the vertices, and
          // we don't want to change the original.
          common::SubMesh subMesh(_mesh->GetSubMesh(i));

          // Recenter the vertices if requested.
          if (_centerSubmesh)
            subMesh.Center(ignition::math::Vector3d::Zero);

          ogreSubMesh = ogreMesh->createSubMesh();
          ogreSubMesh->useSharedVertices = false;
          if (subMesh.GetPrimitiveType() == common::SubMesh::TRIANGLES)
            ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
          else if (subMesh.GetPrimitiveType() == common::SubMesh::LINES)
            ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_LIST;
          else if (subMesh.GetPrimitiveType() == common::SubMesh::LINESTRIPS)
            ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_STRIP;
          else if (subMesh.GetPrimitiveType() == common::SubMesh::TRIFANS)
            ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
          else if (subMesh.GetPrimitiveType() == common::SubMesh::TRISTRIPS)
            ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
          else if (subMesh.GetPrimitiveType() == common::SubMesh::POINTS)
            ogreSubMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
          else
            gzerr << "Unknown primitive type["
                  << subMesh.GetPrimitiveType() << "]\n";

          ogreSubMesh->vertexData = new Ogre::VertexData();
          vertexData = ogreSubMesh->vertexData;
          vertexDecl = vertexData->vertexDeclaration;

          // The vertexDecl should contain positions, blending weights, normals,
          // diffiuse colors, specular colors, tex coords. In that order.
          vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                                Ogre::VES_POSITION);
          currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

          // TODO: blending weights

          // normals
          if (subMesh.GetNormalCount() > 0)
          {
            vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                                  Ogre::VES_NORMAL);
            currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
          }

          // TODO: diffuse colors

          // TODO: specular colors

          // two dimensional texture coordinates
          if (subMesh.GetTexCoordCount() > 0)
          {
            vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                Ogre::VES_TEXTURE_COORDINATES, 0);
            currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
          }

          // allocate the vertex buffer
          vertexData->vertexCount = subMesh.GetVertexCount();

          // See documentation for usage:
          // https://ogrecave.github.io/ogre/api/latest/class_ogre_1_1_hardware_buffer_manager.html#a6009815139b407d20347dd3e2262e5ad
          //
          // Since we intend to read data back from the vertex buffer:
          //
          // usage = Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE
          // useShadowBuffer = true
          vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                    vertexDecl->getVertexSize(0),
                    vertexData->vertexCount,
                    Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE,
                    true); 

          vertexData->vertexBufferBinding->setBinding(0, vBuf);
          vertices = static_cast<float*>(vBuf->lock(
                          Ogre::HardwareBuffer::HBL_DISCARD));

          if (_mesh->HasSkeleton())
          {
            common::Skeleton *skel = _mesh->GetSkeleton();
            for (unsigned int j = 0; j < subMesh.GetNodeAssignmentsCount(); j++)
            {
              common::NodeAssignment na = subMesh.GetNodeAssignment(j);
              Ogre::VertexBoneAssignment vba;
              vba.vertexIndex = na.vertexIndex;
              vba.boneIndex = ogreSkeleton->getBone(skel->GetNodeByHandle(
                                  na.nodeIndex)->GetName())->getHandle();
              vba.weight = na.weight;
              ogreSubMesh->addBoneAssignment(vba);
            }
          }

          // allocate index buffer
          ogreSubMesh->indexData->indexCount = subMesh.GetIndexCount();

          ogreSubMesh->indexData->indexBuffer =
            Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
                Ogre::HardwareIndexBuffer::IT_32BIT,
                ogreSubMesh->indexData->indexCount,
                Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                false);

          iBuf = ogreSubMesh->indexData->indexBuffer;
          indices = static_cast<uint32_t*>(
              iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

          unsigned int j;

          // Add all the vertices
          for (j = 0; j < subMesh.GetVertexCount(); j++)
          {
            *vertices++ = subMesh.Vertex(j).X();
            *vertices++ = subMesh.Vertex(j).Y();
            *vertices++ = subMesh.Vertex(j).Z();

            if (subMesh.GetNormalCount() > 0)
            {
              *vertices++ = subMesh.Normal(j).X();
              *vertices++ = subMesh.Normal(j).Y();
              *vertices++ = subMesh.Normal(j).Z();
            }

            if (subMesh.GetTexCoordCount() > 0)
            {
              *vertices++ = subMesh.TexCoord(j).X();
              *vertices++ = subMesh.TexCoord(j).Y();
            }
          }

          // Add all the indices
          for (j = 0; j < subMesh.GetIndexCount(); j++)
            *indices++ = subMesh.GetIndex(j);

          const common::Material *material;
          material = _mesh->GetMaterial(subMesh.GetMaterialIndex());
          if (material)
          {
            rendering::Material::Update(material);
            ogreSubMesh->setMaterialName(material->GetName());
          }
          else
          {
            ogreSubMesh->setMaterialName("Gazebo/White");
          }

          // Unlock
          vBuf->unlock();
          iBuf->unlock();
        }

        ignition::math::Vector3d max = _mesh->Max();
        ignition::math::Vector3d min = _mesh->Min();

        if (_mesh->HasSkeleton())
        {
          min = ignition::math::Vector3d(-1, -1, -1);
          max = ignition::math::Vector3d(1, 1, 1);
        }

        if (!max.IsFinite())
          gzthrow("Max bounding box is not finite[" << max << "]\n");

        if (!min.IsFinite())
          gzthrow("Min bounding box is not finite[" << min << "]\n");

        ogreMesh->_setBounds(Ogre::AxisAlignedBox(
              Ogre::Vector3(min.X(), min.Y(), min.Z()),
              Ogre::Vector3(max.X(), max.Y(), max.Z())),
              false);

        // this line makes clear the mesh is loaded (avoids memory leaks)
        ogreMesh->load();
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Unable to insert mesh[" << e.getDescription() << "]" << std::endl;
      }
    }


  } // namespace rendering
} // namespace gazebo

///////////////////////////////////////////////////////////////////////////////
