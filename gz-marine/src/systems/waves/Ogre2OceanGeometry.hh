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
#ifndef GZ_RENDERING_OGRE2_OGRE2OCEANGEOMETRY_HH_
#define GZ_RENDERING_OGRE2_OGRE2OCEANGEOMETRY_HH_

#include <gz/rendering/ogre2/Export.hh>
#include <gz/rendering/ogre2/Ogre2Geometry.hh>
#include <gz/rendering/RenderTypes.hh>
// #include <gz/rendering/Scene.hh>

#include <memory>

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class Ogre2OceanGeometryPrivate;

    /*  \class Ogre2OceanGeometry Ogre2OceanGeometry.hh \
     *  gz/rendering/ogre2/Ogre2OceanGeometry.hh
     */
    /// \brief Ocean geometry class based on a dynamic mesh
    class GZ_RENDERING_OGRE2_VISIBLE Ogre2OceanGeometry :
        public Ogre2Geometry
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to scene
      public: explicit Ogre2OceanGeometry();

      /// \brief Virtual destructor
      public: virtual ~Ogre2OceanGeometry();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual Ogre::MovableObject *OgreObject() const override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void
        SetMaterial(MaterialPtr _material, bool _unique) override;

      /// \brief Load from a mesh
      public: void LoadMesh(gz::common::MeshPtr _mesh);

      /// \brief Update from a mesh
      public: void UpdateMesh(gz::common::MeshPtr _mesh);

      /// \brief Work-around the protected accessors and protected methods in Scene
      public: void InitObject(Ogre2ScenePtr _scene,
          unsigned int _id, const std::string &_name);

      /// \brief OceanGeometry should be created by scene.
      private: friend class Ogre2Scene;

      /// \brief Pointer to private data
      private: std::unique_ptr<Ogre2OceanGeometryPrivate> dataPtr;
    };

    typedef std::shared_ptr<Ogre2OceanGeometry> Ogre2OceanGeometryPtr;

    }
  }
}
#endif
