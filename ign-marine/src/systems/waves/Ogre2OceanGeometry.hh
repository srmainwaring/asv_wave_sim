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
#ifndef IGNITION_RENDERING_OGRE2_OGRE2OCEANGEOMETRY_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2OCEANGEOMETRY_HH_

#include "Ogre2DynamicMesh.hh"

#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Scene.hh"

#include <memory>

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class Ogre2OceanGeometryPrivate;

    /*  \class Ogre2OceanGeometry Ogre2OceanGeometry.hh \
     *  ignition/rendering/ogre2/Ogre2OceanGeometry.hh
     */
    /// \brief Ocean geometry class based on a dynamic mesh
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2OceanGeometry :
        public Ogre2DynamicMesh
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to scene
      public: explicit Ogre2OceanGeometry(ScenePtr _scene);

      /// \brief Virtual destructor
      public: virtual ~Ogre2OceanGeometry();



      /// \brief Pointer to private data
      private: std::unique_ptr<Ogre2OceanGeometryPrivate> dataPtr;
    };
    }
  }
}
#endif
