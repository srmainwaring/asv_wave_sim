#ifndef IGNITION_RENDERING_OGRE2_OGRE2OCEANVISUAL_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2OCEANVISUAL_HH_

#include <memory>

#include "ignition/rendering/base/BaseVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"

namespace Ogre
{
  class MovableObject;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

    // Forward declaration
    class Ogre2OceanVisualPrivate;

    /// \brief Ogre2.x implementation of an ocean visual class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2OceanVisual :
      public Ogre2Visual
    {
      /// \brief Constructor
      // protected: Ogre2OceanVisual();
      public: Ogre2OceanVisual();

      /// \brief Destructor
      public: virtual ~Ogre2OceanVisual();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      protected: virtual void Destroy() override;

      /// \brief Load the dynamic renderable
      public: void Load2();

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(
        MaterialPtr _material, bool _unique) override;

      /// \brief Set material to geometry.
      /// \param[in] _material Ogre material.
      protected: virtual void SetMaterialImpl(Ogre2MaterialPtr _material);

      /// \brief Work-around the protected accessors and protected methods in Scene
      public: void InitObject(Ogre2ScenePtr _scene,
          unsigned int _id, const std::string &_name);

      private: friend class Ogre2Scene;

      /// \brief Private data class
      private: std::unique_ptr<Ogre2OceanVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
