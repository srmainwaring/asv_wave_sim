

#ifndef IGNITION_RENDERING_OGRE2_OGRE2OCEANVISUAL_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2OCEANVISUAL_HH_

#include "ignition/rendering/base/BaseOceanVisual.hh"

#include <ignition/rendering/config.hh>
#include <ignition/rendering/ogre2/Ogre2Visual.hh>

#include <memory>

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

    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2OceanVisual :
      public BaseOceanVisual<Ogre2Visual>
    {
      /// \brief Constructor
      protected: Ogre2OceanVisual();

      /// \brief Destructor
      public: virtual ~Ogre2OceanVisual();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      protected: virtual void Destroy() override;

      /// \brief Create the Light Visual in Ogre
      public: void CreateVisual();

      // Documentation inherited
      public: virtual VisualPtr SphereVisual() const override;

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(
        MaterialPtr _material, bool _unique) override;

      /// \brief Set material to grid geometry.
      /// \param[in] _material Ogre material.
      protected: virtual void SetMaterialImpl(Ogre2MaterialPtr _material);

      private: friend class Ogre2Scene;

      /// \brief Private data class
      private: std::unique_ptr<Ogre2OceanVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
