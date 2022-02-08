#ifndef IGNITION_RENDERING_OCEANVISUAL_HH_
#define IGNITION_RENDERING_OCEANVISUAL_HH_

#include <ignition/math/Inertial.hh>
#include <ignition/rendering/config.hh>
#include <ignition/rendering/Object.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/Visual.hh>

#include <string>

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

    /// \class OceanVisual OceanVisual.hh
    /// ignition/rendering/OceanVisual.hh
    /// \brief Represents a center of mass visual
    class IGNITION_RENDERING_VISIBLE OceanVisual :
      public virtual Visual
    {
      /// \brief Destructor
      public: virtual ~OceanVisual() {}

      /// \brief Set the inertial component of the visual
      /// \param[in] _inertial Inertial component of the visual
      public: virtual void SetInertial(
                  const ignition::math::Inertiald &_inertial) = 0;

      /// \brief Set the mass of the parent
      /// \param[in] _mass Parent mass
      public: virtual void SetMass(double _mass) = 0;

      /// \brief Get the mass of the parent
      /// \return Parent mass
      public: virtual double Mass() const = 0;

      /// \brief Get the inertia pose
      /// \return Inertia pose in parent frame.
      public: virtual ignition::math::Pose3d InertiaPose() const = 0;

      /// \brief Get the sphere visual
      /// \return Pointer to the sphere visual
      public: virtual VisualPtr SphereVisual() const = 0;
    };
    }
  }
}
#endif