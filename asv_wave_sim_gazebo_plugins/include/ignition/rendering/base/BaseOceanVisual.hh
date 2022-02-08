#ifndef IGNITION_RENDERING_BASE_BASEOCEANVISUAL_HH_
#define IGNITION_RENDERING_BASE_BASEOCEANVISUAL_HH_

#include "ignition/rendering/OceanVisual.hh"

#include <ignition/common/Console.hh>
#include <ignition/rendering/config.hh>
#include <ignition/rendering/base/BaseObject.hh>
#include <ignition/rendering/base/BaseRenderTypes.hh>
#include <ignition/rendering/Scene.hh>

#include <string>

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of an ocean visual
    template <class T>
    class BaseOceanVisual :
      public virtual OceanVisual,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseOceanVisual();

      /// \brief Destructor
      public: virtual ~BaseOceanVisual();

      // Documentation inherited.
      protected: virtual void Init() override;

      // Documentation inherited.
      protected: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void SetInertial(
                  const ignition::math::Inertiald &_inertial) override;

      // Documentation inherited.
      public: virtual void SetMass(double _mass) override;

      // Documentation inherited
      public: virtual double Mass() const override;

      // Documentation inherited
      public: virtual ignition::math::Pose3d InertiaPose() const override;

      // Documentation inherited
      public: virtual VisualPtr SphereVisual() const override;

      /// \brief Get the radius of the CoM sphere
      /// \return Radius of the CoM sphere
      protected: double SphereRadius() const;

      /// \brief Parent visual name.
      protected: std::string parentName = "";

      /// \brief Parent mass.
      protected: double mass = 1.0;

      /// \brief Inertia pose in parent frame.
      protected: ignition::math::Pose3d inertiaPose =
          ignition::math::Pose3d::Zero;

      /// \brief Flag to indicate parent properties have changed.
      protected: bool dirtyOceanVisual = false;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseOceanVisual<T>::BaseOceanVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseOceanVisual<T>::~BaseOceanVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseOceanVisual<T>::PreRender()
    {
      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseOceanVisual<T>::Init()
    {
      T::Init();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseOceanVisual<T>::SetInertial(
          const ignition::math::Inertiald &_inertial)
    {
      this->inertiaPose = _inertial.Pose();

      this->SetMass(_inertial.MassMatrix().Mass());
    }

    template <class T>
    void BaseOceanVisual<T>::SetMass(double _mass)
    {
      if (_mass <= 0)
      {
        // Unrealistic mass, load with default mass
        if (_mass < 0)
        {
          ignlog << "The parent " << this->parentName
              << " has unrealistic mass, "
              << "unable to visualize sphere of equivalent mass.\n";
        }
        else
        {
          ignlog << "The parent " << this->parentName
              << " is static or has mass of 0, "
              << "so a sphere of equivalent mass will not be shown.\n";
        }
        return;
      }

      this->mass = _mass;
      this->dirtyOceanVisual = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseOceanVisual<T>::Mass() const
    {
      return this->mass;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Pose3d BaseOceanVisual<T>::InertiaPose() const
    {
      return this->inertiaPose;
    }

    //////////////////////////////////////////////////
    template <class T>
    VisualPtr BaseOceanVisual<T>::SphereVisual() const
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseOceanVisual<T>::SphereRadius() const
    {
      // Compute radius of sphere with density of lead and equivalent mass.
      double sphereRadius;
      double densityLead = 11340;
      sphereRadius = cbrt((0.75 * this->Mass()) / (IGN_PI * densityLead));

      return sphereRadius;
    }
    }
  }
}
#endif
