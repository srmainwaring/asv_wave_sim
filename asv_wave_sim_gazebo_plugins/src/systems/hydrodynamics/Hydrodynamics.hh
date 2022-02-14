


#ifndef IGNITION_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_
#define IGNITION_GAZEBO_SYSTEMS_HYDRODYNAMICS_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class HydrodynamicsPrivate;

  /// \brief A plugin for surface hydrodynamics
  class Hydrodynamics
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: Hydrodynamics();

    /// \brief Destructor
    public: ~Hydrodynamics() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<HydrodynamicsPrivate> dataPtr;
  };
  }
}
}
}

#endif
