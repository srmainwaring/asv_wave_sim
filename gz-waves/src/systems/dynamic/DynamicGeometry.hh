
#ifndef GZ_SIM_SYSTEMS_DYNAMICGEOMETRY_HH_
#define GZ_SIM_SYSTEMS_DYNAMICGEOMETRY_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  struct CubeVertices;
  class DynamicGeometryPrivate;

  /// \brief A plugin to demonstrate using Ogre2 for dynamic geometry
  class DynamicGeometry
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: DynamicGeometry();

    /// \brief Destructor
    public: ~DynamicGeometry() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<DynamicGeometryPrivate> dataPtr;
  };
  }
}
}
}

#endif
