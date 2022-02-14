


#ifndef IGNITION_GAZEBO_SYSTEMS_WAVESMODEL_HH_
#define IGNITION_GAZEBO_SYSTEMS_WAVESMODEL_HH_

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
  class WavesModelPrivate;

  /// \brief A plugin for surface waves
  class WavesModel
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: WavesModel();

    /// \brief Destructor
    public: ~WavesModel() override;

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
    private: std::unique_ptr<WavesModelPrivate> dataPtr;
  };
  }
}
}
}

#endif
