
#ifndef GZ_MARINE_COMPONENTS_WAVEFIELD_HH_
#define GZ_MARINE_COMPONENTS_WAVEFIELD_HH_

#include "gz/marine/Wavefield.hh"

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Conversions.hh>

namespace ignition
{
namespace marine
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_MARINE_VERSION_NAMESPACE {
namespace components
{
  /// \brief This component holds an entity's wavefield.
  using Wavefield = gazebo::components::Component<marine::WavefieldPtr, class WavefieldTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_marine_components.Wavefield", Wavefield)
}
}
}
}

#endif
