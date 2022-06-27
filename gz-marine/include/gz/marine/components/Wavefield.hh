
#ifndef GZ_MARINE_COMPONENTS_WAVEFIELD_HH_
#define GZ_MARINE_COMPONENTS_WAVEFIELD_HH_

#include "gz/marine/Wavefield.hh"

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Conversions.hh>

namespace gz
{
namespace marine
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_MARINE_VERSION_NAMESPACE {
namespace components
{
  /// \brief This component holds an entity's wavefield.
  using Wavefield = gz::sim::components::Component<marine::WavefieldPtr, class WavefieldTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_marine_components.Wavefield", Wavefield)
}
}
}
}

#endif
