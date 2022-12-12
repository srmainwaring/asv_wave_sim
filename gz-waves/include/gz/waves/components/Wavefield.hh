
#ifndef GZ_WAVES_COMPONENTS_WAVEFIELD_HH_
#define GZ_WAVES_COMPONENTS_WAVEFIELD_HH_

#include "gz/waves/Wavefield.hh"

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Conversions.hh>

namespace gz
{
namespace waves
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_WAVES_VERSION_NAMESPACE {
namespace components
{
  /// \brief This component holds an entity's wavefield.
  using Wavefield = gz::sim::components::Component<
      waves::WavefieldConstWeakPtr, class WavefieldTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_waves_components.Wavefield", Wavefield)
}
}
}
}

#endif
