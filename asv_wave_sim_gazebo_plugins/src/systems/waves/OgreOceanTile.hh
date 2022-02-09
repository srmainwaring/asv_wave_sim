

#ifndef IGNITION_GAZEBO_SYSTEMS_OGREOCEANTILE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OGREOCEANTILE_HH_

#include <ignition/rendering.hh>

#include <memory>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

  class OgreOceanTilePrivate;

  class OgreOceanTile
  {
    public: virtual ~OgreOceanTile();

    public: OgreOceanTile(size_t _N, double _L, bool _hasVisuals=true);

    void Create();

    void Update(double _time);

    private: std::unique_ptr<OgreOceanTilePrivate> dataPtr;
  };

}
}
}

#endif
