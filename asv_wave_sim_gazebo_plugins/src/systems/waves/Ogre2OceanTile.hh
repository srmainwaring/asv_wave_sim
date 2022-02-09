

#ifndef IGNITION_GAZEBO_SYSTEMS_OGRE2OCEANTILE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OGRE2OCEANTILE_HH_

#include <ignition/rendering.hh>

#include <memory>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

  class Ogre2OceanTilePrivate;

  class Ogre2OceanTile
  {
    public: virtual ~Ogre2OceanTile();

    public: Ogre2OceanTile(size_t _N, double _L, bool _hasVisuals=true);

    void SetWindVelocity(double _ux, double _uy);

    void Create();

    void Update(double _time);

    private: std::unique_ptr<Ogre2OceanTilePrivate> dataPtr;
  };

}
}
}

#endif
