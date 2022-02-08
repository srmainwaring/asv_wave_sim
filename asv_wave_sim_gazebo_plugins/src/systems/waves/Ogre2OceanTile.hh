

#ifndef IGNITION_GAZEBO_SYSTEMS_OGRE2OCEANTILE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OGRE2OCEANTILE_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

  class Ogre2OceanTilePrivate;

  class Ogre2OceanTile
  {
    public: virtual ~Ogre2OceanTile();

    public: Ogre2OceanTile();

    void Update();

    private: std::unique_ptr<Ogre2OceanTilePrivate> dataPtr;
  };

}
}
}

#endif
