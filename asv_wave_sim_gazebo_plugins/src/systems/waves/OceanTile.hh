

#ifndef IGNITION_GAZEBO_SYSTEMS_OCEANTILE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OCEANTILE_HH_

#include <ignition/common.hh>
#include <ignition/rendering.hh>

#include <memory>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

  class OceanTilePrivate;

  class OceanTile
  {
    public: virtual ~OceanTile();

    public: OceanTile(size_t _N, double _L, bool _hasVisuals=true);

    public: void SetWindVelocity(double _ux, double _uy);

    public: void Create();

    public: void Update(double _time);

    public: common::Mesh * Mesh();

    private: std::unique_ptr<OceanTilePrivate> dataPtr;
  };

}
}
}

#endif
