#include "asv_wave_sim_gazebo_plugins/OceanVisualPlugin.hh"
#include "asv_wave_sim_gazebo_plugins/OceanVisual.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>

#include <ignition/math/Vector3.hh>

#include <memory>
#include <thread>
#include <vector>

using namespace gazebo;

namespace asv
{
  GZ_REGISTER_VISUAL_PLUGIN(OceanVisualPlugin)

///////////////////////////////////////////////////////////////////////////////
// OceanVisualPluginPrivate

  class OceanVisualPluginPrivate
  {
    /// \brief The visual containing this plugin.
    public: rendering::VisualPtr parentVisual;

    /// \brief The visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief The ocean visual to load.
    public: std::shared_ptr<OceanVisual> oceanVisual;

    /// \brief Prevent multiple calls to Init.
    public: bool isInitialised = false;

    /// \brief Mutex
    public: std::mutex mutex;
  };

///////////////////////////////////////////////////////////////////////////////
// OceanVisualPlugin

  OceanVisualPlugin::~OceanVisualPlugin()
  {
  }

  OceanVisualPlugin::OceanVisualPlugin() :
    VisualPlugin(),
    data(new OceanVisualPluginPrivate)
  {
  }

  void OceanVisualPlugin::Load(
    rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
  {
    std::lock_guard<std::mutex> lock(this->data->mutex);

    gzmsg << "Loading OceanVisualPlugin..." << std::endl;

    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->parentVisual = _visual;
    this->data->sdf = _sdf;

    gzmsg << "Done loading OceanVisualPlugin." << std::endl;
  }

  void OceanVisualPlugin::Init()
  {
    std::lock_guard<std::mutex> lock(this->data->mutex);

    gzmsg << "Initializing OceanVisualPlugin..." << std::endl;

    if (!this->data->isInitialised)
    {
      // Load the shader visual
      std::string visualName = this->data->parentVisual->Name() + "_OCEAN_VISUAL";

      // typedef std::shared_ptr<OceanVisual> OceanVisualPtr;
      this->data->oceanVisual.reset(new OceanVisual(visualName, this->data->parentVisual));
      this->data->oceanVisual->Load(/*this->data->sdf*/);

      // Cascade this visuals material
      // This will cascade to children (and overwite any value we might assign there)
      // this->data->oceanVisual->SetMaterial(this->data->parentVisual->GetMaterialName());

#if DEBUG
      gzmsg << "OceanVisualPlugin::ParentVisual..." << std::endl;
      gzmsg << "Name: "                 << this->data->parentVisual->Name() << std::endl;
      gzmsg << "Id: "                   << this->data->parentVisual->GetId() << std::endl;
      gzmsg << "MaterialName: "         << this->data->parentVisual->GetMaterialName() << std::endl;
      gzmsg << "MeshName: "             << this->data->parentVisual->GetMeshName() << std::endl;
      gzmsg << "ShaderType: "           << this->data->parentVisual->GetShaderType() << std::endl;
      gzmsg << "AttachedObjectCount: "  << this->data->parentVisual->GetAttachedObjectCount() << std::endl;

      gzmsg << "OceanVisual..." << std::endl;
      gzmsg << "Name: "                 << this->data->oceanVisual->Name() << std::endl;
      gzmsg << "Id: "                   << this->data->oceanVisual->GetId() << std::endl;
      gzmsg << "MaterialName: "         << this->data->oceanVisual->GetMaterialName() << std::endl;
      gzmsg << "MeshName: "             << this->data->oceanVisual->GetMeshName() << std::endl;
      gzmsg << "ShaderType: "           << this->data->oceanVisual->GetShaderType() << std::endl;
      gzmsg << "AttachedObjectCount: "  << this->data->oceanVisual->GetAttachedObjectCount() << std::endl;
#endif

      this->data->isInitialised = true;
    }

    gzmsg << "Done initializing OceanVisualPlugin." << std::endl;
  }

  void OceanVisualPlugin::Reset()
  {
    std::lock_guard<std::mutex> lock(this->data->mutex);
  }

} // namespace asv
