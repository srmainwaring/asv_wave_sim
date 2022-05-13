#include "gz/marine/OceanVisualPlugin.hh"
#include "gz/marine/OceanVisual.hh"

#include <gz/common.hh>
#include <gz/rendering/Visual.hh>

#include <gz/math/Vector3.hh>

#include <memory>
#include <thread>
#include <vector>

namespace ignition
{
namespace marine
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

    ignmsg << "Loading OceanVisualPlugin..." << std::endl;

    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->parentVisual = _visual;
    this->data->sdf = _sdf;

    ignmsg << "Done loading OceanVisualPlugin." << std::endl;
  }

  void OceanVisualPlugin::Init()
  {
    std::lock_guard<std::mutex> lock(this->data->mutex);

    ignmsg << "Initializing OceanVisualPlugin..." << std::endl;

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
      ignmsg << "OceanVisualPlugin::ParentVisual..." << std::endl;
      ignmsg << "Name: "                 << this->data->parentVisual->Name() << std::endl;
      ignmsg << "Id: "                   << this->data->parentVisual->GetId() << std::endl;
      ignmsg << "MaterialName: "         << this->data->parentVisual->GetMaterialName() << std::endl;
      ignmsg << "MeshName: "             << this->data->parentVisual->GetMeshName() << std::endl;
      ignmsg << "ShaderType: "           << this->data->parentVisual->GetShaderType() << std::endl;
      ignmsg << "AttachedObjectCount: "  << this->data->parentVisual->GetAttachedObjectCount() << std::endl;

      ignmsg << "OceanVisual..." << std::endl;
      ignmsg << "Name: "                 << this->data->oceanVisual->Name() << std::endl;
      ignmsg << "Id: "                   << this->data->oceanVisual->GetId() << std::endl;
      ignmsg << "MaterialName: "         << this->data->oceanVisual->GetMaterialName() << std::endl;
      ignmsg << "MeshName: "             << this->data->oceanVisual->GetMeshName() << std::endl;
      ignmsg << "ShaderType: "           << this->data->oceanVisual->GetShaderType() << std::endl;
      ignmsg << "AttachedObjectCount: "  << this->data->oceanVisual->GetAttachedObjectCount() << std::endl;
#endif

      this->data->isInitialised = true;
    }

    ignmsg << "Done initializing OceanVisualPlugin." << std::endl;
  }

  void OceanVisualPlugin::Reset()
  {
    std::lock_guard<std::mutex> lock(this->data->mutex);
  }

}
}
