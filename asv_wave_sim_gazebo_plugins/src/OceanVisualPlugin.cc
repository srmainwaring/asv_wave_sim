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
    public: rendering::VisualPtr visual;

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
    this->data->visual = _visual;
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
      std::string visualName = this->data->visual->Name() + "_OCEAN_VISUAL";

      // typedef std::shared_ptr<OceanVisual> OceanVisualPtr;
      this->data->oceanVisual.reset(new OceanVisual(visualName, this->data->visual));
      this->data->oceanVisual->Load(/*this->data->sdf*/);

      // Cascade this visuals material 
      this->data->oceanVisual->SetMaterial(this->data->visual->GetMaterialName());

      this->data->isInitialised = true;
    }

    gzmsg << "Done initializing OceanVisualPlugin." << std::endl;
  }

  void OceanVisualPlugin::Reset()
  {
    std::lock_guard<std::mutex> lock(this->data->mutex);
  }

} // namespace asv
