// Copyright (C) 2022  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "LinearWaveBody.hh"

#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/msgs/wrench.pb.h>

#include <gz/math/Matrix6.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport.hh>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>

#include <Eigen/Dense>
#include <highfive/H5File.hpp>

#include <chrono>
#include <list>
#include <mutex>
#include <vector>
#include <string>

#include <mlinterp>

using namespace gz;
using namespace sim;
using namespace systems;

namespace Eigen
{
  using Vector6d = Matrix<double, 6, 1>;
  using Matrix6d = Matrix<double, 6, 6>;
}

namespace
{
  /// \brief use stream buffers to populate a vector from a whitespace
  /// delimited string of numbers.
  /// https://stackoverflow.com/questions/4423361/constructing-a-vector-with-istream-iterators
  /// https://stackoverflow.com/questions/2275135/splitting-a-string-by-whitespace-in-c
  std::vector<double> whitespaceDelimitedStringToDoubleVector(
      const std::string &_input)
  { 
    // tokenise
    std::istringstream buffer(_input);
    std::vector<std::string> tokens(
        (std::istream_iterator<std::string>(buffer)), 
        std::istream_iterator<std::string>());

    std::vector<double> ret(tokens.size());
    for (size_t i=0; i<tokens.size(); ++i)
      ret[i] = std::stod(tokens[i]);

    return ret;
  }

  template<typename T>
  T SdfGet(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key)
  {
    return _sdf->Get<T>(_key);
  }

  /// \brief template specialisation for std::vector<double>. 
  ///
  /// Parse data a whitespace delimited string (commas will be ignored in XML).
  template<>
  std::vector<double> SdfGet<std::vector<double>>(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key)
  {
    auto str = _sdf->Get<std::string>(_key);
    std::vector<double> ret = whitespaceDelimitedStringToDoubleVector(str);    
    return ret;
  }

  /// \brief template specialisation for Eigen::Matrix6d. 
  ///
  /// Parse data a whitespace delimited string (commas will be ignored in XML).
  /// Eigen assumes a vector contains successive columns, so transpose.
  template<>
  Eigen::Matrix6d SdfGet<Eigen::Matrix6d>(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key)
  {
    auto str = _sdf->Get<std::string>(_key);
    std::vector<double> v = whitespaceDelimitedStringToDoubleVector(str);
    Eigen::Matrix6d ret(v.data());
    return ret.transpose();
  }

  /// \brief template specialisation for Eigen::Vector6d. 
  ///
  /// Parse data a whitespace delimited string (commas will be ignored in XML).
  template<>
  Eigen::Vector6d SdfGet<Eigen::Vector6d>(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key)
  {
    auto str = _sdf->Get<std::string>(_key);
    std::vector<double> v = whitespaceDelimitedStringToDoubleVector(str);
    Eigen::Vector6d ret(v.data());
    return ret;
  }
}

class gz::sim::systems::LinearWaveBodyPrivate
{
  /// \todo List of items to investigate or fix
  ///
  /// 1. Linear displacements should be calculated for the origin of the
  ///    floating body waterplane. Done.
  ///
  /// 2. Improve pose notation to emphasise frame
  ///    (e.g. Peter Corke's notation).
  ///    Use Kane/monogram (Drake) notation. Done
  ///
  /// 3. Set waterplane pose in parameters. Done.
  ///
  /// 4. Remove hardcoding of 1 heading for excitation data.
  ///
  /// 5. Add support for finding file in model:// or package:// dirs. Done.
  ///
  /// 6. Refactor code to read HDF5 files. Removing duplicated code. Done.
  ///
  /// 7. Improve handing of overrides. This should happen once in config
  ///    not in the update loop. Done.
  ///
  /// 8. Review and document assunmptions about what quantities are treated
  ///    as constants in the linear potential wave-body model.
  ///
  ///    Hydrostatics
  ///     - cg and cb are supplied in the body frame at equilibrium position
  ///       are not updated as the body pose alters, or if the body is given
  ///       an initial pose.
  ///     - the body waterplane origin is located on the free surface. 
  ///     - the displacements used in the hydrostatics restoring force are
  ///       calculated in the world frame. The reference is the initial
  ///       position of origin of the body waterplane.
  ///
  /// 9. Simplify data structures for debugging, publishing forces etc.
  ///       map[forceName] -> { enable, debug, publish, topic, ... }
  ///    and perhaps a bitmasks for switching features on/off
  ///
  /// 10. Interpolate hdf5 date for constant coefficient case. Done.
  ///
  /// 11. Optimise interpolation to eliminate unnecessary copies.
  ///
  ///

  /// \brief WEC-Sim BEMIO hydro data structure (read from HDF5 file)
  ///
  /// For details see:
  /// http://wec-sim.github.io/WEC-Sim/master/user/advanced_features.html#bemio-hydro-data-structure
  ///
  /// This is the raw data loaded from file. It is not accessed directly in
  /// the simulation updates.
  ///
  /// The hydro coefficients are assumed to be non-dimensional.
  ///
  struct HydroData
  {
    /// \brief Hydro data hdf5 file.
    public: std::string hdf5File;

    /// \brief Infinite frequency radiation added mass [Ndof, Ndof]
    Eigen::Matrix6d Ainf = Eigen::Matrix6d::Zero();

    /// \brief Radiation added mass [Ndof, Ndof, Nf]
    std::vector<Eigen::Matrix6d> A;

    /// \brief Radiation wave damping [Ndof, Ndof, Nf]
    std::vector<Eigen::Matrix6d> B;

    /// \brief Radiation IRF [Ndof, Ndof, len(ra_t)]
    std::vector<Eigen::Matrix6d> ra_K;

    /// \brief Radiation IRF time steps
    std::vector<double> ra_t;

    /// \brief Radiation IRF temporal angular frequency steps
    std::vector<double> ra_w;

    /// \todo remove hardcoding of 1 heading for excitation data

    /// \brief Excitation force [Ndof, Nh, Nf]
    std::vector<Eigen::Matrix<double, 6, 1>> ex_re;
    std::vector<Eigen::Matrix<double, 6, 1>> ex_im;
    std::vector<Eigen::Matrix<double, 6, 1>> ex_ma;
    std::vector<Eigen::Matrix<double, 6, 1>> ex_ph;

    /// \brief Froude-Krylov component of excitation force [Ndof, Nh, Nf]
    std::vector<Eigen::Matrix<double, 6, 1>> fk_re;
    std::vector<Eigen::Matrix<double, 6, 1>> fk_im;
    std::vector<Eigen::Matrix<double, 6, 1>> fk_ma;
    std::vector<Eigen::Matrix<double, 6, 1>> fk_ph;

    /// \brief Scattering component of excitation force [Ndof, Nh, Nf]
    std::vector<Eigen::Matrix<double, 6, 1>> sc_re;
    std::vector<Eigen::Matrix<double, 6, 1>> sc_im;
    std::vector<Eigen::Matrix<double, 6, 1>> sc_ma;
    std::vector<Eigen::Matrix<double, 6, 1>> sc_ph;

    /// \brief Excitation IRF [Ndof, Nh, len(ex_t)]
    std::vector<Eigen::Matrix<double, 6, 1>> ex_K;

    /// \brief Excitation IRF time steps
    std::vector<double> ex_t;

    /// \brief Excitation IRF temporal angular frequency steps
    std::vector<double> ex_w;

    /// \brief Hydrostatic linear restoring stiffness [6, 6]
    Eigen::Matrix6d K_hs = Eigen::Matrix6d::Zero();

    // Properties

    /// \brief Number of bodies
    double Nb{1};

    /// \brief Number of wave frequencies
    double Nf{0};

    /// \brief Number of wave headings
    double Nh{1};

    /// \brief Center of buoyancy [3]
    Eigen::Vector3d cb = Eigen::Vector3d::Zero();

    /// \brief Center of gravity [3]
    Eigen::Vector3d cg = Eigen::Vector3d::Zero();

    /// \brief Displaced volume
    double Vo{0.0};
  
    // SimulationParameters

    /// \brief Wave periods [1, Nf]
    Eigen::VectorXd T;

    /// \brief Gravitational acceleration 
    double g{9.81};

    /// \brief Fluid density
    double rho{1025.0};

    /// \brief Are quantities non-dimensional?
    ///
    /// \note: not set correctly in HDF5 by bemio.
    double scaled{1};

    /// \brief Wave temporal angular frequencies [1, Nf]
    Eigen::VectorXd w;

    /// \brief Wave directions in degrees [1, Nh]
    Eigen::VectorXd theta;
  };

  /// \brief Dimensioned hydrodynamics force coefficients (constant).
  ///
  /// The force coefficients used in the simulation update.
  ///
  struct HydroForceCoeffs
  {
    /// \brief Hydrostatic linear restoring stiffness [6, 6]
    Eigen::Matrix6d K_hs = Eigen::Matrix6d::Zero();

    /// \brief Radiation added mass [Ndof, Ndof]
    Eigen::Matrix6d A = Eigen::Matrix6d::Zero();

    /// \brief Radiation damping [Ndof, Ndof]
    Eigen::Matrix6d B = Eigen::Matrix6d::Zero();

    /// \brief Combined excitation force [Ndof, Nh=1]
    Eigen::Vector6d ex_re = Eigen::Vector6d::Zero();
    Eigen::Vector6d ex_im = Eigen::Vector6d::Zero();

    /// \brief Froude-Krylov excitation force [Ndof, Nh=1]
    Eigen::Vector6d fk_re = Eigen::Vector6d::Zero();
    Eigen::Vector6d fk_im = Eigen::Vector6d::Zero();

    /// \brief Scattering excitation force [Ndof, Nh=1]
    Eigen::Vector6d sc_re = Eigen::Vector6d::Zero();
    Eigen::Vector6d sc_im = Eigen::Vector6d::Zero();
  };

  /// \brief Waves parameters (regular waves)
  ///
  /// Override priority (lowest number => highest priority)
  ///   1. SDF <waves>
  ///   4. Default
  ///
  /// \todo integrate with the waves plugin - details should be retrieved
  ///       from the waves entity.
  ///
  struct Waves
  {
    /// \brief Wave period (s)
    double period{6.0};

    /// \brief Wave height (m). Height = 2 * amplitude.
    double height{4.0};

    /// \brief Wave direction (rad). Zero is aligned with world x-axis.
    double direction{0.0};

    /// \brief Wave phase (rad).
    double phase{0.0};
  };

  /// \brief Simulation parameters.
  ///
  /// Override priority (lowest number => highest priority)
  ///   1. SDF <waves>
  ///   2. SDF <simulation_parameters>
  ///   3. SDF <hdf5_file>
  ///   4. Default
  ///
  struct SimulationParameters
  {
    double g{9.81};
    double rho{1025.0};
    double T{6.0};
    double w{2.0 * GZ_PI / T};
  };

  /// \brief Simulation parameters.
  ///
  /// Override priority (lowest number => highest priority)
  ///   1. SDF <geometry>
  ///   2. SDF <hdf5_file>
  ///   3. Default
  ///
  struct Geometry
  {
    /// \brief Position vector of the waterplane origin in body frame.
    public: gz::math::Vector3d p_BoBwp_B = gz::math::Vector3d::Zero;

    /// \brief Position vector of the center of buoyancy in body frame.
    public: gz::math::Vector3d p_BoBcb_B = gz::math::Vector3d::Zero;

    /// \brief Displaced volume.
    double Vo{0.0};
  };

  /// \brief Destructor
  public: ~LinearWaveBodyPrivate();

  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void Init(EntityComponentManager &_ecm);

  /// \brief Read a WECSim HDF5 BEM data file.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void ReadWECSim(EntityComponentManager &_ecm);

  /// \brief Update the physics and markers.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void Update(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update link state.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateLinkState(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update the gravity forces.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateGravityForces(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update the hydrostatic forces.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateHydrostaticForces(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update the radiation forces.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateRadiationForces(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update the radiation forces.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void RadiationForcesConstantCoefficients(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update the radiation forces.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void RadiationForcesImpulseResponseFunction(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Update the excitation forces.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateExcitationForces(
      const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Model interface
  public: sim::Model model{kNullEntity};

  /// \brief Link entity
  public: sim::Entity linkEntity{kNullEntity};

  /// \brief Static pose of the link waterplane in the link frame 
  public: gz::math::Pose3d X_BBwp = gz::math::Pose3d::Zero;

  /// \brief Initial pose of the body waterplane in the world frame 
  public: gz::math::Pose3d X0_WBwp = gz::math::Pose3d::Zero;

  /// \brief Link world linear velocity at previous time-step
  public: gz::math::Vector3d v_WB_W_prev = gz::math::Vector3d::Zero;

  /// \brief Link world angular velocity at previous time-step
  public: gz::math::Vector3d w_WB_W_prev = gz::math::Vector3d::Zero;

  /// \brief Link state to update each time-step.
  struct LinkState
  {
    gz::math::Pose3d X_WB;
    gz::math::Pose3d X_WBcm;
    gz::math::Pose3d X_WBwp;
    gz::math::Pose3d X_BBcm;
    gz::math::Pose3d X_BcmBwp;

    gz::math::Vector3d p_BoBcm_B; 
    gz::math::Vector3d p_BoBcm_W;

    gz::math::Vector3d p_BcmBcb_B;
    gz::math::Vector3d p_BcmBcb_W;

    gz::math::Vector3d p_BcmBwp_Bcm;
    gz::math::Vector3d p_BcmBwp_W;
  };

  /// \brief Common link state required by force calculations. 
  public: LinkState linkState;

  /// \brief Initialization flag
  public: bool initialized{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdf;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Name of the world
  public: std::string worldName;

  /// \brief Hydro data populated from hdf5 file.
  public: HydroData hydroData;

  /// \brief Waves parameters.
  public: Waves waves;

  /// \brief Simulation parameters.
  public: SimulationParameters simParams;

  /// \brief Geometry parameters.
  public: Geometry geometry;

  /// \brief Hydrostatic force constant coefficients used in sim update.
  public: HydroForceCoeffs hydroForceCoeffs;

  /// \brief Flags to enable forces.
  struct ForceFlags
  {
    bool gravityOn{true};
    bool buoyancyOn{true};
    bool hydrostaticRestoringOn{true};
    bool radiationDampingOn{true};
    bool radiationAddedMassOn{true};
    bool excitationOn{true};
    bool excitationFroudeKrylovOn{true};
    bool excitationScatteringOn{true};
  };
  public: ForceFlags forceFlags;

  /// \brief Flags to enable additional debug info.
  struct DebugFlags
  {
    bool gravityOn{false};
    bool buoyancyOn{false};
    bool radiationDampingOn{false};
    bool radiationAddedMassOn{false};
    bool excitationOn{false};
  };
  public: DebugFlags debugFlags;

  // Flags to note if SDF has overridden hdf5 data
  public: bool waveOverrideOn{false};
  public: bool simParamOverrideOn{false};
  public: bool geometryOverrideOn{false};
  public: bool hydrostaticOverrideOn{false};
  public: bool radiationOverrideOn{false};
  public: bool excitationOverrideOn{false};

  /// \brief Mutex to protect wave marker updates.
  public: std::recursive_mutex mutex;

  /// \brief Previous update time (s).
  public: double prevTime;

  /// \brief Publishers
  struct Publishers
  {
    double updateRate = 20; 

    // Enable / disable force publishers
    bool gravityOn{false};
    bool buoyancyOn{false};
    bool hydrostaticRestoringOn{false};
    bool radiationDampingOn{false};
    bool radiationAddedMassOn{false};
    bool excitationOn{false};
    bool excitationFroudeKrylovOn{false};
    bool excitationScatteringOn{false};
    
    // Topic strings
    std::string gravityTopic
        {"/force/gravity"};
    std::string buoyancyTopic
        {"/force/buoyancy"};
    std::string restoringTopic
        {"/force/restoring"};
    std::string radiationDampingTopic
        {"/force/radiation_damping"};
    std::string radiationAddedMassTopic
        {"/force/radiation_added_mass"};
    std::string excitationTopic
        {"/force/excitation"};
    std::string excitationFroudeKrylovTopic
        {"/force/excitation_froude_krylov"};
    std::string excitationScatteringTopic
        {"/force/excitation_scattering"};

    // Publishers
    transport::Node::Publisher gravityPub;
    transport::Node::Publisher buoyancyPub;
    transport::Node::Publisher restoringPub;
    transport::Node::Publisher radiationDampingPubl;
    transport::Node::Publisher radiationAddedMassPubl;
    transport::Node::Publisher excitationPub;
    transport::Node::Publisher excitationFroudeKrylovPubl;
    transport::Node::Publisher excitationScatteringPub;
  };

  /// \brief Publishers
  public: Publishers publishers;

  /// \brief Transport node for wave marker messages
  public: transport::Node node;
};

/////////////////////////////////////////////////
LinearWaveBody::LinearWaveBody() : System(),
    dataPtr(std::make_unique<LinearWaveBodyPrivate>())
{
}

/////////////////////////////////////////////////
LinearWaveBody::~LinearWaveBody()
{
}

namespace
{
  /// \todo Optimise to reduce / eliminate copies.
  ///       The inefficiency is because we read data 
  ///       in Eigen Vectors and Matrices and represent
  ///       the coefficients as a vector of matrices rathen
  ///       a matrix of time series suitable for interpolation.
  
  /// \brief Interpolation helpers
  ///
  /// \param _Xd data vector
  /// \param _Xi interpolation point
  /// \param _Yd data vector of matrix
  /// \param _Yi interpolation matrix
  template <typename Matrix>
  void Interp(
    const Eigen::VectorXd &_Xd,
    double _Xi,
    const std::vector<Matrix> &_Yd,
    Matrix *_Yi)
  {
    // check there is data to interpolate
    if (_Xd.size() == 0)
      return;

    // get dimensions
    size_t nf = _Xd.size();
    size_t n1 = _Yd[0].rows(); 
    size_t n2 = _Yd[0].cols(); 

    // convert to std::vector
    std::vector<double> Xdd;
    for (size_t k=0; k<nf; ++k)
    {
      Xdd.push_back(_Xd[k]);
    }

    // reshape to (n1 * n2) x 1 x Nf vectors for interpolation
    std::vector<std::vector<double>> Ydd(n1*n2);
    for (size_t i=0; i<n1; ++i)
    {
      for (size_t j=0; j<n2; ++j)
      {
        size_t idx = j*n1 + i;
        for (size_t k=0; k<nf; ++k)
        {
          auto& m = _Yd[k];
          double val = m(i, j);
          Ydd[idx].push_back(val);
        }
      }
    }

    // interpolate
    size_t ni = 1;
    double yi[] = { 0.0 };
    double xi[] = { _Xi };
    for (size_t i=0; i<n1; ++i)
    {
      for (size_t j=0; j<n2; ++j)
      {
        size_t idx = j*n1 + i;
        size_t nd[] = { nf };
        double* xd = Xdd.data();
        double* yd = Ydd[idx].data();
        mlinterp::interp(nd, ni, yd, yi, xd, xi);
        (*_Yi)(i, j) = yi[0];
      }
    }
  };
}

/////////////////////////////////////////////////
/// \brief Configure the model
///
/// The plugin reads in data from an hdf5 file. Defaults are used if a file 
/// file is not provided. A number of parameters may be overridden if options 
/// are set in the plugin XML. This is the load order.
///
/// 1. Read the <hdf5_file> into the struct HydroData.
///     - override defaults for g and rho in struct SimulationParameters
///     - override defaults for cg and Vo in struc Geometry
///
/// 2. Read the <waves> element
///     - override defaults in struct Waves
///     - override defaults for T and w in struct SimulationParameters 
/// 
/// 3. Read the <simulation_parameters> element
///     - override defaults in struct SimulationParameters
///
/// 4. Read the <geometry> element
///
/// 5. Look up the hydro coefficients from the hdf5 data
///     - \todo locate the index of w
///     - rescale the coefficients to dimensioned quantities  
///
/// 6. Read the <hydro_coeff> elements
///     - override hydrostatic, radiation and excitation coefficients
///       if elements are present.
///
void LinearWaveBody::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  GZ_PROFILE("LinearWaveBody::Configure");

  gzmsg << "LinearWaveBody: configuring\n";

  // Clone sdf for non-const access.
  this->dataPtr->sdf = _sdf->Clone();

  // Get the name of the world
  if (this->dataPtr->worldName.empty())
  {
    _ecm.Each<components::World, components::Name>(
      [&](const Entity &,
          const components::World *,
          const components::Name *_name) -> bool
      {
        // Assume there's only one world
        this->dataPtr->worldName = _name->Data();
        return false;
      });
  }

  // Capture the model entity
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The LinearWaveBody system should be attached to a model entity. "
          << "Failed to initialize." << "\n";
    return;
  }

  // Set the link entity
  std::string linkName("base_link");
  this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
  if (!_ecm.HasEntity(this->dataPtr->linkEntity))
  {
    gzerr << "Link name" << linkName << "does not exist";
    return;
  }

  /// \note Must enable world velocity and acceleration checks to register
  ///       components for pose, velocity and acceleration.
  ///       Link::AddWorldForce fails silently if not enabled.
  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  baseLink.EnableVelocityChecks(_ecm, true);
  baseLink.EnableAccelerationChecks(_ecm, true);

  // Parameters

  /// \todo set waterplane pose in parameters
  this->dataPtr->X_BBwp.Pos() = gz::math::Vector3d(0, 0, 0);

  /// \todo add support for finding file in model:// or package:// dirs.
  
  //////////
  // 1a. read WEC-Sim hdf5 BEM file
  if (_sdf->HasElement("hdf5_file"))
  {
    std::string fileUri = _sdf->Get<std::string>("hdf5_file");

    // Find the file path
    common::SystemPaths systemPaths;
    this->dataPtr->hydroData.hdf5File = systemPaths.FindFileURI(fileUri);

    this->dataPtr->ReadWECSim(_ecm);
  }
  else
  {
    gzwarn << "Missing element <hdf5_file> for HDF5 file. "
        << "Check this is intentional - using overrides and defaults.\n";
  }

  /// 1b. apply overrides
  if (_sdf->HasElement("hdf5_file"))
  {
      // simulation parameters
      this->dataPtr->simParams.g = this->dataPtr->hydroData.g;
      this->dataPtr->simParams.rho = this->dataPtr->hydroData.rho;

      // geometry
      this->dataPtr->geometry.Vo = this->dataPtr->hydroData.Vo;
      /// \todo override cg, cb, waterplane origin
  }

  //////////
  // 2a. read the <waves> element
  if (_sdf->HasElement("waves"))
  {
    this->dataPtr->waveOverrideOn = true;

    auto sdfWaves = _sdf->GetElementImpl("waves");

    // regular waves
    if (sdfWaves->HasElement("regular"))
    {
      auto sdfRegWaves = sdfWaves->GetElementImpl("regular");

      if (sdfRegWaves->HasElement("period"))
        this->dataPtr->waves.period =
            sdfRegWaves->Get<double>("period");

      if (sdfRegWaves->HasElement("height"))
        this->dataPtr->waves.height =
            sdfRegWaves->Get<double>("height");

      if (sdfRegWaves->HasElement("direction"))
        this->dataPtr->waves.direction =
            sdfRegWaves->Get<double>("direction");

      if (sdfRegWaves->HasElement("phase"))
        this->dataPtr->waves.phase =
            sdfRegWaves->Get<double>("phase");
    }
  }

  // 2a. apply overrides
  if (this->dataPtr->waveOverrideOn)
  {
    // simulation parameters
    this->dataPtr->simParams.T = this->dataPtr->waves.period;
    this->dataPtr->simParams.w = 2.0 * GZ_PI / this->dataPtr->simParams.T;
  }

  //////////
  // 3a. read the <simulation_parameters> element
  if (_sdf->HasElement("simulation_parameters"))
  {
    this->dataPtr->simParamOverrideOn = true;

    auto sdfSimParams = _sdf->GetElementImpl("simulation_parameters");

    if (sdfSimParams->HasElement("gravity"))
      this->dataPtr->simParams.g = sdfSimParams->Get<double>("gravity");

    if (sdfSimParams->HasElement("fluid_density"))
      this->dataPtr->simParams.rho = sdfSimParams->Get<double>("fluid_density");
  }

  //////////
  // 4a. read the <geometry> element.
  if (_sdf->HasElement("geometry"))
  {
    this->dataPtr->geometryOverrideOn = true;

    auto sdfGeom = _sdf->GetElementImpl("geometry");

    if (sdfGeom->HasElement("center_of_waterplane"))
      this->dataPtr->geometry.p_BoBwp_B =
          sdfGeom->Get<gz::math::Vector3d>("center_of_waterplane");

    if (sdfGeom->HasElement("center_of_buoyancy"))
      this->dataPtr->geometry.p_BoBcb_B =
          sdfGeom->Get<gz::math::Vector3d>("center_of_buoyancy");

    if (sdfGeom->HasElement("displaced_volume"))
      this->dataPtr->geometry.Vo =
          sdfGeom->Get<double>("displaced_volume");
  }

  //////////
  // 5a. interpolate hydro coeffs from hdf5 data.
  if (_sdf->HasElement("hdf5_file"))
  {
    /// \note we allow g and rho to override hdf5 values
    double g = this->dataPtr->simParams.g;
    double rho = this->dataPtr->simParams.rho;

    // hydrostatics
    this->dataPtr->hydroForceCoeffs.K_hs =
        this->dataPtr->hydroData.K_hs * rho * g;

    // interpolate and scale.
    double w = this->dataPtr->simParams.w;

    // radiation added mass [Ndof, Ndof, Nf]
    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.A,             // Yd
      &this->dataPtr->hydroForceCoeffs.A      // Yi
    );
    this->dataPtr->hydroForceCoeffs.A *= rho;

    // radiation damping [Ndof, Ndof, Nf]
    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.B,             // Yd
      &this->dataPtr->hydroForceCoeffs.B      // Yi
    );
    this->dataPtr->hydroForceCoeffs.B *= rho * w;

    // excitation combined [Ndof, Nh, Nf]
    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.ex_re,         // Yd
      &this->dataPtr->hydroForceCoeffs.ex_re  // Yi
    );
    this->dataPtr->hydroForceCoeffs.ex_re *= rho * g;

    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.ex_im,         // Yd
      &this->dataPtr->hydroForceCoeffs.ex_im  // Yi
    );
    this->dataPtr->hydroForceCoeffs.ex_im *= rho * g;

    // excitation Froude-Krylov [Ndof, Nh, Nf]
    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.fk_re,         // Yd
      &this->dataPtr->hydroForceCoeffs.fk_re  // Yi
    );
    this->dataPtr->hydroForceCoeffs.fk_re *= rho * g;

    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.fk_im,         // Yd
      &this->dataPtr->hydroForceCoeffs.fk_im  // Yi
    );
    this->dataPtr->hydroForceCoeffs.fk_im *= rho * g;

    // excitation scattering [Ndof, Nh, Nf]
    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.sc_re,         // Yd
      &this->dataPtr->hydroForceCoeffs.sc_re  // Yi
    );
    this->dataPtr->hydroForceCoeffs.sc_re *= rho * g;

    Interp(
      this->dataPtr->hydroData.w, w,          // Xd, Xi
      this->dataPtr->hydroData.sc_im,         // Yd
      &this->dataPtr->hydroForceCoeffs.sc_im  // Yi
    );
    this->dataPtr->hydroForceCoeffs.sc_im *= rho * g;
  }

  //////////
  // 6. read the <hydro_coeffs> elements.
  sdf::ElementPtr sdfHydrostatic;
  sdf::ElementPtr sdfRadiation;
  sdf::ElementPtr sdfExcitation;
  if (_sdf->HasElement("hydro_coeffs"))
  {
    auto sdfHydro = _sdf->GetElementImpl("hydro_coeffs");

    /// \todo use the scaled parameter - current assumption is all
    ///       hydro coeff overrides are dimensioned.
    if (sdfHydro->HasElement("scaled"))
      double scaled = sdfHydro->Get<double>("scaled");

    if (sdfHydro->HasElement("hydrostatic"))
      sdfHydrostatic = sdfHydro->GetElementImpl("hydrostatic");

    if (sdfHydro->HasElement("radiation"))
      sdfRadiation = sdfHydro->GetElementImpl("radiation");

    if (sdfHydro->HasElement("excitation"))
      sdfExcitation = sdfHydro->GetElementImpl("excitation");
  }

  // 6a. hydrostatic coefficients
  if (sdfHydrostatic)
  { 
    this->dataPtr->hydrostaticOverrideOn = true;

    if (sdfHydrostatic->HasElement("linear_restoring"))
    {
      this->dataPtr->hydroForceCoeffs.K_hs =
          SdfGet<Eigen::Matrix6d>(sdfHydrostatic, "linear_restoring");
    }
  }

  // 6b. hydrodynamic radiation coefficients
  if (sdfRadiation)
  {
    this->dataPtr->radiationOverrideOn = true;

    if (sdfRadiation->HasElement("added_mass"))
      this->dataPtr->hydroForceCoeffs.A =
          SdfGet<Eigen::Matrix6d>(sdfRadiation, "added_mass");
          
    if (sdfRadiation->HasElement("damping"))
      this->dataPtr->hydroForceCoeffs.B =
          SdfGet<Eigen::Matrix6d>(sdfRadiation, "damping");
  }

  // 6c. hydrodynamic excitation coefficients
  if (sdfExcitation)
  {
    this->dataPtr->excitationOverrideOn = true;

    if (sdfExcitation->HasElement("combined"))
    {
      auto sdfConstCoeff = sdfExcitation->GetElementImpl("combined");

      if (sdfConstCoeff->HasElement("re"))
        this->dataPtr->hydroForceCoeffs.ex_re =
            SdfGet<Eigen::Vector6d>(sdfConstCoeff, "re");

      if (sdfConstCoeff->HasElement("im"))
        this->dataPtr->hydroForceCoeffs.ex_im =
            SdfGet<Eigen::Vector6d>(sdfConstCoeff, "im");
    }

    if (sdfExcitation->HasElement("froude_krylov"))
    {
      auto sdfConstCoeff = sdfExcitation->GetElementImpl("froude_krylov");

      if (sdfConstCoeff->HasElement("re"))
        this->dataPtr->hydroForceCoeffs.fk_re =
            SdfGet<Eigen::Vector6d>(sdfConstCoeff, "re");

      if (sdfConstCoeff->HasElement("im"))
        this->dataPtr->hydroForceCoeffs.fk_im =
            SdfGet<Eigen::Vector6d>(sdfConstCoeff, "im");
    }

    if (sdfExcitation->HasElement("scattering"))
    {
      auto sdfConstCoeff = sdfExcitation->GetElementImpl("scattering");

      if (sdfConstCoeff->HasElement("re"))
        this->dataPtr->hydroForceCoeffs.sc_re =
            SdfGet<Eigen::Vector6d>(sdfConstCoeff, "re");

      if (sdfConstCoeff->HasElement("im"))
        this->dataPtr->hydroForceCoeffs.sc_im =
            SdfGet<Eigen::Vector6d>(sdfConstCoeff, "im");
    }
  }

  // forces on/off parameters
  if (_sdf->HasElement("forces"))
  {
    auto sdfForces = _sdf->GetElementImpl("forces");

    if (sdfForces->HasElement("gravity_on"))
      this->dataPtr->forceFlags.gravityOn =
          sdfForces->Get<bool>("gravity_on");

    if (sdfForces->HasElement("buoyancy_on"))
      this->dataPtr->forceFlags.buoyancyOn =
          sdfForces->Get<bool>("buoyancy_on");
    
    if (sdfForces->HasElement("hydrostatic_restoring_on"))
      this->dataPtr->forceFlags.hydrostaticRestoringOn =
          sdfForces->Get<bool>("hydrostatic_restoring_on");

    if (sdfForces->HasElement("radiation_damping_on"))
      this->dataPtr->forceFlags.radiationDampingOn =
          sdfForces->Get<bool>("radiation_damping_on");
    
    if (sdfForces->HasElement("radiation_added_mass_on"))
      this->dataPtr->forceFlags.radiationAddedMassOn =
          sdfForces->Get<bool>("radiation_added_mass_on");

    if (sdfForces->HasElement("excitation_on"))
      this->dataPtr->forceFlags.excitationOn =
          sdfForces->Get<bool>("excitation_on");

    if (sdfForces->HasElement("excitation_froude_krylov_on"))
      this->dataPtr->forceFlags.excitationFroudeKrylovOn =
          sdfForces->Get<bool>("excitation_froude_krylov_on");

    if (sdfForces->HasElement("excitation_scattering_on"))
      this->dataPtr->forceFlags.excitationScatteringOn =
          sdfForces->Get<bool>("excitation_scattering_on");
  }

  // debug on/off parameters
  if (_sdf->HasElement("debug"))
  {
    auto sdfDebug = _sdf->GetElementImpl("debug");
  
    if (sdfDebug->HasElement("gravity_on"))
      this->dataPtr->debugFlags.gravityOn =
          sdfDebug->Get<bool>("gravity_on");

    if (sdfDebug->HasElement("buoyancy_on"))
      this->dataPtr->debugFlags.buoyancyOn =
          sdfDebug->Get<bool>("buoyancy_on");

    if (sdfDebug->HasElement("radiation_damping_on"))
      this->dataPtr->debugFlags.radiationDampingOn =
          sdfDebug->Get<bool>("radiation_damping_on");

    if (sdfDebug->HasElement("radiation_added_mass_on"))
      this->dataPtr->debugFlags.radiationAddedMassOn =
          sdfDebug->Get<bool>("radiation_added_mass_on");

    if (sdfDebug->HasElement("excitation_on"))
      this->dataPtr->debugFlags.excitationOn =
          sdfDebug->Get<bool>("excitation_on");
  }

  // publishers
  if (_sdf->HasElement("publishers"))
  {
    auto sdPub = _sdf->GetElementImpl("publishers");

    if (sdPub->HasElement("update_rate"))
      this->dataPtr->publishers.updateRate =
          sdPub->Get<double>("update_rate");

    if (sdPub->HasElement("gravity_on"))
      this->dataPtr->publishers.gravityOn =
          sdPub->Get<bool>("gravity_on");
    if (sdPub->HasElement("gravity_topic"))
      this->dataPtr->publishers.gravityTopic =
          sdPub->Get<std::string>("gravity_topic");

    if (sdPub->HasElement("buoyancy_on"))
      this->dataPtr->publishers.buoyancyOn =
          sdPub->Get<bool>("buoyancy_on");
    if (sdPub->HasElement("buoyancy_topic"))
      this->dataPtr->publishers.buoyancyTopic =
          sdPub->Get<std::string>("buoyancy_topic");

    if (sdPub->HasElement("hydrostatic_restoring_on"))
      this->dataPtr->publishers.hydrostaticRestoringOn =
          sdPub->Get<bool>("hydrostatic_restoring_on");
    if (sdPub->HasElement("hydrostatic_restoring_topic"))
      this->dataPtr->publishers.restoringTopic =
          sdPub->Get<std::string>("hydrostatic_restoring_topic");

    if (sdPub->HasElement("radiation_damping_on"))
      this->dataPtr->publishers.radiationDampingOn =
          sdPub->Get<bool>("radiation_damping_on");
    if (sdPub->HasElement("radiation_damping_topic"))
      this->dataPtr->publishers.radiationDampingTopic =
          sdPub->Get<std::string>("radiation_damping_topic");

    if (sdPub->HasElement("radiation_added_mass_on"))
      this->dataPtr->publishers.radiationAddedMassOn =
          sdPub->Get<bool>("radiation_added_mass_on");
    if (sdPub->HasElement("radiation_added_mass_topic"))
      this->dataPtr->publishers.radiationAddedMassTopic =
          sdPub->Get<std::string>("radiation_added_mass_topic");

    if (sdPub->HasElement("excitation_on"))
      this->dataPtr->publishers.excitationOn =
          sdPub->Get<bool>("excitation_on");
    if (sdPub->HasElement("excitation_topic"))
      this->dataPtr->publishers.excitationTopic =
          sdPub->Get<std::string>("excitation_topic");

    if (sdPub->HasElement("excitation_froude_krylov_on"))
      this->dataPtr->publishers.excitationFroudeKrylovOn =
          sdPub->Get<bool>("excitation_froude_krylov_on");
    if (sdPub->HasElement("excitation_froude_krylov_topic"))
      this->dataPtr->publishers.excitationFroudeKrylovTopic =
          sdPub->Get<std::string>("excitation_froude_krylov_topic");

    if (sdPub->HasElement("excitation_scattering_on"))
      this->dataPtr->publishers.excitationScatteringOn =
          sdPub->Get<bool>("excitation_scattering_on");
    if (sdPub->HasElement("excitation_scattering_topic"))
      this->dataPtr->publishers.excitationScatteringTopic =
          sdPub->Get<std::string>("excitation_scattering_topic");
  }

  /// \todo add checks

  // Create publishers
  if (this->dataPtr->publishers.gravityOn)
  {
    this->dataPtr->publishers.gravityPub =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.gravityTopic);
  }
  if (this->dataPtr->publishers.buoyancyOn)
  {
    this->dataPtr->publishers.buoyancyPub =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.buoyancyTopic);
  }
  if (this->dataPtr->publishers.hydrostaticRestoringOn)
  {
    this->dataPtr->publishers.restoringPub =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.restoringTopic);
  }
  if (this->dataPtr->publishers.radiationDampingOn)
  {
    this->dataPtr->publishers.radiationDampingPubl =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.radiationDampingTopic);
  }
  if (this->dataPtr->publishers.radiationAddedMassOn)
  {
    this->dataPtr->publishers.radiationAddedMassPubl =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.radiationAddedMassTopic);
  }
  if (this->dataPtr->publishers.excitationOn)
  {
    this->dataPtr->publishers.excitationPub =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.excitationTopic);
  }
  if (this->dataPtr->publishers.excitationFroudeKrylovOn)
  {
    this->dataPtr->publishers.excitationFroudeKrylovPubl =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.excitationFroudeKrylovTopic);
  }
  if (this->dataPtr->publishers.excitationScatteringOn)
  {
    this->dataPtr->publishers.excitationScatteringPub =
        this->dataPtr->node.Advertise<gz::msgs::Wrench>(
            this->dataPtr->publishers.excitationScatteringTopic);
  }

  // display plugin parameters
  {
    gzmsg << "LinearWaveBody plugin parameters\n"
          << "<gravity_on>: "
          << this->dataPtr->forceFlags.gravityOn << "\n"
          << "<buoyancy_on>: "
          << this->dataPtr->forceFlags.buoyancyOn << "\n"
          << "<hydrostatic_restoring_on>: "
          << this->dataPtr->forceFlags.hydrostaticRestoringOn << "\n"
          << "<radiation_damping_on>: "
          << this->dataPtr->forceFlags.radiationDampingOn << "\n"
          << "<radiation_added_mass_on>: "
          << this->dataPtr->forceFlags.radiationAddedMassOn << "\n"
          << "<excitation_on>: "
          << this->dataPtr->forceFlags.excitationOn << "\n"
          << "<excitation_froude_krylov_on>: "
          << this->dataPtr->forceFlags.excitationFroudeKrylovOn << "\n"
          << "<excitation_scattering_on>: "
          << this->dataPtr->forceFlags.excitationScatteringOn << "\n";
  }

  // display wave parameters
  {
    gzmsg << "Wave Parameters\n"
          << "<wave_period>: "
          << this->dataPtr->waves.period << "\n"
          << "<wave_height>: "
          << this->dataPtr->waves.height << "\n"
          << "<wave_direction>: "
          << this->dataPtr->waves.direction << "\n"
          << "<wave_phase>: "
          << this->dataPtr->waves.phase << "\n";
  }

  // display simulation parameters
  {
    gzmsg << "Simulation Parameters\n"
          << "<gravity>: "
          << this->dataPtr->simParams.g << "\n"
          << "<fluid_density>: "
          << this->dataPtr->simParams.rho << "\n"
          << "T: "
          << this->dataPtr->simParams.T << "\n"
          << "w: "
          << this->dataPtr->simParams.w << "\n";
  }

  // display geometry parameters
  {
    gzmsg << "Geometry Parameters\n"
          << "<center_of_waterplane>: "
          << this->dataPtr->geometry.p_BoBwp_B << "\n"
          << "<center_of_buoyancy>: "
          << this->dataPtr->geometry.p_BoBcb_B << "\n"
          << "<displaced_volume>: "
          << this->dataPtr->geometry.Vo << "\n";
  }

  // display hydrostatic parameters
  {
    gzmsg << "Hydrostatic Coefficients\n"
          << "<linear_restoring>:\n"
          << this->dataPtr->hydroForceCoeffs.K_hs << "\n";
  }

  // display radiation parameters
  {
    gzmsg << "Radiation Coefficients\n"
          << "<added_mass>:\n"
          << this->dataPtr->hydroForceCoeffs.A << "\n"
          << "<damping>:\n"
          << this->dataPtr->hydroForceCoeffs.B << "\n";
  }

  // display excitation parameters
  {
    gzmsg << "Excitation Coefficients\n"
          << "<ex_re>:\n"
          << this->dataPtr->hydroForceCoeffs.ex_re.transpose() << "\n"
          << "<ex_im>:\n"
          << this->dataPtr->hydroForceCoeffs.ex_im.transpose() << "\n"
          << "<fk_re>:\n"
          << this->dataPtr->hydroForceCoeffs.fk_re.transpose() << "\n"
          << "<fk_im>:\n"
          << this->dataPtr->hydroForceCoeffs.fk_im.transpose() << "\n"
          << "<sc_re>:\n"
          << this->dataPtr->hydroForceCoeffs.sc_re.transpose() << "\n"
          << "<sc_im>:\n"
          << this->dataPtr->hydroForceCoeffs.sc_im.transpose() << "\n";
  }

  /// Radiation added mass - constant coeffients
  ///
  /// \note incorporating added mass terms in the physics solver
  ///
  /// Gazebo Garden (sdformat13) allows an added mass term to be included
  /// in the <inertial> element according to the proposal:
  ///
  /// http://sdformat.org/tutorials?tut=added_mass_proposal&branch=chapulina/added_mass
  ///
  /// An entry for the fluid added mass has been added to the Inertial class
  /// in gz-math7 and is accessed using the method Inertial::FluidAddedMass.
  ///
  /// The following issues and PRs track including the fluid added mass in
  /// the DART physics engine.
  /// - https://github.com/gazebosim/gz-sim/pull/1592
  /// - https://github.com/gazebosim/gz-math/pull/459
  /// - https://github.com/gazebosim/sdformat/pull/1077
  /// - https://github.com/gazebosim/gz-physics/pull/384
  /// - https://github.com/gazebosim/gz-msgs/pull/271
  ///
  if (this->dataPtr->forceFlags.radiationAddedMassOn)
  {
    // parameters
    auto& A = this->dataPtr->hydroForceCoeffs.A;

    // convert from Eigen to gz
    gz::math::Matrix6d gzA;
    for (size_t i=0; i<6; ++i)
      for (size_t j=0; j<6; ++j)
        gzA(i, j) = A(i, j);

    auto inertialComp = _ecm.Component<components::Inertial>(baseLink.Entity());
    if (this->dataPtr->debugFlags.radiationAddedMassOn)
    {
      gzmsg << "Setting Fluid Added Mass\n"
        << gzA << "\n";
    }
    inertialComp->Data().SetFluidAddedMass(gzA);
  }
}

//////////////////////////////////////////////////
void LinearWaveBody::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("LinearWaveBody::PreUpdate");

  /// \todo(anyone) support reset / rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << "\n";
  }

  if (!this->dataPtr->initialized)
  {
    // We call Init here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Init(_ecm);
    this->dataPtr->initialized = true;
  }

  if (_info.paused)
    return;

  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
LinearWaveBodyPrivate::~LinearWaveBodyPrivate()
{
};

/////////////////////////////////////////////////
void LinearWaveBodyPrivate::Init(EntityComponentManager &_ecm)
{
  // initial pose of the link in the world frame
  gz::math::Pose3d X_WB = gz::sim::worldPose(this->linkEntity, _ecm);
  
  // adjust for x-y position only (assume free surface is at z=0)
  this->X0_WBwp = gz::math::Pose3d::Zero;
  this->X0_WBwp.Pos().X() += X_WB.Pos().X();
  this->X0_WBwp.Pos().Y() += X_WB.Pos().Y();

  this->validConfig = true;
}

/// \brief Helpers for reading vectors of Eigen::Matrix6d from data sets.
namespace
{
  void ReadDataSet(
      const HighFive::File& _file,
      const std::string& _name,
      std::vector<Eigen::Matrix6d> *_array)
  {
    auto dset = _file.getDataSet(_name);
    auto dim = dset.getDimensions();
    _array->resize(dim[2]);
    std::vector<size_t> columns{0};
    for (size_t i=0; i<dim[2]; ++i)
    {
      columns[0] = i;
      auto selection = dset.select(columns);
      selection.read((*_array)[i]);
    }
  }

  /// \todo ??? something buggy reading data when Nh = 1 ???
  ///       work-around by reading element-wise
  void ReadDataSet(
      const HighFive::File& _file,
      const std::string& _name,
      std::vector<Eigen::Matrix<double, 6, 1>> *_array)
  {
    auto dset = _file.getDataSet(_name);
    auto dim = dset.getDimensions();
    _array->resize(dim[2]);
    std::vector<size_t> element_ids{0, 0, 0};
    for (size_t i=0; i<dim[0]; ++i)
    {
      element_ids[0] = i;
      for (size_t j=0; j<dim[1]; ++j)
      {
        element_ids[1] = j;
        for (size_t k=0; k<dim[2]; ++k)
        {
          element_ids[2] = k;
          auto selection = dset.select(
              HighFive::ElementSet(element_ids));
          double val;
          selection.read(val);
          (*_array)[k](i, j) = val;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void LinearWaveBodyPrivate::ReadWECSim(EntityComponentManager &_ecm)
{
  auto& hydro = this->hydroData;

  if (hydro.hdf5File.empty())
    return;

  // Read hdf5 data
  HighFive::File file(hydro.hdf5File, HighFive::File::ReadOnly);

  // radiation added mass infinite frequency
  if (file.exist("/body1/hydro_coeffs/added_mass/inf_freq"))
    file.getDataSet("/body1/hydro_coeffs/added_mass/inf_freq")
        .read(hydro.Ainf);

  // radiation added mass has dimension 6 x 6 x Nf
  if (file.exist("/body1/hydro_coeffs/added_mass/all"))
    ReadDataSet(file, "/body1/hydro_coeffs/added_mass/all", &hydro.A);

  // radiation damping has dimension 6 x 6 x Nf
  if (file.exist("/body1/hydro_coeffs/radiation_damping/all"))
    ReadDataSet(file, "/body1/hydro_coeffs/radiation_damping/all", &hydro.B);

  // radiation IRF has dimension 6 x 6 x len(ra_t)
  std::string dsetpath;
  dsetpath = "/body1/hydro_coeffs/radiation_damping/impulse_response_fun";
  if (file.exist(dsetpath + "/K"))
    ReadDataSet(file, dsetpath + "/K", &hydro.ra_K);
  if (file.exist(dsetpath + "/t"))
    file.getDataSet(dsetpath + "/t").read(hydro.ra_t);
  if (file.exist(dsetpath + "/w"))
    file.getDataSet(dsetpath + "/w").read(hydro.ra_w);

  // excitation force has dimension 6 x Nh x Nf
  //
  dsetpath = "/body1/hydro_coeffs/excitation";
  if (file.exist(dsetpath + "/re"))
    ReadDataSet(file, dsetpath + "/re", &hydro.ex_re);
  if (file.exist(dsetpath + "/im"))
    ReadDataSet(file, dsetpath + "/im", &hydro.ex_im);
  if (file.exist(dsetpath + "/mag"))
    ReadDataSet(file, dsetpath + "/mag", &hydro.ex_ma);
  if (file.exist(dsetpath + "/phase"))
    ReadDataSet(file, dsetpath + "/phase", &hydro.ex_ph);

  // excitation froude-krylov force has dimension 6 x Nh x Nf
  //
  dsetpath = "/body1/hydro_coeffs/excitation/froude-krylov";
  if (file.exist(dsetpath + "/re"))
    ReadDataSet(file, dsetpath + "/re", &hydro.fk_re);
  if (file.exist(dsetpath + "/im"))
    ReadDataSet(file, dsetpath + "/im", &hydro.fk_im);
  if (file.exist(dsetpath + "/mag"))
    ReadDataSet(file, dsetpath + "/mag", &hydro.fk_ma);
  if (file.exist(dsetpath + "/phase"))
    ReadDataSet(file, dsetpath + "/phase", &hydro.fk_ph);

  // excitation scattering force has dimension 6 x Nh x Nf
  //
  dsetpath = "/body1/hydro_coeffs/excitation/scattering";
  if (file.exist(dsetpath + "/re"))
    ReadDataSet(file, dsetpath + "/re", &hydro.sc_re);
  if (file.exist(dsetpath + "/im"))
    ReadDataSet(file, dsetpath + "/im", &hydro.sc_im);
  if (file.exist(dsetpath + "/mag"))
    ReadDataSet(file, dsetpath + "/mag", &hydro.sc_ma);
  if (file.exist(dsetpath + "/phase"))
    ReadDataSet(file, dsetpath + "/phase", &hydro.sc_ph);

  // excitation IRF has dimension 6 x Nh x len(ra_t)
  dsetpath = "/body1/hydro_coeffs/excitation/impulse_response_fun";
  if (file.exist(dsetpath + "/f"))
    ReadDataSet(file, dsetpath + "/f", &hydro.ex_K);
  if (file.exist(dsetpath + "/t"))
    file.getDataSet(dsetpath + "/t").read(hydro.ex_t);
  if (file.exist(dsetpath + "/w"))
    file.getDataSet(dsetpath + "/w").read(hydro.ex_w);

  if (file.exist("/body1/hydro_coeffs/linear_restoring_stiffness"))
    file.getDataSet("/body1/hydro_coeffs/linear_restoring_stiffness")
        .read(hydro.K_hs);

  // properties
  dsetpath = "/body1/properties";
  if (file.exist(dsetpath + "/body_number"))
    file.getDataSet(dsetpath + "/body_number").read(hydro.Nb);
  if (file.exist(dsetpath + "/cb"))
    file.getDataSet(dsetpath + "/cb").read(hydro.cb);
  if (file.exist(dsetpath + "/cg"))
    file.getDataSet(dsetpath + "/cg").read(hydro.cg);
  if (file.exist(dsetpath + "/disp_vol"))
    file.getDataSet(dsetpath + "/disp_vol").read(hydro.Vo);

  /// \todo issue reading fixed length strings
  /// https://github.com/BlueBrain/HighFive/issues/94
  /// https://github.com/BlueBrain/HighFive/pull/277
  // HighFive::FixedLenStringArray<50> name;
  // file.getDataSet("body1/properties/name").read(name);

  // simulation_parameters
  dsetpath = "/simulation_parameters";
  if (file.exist(dsetpath + "/T"))
    file.getDataSet(dsetpath + "/T").read(hydro.T);
  if (file.exist(dsetpath + "/g"))
    file.getDataSet(dsetpath + "/g").read(hydro.g);
  if (file.exist(dsetpath + "/rho"))
    file.getDataSet(dsetpath + "/rho").read(hydro.rho);
  if (file.exist(dsetpath + "/scaled"))
    file.getDataSet(dsetpath + "/scaled").read(hydro.scaled);
  if (file.exist(dsetpath + "/w"))
    file.getDataSet(dsetpath + "/w").read(hydro.w);
  if (file.exist(dsetpath + "/wave_dir"))
    file.getDataSet(dsetpath + "/wave_dir").read(hydro.theta);

  // set number of wave frequencies
  hydro.Nf = hydro.w.size();

  gzmsg << "Loaded HDF5 data:\n"
        << "hdf5_file: " << hydro.hdf5File << "\n";

  // radiation added mass
  {
    gzmsg << "HDF5 Added Mass Inf Freq:\n"
          << "/body1/hydro_coeffs/added_mass/inf_freq\n"
          << "Ainf_00: " << hydro.Ainf(0, 0) << "\n"
          << "Ainf_11: " << hydro.Ainf(1, 1) << "\n"
          << "Ainf_22: " << hydro.Ainf(2, 2) << "\n";
  }

  // radiation added mass
  if (!hydro.A.empty())
  {
    gzmsg << "HDF5 Added Mass Coefficients:\n"
          << "/body1/hydro_coeffs/added_mass/all\n"
          << "A.size(): " << hydro.A.size() << "\n"
          << "A_000: " << hydro.A[0](0, 0) << "\n"
          << "A_011: " << hydro.A[0](1, 1) << "\n"
          << "A_022: " << hydro.A[0](2, 2) << "\n";
  }

  // radiation damping
  if (!hydro.B.empty())
  {
    gzmsg << "HDF5 Radiation Damping Coefficients:\n"
          << "/body1/hydro_coeffs/radiation_damping/all\n"
          << "B.size(): " << hydro.B.size() << "\n"
          << "B_000: " << hydro.B[0](0, 0) << "\n"
          << "B_011: " << hydro.B[0](1, 1) << "\n"
          << "B_022: " << hydro.B[0](2, 2) << "\n";
  }

  // radiation IRF
  if (!hydro.ra_K.empty() && !hydro.ra_t.empty() && !hydro.ra_t.empty())
  {
    gzmsg << "HDF5 Radiation IRF Coefficients:\n"
          << "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/K\n"
          << "ra_K.size(): " << hydro.ra_K.size() << "\n"
          << "ra_K_000: " << hydro.ra_K[0](0, 0) << "\n"
          << "ra_K_011: " << hydro.ra_K[0](1, 1) << "\n"
          << "ra_K_022: " << hydro.ra_K[0](2, 2) << "\n"

          << "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/t\n"
          << "ra_t.size(): " << hydro.ra_t.size() << "\n"
          << "ra_t_0: " << hydro.ra_t[0] << "\n"
          << "ra_t_n: " << hydro.ra_t[hydro.ra_t.size()-1] << "\n"

          << "/body1/hydro_coeffs/radiation_damping/impulse_response_fun/w\n"
          << "ra_w.size(): " << hydro.ra_w.size() << "\n"
          << "ra_w_0: " << hydro.ra_w[0] << "\n"
          << "ra_w_n: " << hydro.ra_w[hydro.ra_w.size()-1] << "\n";
  }

  // combined excitation
  if (!hydro.ex_re.empty() && !hydro.ex_im.empty() &&
      !hydro.ex_ma.empty() && !hydro.ex_ph.empty())
  {
    gzmsg << "HDF5 Combined Excitation Coefficients:\n"
          << "/body1/hydro_coeffs/excitation/re\n"
          << "ex_re.size(): " << hydro.ex_re.size() << "\n"
          << "ex_re_000: " << hydro.ex_re[0](0, 0) << "\n"
          << "ex_re_001: " << hydro.ex_re[0](1, 0) << "\n"

          << "/body1/hydro_coeffs/excitation/im\n"
          << "ex_im.size(): " << hydro.ex_im.size() << "\n"
          << "ex_im_000: " << hydro.ex_im[0](0, 0) << "\n"
          << "ex_im_001: " << hydro.ex_im[0](1, 0) << "\n"

          << "/body1/hydro_coeffs/excitation/ma\n"
          << "ex_ma.size(): " << hydro.ex_ma.size() << "\n"
          << "ex_ma_000: " << hydro.ex_ma[0](0, 0) << "\n"
          << "ex_ma_001: " << hydro.ex_ma[0](1, 0) << "\n"

          << "/body1/hydro_coeffs/excitation/ph\n"
          << "ex_ph.size(): " << hydro.ex_ph.size() << "\n"
          << "ex_ph_000: " << hydro.ex_ph[0](0, 0) << "\n"
          << "ex_ph_001: " << hydro.ex_ph[0](1, 0) << "\n";
  }

  // froude-krylov
  if (!hydro.fk_re.empty() && !hydro.fk_im.empty() &&
      !hydro.fk_ma.empty() && !hydro.fk_ph.empty())
  {
    gzmsg << "HDF5 Excitation Froude-Krylov Coefficients:\n"
          << "/body1/hydro_coeffs/excitation/froude-krylov/re\n"
          << "fk_re.size(): " << hydro.fk_re.size() << "\n"
          << "fk_re_000: " << hydro.fk_re[0](0, 0) << "\n"
          << "fk_re_001: " << hydro.fk_re[0](1, 0) << "\n"

          << "/body1/hydro_coeffs/excitation/froude-krylov/im\n"
          << "fk_im.size(): " << hydro.fk_im.size() << "\n"
          << "fk_im_000: " << hydro.fk_im[0](0, 0) << "\n"
          << "fk_im_001: " << hydro.fk_im[0](1, 0) << "\n"

          << "/body1/hydro_coeffs/excitation/froude-krylov/ma\n"
          << "fk_ma.size(): " << hydro.fk_ma.size() << "\n"
          << "fk_ma_000: " << hydro.fk_ma[0](0, 0) << "\n"
          << "fk_ma_001: " << hydro.fk_ma[0](1, 0) << "\n"

          << "/body1/hydro_coeffs/excitation/froude-krylov/ph\n"
          << "fk_ph.size(): " << hydro.fk_ph.size() << "\n"
          << "fk_ph_000: " << hydro.fk_ph[0](0, 0) << "\n"
          << "fk_ph_001: " << hydro.fk_ph[0](1, 0) << "\n";
  }

  // scattering
  if (!hydro.sc_re.empty() && !hydro.sc_im.empty() &&
      !hydro.sc_ma.empty() && !hydro.sc_ph.empty())
  {
    gzmsg << "HDF5 Excitation Scattering Coefficients:\n"
        << "/body1/hydro_coeffs/excitation/scattering/re\n"
        << "sc_re.size(): " << hydro.sc_re.size() << "\n"
        << "sc_re_000: " << hydro.sc_re[0](0, 0) << "\n"
        << "sc_re_001: " << hydro.sc_re[0](1, 0) << "\n"

        << "/body1/hydro_coeffs/excitation/scattering/im\n"
        << "sc_im.size(): " << hydro.sc_im.size() << "\n"
        << "sc_im_000: " << hydro.sc_im[0](0, 0) << "\n"
        << "sc_im_001: " << hydro.sc_im[0](1, 0) << "\n"

        << "/body1/hydro_coeffs/excitation/scattering/ma\n"
        << "sc_ma.size(): " << hydro.sc_ma.size() << "\n"
        << "sc_ma_000: " << hydro.sc_ma[0](0, 0) << "\n"
        << "sc_ma_001: " << hydro.sc_ma[0](1, 0) << "\n"

        << "/body1/hydro_coeffs/excitation/scattering/ph\n"
        << "sc_ph.size(): " << hydro.sc_ph.size() << "\n"
        << "sc_ph_000: " << hydro.sc_ph[0](0, 0) << "\n"
        << "sc_ph_001: " << hydro.sc_ph[0](1, 0) << "\n";
  }

  // excitation IRF
  if (!hydro.ex_K.empty() && !hydro.ex_t.empty() && !hydro.ex_w.empty())
  {
    gzmsg << "HDF5 Excitation IRF Coefficients:\n"
          << "body1/hydro_coeffs/excitation/impulse_response_fun/f\n"
          << "ex_K.size(): " << hydro.ex_K.size() << "\n"
          << "ex_K_000: " << hydro.ex_K[0](0, 0) << "\n"
          << "ex_K_011: " << hydro.ex_K[0](1, 1) << "\n"
          << "ex_K_022: " << hydro.ex_K[0](2, 2) << "\n"

          << "body1/hydro_coeffs/excitation/impulse_response_fun/t\n"
          << "ex_t.size(): " << hydro.ex_t.size() << "\n"
          << "ex_t_0: " << hydro.ex_t[0] << "\n"
          << "ex_t_n: " << hydro.ex_t[hydro.ex_t.size()-1] << "\n"

          << "body1/hydro_coeffs/excitation/impulse_response_fun/w\n"
          << "ex_w.size(): " << hydro.ex_w.size() << "\n"
          << "ex_w_0: " << hydro.ex_w[0] << "\n"
          << "ex_w_n: " << hydro.ex_w[hydro.ex_w.size()-1] << "\n";
  }

  // linear hydrostatic restoring coefficients
  {
    gzmsg << "HDF5 Hydrostatic Coefficients:\n"
          << "body1/hydro_coeffs/linear_restoring_stiffness\n"
          << "K_hs_22: " << hydro.K_hs(2, 2) << "\n"
          << "K_hs_33: " << hydro.K_hs(3, 3) << "\n"
          << "K_hs_44: " << hydro.K_hs(4, 4) << "\n";
  }

  // body properties
  {
    gzmsg << "HDF5 Body Properties:\n"
          << "body1/properties/body_number\n"
          << "body_number: " << hydro.Nb << "\n"

          << "body1/properties/cb\n"
          << "cb: " << hydro.cb(0) << ", "
                    << hydro.cb(1) << ", "
                    << hydro.cb(2) << "\n"

          << "body1/properties/cg\n"
          << "cb: " << hydro.cg(0) << ", "
                    << hydro.cg(1) << ", "
                    << hydro.cg(2) << "\n"

          << "body1/properties/disp_vol\n"
          << "disp_vol: " << hydro.Vo << "\n";
  }

  // simulation properties
  {
    gzmsg << "HDF5 Simulation Parameters:\n"
          << "simulation_parameters/T\n"
          << "T.size(): " << hydro.T.size() << "\n"

          << "simulation_parameters/g\n"
          << "g: " << hydro.g << "\n"

          << "simulation_parameters/rho\n"
          << "rho: " << hydro.rho << "\n"

          << "simulation_parameters/scaled\n"
          << "scaled: " << hydro.scaled << "\n"

          << "simulation_parameters/w\n"
          << "Nf: " << hydro.Nf << "\n"

          << "simulation_parameters/wave_dir\n"
          << "Nh: " << hydro.Nh << "\n"
          << "\n";
  }
}

/////////////////////////////////////////////////
void LinearWaveBodyPrivate::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->UpdateLinkState(_info, _ecm);
  this->UpdateGravityForces(_info, _ecm);
  this->UpdateHydrostaticForces(_info, _ecm);
  this->UpdateRadiationForces(_info, _ecm);
  this->UpdateExcitationForces(_info, _ecm);

  /// \todo improve / consolidate
  // update prevTime after all updates have checked for publishing
  double updatePeriod = 1.0/this->publishers.updateRate;
  double currentTime = std::chrono::duration<double>(_info.simTime).count();
  if ((currentTime - this->prevTime) < updatePeriod)
  {
    this->prevTime = currentTime; 
  }
}

/////////////////////////////////////////////////
void LinearWaveBodyPrivate::UpdateLinkState(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // link properties
  gz::sim::Link baseLink(this->linkEntity);
  auto inertial = _ecm.Component<components::Inertial>(this->linkEntity);
  // double mass = inertial->Data().MassMatrix().Mass();

  // update pose of body in world frame
  this->linkState.X_WB = gz::sim::worldPose(this->linkEntity, _ecm);

  // pose of body CoM in world frame
  this->linkState.X_BBcm = inertial->Data().Pose();
  this->linkState.X_WBcm = this->linkState.X_WB * this->linkState.X_BBcm;

  // pose of body waterplane in body CoM frame
  this->linkState.X_BcmBwp = this->linkState.X_BBcm.Inverse() * this->X_BBwp;

  // pose of body waterplane in world frame
  this->linkState.X_WBwp = this->linkState.X_WB * this->X_BBwp;

  // position vector of body CoM wrt body origin in body/world frame
  this->linkState.p_BoBcm_B = this->linkState.X_BBcm.Pos(); 
  this->linkState.p_BoBcm_W =
      this->linkState.X_WB.Rot().RotateVector(this->linkState.p_BoBcm_B);

  // position vector of body waterplane wrt body CoM in world body/world frame
  this->linkState.p_BcmBwp_Bcm = this->linkState.X_BcmBwp.Pos();
  this->linkState.p_BcmBwp_W =
      this->linkState.X_WBcm.Rot().RotateVector(this->linkState.p_BcmBwp_Bcm);

  // position vector of body center of buoyancy  
  auto& cb = this->hydroData.cb;
  auto& cg = this->hydroData.cg;
  auto p_BcmBcb = cb - cg;

  // position vector of body CoB wrt body CoM in body/world frame  
  this->linkState.p_BcmBcb_B =
      gz::math::Vector3d(p_BcmBcb(0), p_BcmBcb(1), p_BcmBcb(2));
  this->linkState.p_BcmBcb_W =
      this->linkState.X_WB.Rot().RotateVector(this->linkState.p_BcmBcb_B);

  // enable for additional debug info
  if (this->debugFlags.buoyancyOn ||
      this->debugFlags.radiationDampingOn ||
      this->debugFlags.radiationAddedMassOn ||
      this->debugFlags.excitationOn)
  {
    gzdbg << "Link State\n"
          << "X_BcmBwp.Pos: " << this->linkState.X_BcmBwp.Pos() << " m\n"

          << "X_BBcm.Pos:   " << this->linkState.X_BBcm.Pos()   << " m\n"
          << "X_BBwp.Pos:   " << this->X_BBwp.Pos()             << " m\n"

          << "X_WB.Pos:     " << this->linkState.X_WB.Pos()     << " m\n"
          << "X_WBcm.Pos:   " << this->linkState.X_WBcm.Pos()   << " m\n"
          << "X_WBwp.Pos:   " << this->linkState.X_WBwp.Pos()   << " m\n"

          << "X_BcmBwp.Rot: " << this->linkState.X_BcmBwp.Rot().Roll() << " "
                              << this->linkState.X_BcmBwp.Rot().Pitch() << " "
                              << this->linkState.X_BcmBwp.Rot().Yaw()
                              << " rad\n"

          << "X_BBcm.Rot:   " << this->linkState.X_BBcm.Rot().Roll() << " "
                              << this->linkState.X_BBcm.Rot().Pitch() << " "
                              << this->linkState.X_BBcm.Rot().Yaw()
                              << " rad\n"
          << "X_BBwp.Rot:   " << this->X_BBwp.Rot().Roll() << " "
                              << this->X_BBwp.Rot().Pitch() << " "
                              << this->X_BBwp.Rot().Yaw()
                              << " rad\n"
          
          << "X_WB.Rot:     " << this->linkState.X_WB.Rot().Roll() << " "
                              << this->linkState.X_WB.Rot().Pitch() << " "
                              << this->linkState.X_WB.Rot().Yaw()
                              << " rad\n"
          << "X_WBcm.Rot:   " << this->linkState.X_WBcm.Rot().Roll() << " "
                              << this->linkState.X_WBcm.Rot().Pitch() << " "
                              << this->linkState.X_WBcm.Rot().Yaw()
                              << " rad\n"
          << "X_WBwp.Rot:   " << this->linkState.X_WBwp.Rot().Roll() << " "
                              << this->linkState.X_WBwp.Rot().Pitch() << " "
                              << this->linkState.X_WBwp.Rot().Yaw()
                              << " rad\n"

          << "p_BcmBwp_Bcm: " << this->linkState.p_BcmBwp_Bcm << " m\n"
          << "p_BcmBcb_B:   " << this->linkState.p_BcmBcb_B << " m\n"
          << "p_BoBcm_B:    " << this->linkState.p_BoBcm_B << " m\n"

          << "p_BcmBwp_W:   " << this->linkState.p_BcmBwp_W << " m\n"
          << "p_BcmBcb_W:   " << this->linkState.p_BcmBcb_W << " m\n"
          << "p_BoBcm_W:    " << this->linkState.p_BoBcm_W << " m\n"

          << "\n";
  }
}

/////////////////////////////////////////////////
/// \brief Calculate the gravity force.
///
/// Global gravity must be disabled when using added mass because the physics
/// engine will combine the added mass with the body inertial mass to calculate
/// the gravitational force - and the force will be too large.  
void LinearWaveBodyPrivate::UpdateGravityForces(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // short circuit if forces disabled
  if (!this->forceFlags.gravityOn)
    return;

  // simulation params
  double g = this->simParams.g;

  // link properties
  gz::sim::Link baseLink(this->linkEntity);
  auto inertial = _ecm.Component<components::Inertial>(this->linkEntity);
  double mass = inertial->Data().MassMatrix().Mass();

  // gravity force
  gz::math::Vector3d fgr_Bcm_W(0.0, 0.0, - mass * g);

  if (this->debugFlags.gravityOn)
  {
    gzdbg << "Gravity Forces\n"
      << "fgr_Bcm_W     " << fgr_Bcm_W << " N\n"
      << "\n";
  }

  // apply forces at CoM
  if (fgr_Bcm_W.IsFinite())
  {
    baseLink.AddWorldForce(_ecm, fgr_Bcm_W);
  }

  // publish forces periodically
  double updatePeriod = 1.0/this->publishers.updateRate;
  double currentTime = std::chrono::duration<double>(_info.simTime).count();
  if (this->publishers.gravityOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), fgr_Bcm_W);
    msgs::Set(msg.mutable_torque(), gz::math::Vector3d::Zero);
    this->publishers.gravityPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
/// \brief Calculate the hydrostatic equilibrium and restoring forces.
///
/// There are two comtributions to the hydrostatic force in the linear model.
///
/// 1. The first term is the equilibrium hydrostatic force. In Newman p293
///    these terms cancel because the force and moments due to gravity are
///    included, however in gz-sim gravity forces are computed externally so
///    the equilibrium contributions must be included. 
///
/// 2. The second term is a restoring force arising from small displacements
///    from the equilibrium position. These forces are assumed linear in the
///    dispacement and take the form of a generalised spring equation of the
///    form
///
///    F_hr = - K_hs * Xsi
///
///    where Xsi is a 6 x 1 vector of displacements and K_hs is the 6 x 6
///    hydrostatic stiffness matrix. 
///
void LinearWaveBodyPrivate::UpdateHydrostaticForces(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // short circuit if forces disabled
  if (!this->forceFlags.buoyancyOn &&
      !this->forceFlags.hydrostaticRestoringOn)
    return;

  // parameters
  double g = this->simParams.g;
  double rho = this->simParams.rho;
  double Vo = this->geometry.Vo;
  auto& K_hs = this->hydroForceCoeffs.K_hs;

  /// \todo check whether to add correction - in WEC-Sim buoyancy force is
  ///       applied at CoB not CoM, so there is an additional moment. This
  ///       to be compared with the point of application in the linear model,
  ///       which seems to apply all forces at the origin of the waterplane?

  // center of buoyancy in link frame (i.e. wrt link origin)
  // auto& cb = this->hydroData.cb;
  // auto& cg = this->hydroData.cg;
  // auto p_BcmBcb = cb - cg;

  // link properties
  gz::sim::Link baseLink(this->linkEntity);

  // displacements in the world frame (assumed small)
  Eigen::Vector6d Xsi_WwpBwp_W(
      this->linkState.X_WBwp.X()     - this->X0_WBwp.X(),
      this->linkState.X_WBwp.Y()     - this->X0_WBwp.Y(),
      this->linkState.X_WBwp.Z()     - this->X0_WBwp.Z(),
      this->linkState.X_WBwp.Roll()  - this->X0_WBwp.Roll(),
      this->linkState.X_WBwp.Pitch() - this->X0_WBwp.Pitch(),
      this->linkState.X_WBwp.Yaw()   - this->X0_WBwp.Yaw()
  ); 

  /// \note compare to Newman p293: here weight forces and moments are
  ///       calculated externally to the hydrostatics - so we must include
  ///       the equilibrium forces as well as the restoring ones. 

  // hydrostatic buoyancy forces at equilibrium (independent of displacements)
  gz::math::Vector3d fhb_Bcm_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d thb_Bcm_W = gz::math::Vector3d::Zero;

  if (this->forceFlags.buoyancyOn)
  {
    fhb_Bcm_W = gz::math::Vector3d(0.0, 0.0, rho * g * Vo);

    /// \todo In WEC-Sim the CoB and CoM for the buoyancy
    ///       torque contribution are held constant - as they are
    ///       in the linear hydrostatic restoring force coefficients.
    thb_Bcm_W = this->linkState.p_BcmBcb_B.Cross(fhb_Bcm_W);
    // thb_Bcm_W = this->linkState.p_BcmBcb_W.Cross(fhb_Bcm_W);
  }

  // hydrostatic restoring forces (valid for small displacements)
  gz::math::Vector3d fhr_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d thr_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d thr_Bcm_W = gz::math::Vector3d::Zero;

  if (this->forceFlags.hydrostaticRestoringOn)
  {
    // hydrostatic restoring forces
    Eigen::Vector6d F_Bwp_W = -1 * K_hs * Xsi_WwpBwp_W;
    fhr_Bwp_W = gz::math::Vector3d(F_Bwp_W(0), F_Bwp_W(1), F_Bwp_W(2));
    thr_Bwp_W = gz::math::Vector3d(F_Bwp_W(3), F_Bwp_W(4), F_Bwp_W(5));

    /// \todo See comment above re CoM and CoM
    thr_Bcm_W = this->linkState.p_BcmBwp_Bcm.Cross(fhr_Bwp_W);
    // thr_Bcm_W = this->linkState.p_BcmBwp_W.Cross(fhr_Bwp_W);
  }

  // enable for additional debug info
  if (this->debugFlags.buoyancyOn)
  {
    gzdbg << "Hydrostatic Forces\n"
      << "Xsi_WwpBwp_W  " << Xsi_WwpBwp_W.transpose().head(3) << " m\n"
      << "              " << Xsi_WwpBwp_W.transpose().tail(3) << " rad\n"
      << "fhb_Bcm_W:    " << fhb_Bcm_W << " N\n"
      << "fhr_Bwp_W:    " << fhr_Bwp_W << " N\n"
      << "fhs_Bcm_W:    " << fhb_Bcm_W + fhr_Bwp_W << " N\n"
      << "thb_Bcm_W:    " << thb_Bcm_W << " N\n"
      << "thr_Bwp_W:    " << thr_Bwp_W << " N\n"
      << "thr_Bcm_W:    " << thr_Bcm_W << " N\n"
      << "ths_Bo_W:     " << thb_Bcm_W + thr_Bwp_W + thr_Bcm_W  << " N\n"
      // << "\n" << K_hs << "\n"
      << "\n";
  }

  // apply forces at CoM
  if (fhb_Bcm_W.IsFinite() && fhr_Bwp_W.IsFinite())
  {
    baseLink.AddWorldForce(_ecm, fhb_Bcm_W + fhr_Bwp_W);
  }

  // apply torques (forces applied at the link origin)
  if (thb_Bcm_W.IsFinite() && thr_Bwp_W.IsFinite() && thr_Bcm_W.IsFinite())
  {
    baseLink.AddWorldWrench(
        _ecm, gz::math::Vector3d::Zero, thb_Bcm_W + thr_Bwp_W + thr_Bcm_W);
  }

  // publish forces periodically
  double updatePeriod = 1.0/this->publishers.updateRate;
  double currentTime = std::chrono::duration<double>(_info.simTime).count();
  if (this->publishers.buoyancyOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), fhb_Bcm_W);
    msgs::Set(msg.mutable_torque(), thb_Bcm_W);
    this->publishers.buoyancyPub.Publish(msg);
  }

  if (this->publishers.hydrostaticRestoringOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), fhr_Bwp_W);
    msgs::Set(msg.mutable_torque(), thr_Bwp_W + thr_Bcm_W);
    this->publishers.restoringPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
/// \brief Calculate the hydrodynamic radiation forces.
///
///
void LinearWaveBodyPrivate::UpdateRadiationForces(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  RadiationForcesConstantCoefficients(_info, _ecm);
}

/////////////////////////////////////////////////
void LinearWaveBodyPrivate::RadiationForcesConstantCoefficients(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // short circuit if forces disabled
  if (!this->forceFlags.radiationDampingOn)
    return;

  // parameters
  auto& B = this->hydroForceCoeffs.B;

  // link properties
  gz::sim::Link baseLink(this->linkEntity);

  //
  // Damping
  //

  // velocity in world frame
  auto v_WB_W = baseLink.WorldLinearVelocity(_ecm).value();
  auto w_WB_W = baseLink.WorldAngularVelocity(_ecm).value();

  // velocity in link waterplane frame
  auto v_WB_Bwp = this->linkState.X_WBwp.Inverse().Rot().RotateVector(v_WB_W);
  auto w_WB_Bwp = this->linkState.X_WBwp.Inverse().Rot().RotateVector(w_WB_W);

  // spatial velocity
  Eigen::Vector6d V_WB_W;
  V_WB_W << v_WB_W.X(), v_WB_W.Y(), v_WB_W.Z(),
            w_WB_W.X(), w_WB_W.Y(), w_WB_W.Z();

  Eigen::Vector6d V_WB_Bwp;
  V_WB_Bwp << v_WB_Bwp.X(), v_WB_Bwp.Y(), v_WB_Bwp.Z(),
              w_WB_Bwp.X(), w_WB_Bwp.Y(), w_WB_Bwp.Z();

  /// \note B is wrt to the fixed inertial frame at the initial
  ///       equilibrium position of the body waterplane.

  // radiation damping force in waterplane frame
  Eigen::Vector6d F_Bwp_Bwp = -1 * B * V_WB_Bwp;
  gz::math::Vector3d frd_Bwp_Bwp(F_Bwp_Bwp(0), F_Bwp_Bwp(1), F_Bwp_Bwp(2));
  gz::math::Vector3d trd_Bo_Bwp(F_Bwp_Bwp(3), F_Bwp_Bwp(4), F_Bwp_Bwp(5));

  // radiation damping force in world frame
  Eigen::Vector6d F_Bwp_W = -1 * B * V_WB_W;
  gz::math::Vector3d frd_Bwp_W(F_Bwp_W(0), F_Bwp_W(1), F_Bwp_W(2));
  gz::math::Vector3d trd_Bwp_W(F_Bwp_W(3), F_Bwp_W(4), F_Bwp_W(5));
  gz::math::Vector3d trd_Bcm_W = this->linkState.p_BcmBwp_W.Cross(frd_Bwp_W);

  /// \todo zero out contribitions
  trd_Bcm_W = gz::math::Vector3d::Zero;

  // enable for additional debug info
  if (this->debugFlags.radiationDampingOn)
  {
    gzdbg << "Radiation Damping Forces\n"
          << "v_WB_Bwp:     " << v_WB_Bwp << " m/s\n"
          << "v_WB_W:       " << v_WB_W << " m/s\n"
          << "w_WB_Bwp:     " << w_WB_Bwp << " rad/s\n"
          << "w_WB_W:       " << w_WB_W << " rad/s\n"
          << "frd_Bwp_W:    " << frd_Bwp_W << " N\n"
          << "trd_Bwp_W:    " << trd_Bwp_W << " N m\n"
          << "trd_Bcm_W:    " << trd_Bcm_W << " N m\n"
          << "\n";
  }

  // apply forces at CoM
  if (frd_Bwp_W.IsFinite())
  {
    baseLink.AddWorldForce(_ecm, frd_Bwp_W);
  }

  // apply torques (forces applied at the link origin)
  if (trd_Bwp_W.IsFinite() && trd_Bcm_W.IsFinite())
  {
    baseLink.AddWorldWrench(_ecm,
        gz::math::Vector3d::Zero, trd_Bwp_W + trd_Bcm_W);
  }

  // publish forces periodically
  double updatePeriod = 1.0/this->publishers.updateRate;
  double currentTime = std::chrono::duration<double>(_info.simTime).count();
  if (this->publishers.radiationDampingOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), frd_Bwp_W);
    msgs::Set(msg.mutable_torque(), trd_Bwp_W + trd_Bcm_W);
    this->publishers.radiationDampingPubl.Publish(msg);
  }

  // force / moment tests
#if 0  

  // 0. pose of CoM in world frame.
  auto X_WBcm = X_WB * X_BBcm;
  auto f_Bcm_Bcm = gz::math::Vector3d::Zero;
  auto f_Bcm_W = gz::math::Vector3d::Zero;
  auto t_Bcm_Bcm    = gz::math::Vector3d::Zero;
  auto t_Bcm_W    = gz::math::Vector3d::Zero;

  // 1. constant force along x-axis applied at CoM in world frame.
  // f_Bcm_W = gz::math::Vector3d(10000, 0 , 0);

  // 2. constant force along x-axis applied at CoM in CoM frame.
  // f_Bcm_Bcm = gz::math::Vector3d(10000, 0 , 0);
  // f_Bcm_W = X_WBcm.Rot().RotateVector(f_Bcm_Bcm);

  // 3. constant torque about z-axis in world frame.
  // t_Bcm_W = gz::math::Vector3d(0, 0, 50000);

  // 4. constant torque about z-axis in CoM frame.
  // t_Bcm_Bcm = gz::math::Vector3d(0, 0, 50000);
  // t_Bcm_W = X_WBcm.Rot().RotateVector(t_Bcm_Bcm);

  // 5. constant force along x-axis applied at CoM in world frame.
  //    constant torque about z-axis in world frame
  // f_Bcm_W = gz::math::Vector3d(10000, 0 , 0);
  // t_Bcm_W = gz::math::Vector3d(0, 0, 50000);

  // 6. constant force along x-axis applied at CoM in world frame.
  //    constant torque about z-axis in world frame
  // f_Bcm_Bcm = gz::math::Vector3d(10000, 0 , 0);
  // f_Bcm_W = X_WBcm.Rot().RotateVector(f_Bcm_Bcm);
  // t_Bcm_Bcm = gz::math::Vector3d(0, 0, 50000);
  // t_Bcm_W = X_WBcm.Rot().RotateVector(t_Bcm_Bcm);

  // apply force and torque
  baseLink.AddWorldForce(_ecm, f_Bcm_W);
  baseLink.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, t_Bcm_W);
#endif
}

/////////////////////////////////////////////////
void LinearWaveBodyPrivate::RadiationForcesImpulseResponseFunction(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // short circuit if forces disabled
  if (!this->forceFlags.radiationDampingOn &&
      !this->forceFlags.radiationAddedMassOn)
    return;

  /// \todo implement convolution integral
}

/////////////////////////////////////////////////
/// \brief Calculate the hydrodynamic excitation forces.
///
///
void LinearWaveBodyPrivate::UpdateExcitationForces(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // short circuit if forces disabled
  if (!this->forceFlags.excitationOn &&
      !this->forceFlags.excitationFroudeKrylovOn &&
      !this->forceFlags.excitationScatteringOn)
    return;

  // parameters
  double time = std::chrono::duration<double>(_info.simTime).count();
  double height = this->waves.height;
  double phase = this->waves.phase;
  double omega = this->simParams.w;
  auto& ex_re = this->hydroForceCoeffs.ex_re;
  auto& ex_im = this->hydroForceCoeffs.ex_im;
  auto& fk_re = this->hydroForceCoeffs.fk_re;
  auto& fk_im = this->hydroForceCoeffs.fk_im;
  auto& sc_re = this->hydroForceCoeffs.sc_re;
  auto& sc_im = this->hydroForceCoeffs.sc_im;

  // link properties
  gz::sim::Link baseLink(this->linkEntity);
  
  // ramp function
  // (http://wec-sim.github.io/WEC-Sim/master/theory/theory.html#ramp-function)
  auto ramp = [](double t) -> double
  {
    // relaxation time
    double tr = 10.0;
    double tau = t/tr;

    if (tau < 1)
      return 0.5*(1.0 + std::sin(3.0/2.0*GZ_PI + GZ_PI*tau));
    else
      return 1.0;
  };

  /// \todo check sign (and convention for wave direction)
  // force calculations
  double wt = omega * time;
  double c = std::cos(wt + phase);
  double s = std::sin(wt + phase);

  gz::math::Vector3d fex_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d tex_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d tex_Bcm_W = gz::math::Vector3d::Zero;

  gz::math::Vector3d ffk_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d tfk_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d tfk_Bcm_W = gz::math::Vector3d::Zero;

  gz::math::Vector3d fsc_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d tsc_Bwp_W = gz::math::Vector3d::Zero;
  gz::math::Vector3d tsc_Bcm_W = gz::math::Vector3d::Zero;

  // Net Excitation - disable if either or both components enabled
  if (!this->forceFlags.excitationFroudeKrylovOn &&
      !this->forceFlags.excitationScatteringOn)
  {
    Eigen::Vector6d F_Bwp_W =
        0.5 * height * ramp(time) * (ex_re * c - ex_im * s);
    fex_Bwp_W = gz::math::Vector3d(F_Bwp_W(0), F_Bwp_W(1), F_Bwp_W(2));
    tex_Bwp_W = gz::math::Vector3d(F_Bwp_W(3), F_Bwp_W(4), F_Bwp_W(5));
    tex_Bcm_W = this->linkState.p_BcmBwp_W.Cross(fex_Bwp_W);
  }

  // Froude-Kylov
  if (this->forceFlags.excitationFroudeKrylovOn)
  {
    Eigen::Vector6d F_Bwp_W =
        0.5 * height * ramp(time) * (fk_re * c - fk_im * s);
    ffk_Bwp_W = gz::math::Vector3d(F_Bwp_W(0), F_Bwp_W(1), F_Bwp_W(2));
    tfk_Bwp_W = gz::math::Vector3d(F_Bwp_W(3), F_Bwp_W(4), F_Bwp_W(5));
    tfk_Bcm_W = this->linkState.p_BcmBwp_W.Cross(ffk_Bwp_W);
  }

  // Scattering
  if (this->forceFlags.excitationScatteringOn)
  {
    Eigen::Vector6d F_Bwp_W =
        0.5 * height * ramp(time) * (sc_re * c - sc_im * s);
    fsc_Bwp_W = gz::math::Vector3d(F_Bwp_W(0), F_Bwp_W(1), F_Bwp_W(2));
    tsc_Bwp_W = gz::math::Vector3d(F_Bwp_W(3), F_Bwp_W(4), F_Bwp_W(5));
    tsc_Bcm_W = this->linkState.p_BcmBwp_W.Cross(fsc_Bwp_W);
  }

  // enable for additional debug info
  if (this->debugFlags.excitationOn)
  {
    gzdbg << "Excitation Forces\n"
          << "fex_Bwp_W:    " << fex_Bwp_W << " N\n"
          << "tex_Bwp_W:    " << tex_Bwp_W << " N m\n"
          << "tex_Bcm_W:    " << tex_Bcm_W << " N m\n"
          << "ffk_Bwp_W:    " << ffk_Bwp_W << " N\n"
          << "tfk_Bwp_W:    " << tfk_Bwp_W << " N m\n"
          << "tfk_Bcm_W:    " << tfk_Bcm_W << " N m\n"
          << "fsc_Bwp_W:    " << fsc_Bwp_W << " N\n"
          << "tsc_Bwp_W:    " << tsc_Bwp_W << " N m\n"
          << "tsc_Bcm_W:    " << tsc_Bcm_W << " N m\n"
          << "\n";
  }

  // apply forces at CoM
  if (fex_Bwp_W.IsFinite() && ffk_Bwp_W.IsFinite() && fsc_Bwp_W.IsFinite())
  {
    baseLink.AddWorldForce(_ecm, fex_Bwp_W + ffk_Bwp_W + fsc_Bwp_W);
  }

  // apply torques (forces applied at the link origin)
  if (tex_Bwp_W.IsFinite() && tex_Bcm_W.IsFinite() &&
      tfk_Bwp_W.IsFinite() && tfk_Bcm_W.IsFinite() && 
      tsc_Bwp_W.IsFinite() && tsc_Bcm_W.IsFinite())
  {
    baseLink.AddWorldWrench(_ecm, gz::math::Vector3d::Zero,
      tex_Bwp_W + tex_Bcm_W + tfk_Bwp_W + tfk_Bcm_W + tsc_Bwp_W + tsc_Bcm_W);
  }

  // publish forces periodically
  double updatePeriod = 1.0/this->publishers.updateRate;
  double currentTime = std::chrono::duration<double>(_info.simTime).count();
  if (this->publishers.excitationOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), fex_Bwp_W);
    msgs::Set(msg.mutable_torque(), tex_Bwp_W + tex_Bcm_W);
    this->publishers.excitationPub.Publish(msg);
  }

  if (this->publishers.excitationFroudeKrylovOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), ffk_Bwp_W);
    msgs::Set(msg.mutable_torque(), tfk_Bwp_W + tfk_Bcm_W);
    this->publishers.excitationFroudeKrylovPubl.Publish(msg);
  }

  if (this->publishers.excitationScatteringOn &&
      (currentTime - this->prevTime) < updatePeriod)
  {
    gz::msgs::Wrench msg;
    msgs::Set(msg.mutable_force(), fsc_Bwp_W);
    msgs::Set(msg.mutable_torque(), tsc_Bwp_W + tsc_Bcm_W);
    this->publishers.excitationScatteringPub.Publish(msg);
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(LinearWaveBody,
              gz::sim::System,
              LinearWaveBody::ISystemConfigure,
              LinearWaveBody::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LinearWaveBody,
                    "gz::sim::systems::LinearWaveBody")
