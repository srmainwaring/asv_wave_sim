// Copyright (C) 2019  Rhys Mainwaring
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

#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Geometry.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/Physics.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"

#include <CGAL/Aff_transformation_3.h>
#include <CGAL/number_utils.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>

#include <Eigen/Dense>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <tbb/tbb.h>

#include <array>
#include <iostream>
#include <cmath>
#include <string>

namespace asv 
{
  typedef CGAL::Aff_transformation_2<Kernel> TransformMatrix;

///////////////////////////////////////////////////////////////////////////////
// Utilities

  std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
  { 
    for (auto&& v : _vec )
      os << v << ", ";
    return os;
  }

///////////////////////////////////////////////////////////////////////////////
// WaveParametersPrivate

  /// \internal
  /// \brief Private data for the WavefieldParameters.
  class WaveParametersPrivate
  {
    /// \brief Constructor.
    public: WaveParametersPrivate():
      number(1), 
      scale(2.0),
      angle(2.0*M_PI/10.0),
      steepness(1.0),
      amplitude(0.0), 
      period(1.0), 
      phase(0.0), 
      direction(1, 0),
      angularFrequency(2.0*M_PI),
      wavelength(2*M_PI/Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)), 
      wavenumber(Physics::DeepWaterDispersionToWavenumber(2.0*M_PI))
    {
    }

    /// \brief The number of component waves.
    public: size_t number;

    /// \brief Set the scale of the largest and smallest waves. 
    public: double scale;

    /// \brief Set the angle between component waves and the mean direction.
    public: double angle;

    /// \brief Control the wave steepness. 0 is sine waves, 1 is Gerstner waves.
    public: double steepness;

    /// \brief The mean wave amplitude [m].
    public: double amplitude;

    /// \brief The mean wave period [s]
    public: double period;

    /// \brief The mean wve phase (not currently enabled).
    public: double phase;

    /// \brief The mean wave direction.
    public: Vector2 direction;

    /// \brief The mean wave angular frequency (derived).    
    public: double angularFrequency;

    /// \brief The mean wavelength (derived).
    public: double wavelength;

    /// \brief The mean wavenumber (derived).
    public: double wavenumber;
  
    /// \brief The component wave angular frequencies (derived).
    public: std::vector<double> angularFrequencies;

    /// \brief The component wave amplitudes (derived).
    public: std::vector<double> amplitudes;

    /// \brief The component wave phases (derived).
    public: std::vector<double> phases;

    /// \brief The component wave steepness factors (derived).
    public: std::vector<double> steepnesses;

    /// \brief The component wavenumbers (derived).
    public: std::vector<double> wavenumbers;

    /// \brief The component wave dirctions (derived).
    public: std::vector<Vector2> directions;

    /// \brief Recalculate all derived quantities from inputs.
    public: void Recalculate()
    {
      // Normalize direction
      this->direction = Geometry::Normalize(this->direction);

      // Derived mean values
      this->angularFrequency = 2.0 * M_PI / this->period;
      this->wavenumber = Physics::DeepWaterDispersionToWavenumber(this->angularFrequency);
      this->wavelength = 2.0 * M_PI / this->wavenumber;

      // Update components
      this->angularFrequencies.clear();
      this->amplitudes.clear();
      this->phases.clear();
      this->wavenumbers.clear();
      this->steepnesses.clear();
      this->directions.clear();

      for (size_t i=0; i<this->number; ++i)
      {
        const int n = i - this->number/2;
        const double scaleFactor = std::pow(this->scale, n);
        const double a = scaleFactor * this->amplitude;
        const double k = this->wavenumber / scaleFactor;
        const double omega = Physics::DeepWaterDispersionToOmega(k);
        const double phi = this->phase;
        double q = 0.0;
        if (a != 0)
        {
          q = std::min(1.0, this->steepness / (a * k * this->number));
        }

        this->amplitudes.push_back(a);        
        this->angularFrequencies.push_back(omega);
        this->phases.push_back(phi);
        this->steepnesses.push_back(q);
        this->wavenumbers.push_back(k);
      
        // Direction
        const double c = std::cos(n * this->angle);
        const double s = std::sin(n * this->angle);
        const TransformMatrix T(
          c, -s,
          s,  c
        );
        const Vector2 d = T(this->direction);
        directions.push_back(d);
      }
    }
  };

///////////////////////////////////////////////////////////////////////////////
// WaveParameters

  WaveParameters::~WaveParameters()
  {
  }

  WaveParameters::WaveParameters()
    : data(new WaveParametersPrivate())
  {
    this->data->Recalculate();
  }

  void WaveParameters::FillMsg(gazebo::msgs::Param_V& _msg) const
  {
    // Clear 
    _msg.mutable_param()->Clear();

    // "number"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("number");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::INT32);
      nextParam->mutable_value()->set_int_value(this->data->number);
    }
    // "scale"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("scale");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->scale);
    }
    // "angle"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("angle");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->angle);
    }
    // "steepness"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("steepness");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->steepness);
    }
    // "amplitude"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("amplitude");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->amplitude);
    }
    // "period"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("period");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->period);
    }
    // "direction"
    {
      const auto& direction = this->data->direction;
      auto nextParam = _msg.add_param();
      nextParam->set_name("direction");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::VECTOR3D);
      nextParam->mutable_value()->mutable_vector3d_value()->set_x(direction.x());
      nextParam->mutable_value()->mutable_vector3d_value()->set_y(direction.y());
      nextParam->mutable_value()->mutable_vector3d_value()->set_z(0);
    }
  }

  void WaveParameters::SetFromMsg(const gazebo::msgs::Param_V& _msg)
  {
    this->data->number    = Utilities::MsgParamSizeT(_msg,    "number",     this->data->number);
    this->data->amplitude = Utilities::MsgParamDouble(_msg,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::MsgParamDouble(_msg,   "period",     this->data->period);
    this->data->phase     = Utilities::MsgParamDouble(_msg,   "phase",      this->data->phase);
    this->data->direction = Utilities::MsgParamVector2(_msg,  "direction",  this->data->direction);
    this->data->scale     = Utilities::MsgParamDouble(_msg,   "scale",      this->data->scale);
    this->data->angle     = Utilities::MsgParamDouble(_msg,   "angle",      this->data->angle);
    this->data->steepness = Utilities::MsgParamDouble(_msg,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  void WaveParameters::SetFromSDF(sdf::Element& _sdf)
  {
    this->data->number    = Utilities::SdfParamSizeT(_sdf,    "number",     this->data->number);
    this->data->amplitude = Utilities::SdfParamDouble(_sdf,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::SdfParamDouble(_sdf,   "period",     this->data->period);
    this->data->phase     = Utilities::SdfParamDouble(_sdf,   "phase",      this->data->phase);
    this->data->direction = Utilities::SdfParamVector2(_sdf,  "direction",  this->data->direction);
    this->data->scale     = Utilities::SdfParamDouble(_sdf,   "scale",      this->data->scale);
    this->data->angle     = Utilities::SdfParamDouble(_sdf,   "angle",      this->data->angle);
    this->data->steepness = Utilities::SdfParamDouble(_sdf,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  size_t WaveParameters::Number() const
  {
    return this->data->number;
  }

  double WaveParameters::Angle() const
  {
    return this->data->angle;
  }

  double WaveParameters::Scale() const
  {
    return this->data->scale;
  }

  double WaveParameters::Steepness() const
  {
    return this->data->steepness;
  }

  double WaveParameters::AngularFrequency() const
  {
    return this->data->angularFrequency;
  }

  double WaveParameters::Amplitude() const
  {
    return this->data->amplitude;
  }
  
  double WaveParameters::Period() const
  {
    return this->data->period;
  }
  
  double WaveParameters::Phase() const
  {
    return this->data->phase;
  }

  double WaveParameters::Wavelength() const
  {
    return this->data->wavelength;
  }

  double WaveParameters::Wavenumber() const
  {
    return this->data->wavenumber;
  }    

  Vector2 WaveParameters::Direction() const
  {
    return this->data->direction;
  }
  
  void WaveParameters::SetNumber(size_t _number)
  {
    this->data->number = _number;
    this->data->Recalculate();
  }

  void WaveParameters::SetAngle(double _angle)
  {
    this->data->angle = _angle;
    this->data->Recalculate();
  }

  void WaveParameters::SetScale(double _scale)
  {
    this->data->scale = _scale;
    this->data->Recalculate();
  }

  void WaveParameters::SetSteepness(double _steepness)
  {
    this->data->steepness = _steepness;
    this->data->Recalculate();
  }

  void WaveParameters::SetAmplitude(double _amplitude)
  {
    this->data->amplitude = _amplitude;
    this->data->Recalculate();
  }
  
  void WaveParameters::SetPeriod(double _period)
  {
    this->data->period = _period;
    this->data->Recalculate();
  }
    
  void WaveParameters::SetPhase(double _phase)
  {
    this->data->phase = _phase;
    this->data->Recalculate();
  }
  
  void WaveParameters::SetDirection(const Vector2& _direction)
  {
    this->data->direction = _direction;
    this->data->Recalculate();
  }

  const std::vector<double>& WaveParameters::AngularFrequency_V() const
  {
    return this->data->angularFrequencies;
  }

  const std::vector<double>& WaveParameters::Amplitude_V() const
  {
    return this->data->amplitudes;
  }
  
  const std::vector<double>& WaveParameters::Phase_V() const
  {
    return this->data->phases;
  }
  
  const std::vector<double>& WaveParameters::Steepness_V() const
  {
    return this->data->steepnesses;
  }

  const std::vector<double>& WaveParameters::Wavenumber_V() const
  {
    return this->data->wavenumbers;
  }

  const std::vector<Vector2>& WaveParameters::Direction_V() const
  {
    return this->data->directions;
  }
 
  void WaveParameters::DebugPrint() const
  {
    gzmsg << "number:     " << this->data->number << std::endl;
    gzmsg << "scale:      " << this->data->scale << std::endl;
    gzmsg << "angle:      " << this->data->angle << std::endl;
    gzmsg << "period:     " << this->data->period << std::endl;
    gzmsg << "amplitude:  " << this->data->amplitudes << std::endl;
    gzmsg << "wavenumber: " << this->data->wavenumbers << std::endl;
    gzmsg << "omega:      " << this->data->angularFrequencies << std::endl;
    gzmsg << "phase:      " << this->data->phases << std::endl;
    gzmsg << "steepness:  " << this->data->steepnesses << std::endl;
    for (auto&& d : this->data->directions)
    {
      gzmsg << "direction:  " << d << std::endl;
    }
  }

///////////////////////////////////////////////////////////////////////////////
// WavefieldPrivate

  /// \internal
  /// \brief Private data for the WavefieldParameters.
  class WavefieldPrivate
  {
    /// \brief Constructor.
    public: WavefieldPrivate() :
      params(new WaveParameters()),
      size({ 1000, 1000 }),
      cellCount({ 50, 50 })
      // isDirty(false)
    {
    }

    /// \brief Constructor.
    ///
    /// \brief param[in] _size      The dimensions of the wave field [m].
    /// \brief param[in] _cellCount The number of cells in each direction.
    public: WavefieldPrivate(
      const std::array<double, 2>& _size,
      const std::array<size_t, 2>& _cellCount 
    ) :
      params(new WaveParameters()),
      size(_size),
      cellCount(_cellCount)
      // isDirty(false)
    {
    }

    /// \brief Wave parameters
    public: std::shared_ptr<WaveParameters> params;

    /// brief The dimensions of the wave field in each direction [m].    
    public: std::array<double, 2> size;

    /// \brief The number of grid cells in each direction.
    public: std::array<size_t, 2> cellCount;

    /// \brief The initial wave field mesh. This is a triangulated regular grid.
    public: std::shared_ptr<const Grid> initialGrid;

    /// \brief The current position of the wave field.
    public: std::shared_ptr<Grid> grid;
    
    /// \brief Retain the Gazebo mesh as it's required to update visuals.
    // public: std::string name;
    // public: std::shared_ptr<gazebo::common::Mesh> gzMesh;
    // public: gazebo::common::SubMesh* gzSubMesh;
    // public: bool isDirty;
  };

///////////////////////////////////////////////////////////////////////////////
// Wavefield

  Wavefield::~Wavefield()
  {
  }

  Wavefield::Wavefield(
    const std::string& _name) : 
    data(new WavefieldPrivate())
  {
    // Grid
    this->data->initialGrid.reset(new Grid(
      this->data->size, this->data->cellCount));
    this->data->grid.reset(new Grid(
      this->data->size, this->data->cellCount));
    
    // GzMesh
    // this->data->name = _name;
    // this->data->gzMesh = std::make_shared<gazebo::common::Mesh>();
    // this->InitGzMesh();

    // Update
    this->Update(0.0);
  }

  Wavefield::Wavefield(
    const std::string& _name,
    const std::array<double, 2>& _size,
    const std::array<size_t, 2>& _cellCount) : 
    data(new WavefieldPrivate(_size, _cellCount))
  {
    // Grid
    this->data->initialGrid.reset(new Grid(
      this->data->size, this->data->cellCount));
    this->data->grid.reset(new Grid(
      this->data->size, this->data->cellCount));
    
    // GzMesh
    // this->data->name = _name;
    // this->data->gzMesh = std::make_shared<gazebo::common::Mesh>();
    // this->InitGzMesh();

    // Update
    this->Update(0.0);
  }

  std::shared_ptr<const Mesh> Wavefield::GetMesh() const
  {
    return this->data->grid->GetMesh();
  }

  std::shared_ptr<const Grid> Wavefield::GetGrid() const
  {
    return this->data->grid;
  }

  // std::shared_ptr<const gazebo::common::Mesh> Wavefield::GetGzMesh() const
  // {
  //   this->LazyUpdateGzMesh();
  //   return this->data->gzMesh;
  // }

  std::shared_ptr<const WaveParameters> Wavefield::GetParameters() const
  {
    return this->data->params;
  }

  void Wavefield::SetParameters(std::shared_ptr<WaveParameters> _params) const
  {
    GZ_ASSERT(_params != nullptr, "Invalid parameter _params");
    this->data->params = _params;    
  }

  void Wavefield::Update(double _time)
  {
    this->UpdateGerstnerWave(_time);
  
    // this->data->isDirty = true;
  }

  void Wavefield::UpdateGerstnerWave(double _time)
  {
    // Single wave params
    // auto amplitude  = this->data->params->Amplitude();
    // auto wavenumber = this->data->params->Wavenumber();
    // auto omega      = this->data->params->AngularFrequency();
    // auto phase      = this->data->params->Phase();
    // auto q          = this->data->params->Steepness();
    // auto direction  = this->data->params->Direction();

    // Multiple wave params
    const auto  number     = this->data->params->Number();
    const auto& amplitude  = this->data->params->Amplitude_V();
    const auto& wavenumber = this->data->params->Wavenumber_V();
    const auto& omega      = this->data->params->AngularFrequency_V();
    const auto& phase      = this->data->params->Phase_V();
    const auto& q          = this->data->params->Steepness_V();
    const auto& direction  = this->data->params->Direction_V();

    const auto& initMesh = *this->data->initialGrid->GetMesh();
    auto& mesh = *this->data->grid->GetMesh();

    // Reset points to original positions
    for (
      auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
      it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
      ++it.first, ++it.second)
    {
      auto& vtx0 = *it.first;
      auto& vtx1 = *it.second;
      mesh.point(vtx1) = initMesh.point(vtx0);
    }
    
    // Multiple wave update 
    for (size_t i=0; i<number; ++i)
    // size_t i = 2;
    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      const auto& q_i = q[i];

      for (
        auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
        it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
        ++it.first, ++it.second)
      {
        auto& vtx0 = *it.first;
        auto& vtx1 = *it.second;

        const Point3& p0 = initMesh.point(vtx0);
        Vector2 v0(p0.x(), p0.y()); // - CGAL::ORIGIN;        

        // Multiple waves
        const double angle  = CGAL::to_double(direction_i * v0) * wavenumber_i - omega_i * _time + phase_i;
        const double s = std::sin(angle);
        const double c = std::cos(angle);
        Vector3 v1(
          - direction_i.x() * q_i * amplitude_i * s,
          - direction_i.y() * q_i * amplitude_i * s,
          + amplitude_i * c
        );

        mesh.point(vtx1) += v1;
      }
    }

    // Single wave update 
    // for (
    //   auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
    //   it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
    //   ++it.first, ++it.second)
    // {
    //   auto& vtx0 = *it.first;
    //   auto& vtx1 = *it.second;

    //   const Point3& p0 = initMesh.point(vtx0);
    //   Vector3 v0 = p0 - CGAL::ORIGIN;
    //   Vector3 v1 = CGAL::NULL_VECTOR;

    //   auto prj    = direction * v0;
    //   double ang  = CGAL::to_double(prj) * wavenumber - omega * _time + phase;
    //   double sang = std::sin(ang);
    //   double cang = std::cos(ang);
    //   v1 += Vector3(
    //     - direction.x() * q * amplitude * sang,
    //     - direction.y() * q * amplitude * sang,
    //     + amplitude * cang
    //   );
      
    //   mesh.point(vtx1) = p0 + v1;
    // }
  
  }

  // void Wavefield::InitGzMesh()
  // {
  //   auto& grid = *this->data->grid;
  //   auto& mesh = *grid.GetMesh();

  //   // Update Normals
  //   grid.RecalculateNormals();

  //   // Make GzMesh
  //   auto& name = this->data->name;
  //   this->data->gzMesh.reset(new gazebo::common::Mesh());
  //   this->data->gzMesh->SetName(name);

  //   const double Lx = this->data->size[0];
  //   const double Ly = this->data->size[1];

  //   // Create the submesh and retain a pointer 
  //   std::unique_ptr<gazebo::common::SubMesh> gzSubMesh(new gazebo::common::SubMesh());
  //   this->data->gzSubMesh = gzSubMesh.get();

  //   size_t iv = 0;
  //   size_t it = 0;
  //   for(auto&& face : mesh.faces())
  //   {
  //     Triangle tri  = Geometry::MakeTriangle(mesh, face);
  //     auto& normal = grid.GetNormal(it++);

  //     ignition::math::Vector3d ignP0(ToIgn(tri[0]));
  //     ignition::math::Vector3d ignP1(ToIgn(tri[1]));
  //     ignition::math::Vector3d ignP2(ToIgn(tri[2]));
  //     ignition::math::Vector3d ignNormal(ToIgn(normal));

  //     // Vertices
  //     gzSubMesh->AddVertex(ignP0);
  //     gzSubMesh->AddVertex(ignP1);
  //     gzSubMesh->AddVertex(ignP2);

  //     // Vertex normals
  //     gzSubMesh->AddNormal(ignNormal);
  //     gzSubMesh->AddNormal(ignNormal);
  //     gzSubMesh->AddNormal(ignNormal);

  //     // Face
  //     gzSubMesh->AddIndex(iv++);
  //     gzSubMesh->AddIndex(iv++);
  //     gzSubMesh->AddIndex(iv++);

  //     // Vertex texture coordinates
  //     gzSubMesh->AddTexCoord(0.5 + ignP0.X()/Lx, 0.5 - ignP0.Y()/Ly);
  //     gzSubMesh->AddTexCoord(0.5 + ignP1.X()/Lx, 0.5 - ignP1.Y()/Ly);
  //     gzSubMesh->AddTexCoord(0.5 + ignP2.X()/Lx, 0.5 - ignP2.Y()/Ly);
  //   }

  //   this->data->gzMesh->AddSubMesh(gzSubMesh.release());      
  // }

  // void Wavefield::LazyUpdateGzMesh() const
  // {
  //   // Update the Gazebo mesh vertices
  //   if (this->data->isDirty)
  //   {
  //     auto& grid = *this->data->grid;
  //     auto& mesh = *grid.GetMesh();
  //     auto& gzSubMesh = *this->data->gzSubMesh;

  //     // Update Normals
  //     grid.RecalculateNormals();

  //     // Update GzMesh
  //     size_t iv = 0;
  //     size_t it = 0;
  //     for(auto&& face : mesh.faces())
  //     {
  //       Triangle tri  = Geometry::MakeTriangle(mesh, face);
  //       auto& normal = grid.GetNormal(it++);

  //       ignition::math::Vector3d ignP0(ToIgn(tri[0]));
  //       ignition::math::Vector3d ignP1(ToIgn(tri[1]));
  //       ignition::math::Vector3d ignP2(ToIgn(tri[2]));
  //       ignition::math::Vector3d ignNormal(ToIgn(normal));

  //       // Update Vertices and Normals
  //       gzSubMesh.SetVertex(iv, ignP0);
  //       gzSubMesh.SetNormal(iv++, ignNormal);

  //       gzSubMesh.SetVertex(iv, ignP1);
  //       gzSubMesh.SetNormal(iv++, ignNormal);

  //       gzSubMesh.SetVertex(iv, ignP2);
  //       gzSubMesh.SetNormal(iv++, ignNormal);
  //     }
  //     this->data->isDirty = false;
  //   }
  // }

///////////////////////////////////////////////////////////////////////////////    
// WavefieldSamplerPrivate

  /// \internal
  /// \brief Private data for the WavefieldSampler.
  class WavefieldSamplerPrivate
  {
    /// \brief The wavefield grid.
    public: std::shared_ptr<const Wavefield> wavefield;

    /// \brief A sample of the wavefield. This copy is located at the initial pose. 
    public: std::shared_ptr<const Grid> initWaterPatch;

    /// \brief A sample of the wavefield. This copy is updated.
    public: std::shared_ptr<Grid> waterPatch;    
  };

///////////////////////////////////////////////////////////////////////////////    
// WavefieldSampler

  WavefieldSampler::~WavefieldSampler()
  {        
  }

  WavefieldSampler::WavefieldSampler(
    std::shared_ptr<const Wavefield> _wavefield,
    std::shared_ptr<const Grid> _waterPatch
  ) : data(new WavefieldSamplerPrivate())
  {
    this->data->wavefield = _wavefield;
    this->data->initWaterPatch = _waterPatch;
    this->data->waterPatch.reset(new Grid(*_waterPatch));
  }

  std::shared_ptr<const Grid> WavefieldSampler::GetWaterPatch() const
  {
    return this->data->waterPatch;
  }

  void WavefieldSampler::ApplyPose(const ignition::math::Pose3d& _pose)
  {
    // @TODO_FRAGILE - Move to Grid as changing internal state 
    // Apply pose to center
    const Point3& c0 = this->data->initWaterPatch->GetCenter();
    Point3 c1(c0.x() + _pose.Pos().X(), c0.y() + _pose.Pos().Y(), c0.z());
    this->data->waterPatch->SetCenter(c1);

    // Iterate over vertices
    auto& source = *this->data->initWaterPatch->GetMesh();
    auto& target = *this->data->waterPatch->GetMesh();
    for (
      auto&& it = std::make_pair(std::begin(source.vertices()), std::begin(target.vertices()));
      it.first != std::end(source.vertices()) && it.second != std::end(target.vertices());
      ++it.first, ++it.second)
    {
      auto& v0 = *it.first;
      auto& v1 = *it.second;
      const Point3& p0 = source.point(v0);

      // Transformation: slide the patch in the xy - plane only
      Point3 p1(p0.x() + _pose.Pos().X(), p0.y() + _pose.Pos().Y(), p0.z());
      target.point(v1) = p1;
    }
  }

  void WavefieldSampler::UpdatePatch()
  {
    // Direction of the line search (i.e. positive z-axis)
    Direction3 direction(0, 0, 1);

    // Update the water patch Mesh
    const auto& target = this->data->waterPatch->GetMesh();
    for (
      auto&& vb = std::begin(target->vertices()); 
      vb != std::end(target->vertices());
      ++vb
    )
    {
      auto& v1 = *vb;

      Point3& origin = target->point(v1);
      Point3 point = CGAL::ORIGIN;

      auto& wavefieldGrid = *this->data->wavefield->GetGrid();
      std::array<size_t, 3> cellIndex = { 0, 0, 0 };
      bool isFound = GridTools::FindIntersectionIndex(
        wavefieldGrid, origin.x(), origin.y(), cellIndex);
      if (!isFound)
      {
        // @DEBUG_INFO
        gzmsg << "origin:   " << origin << std::endl;
        gzerr << "Wavefield is too small" << std::endl;
        return;
      }
      isFound = GridTools::FindIntersectionGrid(
        wavefieldGrid, origin, direction, cellIndex, point);

      if (!isFound)
      {
        // @DEBUG_INFO
        gzmsg << "origin:   " << origin << std::endl;
        gzerr << "Wavefield is too small" << std::endl;
        return;
      }

      target->point(v1) = point;
    }
  }

  double WavefieldSampler::ComputeDepth(const Point3& _point) const
  {
    auto& grid = *this->data->waterPatch;
    return WavefieldSampler::ComputeDepth(grid, _point);

    // @TODO_EXPERIMENTAL - direct calculation (currently static only)
    // auto& waveParams = *this->data->wavefield->GetParameters();
    // return WavefieldSampler::ComputeDepthDirectly(waveParams, _point, 0.0);
  }
  
  double WavefieldSampler::ComputeDepth(  
    const Grid& _patch,
    const Point3& _point
  )
  {
    // Calculate the depth
    Direction3 direction(0, 0, 1);
    Point3 wavePoint = CGAL::ORIGIN;
    std::array<size_t, 3> index;
    bool isFound = GridTools::FindIntersectionIndex(
      _patch, _point.x(), _point.y(), index);
    if (!isFound)
    {
      // @DEBUG_INFO
      gzerr << "point:  " << _point << std::endl;
      // _patch.DebugPrint();
      gzerr << "Water patch is too small" << std::endl;
      return 0;
    }
    isFound = GridTools::FindIntersectionGrid(
      _patch, _point, direction, index, wavePoint);
    if (!isFound)
    {
      // @DEBUG_INFO
      gzerr << "point:  " << _point << std::endl;
      // _patch.DebugPrint();
      gzerr << "Water patch is too small" << std::endl;
      return 0;
    }
    double h = wavePoint.z() - _point.z();
    return h;
  }

  double WavefieldSampler::ComputeDepthDirectly(  
    const WaveParameters& _waveParams,
    const Point3& _point,
    double time
  )
  {
    // Struture for passing wave parameters to lambdas
    struct WaveParams
    {
      WaveParams(
        const std::vector<double>& _a,
        const std::vector<double>& _k,
        const std::vector<double>& _omega,
        const std::vector<double>& _phi,
        const std::vector<double>& _q,
        const std::vector<Vector2>& _dir) :
        a(_a), k(_k), omega(_omega), phi(_phi), q(_q), dir(_dir) {}

      const std::vector<double>& a;
      const std::vector<double>& k;
      const std::vector<double>& omega;
      const std::vector<double>& phi;
      const std::vector<double>& q;
      const std::vector<Vector2>& dir;
    };

    // Compute the target function and Jacobian. Also calculate pz,
    // the z-componen of the Gerstner wave, which we essentially get for free.
    auto wave_fdf = [=](auto x, auto p, auto t, auto& wp, auto& F, auto& J)
    {
      double pz = 0;
      F(0) = p.x() - x.x();
      F(1) = p.y() - x.y();
      J(0, 0) = -1;
      J(0, 1) =  0;
      J(1, 0) =  0;
      J(1, 1) = -1;
      const size_t n = wp.a.size();
      for (auto&& i=0; i<n; ++i)
      {
        const double dx = wp.dir[i].x();
        const double dy = wp.dir[i].y();
        const double q = wp.q[i];
        const double a = wp.a[i];
        const double k = wp.k[i];
        const double dot = x.x() * dx + x.y() * dy;
        const double theta = k * dot - wp.omega[i] * t;
        const double s = std::sin(theta);
        const double c = std::cos(theta);
        const double qakc = q * a * k * c;
        const double df1x = qakc * dx * dx;
        const double df1y = qakc * dx * dy;
        const double df2x = df1y;
        const double df2y = qakc * dy * dy;
        pz += a * c;
        F(0) += a * dx * s;
        F(1) += a * dy * s;
        J(0, 0) += df1x;
        J(0, 1) += df1y;
        J(1, 0) += df2x;
        J(1, 1) += df2y;
      }
      return pz;
    };

    // Simple multi-variate Newton solver - this version returns the z-component of the
    // wave field at the desired point p.
    auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, auto& wp, auto tol, auto nmax)
    {
      int n = 0;
      double err = 1;
      double pz = 0;
      auto xn = x0;
      Eigen::Vector2d F;
      Eigen::Matrix2d J;
      while (std::abs(err) > tol && n < nmax)
      {
        pz = fdfunc(x0, p, t, wp, F, J);
        xn = x0 - J.inverse() * F;
        x0 = xn;
        err = F.norm();
        n++;
      }
      return pz;
    };

    // Set up parameter references
    WaveParams wp(
      _waveParams.Amplitude_V(),
      _waveParams.Wavenumber_V(),
      _waveParams.AngularFrequency_V(),
      _waveParams.Phase_V(),
      _waveParams.Steepness_V(),
      _waveParams.Direction_V()
    );

    // Tolerances etc.
    const double tol = 1.0E-10;
    const double nmax = 30;

    // Use the target point as the initial guess (this is within sum{amplitudes} of the solution)
    Eigen::Vector2d p2(_point.x(), _point.y());
    const double pz = solver(wave_fdf, p2, p2, time, wp, tol, nmax);
    const double h = pz - _point.z();
    return h;
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv
