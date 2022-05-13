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

#include "gz/marine/WaveParameters.hh"
#include "gz/marine/Convert.hh"
#include "gz/marine/Geometry.hh"
#include "gz/marine/Grid.hh"
#include "gz/marine/Physics.hh"
#include "gz/marine/Utilities.hh"

#include <gz/common.hh>
#include <gz/msgs.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include <iostream>
#include <cmath>
#include <string>

namespace ignition
{
namespace marine
{
  // typedef CGAL::Aff_transformation_2<cgal::Kernel> TransformMatrix;

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
      angularFrequency(2.0*M_PI),
      wavelength(2*M_PI/Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)), 
      wavenumber(Physics::DeepWaterDispersionToWavenumber(2.0*M_PI))
    {
    }

    /// \brief The wave algorithm.
    public: std::string algorithm{"fft"};

    /// \brief The size of the wave tile.
    public: double tileSize{256.0};

    /// \brief The size of the wave tile.
    public: size_t cellCount{128};

    /// \brief The number of component waves.
    public: size_t number{1};

    /// \brief Set the scale of the largest and smallest waves. 
    public: double scale{2.0};

    /// \brief Set the angle between component waves and the mean direction.
    public: double angle{2.0*M_PI/10.0};

    /// \brief Control the wave steepness. 0 is sine waves, 1 is Gerstner waves.
    public: double steepness{1.0};

    /// \brief The mean wave amplitude [m].
    public: double amplitude{0.0};

    /// \brief The mean wave period [s]
    public: double period{1.0};

    /// \brief The mean wve phase (not currently enabled).
    public: double phase{0.0};

    /// \brief The mean wave direction.
    public: math::Vector2d direction = math::Vector2d(1.0, 0.0);

    /// \brief The horizontal wind velocity [m/s].
    public: math::Vector2d windVelocity = math::Vector2d(5.0, 0.0);

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
    public: std::vector<ignition::math::Vector2d> directions;

    /// \brief Recalculate all derived quantities from inputs.
    public: void Recalculate()
    {
      // Normalize direction
      this->direction.Normalize();

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
        // const TransformMatrix T(
        //   c, -s,
        //   s,  c
        // );
        // const Vector2 d = T(this->direction);

        double x = c * this->direction.X() - s * this->direction.Y();
        double y = s * this->direction.X() + c * this->direction.Y();
        ignition::math::Vector2d d(x, y);

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

  void WaveParameters::FillMsg(ignition::msgs::Param_V& _msg) const
  {
    // Clear 
    _msg.mutable_param()->Clear();

    // "number"
    {
      auto nextParam = _msg.add_param();

      (*nextParam->mutable_params())["number"].set_type(ignition::msgs::Any::INT32);
      (*nextParam->mutable_params())["number"].set_int_value(this->data->number);
    }
    // "scale"
    {
      auto nextParam = _msg.add_param();
      (*nextParam->mutable_params())["scale"].set_type(ignition::msgs::Any::DOUBLE);
      (*nextParam->mutable_params())["scale"].set_double_value(this->data->scale);
    }
    // "angle"
    {
      auto nextParam = _msg.add_param();
      (*nextParam->mutable_params())["angle"].set_type(ignition::msgs::Any::DOUBLE);
      (*nextParam->mutable_params())["angle"].set_double_value(this->data->angle);
    }
    // "steepness"
    {
      auto nextParam = _msg.add_param();
      (*nextParam->mutable_params())["steepness"].set_type(ignition::msgs::Any::DOUBLE);
      (*nextParam->mutable_params())["steepness"].set_double_value(this->data->steepness);
    }
    // "amplitude"
    {
      auto nextParam = _msg.add_param();
      (*nextParam->mutable_params())["amplitude"].set_type(ignition::msgs::Any::DOUBLE);
      (*nextParam->mutable_params())["amplitude"].set_double_value(this->data->amplitude);
    }
    // "period"
    {
      auto nextParam = _msg.add_param();
      (*nextParam->mutable_params())["period"].set_type(ignition::msgs::Any::DOUBLE);
      (*nextParam->mutable_params())["period"].set_double_value(this->data->period);
    }
    // "direction"
    {
      const auto& direction = this->data->direction;
      auto nextParam = _msg.add_param();
      (*nextParam->mutable_params())["direction"].set_type(ignition::msgs::Any::VECTOR3D);
      (*nextParam->mutable_params())["direction"].mutable_vector3d_value()->set_x(direction.X());
      (*nextParam->mutable_params())["direction"].mutable_vector3d_value()->set_y(direction.Y());
      (*nextParam->mutable_params())["direction"].mutable_vector3d_value()->set_z(0);
    }
  }

  void WaveParameters::SetFromMsg(const ignition::msgs::Param_V& _msg)
  {
    // todo(srmainwaring) add missing entries
    this->data->number    = Utilities::MsgParamSizeT(_msg,    "number",     this->data->number);
    this->data->amplitude = Utilities::MsgParamDouble(_msg,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::MsgParamDouble(_msg,   "period",     this->data->period);
    this->data->phase     = Utilities::MsgParamDouble(_msg,   "phase",      this->data->phase);
    this->data->direction = Utilities::MsgParamVector2d(_msg, "direction",  this->data->direction);
    this->data->scale     = Utilities::MsgParamDouble(_msg,   "scale",      this->data->scale);
    this->data->angle     = Utilities::MsgParamDouble(_msg,   "angle",      this->data->angle);
    this->data->steepness = Utilities::MsgParamDouble(_msg,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  void WaveParameters::SetFromSDF(sdf::Element& _sdf)
  {
    this->data->algorithm     = Utilities::SdfParamString(_sdf,   "algorithm",  this->data->algorithm);
    this->data->tileSize      = Utilities::SdfParamDouble(_sdf,   "tile_size",  this->data->tileSize);
    this->data->cellCount     = Utilities::SdfParamSizeT(_sdf,    "cell_count", this->data->tileSize);
    this->data->number        = Utilities::SdfParamSizeT(_sdf,    "number",     this->data->number);
    this->data->amplitude     = Utilities::SdfParamDouble(_sdf,   "amplitude",  this->data->amplitude);
    this->data->period        = Utilities::SdfParamDouble(_sdf,   "period",     this->data->period);
    this->data->phase         = Utilities::SdfParamDouble(_sdf,   "phase",      this->data->phase);
    this->data->direction     = Utilities::SdfParamVector2d(_sdf, "direction",  this->data->direction);
    this->data->scale         = Utilities::SdfParamDouble(_sdf,   "scale",      this->data->scale);
    this->data->angle         = Utilities::SdfParamDouble(_sdf,   "angle",      this->data->angle);
    this->data->steepness     = Utilities::SdfParamDouble(_sdf,   "steepness",  this->data->steepness);
    this->data->windVelocity  = Utilities::SdfParamVector2d(_sdf, "wind_velocity",  this->data->windVelocity);

    // override wind speed and angle if parameters are provided
    if (_sdf.HasElement("wind_speed") || _sdf.HasElement("wind_angle_deg"))
    {
      ignmsg << "Overriding 'wind_velocity' using 'wind_speed' and 'wind_angle_deg'\n";

      double vx = this->data->windVelocity.X();
      double vy = this->data->windVelocity.Y();
      double u = sqrt(vx*vx + vy*vy);
      double phi_rad = atan2(vy, vx);
      double phi_deg = 180.0 / M_PI * phi_rad;

      // override wind speed and angle if parameters are provided
      u       = Utilities::SdfParamDouble(_sdf, "wind_speed", u);
      phi_deg = Utilities::SdfParamDouble(_sdf, "wind_angle_deg", phi_deg);
      phi_rad = M_PI / 180.0 * phi_deg;


      // update wind velocity
      vx = u * cos(phi_rad);
      vy = u * sin(phi_rad);
      this->data->windVelocity.X() = vx;
      this->data->windVelocity.Y() = vy;
    }

    this->data->Recalculate();
  }

  std::string WaveParameters::Algorithm() const
  {
    return this->data->algorithm;
  }

  double WaveParameters::TileSize() const
  {
    return this->data->tileSize;
  }

  size_t WaveParameters::CellCount() const
  {
    return this->data->cellCount;
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

  ignition::math::Vector2d WaveParameters::Direction() const
  {
    return this->data->direction;
  }
  
  ignition::math::Vector2d WaveParameters::WindVelocity() const
  {
    return this->data->windVelocity;
  }

  double WaveParameters::WindSpeed() const
  {
    double vx = this->data->windVelocity.X();
    double vy = this->data->windVelocity.Y();
    double u = sqrt(vx*vx + vy*vy);
    return u;
  }

  double WaveParameters::WindAngleRad() const
  {
    double vx = this->data->windVelocity.X();
    double vy = this->data->windVelocity.Y();
    double phi_rad = atan2(vy, vx);
    return phi_rad;
  }

  void WaveParameters::SetAlgorithm(const std::string &_algorithm)
  {
    this->data->algorithm = _algorithm;
    this->data->Recalculate();
  }

  void WaveParameters::SetTileSize(double _tileSize)
  {
    this->data->tileSize = _tileSize;
    this->data->Recalculate();
  }

  void WaveParameters::SetCellCount(size_t _cellCount)
  {
    this->data->cellCount = _cellCount;
    this->data->Recalculate();
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
  
  void WaveParameters::SetDirection(const ignition::math::Vector2d& _direction)
  {
    this->data->direction = _direction;
    this->data->Recalculate();
  }

  void WaveParameters::SetWindVelocity(const ignition::math::Vector2d& _windVelocity)
  {
    this->data->windVelocity = _windVelocity;
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

  const std::vector<ignition::math::Vector2d>& WaveParameters::Direction_V() const
  {
    return this->data->directions;
  }
 
  void WaveParameters::DebugPrint() const
  {
    // todo(srmainwaring) add missing entries
    ignmsg << "number:     " << this->data->number << std::endl;
    ignmsg << "scale:      " << this->data->scale << std::endl;
    ignmsg << "angle:      " << this->data->angle << std::endl;
    ignmsg << "period:     " << this->data->period << std::endl;
    ignmsg << "amplitude:  " << this->data->amplitudes << std::endl;
    ignmsg << "wavenumber: " << this->data->wavenumbers << std::endl;
    ignmsg << "omega:      " << this->data->angularFrequencies << std::endl;
    ignmsg << "phase:      " << this->data->phases << std::endl;
    ignmsg << "steepness:  " << this->data->steepnesses << std::endl;
    for (auto&& d : this->data->directions)
    {
      ignmsg << "direction:  " << d << std::endl;
    }
  }

}
}
