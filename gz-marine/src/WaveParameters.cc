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

namespace gz
{
namespace marine
{
  //////////////////////////////////////////////////
  // Utilities
  std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
  { 
    for (auto&& v : _vec )
      os << v << ", ";
    return os;
  }

  //////////////////////////////////////////////////
  // WaveParametersPrivate

  /// \internal
  /// \brief Private data for the WavefieldParameters.
  class WaveParametersPrivate
  {
    /// \brief Constructor.
    public: WaveParametersPrivate() { }

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
    public: gz::math::Vector2d direction = gz::math::Vector2d(1.0, 0.0);

    /// \brief The horizontal wind velocity [m/s].
    public: gz::math::Vector2d windVelocity = gz::math::Vector2d(5.0, 0.0);

    /// \brief The mean wave angular frequency (derived).    
    public: double angularFrequency{2.0*M_PI};

    /// \brief The mean wavelength (derived).
    public: double wavelength{
        2*M_PI/Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)};

    /// \brief The mean wavenumber (derived).
    public: double wavenumber{
        Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)};

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
    public: std::vector<gz::math::Vector2d> directions;

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
        double x = c * this->direction.X() - s * this->direction.Y();
        double y = s * this->direction.X() + c * this->direction.Y();
        gz::math::Vector2d d(x, y);

        directions.push_back(d);
      }
    }
  };
}
}

using namespace gz;
using namespace marine;

//////////////////////////////////////////////////
WaveParameters::~WaveParameters()
{
}

//////////////////////////////////////////////////
WaveParameters::WaveParameters()
  : dataPtr(new WaveParametersPrivate())
{
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::FillMsg(gz::msgs::Param_V& _msg) const
{
  // Clear 
  _msg.mutable_param()->Clear();

  // "number"
  {
    auto nextParam = _msg.add_param();

    (*nextParam->mutable_params())["number"].set_type(gz::msgs::Any::INT32);
    (*nextParam->mutable_params())["number"].set_int_value(this->dataPtr->number);
  }
  // "scale"
  {
    auto nextParam = _msg.add_param();
    (*nextParam->mutable_params())["scale"].set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["scale"].set_double_value(this->dataPtr->scale);
  }
  // "angle"
  {
    auto nextParam = _msg.add_param();
    (*nextParam->mutable_params())["angle"].set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["angle"].set_double_value(this->dataPtr->angle);
  }
  // "steepness"
  {
    auto nextParam = _msg.add_param();
    (*nextParam->mutable_params())["steepness"].set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["steepness"].set_double_value(this->dataPtr->steepness);
  }
  // "amplitude"
  {
    auto nextParam = _msg.add_param();
    (*nextParam->mutable_params())["amplitude"].set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["amplitude"].set_double_value(this->dataPtr->amplitude);
  }
  // "period"
  {
    auto nextParam = _msg.add_param();
    (*nextParam->mutable_params())["period"].set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["period"].set_double_value(this->dataPtr->period);
  }
  // "direction"
  {
    const auto& direction = this->dataPtr->direction;
    auto nextParam = _msg.add_param();
    (*nextParam->mutable_params())["direction"].set_type(gz::msgs::Any::VECTOR3D);
    (*nextParam->mutable_params())["direction"].mutable_vector3d_value()->set_x(direction.X());
    (*nextParam->mutable_params())["direction"].mutable_vector3d_value()->set_y(direction.Y());
    (*nextParam->mutable_params())["direction"].mutable_vector3d_value()->set_z(0);
  }
}

//////////////////////////////////////////////////
void WaveParameters::SetFromMsg(const gz::msgs::Param_V& _msg)
{
  // todo(srmainwaring) add missing entries
  this->dataPtr->number    = Utilities::MsgParamSizeT(_msg,    "number",     this->dataPtr->number);
  this->dataPtr->amplitude = Utilities::MsgParamDouble(_msg,   "amplitude",  this->dataPtr->amplitude);
  this->dataPtr->period    = Utilities::MsgParamDouble(_msg,   "period",     this->dataPtr->period);
  this->dataPtr->phase     = Utilities::MsgParamDouble(_msg,   "phase",      this->dataPtr->phase);
  this->dataPtr->direction = Utilities::MsgParamVector2d(_msg, "direction",  this->dataPtr->direction);
  this->dataPtr->scale     = Utilities::MsgParamDouble(_msg,   "scale",      this->dataPtr->scale);
  this->dataPtr->angle     = Utilities::MsgParamDouble(_msg,   "angle",      this->dataPtr->angle);
  this->dataPtr->steepness = Utilities::MsgParamDouble(_msg,   "steepness",  this->dataPtr->steepness);

  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetFromSDF(sdf::Element& _sdf)
{
  this->dataPtr->algorithm     = Utilities::SdfParamString(_sdf,   "algorithm",  this->dataPtr->algorithm);
  this->dataPtr->tileSize      = Utilities::SdfParamDouble(_sdf,   "tile_size",  this->dataPtr->tileSize);
  this->dataPtr->cellCount     = Utilities::SdfParamSizeT(_sdf,    "cell_count", this->dataPtr->tileSize);
  this->dataPtr->number        = Utilities::SdfParamSizeT(_sdf,    "number",     this->dataPtr->number);
  this->dataPtr->amplitude     = Utilities::SdfParamDouble(_sdf,   "amplitude",  this->dataPtr->amplitude);
  this->dataPtr->period        = Utilities::SdfParamDouble(_sdf,   "period",     this->dataPtr->period);
  this->dataPtr->phase         = Utilities::SdfParamDouble(_sdf,   "phase",      this->dataPtr->phase);
  this->dataPtr->direction     = Utilities::SdfParamVector2d(_sdf, "direction",  this->dataPtr->direction);
  this->dataPtr->scale         = Utilities::SdfParamDouble(_sdf,   "scale",      this->dataPtr->scale);
  this->dataPtr->angle         = Utilities::SdfParamDouble(_sdf,   "angle",      this->dataPtr->angle);
  this->dataPtr->steepness     = Utilities::SdfParamDouble(_sdf,   "steepness",  this->dataPtr->steepness);
  this->dataPtr->windVelocity  = Utilities::SdfParamVector2d(_sdf, "wind_velocity",  this->dataPtr->windVelocity);
  this->dataPtr->Recalculate();

  // override wind speed and angle if parameters are provided
  if (_sdf.HasElement("wind_speed") || _sdf.HasElement("wind_angle_deg"))
  {
    gzmsg << "Overriding 'wind_velocity' using 'wind_speed' and 'wind_angle_deg'\n";

    // current wind speed and angle
    double windSpeed = this->WindSpeed();
    double windAngleRad = this->WindAngleRad();
    double windAngleDeg = 180.0 / M_PI * windAngleRad;

    // override wind speed and angle if parameters are provided
    windSpeed    = Utilities::SdfParamDouble(_sdf, "wind_speed", windSpeed);
    windAngleDeg = Utilities::SdfParamDouble(_sdf, "wind_angle_deg", windAngleDeg);
    windAngleRad = M_PI / 180.0 * windAngleDeg;
    this->SetWindSpeedAndAngle(windSpeed, windAngleRad);
  }
}

//////////////////////////////////////////////////
std::string WaveParameters::Algorithm() const
{
  return this->dataPtr->algorithm;
}

//////////////////////////////////////////////////
double WaveParameters::TileSize() const
{
  return this->dataPtr->tileSize;
}

//////////////////////////////////////////////////
size_t WaveParameters::CellCount() const
{
  return this->dataPtr->cellCount;
}

//////////////////////////////////////////////////
size_t WaveParameters::Number() const
{
  return this->dataPtr->number;
}

//////////////////////////////////////////////////
double WaveParameters::Angle() const
{
  return this->dataPtr->angle;
}

//////////////////////////////////////////////////
double WaveParameters::Scale() const
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
double WaveParameters::Steepness() const
{
  return this->dataPtr->steepness;
}

//////////////////////////////////////////////////
double WaveParameters::AngularFrequency() const
{
  return this->dataPtr->angularFrequency;
}

//////////////////////////////////////////////////
double WaveParameters::Amplitude() const
{
  return this->dataPtr->amplitude;
}

//////////////////////////////////////////////////
double WaveParameters::Period() const
{
  return this->dataPtr->period;
}

//////////////////////////////////////////////////
double WaveParameters::Phase() const
{
  return this->dataPtr->phase;
}

//////////////////////////////////////////////////
double WaveParameters::Wavelength() const
{
  return this->dataPtr->wavelength;
}

//////////////////////////////////////////////////
double WaveParameters::Wavenumber() const
{
  return this->dataPtr->wavenumber;
}    

//////////////////////////////////////////////////
gz::math::Vector2d WaveParameters::Direction() const
{
  return this->dataPtr->direction;
}

//////////////////////////////////////////////////
gz::math::Vector2d WaveParameters::WindVelocity() const
{
  return this->dataPtr->windVelocity;
}

//////////////////////////////////////////////////
double WaveParameters::WindSpeed() const
{
  double vx = this->dataPtr->windVelocity.X();
  double vy = this->dataPtr->windVelocity.Y();
  double u = sqrt(vx*vx + vy*vy);
  return u;
}

//////////////////////////////////////////////////
double WaveParameters::WindAngleRad() const
{
  double vx = this->dataPtr->windVelocity.X();
  double vy = this->dataPtr->windVelocity.Y();
  double phi_rad = atan2(vy, vx);
  return phi_rad;
}

//////////////////////////////////////////////////
void WaveParameters::SetAlgorithm(const std::string &_algorithm)
{
  this->dataPtr->algorithm = _algorithm;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetTileSize(double _tileSize)
{
  this->dataPtr->tileSize = _tileSize;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetCellCount(size_t _cellCount)
{
  this->dataPtr->cellCount = _cellCount;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetNumber(size_t _number)
{
  this->dataPtr->number = _number;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetAngle(double _angle)
{
  this->dataPtr->angle = _angle;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetScale(double _scale)
{
  this->dataPtr->scale = _scale;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetSteepness(double _steepness)
{
  this->dataPtr->steepness = _steepness;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetAmplitude(double _amplitude)
{
  this->dataPtr->amplitude = _amplitude;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetPeriod(double _period)
{
  this->dataPtr->period = _period;
  this->dataPtr->Recalculate();
}
  
//////////////////////////////////////////////////
void WaveParameters::SetPhase(double _phase)
{
  this->dataPtr->phase = _phase;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetDirection(const gz::math::Vector2d& _direction)
{
  this->dataPtr->direction = _direction;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetWindVelocity(const gz::math::Vector2d& _windVelocity)
{
  this->dataPtr->windVelocity = _windVelocity;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetWindSpeedAndAngle(double _windSpeed, double _windAngleRad)
{
  // update wind velocity
  double ux = _windSpeed * cos(_windAngleRad);
  double uy = _windSpeed * sin(_windAngleRad);
  this->dataPtr->windVelocity.X() = ux;
  this->dataPtr->windVelocity.Y() = uy;
  this->dataPtr->Recalculate();
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::AngularFrequency_V() const
{
  return this->dataPtr->angularFrequencies;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Amplitude_V() const
{
  return this->dataPtr->amplitudes;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Phase_V() const
{
  return this->dataPtr->phases;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Steepness_V() const
{
  return this->dataPtr->steepnesses;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Wavenumber_V() const
{
  return this->dataPtr->wavenumbers;
}

//////////////////////////////////////////////////
const std::vector<gz::math::Vector2d>& WaveParameters::Direction_V() const
{
  return this->dataPtr->directions;
}

//////////////////////////////////////////////////
void WaveParameters::DebugPrint() const
{
  // todo(srmainwaring) add missing entries
  gzmsg << "number:     " << this->dataPtr->number << std::endl;
  gzmsg << "scale:      " << this->dataPtr->scale << std::endl;
  gzmsg << "angle:      " << this->dataPtr->angle << std::endl;
  gzmsg << "period:     " << this->dataPtr->period << std::endl;
  gzmsg << "amplitude:  " << this->dataPtr->amplitudes << std::endl;
  gzmsg << "wavenumber: " << this->dataPtr->wavenumbers << std::endl;
  gzmsg << "omega:      " << this->dataPtr->angularFrequencies << std::endl;
  gzmsg << "phase:      " << this->dataPtr->phases << std::endl;
  gzmsg << "steepness:  " << this->dataPtr->steepnesses << std::endl;
  for (auto&& d : this->dataPtr->directions)
  {
    gzmsg << "direction:  " << d << std::endl;
  }
}

