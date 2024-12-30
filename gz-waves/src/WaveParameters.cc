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

#include "gz/waves/WaveParameters.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include "gz/waves/Convert.hh"
#include "gz/waves/Geometry.hh"
#include "gz/waves/Physics.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/Utilities.hh"


namespace gz
{
namespace waves
{
//////////////////////////////////////////////////
// Utilities
std::ostream& operator<<(std::ostream& os, const std::vector<double>& vec)
{
  for (auto& v : vec)
    os << v << ", ";
  return os;
}

//////////////////////////////////////////////////
// WaveParametersPrivate

/// \internal
/// \brief Private data for the WavefieldParameters.
class WaveParametersPrivate
{
 public:
  /// \brief The wave algorithm.
  std::string algorithm_{"fft"};

  /// \brief The size of the wave tile.
  std::array<double, 2> tile_size_{256.0, 256.0};

  /// \brief The size of the wave tile.
  std::array<Index, 2> cell_count_{128, 128};

  /// \brief The number of component waves.
  Index number_{1};

  /// \brief Set the scale of the largest and smallest waves.
  double scale_{2.0};

  /// \brief Set the angle between component waves and the mean direction.
  double angle_{2.0*M_PI/10.0};

  /// \brief Control the wave steepness. 0 is sine waves, 1 is Gerstner waves.
  double steepness_{1.0};

  /// \brief The mean wave amplitude [m].
  double amplitude_{0.0};

  /// \brief The mean wave period [s]
  double period_{1.0};

  /// \brief The mean wve phase (not currently enabled).
  double phase_{0.0};

  /// \brief The mean wave direction.
  gz::math::Vector2d direction_ = gz::math::Vector2d(1.0, 0.0);

  /// \brief The horizontal wind velocity [m/s].
  gz::math::Vector2d wind_velocity_ = gz::math::Vector2d(5.0, 0.0);

  /// \brief The mean wave angular frequency (derived).
  double angular_frequency_{2.0*M_PI};

  /// \brief The mean wavelength (derived).
  double wavelength_{
      2*M_PI/Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)};

  /// \brief The mean wavenumber (derived).
  double wavenumber_{
      Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)};

  /// \brief The component wave angular frequencies (derived).
  std::vector<double> angular_frequencies_;

  /// \brief The component wave amplitudes (derived).
  std::vector<double> amplitudes_;

  /// \brief The component wave phases (derived).
  std::vector<double> phases_;

  /// \brief The component wave steepness factors (derived).
  std::vector<double> steepnesses_;

  /// \brief The component wavenumbers (derived).
  std::vector<double> wavenumbers_;

  /// \brief The component wave dirctions (derived).
  std::vector<gz::math::Vector2d> directions_;

  /// \brief Recalculate all derived quantities from inputs.
  void Recalculate()
  {
    // Normalize direction
    direction_.Normalize();

    // Derived mean values
    angular_frequency_ = 2.0 * M_PI / period_;
    wavenumber_ = Physics::DeepWaterDispersionToWavenumber(
        angular_frequency_);
    wavelength_ = 2.0 * M_PI / wavenumber_;

    // Update components
    angular_frequencies_.clear();
    amplitudes_.clear();
    phases_.clear();
    wavenumbers_.clear();
    steepnesses_.clear();
    directions_.clear();

    for (Index i=0; i < number_; ++i)
    {
      const Index n = i - number_/2;
      const double scale_factor = std::pow(scale_, n);
      const double a = scale_factor * amplitude_;
      const double k = wavenumber_ / scale_factor;
      const double omega = Physics::DeepWaterDispersionToOmega(k);
      const double phi = phase_;
      double q = 0.0;
      constexpr double tol = 1.0E-16;
      if (std::fabs(a) > tol)
      {
        q = std::min(1.0, steepness_ / (a * k * number_));
      }

      amplitudes_.push_back(a);
      angular_frequencies_.push_back(omega);
      phases_.push_back(phi);
      steepnesses_.push_back(q);
      wavenumbers_.push_back(k);

      // Direction
      const double c = std::cos(n * angle_);
      const double s = std::sin(n * angle_);
      double x = c * direction_.X() - s * direction_.Y();
      double y = s * direction_.X() + c * direction_.Y();
      gz::math::Vector2d d(x, y);

      directions_.push_back(d);
    }
  }
};

//////////////////////////////////////////////////
WaveParameters::~WaveParameters()
{
}

//////////////////////////////////////////////////
WaveParameters::WaveParameters()
  : impl_(new WaveParametersPrivate())
{
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::FillMsg(gz::msgs::Param_V& msg) const
{
  // Clear
  msg.mutable_param()->Clear();

  // "number"
  {
    auto nextParam = msg.add_param();

    (*nextParam->mutable_params())["number"]
        .set_type(gz::msgs::Any::INT32);
    (*nextParam->mutable_params())["number"]
        .set_int_value(impl_->number_);
  }
  // "scale"
  {
    auto nextParam = msg.add_param();
    (*nextParam->mutable_params())["scale"]
        .set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["scale"]
        .set_double_value(impl_->scale_);
  }
  // "angle"
  {
    auto nextParam = msg.add_param();
    (*nextParam->mutable_params())["angle"]
        .set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["angle"]
        .set_double_value(impl_->angle_);
  }
  // "steepness"
  {
    auto nextParam = msg.add_param();
    (*nextParam->mutable_params())["steepness"]
        .set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["steepness"]
        .set_double_value(impl_->steepness_);
  }
  // "amplitude"
  {
    auto nextParam = msg.add_param();
    (*nextParam->mutable_params())["amplitude"]
        .set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["amplitude"]
        .set_double_value(impl_->amplitude_);
  }
  // "period"
  {
    auto nextParam = msg.add_param();
    (*nextParam->mutable_params())["period"]
        .set_type(gz::msgs::Any::DOUBLE);
    (*nextParam->mutable_params())["period"]
        .set_double_value(impl_->period_);
  }
  // "direction"
  {
    const auto& direction = impl_->direction_;
    auto nextParam = msg.add_param();
    (*nextParam->mutable_params())["direction"]
        .set_type(gz::msgs::Any::VECTOR3D);
    (*nextParam->mutable_params())["direction"]
        .mutable_vector3d_value()->set_x(direction.X());
    (*nextParam->mutable_params())["direction"]
        .mutable_vector3d_value()->set_y(direction.Y());
    (*nextParam->mutable_params())["direction"]
        .mutable_vector3d_value()->set_z(0);
  }
}

//////////////////////////////////////////////////
void WaveParameters::SetFromMsg(const gz::msgs::Param_V& msg)
{
  /// \todo(srmainwaring) add missing entries
  impl_->number_    = Utilities::MsgParamSizeT(
      msg,    "number",     impl_->number_);
  impl_->amplitude_ = Utilities::MsgParamDouble(
      msg,   "amplitude",  impl_->amplitude_);
  impl_->period_    = Utilities::MsgParamDouble(
      msg,   "period",     impl_->period_);
  impl_->phase_     = Utilities::MsgParamDouble(
      msg,   "phase",      impl_->phase_);
  impl_->direction_ = Utilities::MsgParamVector2d(
      msg, "direction",  impl_->direction_);
  impl_->scale_     = Utilities::MsgParamDouble(
      msg,   "scale",      impl_->scale_);
  impl_->angle_     = Utilities::MsgParamDouble(
      msg,   "angle",      impl_->angle_);
  impl_->steepness_ = Utilities::MsgParamDouble(
      msg,   "steepness",  impl_->steepness_);

  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetFromSDF(sdf::Element& sdf)
{
  // find as vector2d or double
  if (sdf.HasElement("tile_size"))
  {
    auto element = sdf.GetElement("tile_size");
    auto param = element->GetValue();
    bool found{false};
    if (!found)
    {
      math::Vector2d value;
      sdf::Errors errors;
      if (param->Get(value, errors))
      {
        impl_->tile_size_ = {value[0], value[1]};
        found = true;
      }
    }
    if (!found)
    {
      double value;
      sdf::Errors errors;
      if (param->Get(value, errors))
      {
        impl_->tile_size_ = {value, value};
        found = true;
      }
    }
    gzmsg << "tile_size: "
        << std::get<0>(impl_->tile_size_) << ", "
        << std::get<1>(impl_->tile_size_) << "\n";
  }

  // find as vector2i or int
  if (sdf.HasElement("cell_count"))
  {
    auto element = sdf.GetElement("cell_count");
    auto param = element->GetValue();
    bool found{false};
    if (!found)
    {
      math::Vector2i value;
      sdf::Errors errors;
      if (param->Get(value, errors))
      {
        impl_->cell_count_ = {value[0], value[1]};
        found = true;
      }
    }
    if (!found)
    {
      int value;
      sdf::Errors errors;
      if (param->Get(value, errors))
      {
        Index index = value;
        impl_->cell_count_ = {index, index};
        found = true;
      }
    }
    gzmsg << "cell_count: "
        << std::get<0>(impl_->cell_count_) << ", "
        << std::get<1>(impl_->cell_count_) << "\n";
  }

  impl_->algorithm_     = Utilities::SdfParamString(
      sdf,    "algorithm",  impl_->algorithm_);
  impl_->number_        = Utilities::SdfParamSizeT(
      sdf,     "number",     impl_->number_);
  impl_->amplitude_     = Utilities::SdfParamDouble(
      sdf,    "amplitude",  impl_->amplitude_);
  impl_->period_        = Utilities::SdfParamDouble(
      sdf,    "period",     impl_->period_);
  impl_->phase_         = Utilities::SdfParamDouble(
      sdf,    "phase",      impl_->phase_);
  impl_->direction_     = Utilities::SdfParamVector2d(
      sdf,  "direction",  impl_->direction_);
  impl_->scale_         = Utilities::SdfParamDouble(
      sdf,    "scale",      impl_->scale_);
  impl_->angle_         = Utilities::SdfParamDouble(
      sdf,    "angle",      impl_->angle_);
  impl_->steepness_     = Utilities::SdfParamDouble(
      sdf,    "steepness",  impl_->steepness_);
  impl_->wind_velocity_  = Utilities::SdfParamVector2d(
      sdf, "wind_velocity",  impl_->wind_velocity_);
  impl_->Recalculate();

  // override wind speed and angle if parameters are provided
  if (sdf.HasElement("wind_speed") || sdf.HasElement("wind_angle_deg"))
  {
    gzmsg << "Overriding 'wind_velocity' using 'wind_speed'"
        << " and 'wind_angle_deg'\n";

    // current wind speed and angle
    double wind_speed = WindSpeed();
    double wind_angle_rad = WindAngleRad();
    double wind_angle_deg = 180.0 / M_PI * wind_angle_rad;

    // override wind speed and angle if parameters are provided
    wind_speed    = Utilities::SdfParamDouble(
        sdf, "wind_speed", wind_speed);
    wind_angle_deg = Utilities::SdfParamDouble(
        sdf, "wind_angle_deg", wind_angle_deg);
    wind_angle_rad = M_PI / 180.0 * wind_angle_deg;
    SetWindSpeedAndAngle(wind_speed, wind_angle_rad);
  }
}

//////////////////////////////////////////////////
std::string WaveParameters::Algorithm() const
{
  return impl_->algorithm_;
}

//////////////////////////////////////////////////
std::array<double, 2> WaveParameters::TileSize() const
{
  return impl_->tile_size_;
}

//////////////////////////////////////////////////
std::array<Index, 2> WaveParameters::CellCount() const
{
  return impl_->cell_count_;
}

//////////////////////////////////////////////////
Index WaveParameters::Number() const
{
  return impl_->number_;
}

//////////////////////////////////////////////////
double WaveParameters::Angle() const
{
  return impl_->angle_;
}

//////////////////////////////////////////////////
double WaveParameters::Scale() const
{
  return impl_->scale_;
}

//////////////////////////////////////////////////
double WaveParameters::Steepness() const
{
  return impl_->steepness_;
}

//////////////////////////////////////////////////
double WaveParameters::AngularFrequency() const
{
  return impl_->angular_frequency_;
}

//////////////////////////////////////////////////
double WaveParameters::Amplitude() const
{
  return impl_->amplitude_;
}

//////////////////////////////////////////////////
double WaveParameters::Period() const
{
  return impl_->period_;
}

//////////////////////////////////////////////////
double WaveParameters::Phase() const
{
  return impl_->phase_;
}

//////////////////////////////////////////////////
double WaveParameters::Wavelength() const
{
  return impl_->wavelength_;
}

//////////////////////////////////////////////////
double WaveParameters::Wavenumber() const
{
  return impl_->wavenumber_;
}

//////////////////////////////////////////////////
gz::math::Vector2d WaveParameters::Direction() const
{
  return impl_->direction_;
}

//////////////////////////////////////////////////
gz::math::Vector2d WaveParameters::WindVelocity() const
{
  return impl_->wind_velocity_;
}

//////////////////////////////////////////////////
double WaveParameters::WindSpeed() const
{
  double vx = impl_->wind_velocity_.X();
  double vy = impl_->wind_velocity_.Y();
  double u = sqrt(vx*vx + vy*vy);
  return u;
}

//////////////////////////////////////////////////
double WaveParameters::WindAngleRad() const
{
  double vx = impl_->wind_velocity_.X();
  double vy = impl_->wind_velocity_.Y();
  double phi_rad = atan2(vy, vx);
  return phi_rad;
}

//////////////////////////////////////////////////
void WaveParameters::SetAlgorithm(const std::string& value)
{
  impl_->algorithm_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetTileSize(double value)
{
  impl_->tile_size_ = {value, value};
  impl_->Recalculate();
}

void WaveParameters::SetTileSize(double lx, double ly)
{
  impl_->tile_size_ = {lx, ly};
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetCellCount(Index value)
{
  impl_->cell_count_ = {value, value};
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetCellCount(Index nx, Index ny)
{
  impl_->cell_count_ = {nx, ny};
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetNumber(Index value)
{
  impl_->number_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetAngle(double value)
{
  impl_->angle_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetScale(double value)
{
  impl_->scale_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetSteepness(double value)
{
  impl_->steepness_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetAmplitude(double value)
{
  impl_->amplitude_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetPeriod(double value)
{
  impl_->period_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetPhase(double value)
{
  impl_->phase_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetDirection(const gz::math::Vector2d& value)
{
  impl_->direction_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetWindVelocity(const gz::math::Vector2d& value)
{
  impl_->wind_velocity_ = value;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
void WaveParameters::SetWindSpeedAndAngle(
    double wind_speed, double wind_angle_rad)
{
  // update wind velocity
  double ux = wind_speed * cos(wind_angle_rad);
  double uy = wind_speed * sin(wind_angle_rad);
  impl_->wind_velocity_.X() = ux;
  impl_->wind_velocity_.Y() = uy;
  impl_->Recalculate();
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::AngularFrequency_V() const
{
  return impl_->angular_frequencies_;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Amplitude_V() const
{
  return impl_->amplitudes_;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Phase_V() const
{
  return impl_->phases_;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Steepness_V() const
{
  return impl_->steepnesses_;
}

//////////////////////////////////////////////////
const std::vector<double>& WaveParameters::Wavenumber_V() const
{
  return impl_->wavenumbers_;
}

//////////////////////////////////////////////////
const std::vector<gz::math::Vector2d>& WaveParameters::Direction_V() const
{
  return impl_->directions_;
}

//////////////////////////////////////////////////
void WaveParameters::DebugPrint() const
{
  // todo(srmainwaring) add missing entries
  gzmsg << "number:     " << impl_->number_ << std::endl;
  gzmsg << "scale:      " << impl_->scale_ << std::endl;
  gzmsg << "angle:      " << impl_->angle_ << std::endl;
  gzmsg << "period:     " << impl_->period_ << std::endl;
  gzmsg << "amplitude:  " << impl_->amplitudes_ << std::endl;
  gzmsg << "wavenumber: " << impl_->wavenumbers_ << std::endl;
  gzmsg << "omega:      " << impl_->angular_frequencies_ << std::endl;
  gzmsg << "phase:      " << impl_->phases_ << std::endl;
  gzmsg << "steepness:  " << impl_->steepnesses_ << std::endl;
  for (auto& d : impl_->directions_)
  {
    gzmsg << "direction:  " << d << std::endl;
  }
}

}  // namespace waves
}  // namespace gz
