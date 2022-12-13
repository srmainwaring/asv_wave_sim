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

#include "gz/waves/TrochoidIrregularWaveSimulation.hh"

#include <vector>

#include "gz/waves/Types.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
class TrochoidIrregularWaveSimulation::Impl
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~Impl();

  Impl(
      Index N,
      double L,
      std::shared_ptr<WaveParameters> params);

  void SetWindVelocity(double ux, double uy);

  void SetTime(double time);

  void ElevationAt(
      Eigen::Ref<Eigen::ArrayXXd> _heights);

  void ElevationDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> _dhdx,
      Eigen::Ref<Eigen::ArrayXXd> _dhdy);

  void DisplacementAt(
      Eigen::Ref<Eigen::ArrayXXd> _sx,
      Eigen::Ref<Eigen::ArrayXXd> _sy);

  void DisplacementDerivAt(
      Eigen::Ref<Eigen::ArrayXXd> _dsxdx,
      Eigen::Ref<Eigen::ArrayXXd> _dsydy,
      Eigen::Ref<Eigen::ArrayXXd> _dsxdy);

  Index N_;
  Index N2_;
  Index NOver2_;
  double L_;
  double time_;
  std::shared_ptr<WaveParameters> params_;
};

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::Impl::~Impl()
{
}

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::Impl::Impl(
  Index N,
  double L,
  std::shared_ptr<WaveParameters> params) :
  N_(N),
  N2_(N * N),
  NOver2_(N / 2),
  L_(L),
  params_(params)
{
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::SetWindVelocity(
    double /*ux*/, double /*uy*/)
{
  // @TODO NO IMPLEMENTATION
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::SetTime(double time)
{
  // @TODO NO IMPLEMENTATION
  this->time_ = time;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::ElevationAt(
  Eigen::Ref<Eigen::ArrayXXd> h)
{
  // Multiple wave params
  const auto  number     = this->params_->Number();
  const auto& amplitude  = this->params_->Amplitude_V();
  const auto& wavenumber = this->params_->Wavenumber_V();
  const auto& omega      = this->params_->AngularFrequency_V();
  const auto& phase      = this->params_->Phase_V();
  // const auto& q          = this->params_->Steepness_V();
  const auto& direction  = this->params_->Direction_V();

  // Multiple wave update
  h = Eigen::ArrayXXd::Zero(this->N2_, 0);
  for (Index i=0; i < number; ++i)
  {
    const auto& amplitude_i = amplitude[i];
    const auto& wavenumber_i = wavenumber[i];
    const auto& omega_i = omega[i];
    const auto& phase_i = phase[i];
    const auto& direction_i = direction[i];
    // const auto& q_i = q[i];

    for (Index iy=0; iy < this->N_; ++iy)
    {
      for (Index ix=0; i < this->N_; ++ix)
      {
        // Col major index
        Index idx = iy * this->N_ + ix;

        // Regular grid
        double vx = ix * this->L_ / this->N_ - this->L_ / 2.0;
        double vy = iy * this->L_ / this->N_ - this->L_ / 2.0;

        // Multiple waves
        double ddotx = direction_i.X() * vx + direction_i.Y() * vy;
        double angle  = ddotx * wavenumber_i - omega_i * time_ + phase_i;
        // double s = std::sin(angle);
        double c = std::cos(angle);
        // double sx = - direction_i.X() * q_i * amplitude_i * s;
        // double sy = - direction_i.Y() * q_i * amplitude_i * s;
        double h1 = amplitude_i * c;

        h(idx, 0) += h1;
      }
    }
  }
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::ElevationDerivAt(
  Eigen::Ref<Eigen::ArrayXXd> /*dhdx*/,
  Eigen::Ref<Eigen::ArrayXXd> /*dhdy*/)
{
  // @TODO NO IMPLEMENTATION
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::DisplacementAt(
  Eigen::Ref<Eigen::ArrayXXd> sx,
  Eigen::Ref<Eigen::ArrayXXd> sy)
{
  // Multiple wave params
  const auto  number     = this->params_->Number();
  const auto& amplitude  = this->params_->Amplitude_V();
  const auto& wavenumber = this->params_->Wavenumber_V();
  const auto& omega      = this->params_->AngularFrequency_V();
  const auto& phase      = this->params_->Phase_V();
  const auto& q          = this->params_->Steepness_V();
  const auto& direction  = this->params_->Direction_V();

  // Multiple wave update
  sx = Eigen::ArrayXXd::Zero(this->N2_, 0);
  sy = Eigen::ArrayXXd::Zero(this->N2_, 0);
  for (Index i=0; i < number; ++i)
  {
    const auto& amplitude_i = amplitude[i];
    const auto& wavenumber_i = wavenumber[i];
    const auto& omega_i = omega[i];
    const auto& phase_i = phase[i];
    const auto& direction_i = direction[i];
    const auto& q_i = q[i];

    for (Index iy=0; iy < this->N_; ++iy)
    {
      for (Index ix=0; ix < this->N_; ++ix)
      {
        // Col major index
        Index idx = iy * this->N_ + ix;

        // Regular grid
        double vx = ix * this->L_ / this->N_ - this->L_ / 2.0;
        double vy = iy * this->L_ / this->N_ - this->L_ / 2.0;

        // Multiple waves
        double ddotx = direction_i.X() * vx + direction_i.Y() * vy;
        double angle  = ddotx * wavenumber_i - omega_i * time_ + phase_i;
        double s = std::sin(angle);
        // double c = std::cos(angle);
        double sx1 = - direction_i.X() * q_i * amplitude_i * s;
        double sy1 = - direction_i.Y() * q_i * amplitude_i * s;
        // double h = amplitude_i * c;

        sx(idx, 0) += sx1;
        sy(idx, 0) += sy1;
      }
    }
  }
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::Impl::DisplacementDerivAt(
  Eigen::Ref<Eigen::ArrayXXd> /*dsxdx*/,
  Eigen::Ref<Eigen::ArrayXXd> /*dsydy*/,
  Eigen::Ref<Eigen::ArrayXXd> /*dsxdy*/)
{
  // @TODO NO IMPLEMENTATION
}

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::~TrochoidIrregularWaveSimulation()
{
}

//////////////////////////////////////////////////
TrochoidIrregularWaveSimulation::TrochoidIrregularWaveSimulation(
  Index N,
  double L,
  std::shared_ptr<WaveParameters> params) :
  impl_(new TrochoidIrregularWaveSimulation::Impl(N, L, params))
{
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetWindVelocity(double ux, double uy)
{
  impl_->SetWindVelocity(ux, uy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::SetTime(double time)
{
  impl_->SetTime(time);
}

//////////////////////////////////////////////////
Index TrochoidIrregularWaveSimulation::SizeX() const
{
  return impl_->N_;
}

//////////////////////////////////////////////////
Index TrochoidIrregularWaveSimulation::SizeY() const
{
  return impl_->N_;
}

//////////////////////////////////////////////////
Index TrochoidIrregularWaveSimulation::SizeZ() const
{
  return 0;
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::ElevationAt(
  Eigen::Ref<Eigen::ArrayXXd> h) const
{
  impl_->ElevationAt(h);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::ElevationDerivAt(
  Eigen::Ref<Eigen::ArrayXXd> dhdx,
  Eigen::Ref<Eigen::ArrayXXd> dhdy) const
{
  impl_->ElevationDerivAt(dhdx, dhdy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementAt(
  Eigen::Ref<Eigen::ArrayXXd> sx,
  Eigen::Ref<Eigen::ArrayXXd> sy) const
{
  impl_->DisplacementAt(sx, sy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementDerivAt(
  Eigen::Ref<Eigen::ArrayXXd> dsxdx,
  Eigen::Ref<Eigen::ArrayXXd> dsydy,
  Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
{
  impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementAndDerivAt(
  Eigen::Ref<Eigen::ArrayXXd> h,
  Eigen::Ref<Eigen::ArrayXXd> sx,
  Eigen::Ref<Eigen::ArrayXXd> sy,
  Eigen::Ref<Eigen::ArrayXXd> dhdx,
  Eigen::Ref<Eigen::ArrayXXd> dhdy,
  Eigen::Ref<Eigen::ArrayXXd> dsxdx,
  Eigen::Ref<Eigen::ArrayXXd> dsydy,
  Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
{
  impl_->ElevationAt(h);
  impl_->ElevationDerivAt(dhdx, dhdy);
  impl_->DisplacementAt(sx, sy);
  impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::PressureAt(
    Index ,
    Eigen::Ref<Eigen::ArrayXXd>) const
{
  assert(0 && "Not implemented");
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::ElevationAt(
    Index , Index ,
    double&) const
{
  assert(0 && "Not implemented");
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::DisplacementAt(
    Index , Index ,
    double&, double&) const
{
  assert(0 && "Not implemented");
}

//////////////////////////////////////////////////
void TrochoidIrregularWaveSimulation::PressureAt(
    Index , Index , Index ,
    double&) const
{
  assert(0 && "Not implemented");
}

}  // namespace waves
}  // namespace gz
