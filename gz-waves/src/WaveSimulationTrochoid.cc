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

#include "gz/waves/WaveSimulationTrochoid.hh"

#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"

#include <vector>

namespace gz
{
namespace waves
{

  //////////////////////////////////////////////////
  class WaveSimulationTrochoidImpl
  {
  public:
    ~WaveSimulationTrochoidImpl();

    WaveSimulationTrochoidImpl(
        int N,
        double L,
        std::shared_ptr<WaveParameters> params);

    void SetWindVelocity(double ux, double uy);

    void SetTime(double time);

    void ComputeElevation(
        Eigen::Ref<Eigen::MatrixXd> _heights);

    void ComputeElevationDerivatives(
        Eigen::Ref<Eigen::MatrixXd> _dhdx,
        Eigen::Ref<Eigen::MatrixXd> _dhdy);

    void ComputeDisplacements(
        Eigen::Ref<Eigen::MatrixXd> _sx,
        Eigen::Ref<Eigen::MatrixXd> _sy);

    void ComputeDisplacementsDerivatives(
        Eigen::Ref<Eigen::MatrixXd> _dsxdx,
        Eigen::Ref<Eigen::MatrixXd> _dsydy,
        Eigen::Ref<Eigen::MatrixXd> _dsxdy);
  
    int N_;
    int N2_;
    int NOver2_;
    double L_;
    double time_;
    std::shared_ptr<WaveParameters> params_;
  };

  //////////////////////////////////////////////////
  WaveSimulationTrochoidImpl::~WaveSimulationTrochoidImpl()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationTrochoidImpl::WaveSimulationTrochoidImpl(
    int N,
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
  void WaveSimulationTrochoidImpl::SetWindVelocity(
      double /*ux*/, double /*uy*/)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::SetTime(double time)
  {
    // @TODO NO IMPLEMENTATION
    this->time_ = time;
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> h)
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
    h = Eigen::MatrixXd::Zero(this->N2_, 0);
    for (size_t i=0; i<number; ++i)
    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      // const auto& q_i = q[i];

      for (int iy=0; iy<this->N_; ++iy)
      {
        for (int ix=0; ix<this->N_; ++ix)
        {
          // Col major index
          int idx = iy * this->N_ + ix;

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
  void WaveSimulationTrochoidImpl::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> /*dhdx*/,
    Eigen::Ref<Eigen::MatrixXd> /*dhdy*/)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoidImpl::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy)
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
    sx = Eigen::MatrixXd::Zero(this->N2_, 0);
    sy = Eigen::MatrixXd::Zero(this->N2_, 0);
    for (size_t i=0; i<number; ++i)
    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      const auto& q_i = q[i];

      for (int iy=0; iy<this->N_; ++iy)
      {
        for (int ix=0; ix<this->N_; ++ix)
        {
          // Col major index
          size_t idx = iy * this->N_ + ix;

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
  void WaveSimulationTrochoidImpl::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> /*dsxdx*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsydy*/,
    Eigen::Ref<Eigen::MatrixXd> /*dsxdy*/)
  {
    // @TODO NO IMPLEMENTATION
  }

  //////////////////////////////////////////////////
  WaveSimulationTrochoid::~WaveSimulationTrochoid()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationTrochoid::WaveSimulationTrochoid(
    int N,
    double L,
    std::shared_ptr<WaveParameters> params) :
    impl_(new WaveSimulationTrochoidImpl(N, L, params))
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::SetWindVelocity(double ux, double uy)
  {
    impl_->SetWindVelocity(ux, uy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::SetTime(double time)
  {
    impl_->SetTime(time);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    impl_->ComputeElevation(h);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    impl_->ComputeElevationDerivatives(dhdx, dhdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy)
  {
    impl_->ComputeDisplacements(sx, sy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    impl_->ComputeDisplacementsDerivatives(dsxdx, dsydy, dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationTrochoid::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> h,
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy,
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy,
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    impl_->ComputeElevation(h);
    impl_->ComputeElevationDerivatives(dhdx, dhdy);
    impl_->ComputeDisplacements(sx, sy);
    impl_->ComputeDisplacementsDerivatives(dsxdx, dsydy, dsxdy);
  }
}
}
