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


#include "gz/waves/LinearRandomFFTWaveSimulation.hh"

#include <Eigen/Dense>

#include <fftw3.h>

#include <complex>
#include <random>
#include <unordered_map>
#include <vector>

#include <gz/common/Console.hh>

#include "gz/waves/Algorithm.hh"
#include "gz/waves/Types.hh"
#include "gz/waves/WaveSpectrum.hh"
#include "gz/waves/WaveSpreadingFunction.hh"
#include "LinearRandomFFTWaveSimulationImpl.hh"

namespace gz
{
namespace waves
{
//////////////////////////////////////////////////
LinearRandomFFTWaveSimulation::Impl::~Impl()
{
  DestroyFFTWPlans();
}

//////////////////////////////////////////////////
LinearRandomFFTWaveSimulation::Impl::Impl(
  double lx, double ly, Index nx, Index ny) :
  lx_(lx),
  ly_(ly),
  nx_(nx),
  ny_(ny)
{
  CreateFFTWPlans();
  ComputeBaseAmplitudes();
}

//////////////////////////////////////////////////
LinearRandomFFTWaveSimulation::Impl::Impl(
  double lx, double ly, double lz, Index nx, Index ny, Index nz) :
  lx_(lx),
  ly_(ly),
  lz_(lz),
  nx_(nx),
  ny_(ny),
  nz_(nz)
{
  CreateFFTWPlans();
  ComputeBaseAmplitudes();
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::SetLambda(double value)
{
  lambda_ = value;
  ComputeBaseAmplitudes();
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::SetWindVelocity(
    double ux, double uy)
{
  // Update wind velocity and recompute base amplitudes.
  u10_ = sqrt(ux*ux + uy *uy);
  phi10_ = atan2(uy, ux);

  ComputeBaseAmplitudes();
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::SetSteepness(double value)
{
  lambda_ = value;
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::SetTime(double time)
{
  ComputeCurrentAmplitudes(time);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h)
{
  // run the FFT
  if (fft_needs_update_[0])
  {
    fftw_execute(fft_plan0_);
    fft_needs_update_[0] = false;
  }

  // change from row to column major storage
  Index n2 = nx_ * ny_;
  h = fft_out0_.reshaped<Eigen::ColMajor>(n2, 1);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy)
{
  // run the FFTs
  if (fft_needs_update_[1])
  {
    fftw_execute(fft_plan1_);
    fft_needs_update_[1] = false;
  }
  if (fft_needs_update_[2])
  {
    fftw_execute(fft_plan2_);
    fft_needs_update_[2] = false;
  }
  // change from row to column major storage
  Index n2 = nx_ * ny_;
  dhdy = fft_out1_.reshaped<Eigen::ColMajor>(n2, 1);
  dhdx = fft_out2_.reshaped<Eigen::ColMajor>(n2, 1);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy)
{
  // run the FFTs
  if (fft_needs_update_[3])
  {
    fftw_execute(fft_plan3_);
    fft_needs_update_[3] = false;
  }
  if (fft_needs_update_[4])
  {
    fftw_execute(fft_plan4_);
    fft_needs_update_[4] = false;
  }

  // change from row to column major storage
  Index n2 = nx_ * ny_;
  sy = fft_out3_.reshaped<Eigen::ColMajor>(n2, 1) * lambda_ * -1.0;
  sx = fft_out4_.reshaped<Eigen::ColMajor>(n2, 1) * lambda_ * -1.0;
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy)
{
  // run the FFTs
  if (fft_needs_update_[5])
  {
    fftw_execute(fft_plan5_);
    fft_needs_update_[5] = false;
  }
  if (fft_needs_update_[6])
  {
    fftw_execute(fft_plan6_);
    fft_needs_update_[6] = false;
  }
  if (fft_needs_update_[7])
  {
    fftw_execute(fft_plan7_);
    fft_needs_update_[7] = false;
  }

  // change from row to column major storage
  Index n2 = nx_ * ny_;
  dsydy = fft_out5_.reshaped<Eigen::ColMajor>(n2, 1) * lambda_ * -1.0;
  dsxdx = fft_out6_.reshaped<Eigen::ColMajor>(n2, 1) * lambda_ * -1.0;
  dsxdy = fft_out7_.reshaped<Eigen::ColMajor>(n2, 1) * lambda_ *  1.0;
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::PressureAt(
    Index iz,
    Eigen::Ref<Eigen::ArrayXXd> pressure)
{
  // run the FFTs
  if (fft_needs_update_[8 + iz])
  {
    fftw_execute(fft_plan_p_[iz]);
    fft_needs_update_[8 + iz] = false;
  }

  // change from row to column major storage
  Index n2 = nx_ * ny_;
  pressure = fft_out_p_[iz].reshaped<Eigen::ColMajor>(n2, 1);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::ElevationAt(
    Index ix, Index iy,
    double &eta)
{
  /// \todo(srmainwaring) running the FFT destroys the inputs for c2r plans

  // run the FFT
  if (fft_needs_update_[0])
  {
    fftw_execute(fft_plan0_);
    fft_needs_update_[0] = false;
  }

  // select value
  eta = fft_out0_(ix, iy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::DisplacementAt(
    Index ix, Index iy,
    double &sx, double &sy)
{
  // run the FFTs
  if (fft_needs_update_[3])
  {
    fftw_execute(fft_plan3_);
    fft_needs_update_[3] = false;
  }
  if (fft_needs_update_[4])
  {
    fftw_execute(fft_plan4_);
    fft_needs_update_[4] = false;
  }

  // change from row to column major storage and scale
  sy = fft_out3_(ix, iy) * lambda_ * -1.0;
  sx = fft_out4_(ix, iy) * lambda_ * -1.0;
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::PressureAt(
    Index ix, Index iy, Index iz,
    double &pressure)
{
  // run the FFT
  if (fft_needs_update_[8 + iz])
  {
    fftw_execute(fft_plan_p_[iz]);
    fft_needs_update_[8 + iz] = false;
  }

  // select value
  pressure = fft_out_p_[iz](ix, iy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::ComputeBaseAmplitudes()
{
  InitWaveNumbers();
  InitPressureGrid();

  // initialise arrays - always update as algo switch may change shape.
  Index n2 = nx_ * ny_;
  Eigen::ArrayXd omega_k;
  {
    cap_psi_2s_root_  = Eigen::ArrayXd::Zero(n2);
    rho_              = Eigen::ArrayXd::Zero(n2);
    sigma_            = Eigen::ArrayXd::Zero(n2);
    omega_k           = Eigen::ArrayXd::Zero(n2);
    zhat_             = Eigen::ArrayXd::Zero(n2);
  }

  // spectrum and spreading functions
  gz::waves::ECKVWaveSpectrum spectrum;
  spectrum.SetGravity(gravity_);
  spectrum.SetU10(u10_);
  spectrum.SetCapOmegaC(cap_omega_c_);

  // standing waves - symmetric spreading function
  gz::waves::ECKVSpreadingFunction spreadingFn1;
  spreadingFn1.SetGravity(gravity_);
  spreadingFn1.SetU10(u10_);
  spreadingFn1.SetCapOmegaC(cap_omega_c_);

  // travelling waves - asymmetric spreading function
  gz::waves::Cos2sSpreadingFunction spreadingFn2;
  spreadingFn2.SetSpread(s_param_);

  // normalisation for sqrt of two-sided discrete elevation variance spectrum
  double cap_psi_norm = 0.5;
  double delta_kx = kx_f_;
  double delta_ky = ky_f_;

  // iid random normals for real and imaginary parts of the amplitudes
  auto seed = std::default_random_engine::default_seed;
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, 1.0);

  // calculate spectrum in fft-order
  for (Index ikx = 0; ikx < nx_; ++ikx)
  {
    double kx = kx_fft_(ikx);
    double kx2 = kx*kx;
    for (Index iky = 0; iky < ny_; ++iky)
    {
      double ky = ky_fft_(iky);
      double ky2 = ky*ky;

      double k = sqrt(kx2 + ky2);
      double phi = atan2(ky, kx);

      // index for flattened array
      Index idx = ikx * ny_ + iky;

      double cap_psi = 0.0;
      if (use_symmetric_spreading_fn_)
      {
        // standing waves - symmetric spreading function
        cap_psi = spreadingFn1.Evaluate(phi, phi10_, k);
      } else {
        // travelling waves - asymmetric spreading function
        cap_psi = spreadingFn2.Evaluate(phi, phi10_, k);
      }
      double cap_s = spectrum.Evaluate(k);
      double cap_psi_2s_fft = cap_s * cap_psi / k;

      // square-root of two-sided discrete elevation variance spectrum
      cap_psi_2s_root_(idx) =
          cap_psi_norm * sqrt(cap_psi_2s_fft * delta_kx * delta_ky);

      // iid random normals
      rho_(idx) = distribution(generator);
      sigma_(idx) = distribution(generator);

      // angular temporal frequency using deep water dispersion
      omega_k(idx) = sqrt(gravity_ * k);
    }
  }

  // compute unique elements of omega_k
  algorithm::unordered_unique(
      omega_k.cbegin(),
      omega_k.cend(),
      &uomega_k_,
      &uindex_,
      &uinverse_);

  // resize workspace for time dependent phase
  ucos_wt_.resize(uomega_k_.size());
  usin_wt_.resize(uomega_k_.size());
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::ComputeCurrentAmplitudes(
    double time)
{
  // set all true
  std::transform(
    fft_needs_update_.cbegin(),
    fft_needs_update_.cend(),
    fft_needs_update_.begin(),
    [] (bool) -> bool { return true; });

  // create 1d views
  auto r = rho_.reshaped();
  auto s = sigma_.reshaped();
  auto psi_root = cap_psi_2s_root_.reshaped();

  // time update for unique omega_k (reduce number of calls to sincos)
  for (size_t uidx = 0; uidx < uomega_k_.size(); ++uidx)
  {
    double wt = uomega_k_[uidx] * time;
    ucos_wt_(uidx) = cos(wt);
    usin_wt_(uidx) = sin(wt);
  }

  // flattened index version
  for (Index ikx = 1; ikx < nx_; ++ikx)
  {
    for (Index iky = 1; iky < ny_/2 + 1; ++iky)
    {
      // index for flattened array (ikx, iky)
      Index idx = ikx * ny_ + iky;
      Index uidx = uinverse_[idx];

      // index for conjugate (nx_-ikx, ny_-iky)
      Index cdx = (nx_-ikx) * ny_ + (ny_-iky);

      zhat_(idx) = complex(
          + (r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx)) * ucos_wt_(uidx)
          + (s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx)) * usin_wt_(uidx),
          - (r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx)) * usin_wt_(uidx)
          + (s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx)) * ucos_wt_(uidx));
    }
  }

  for (Index iky = 1; iky < ny_/2 + 1; ++iky)
  {
    Index ikx = 0;

    // index for flattened array (ikx, iky)
    Index idx = ikx * ny_ + iky;
    Index uidx = uinverse_[idx];

    // index for conjugate (ikx, ny_-iky)
    Index cdx = ikx * ny_ + (ny_-iky);

    zhat_(idx) = complex(
        + (r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx)) * ucos_wt_(uidx)
        + (s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx)) * usin_wt_(uidx),
        - (r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx)) * usin_wt_(uidx)
        + (s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx)) * ucos_wt_(uidx));
    zhat_(cdx, 0) = std::conj(zhat_(idx));
  }

  for (Index ikx = 1; ikx < nx_/2 + 1; ++ikx)
  {
    Index iky = 0;

    // index for flattened array (ikx, iky)
    Index idx = ikx * ny_ + iky;
    Index uidx = uinverse_[idx];

    // index for conjugate (nx_-ikx, iky)
    Index cdx = (nx_-ikx) * ny_ + iky;

    zhat_(idx) = complex(
        + (r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx)) * ucos_wt_(uidx)
        + (s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx)) * usin_wt_(uidx),
        - (r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx)) * usin_wt_(uidx)
        + (s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx)) * ucos_wt_(uidx));
    zhat_(cdx, 0) = std::conj(zhat_(idx));
  }

  zhat_(0, 0) = complex(0.0, 0.0);

  /// write into fft_h_, fft_h_ikx_, fft_h_iky_, etc.
  /// \note the inner loop has iky < ny_ / 2 + 1
  ///       as we exploit the Hermitian symmetry in the FFT
  const complex iunit(0.0, 1.0);
  const complex czero(0.0, 0.0);

  for (Index ikx = 0; ikx < nx_; ++ikx)
  {
    double kx = kx_fft_(ikx);
    double kx2 = kx*kx;
    for (Index iky = 0; iky < ny_/2 + 1; ++iky)
    {
      double ky = ky_fft_(iky);
      double ky2 = ky*ky;
      double k = sqrt(kx2 + ky2);

      // index for flattened arrays
      Index idx = ikx * ny_ + iky;

      complex h  = zhat_(idx);
      complex hi = h * iunit;
      complex hikx = hi * kx;
      complex hiky = hi * ky;

      // Nyquist terms for derivatives must be zero.
      // For an explanation see:
      // https://math.mit.edu/~stevenj/fft-deriv.pdf
      if (ikx == nx_ / 2)
        hikx = czero;
      if (iky == ny_ / 2)
        hiky = czero;

      // elevation
      fft_h_(ikx, iky) = h;
      fft_h_ikx_(ikx, iky) = hikx;
      fft_h_iky_(ikx, iky) = hiky;

      /// \todo(srmainwaring) pressure optimisation - adjust so that the
      /// entry for z = 0 is obtained from  fft_h_ / fft_out0_ / fft_plan0_

      // pressure
      for (Index iz = 0; iz < nz_; ++iz)
      {
        double z = z_(iz);
        double e = std::exp(k * z);
        complex p = e * h;
        fft_in_p_[iz](ikx, iky) = p;
      }

      // displacement and derivatives
      if (std::abs(k) < 1.0E-8)
      {
        fft_sx_(ikx, iky)     = czero;
        fft_sy_(ikx, iky)     = czero;
        fft_h_kxkx_(ikx, iky) = czero;
        fft_h_kyky_(ikx, iky) = czero;
        fft_h_kxky_(ikx, iky) = czero;
      } else {
        complex ook = 1.0 / k;
        complex hok = h * ook;
        complex hiok = hi * ook;
        complex dx = - hiok * kx;
        complex dy = - hiok * ky;
        complex hkxkx = hok * kx2;
        complex hkyky = hok * ky2;
        complex hkxky = hok * kx * ky;

        if (ikx == nx_ / 2)
          dx = czero;
        if (iky == ny_ / 2)
          dy = czero;

        fft_sx_(ikx, iky)     = dx;
        fft_sy_(ikx, iky)     = dy;
        fft_h_kxkx_(ikx, iky) = hkxkx;
        fft_h_kyky_(ikx, iky) = hkyky;
        fft_h_kxky_(ikx, iky) = hkxky;
      }
    }
  }
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::InitWaveNumbers()
{
  kx_fft_  = Eigen::ArrayXd::Zero(nx_);
  ky_fft_  = Eigen::ArrayXd::Zero(ny_);

  // wavenumbers in fft and math ordering
  for (Index ikx = 0; ikx < nx_; ++ikx)
  {
    double kx = (ikx - nx_/2) * kx_f_;
    kx_fft_((ikx + nx_/2) % nx_) = kx;
  }

  for (Index iky = 0; iky < ny_; ++iky)
  {
    double ky = (iky - ny_/2) * ky_f_;
    ky_fft_((iky + ny_/2) % ny_) = ky;
  }
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::InitPressureGrid()
{
  // pressure sample points (z is below the free surface)
  Eigen::ArrayXd zr = Eigen::ArrayXd::Zero(nz_);
  if (nz_ > 1)
  {
    // first element is zero - fill nz - 1 remaining elements
    Eigen::ArrayXd ln_z = Eigen::ArrayXd::LinSpaced(
        nz_ - 1, -std::log(lz_), std::log(lz_));
    zr(Eigen::seq(1, nz_ - 1)) = -1 * Eigen::exp(ln_z);
  }
  z_ = zr.reverse();
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::CreateFFTWPlans()
{
  /// \note the input and output arrays may be overridden during
  ///       planning, so allocate here before initialising.
  ///       https://www.fftw.org/fftw3_doc/Complex-DFTs.html

  // allocate storage for Fourier coefficients
  fft_h_      = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_h_ikx_  = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_h_iky_  = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_sx_     = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_sy_     = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_h_kxkx_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_h_kyky_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);
  fft_h_kxky_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1);

  // elevation
  fft_out0_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);
  fft_out1_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);
  fft_out2_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);

  // xy-displacements
  fft_out3_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);
  fft_out4_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);
  fft_out5_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);
  fft_out6_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);
  fft_out7_ = Eigen::ArrayXXdRowMajor::Zero(nx_, ny_);

  // elevation
  fft_plan0_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_h_.data()),
      reinterpret_cast<double*>(fft_out0_.data()),
      FFTW_ESTIMATE);
  fft_plan1_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_h_ikx_.data()),
      reinterpret_cast<double*>(fft_out1_.data()),
      FFTW_ESTIMATE);
  fft_plan2_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_h_iky_.data()),
      reinterpret_cast<double*>(fft_out2_.data()),
      FFTW_ESTIMATE);

  // xy-displacements
  fft_plan3_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_sx_.data()),
      reinterpret_cast<double*>(fft_out3_.data()),
      FFTW_ESTIMATE);
  fft_plan4_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_sy_.data()),
      reinterpret_cast<double*>(fft_out4_.data()),
      FFTW_ESTIMATE);
  fft_plan5_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_h_kxkx_.data()),
      reinterpret_cast<double*>(fft_out5_.data()),
      FFTW_ESTIMATE);
  fft_plan6_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_h_kyky_.data()),
      reinterpret_cast<double*>(fft_out6_.data()),
      FFTW_ESTIMATE);
  fft_plan7_ = fftw_plan_dft_c2r_2d(nx_, ny_,
      reinterpret_cast<fftw_complex*>(fft_h_kxky_.data()),
      reinterpret_cast<double*>(fft_out7_.data()),
      FFTW_ESTIMATE);

  /// \todo(srmainwaring) pressure optimisation - adjust so that the
  /// entry for z = 0 is obtained from  fft_h_ / fft_out0_ / fft_plan0_

  // pressure
  for (Index iz=0; iz < nz_; ++iz)
  {
    fft_in_p_.push_back(Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_/2+1));
    fft_out_p_.push_back(Eigen::ArrayXXdRowMajor::Zero(nx_, ny_));
    fft_plan_p_.push_back(fftw_plan_dft_c2r_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_in_p_[iz].data()),
        reinterpret_cast<double*>(fft_out_p_[iz].data()),
        FFTW_ESTIMATE));
  }

  // set lazy evaluation flags.
  fft_needs_update_.resize(8 + nz_);
  std::transform(
    fft_needs_update_.cbegin(),
    fft_needs_update_.cend(),
    fft_needs_update_.begin(),
    [] (bool) -> bool { return true; });
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::Impl::DestroyFFTWPlans()
{
  fftw_destroy_plan(fft_plan0_);
  fftw_destroy_plan(fft_plan1_);
  fftw_destroy_plan(fft_plan2_);
  fftw_destroy_plan(fft_plan3_);
  fftw_destroy_plan(fft_plan4_);
  fftw_destroy_plan(fft_plan5_);
  fftw_destroy_plan(fft_plan6_);
  fftw_destroy_plan(fft_plan7_);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
LinearRandomFFTWaveSimulation::~LinearRandomFFTWaveSimulation()
{
}

//////////////////////////////////////////////////
LinearRandomFFTWaveSimulation::LinearRandomFFTWaveSimulation(
    double lx, double ly, Index nx, Index ny) :
  impl_(new LinearRandomFFTWaveSimulation::Impl(lx, ly, nx, ny))
{
}

//////////////////////////////////////////////////
LinearRandomFFTWaveSimulation::LinearRandomFFTWaveSimulation(
    double lx, double ly, double lz, Index nx, Index ny, Index nz) :
  impl_(new LinearRandomFFTWaveSimulation::Impl(lx, ly, lz, nx, ny, nz))
{
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::SetLambda(double value)
{
  impl_->SetLambda(value);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::SetWindVelocity(double ux, double uy)
{
  impl_->SetWindVelocity(ux, uy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::SetSteepness(double value)
{
  impl_->SetSteepness(value);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::SetTime(double value)
{
  impl_->SetTime(value);
}

//////////////////////////////////////////////////
Index LinearRandomFFTWaveSimulation::SizeX() const
{
  return impl_->nx_;
}

//////////////////////////////////////////////////
Index LinearRandomFFTWaveSimulation::SizeY() const
{
  return impl_->ny_;
}

//////////////////////////////////////////////////
Index LinearRandomFFTWaveSimulation::SizeZ() const
{
  return impl_->nz_;
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h) const
{
  impl_->ElevationAt(h);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy) const
{
  impl_->ElevationDerivAt(dhdx, dhdy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy) const
{
  impl_->DisplacementAt(sx, sy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
{
  impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::DisplacementAndDerivAt(
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
void LinearRandomFFTWaveSimulation::PressureAt(
    Index iz,
    Eigen::Ref<Eigen::ArrayXXd> pressure) const
{
  impl_->PressureAt(iz, pressure);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::ElevationAt(
    Index ix, Index iy,
    double& eta) const
{
  impl_->ElevationAt(ix, iy, eta);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::DisplacementAt(
    Index ix, Index iy,
    double& sx, double& sy) const
{
  impl_->DisplacementAt(ix, iy, sx, sy);
}

//////////////////////////////////////////////////
void LinearRandomFFTWaveSimulation::PressureAt(
    Index ix, Index iy, Index iz,
    double& pressure) const
{
  impl_->PressureAt(ix, iy, iz, pressure);
}

}  // namespace waves
}  // namespace gz
