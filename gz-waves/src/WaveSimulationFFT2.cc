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


// The non-vectorised time-dependent update step labelled 'non-vectorised reference version'
// in WaveSimulationFFT2Impl.ComputeCurrentAmplitudesReference is based on the Curtis Mobley's
// IDL code cgAnimate_2D_SeaSurface.py

//***************************************************************************************************
//* This code is copyright (c) 2016 by Curtis D. Mobley.                                            *
//* Permission is hereby given to reproduce and use this code for non-commercial academic research, *
//* provided that the user suitably acknowledges Curtis D. Mobley in any presentations, reports,    *
//* publications, or other works that make use of the code or its output.  Depending on the extent  *
//* of use of the code or its outputs, suitable acknowledgement can range from a footnote to offer  *
//* of coauthorship.  Further questions can be directed to curtis.mobley@sequoiasci.com.            *
//***************************************************************************************************

#include "gz/waves/WaveSimulationFFT2.hh"

#include <complex>
#include <random>

#include <Eigen/Dense>

#include <fftw3.h>

#include <gz/common.hh>

#include "gz/waves/WaveSpectrum.hh"
#include "WaveSimulationFFT2Impl.hh"

using Eigen::MatrixXcd;
using Eigen::MatrixXd;
using Eigen::VectorXcd;
using Eigen::VectorXd;

namespace gz
{
namespace waves
{
  //////////////////////////////////////////////////
  WaveSimulationFFT2Impl::~WaveSimulationFFT2Impl()
  {
    fftw_destroy_plan(fft_plan0_);
    fftw_destroy_plan(fft_plan1_);
    fftw_destroy_plan(fft_plan2_);
    fftw_destroy_plan(fft_plan3_);
    fftw_destroy_plan(fft_plan4_);
    fftw_destroy_plan(fft_plan5_);
    fftw_destroy_plan(fft_plan6_);
    fftw_destroy_plan(fft_plan7_);

    fftw_free(fft_out0_);
    fftw_free(fft_out1_);
    fftw_free(fft_out2_);
    fftw_free(fft_out3_);
    fftw_free(fft_out4_);
    fftw_free(fft_out5_);
    fftw_free(fft_out6_);
    fftw_free(fft_out7_);

    fftw_free(fft_in0_);
    fftw_free(fft_in1_);
    fftw_free(fft_in2_);
    fftw_free(fft_in3_);
    fftw_free(fft_in4_);
    fftw_free(fft_in5_);
    fftw_free(fft_in6_);
    fftw_free(fft_in7_);
  }

  //////////////////////////////////////////////////
  WaveSimulationFFT2Impl::WaveSimulationFFT2Impl(
    double lx, double ly, int nx, int ny) :
    nx_(nx),
    ny_(ny),
    lx_(lx),
    ly_(ly),
    lambda_(0.6)
  {
    ComputeBaseAmplitudes();

    // FFT 2D.
    size_t n2 = this->nx_ * this->ny_;

    // For height
    fft_in0_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in1_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in2_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));

    // For xy-displacements
    fft_in3_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in4_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in5_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in6_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in7_  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));

    // For height
    fft_out0_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out1_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out2_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  

    // For xy-displacements
    fft_out3_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out4_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out5_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out6_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out7_ = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  

    // For height
    fft_plan0_ = fftw_plan_dft_2d(nx_, ny_, fft_in0_, fft_out0_, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan1_ = fftw_plan_dft_2d(nx_, ny_, fft_in1_, fft_out1_, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan2_ = fftw_plan_dft_2d(nx_, ny_, fft_in2_, fft_out2_, FFTW_BACKWARD, FFTW_ESTIMATE);

    // For xy-displacements
    fft_plan3_ = fftw_plan_dft_2d(nx_, ny_, fft_in3_, fft_out3_, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan4_ = fftw_plan_dft_2d(nx_, ny_, fft_in4_, fft_out4_, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan5_ = fftw_plan_dft_2d(nx_, ny_, fft_in5_, fft_out5_, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan6_ = fftw_plan_dft_2d(nx_, ny_, fft_in6_, fft_out6_, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan7_ = fftw_plan_dft_2d(nx_, ny_, fft_in7_, fft_out7_, FFTW_BACKWARD, FFTW_ESTIMATE);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetUseVectorised(bool value)
  {
    this->use_vectorised = value;
    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetLambda(double value)
  {
    lambda_ = value;
    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetWindVelocity(double ux, double uy)
  {
    // Update wind velocity and recompute base amplitudes.
    this->u10_ = sqrt(ux*ux + uy *uy);
    this->phi10_ = atan2(uy, ux);

    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetTime(double time)
  {
    ComputeCurrentAmplitudes(time);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    // Populate input array
    for (size_t i=0; i<this->nx_ * this->ny_; ++i)
    {
      fft_in0_[i][0] = fft_h_[i].real();
      fft_in0_[i][1] = fft_h_[i].imag();
    }

    // Run the FFT
    fftw_execute(fft_plan0_);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   _heights[i] = fft_out0_[i][0];
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // z(i,j) => z(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny_; ++iky)
      {
        int ij = ikx * this->ny_ + iky;
        int xy = iky * this->nx_ + ikx;
        h(xy, 0) = fft_out0_[ij][0];
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    size_t n2 = this->nx_ * this->ny_;

    // Populate input array
    for (size_t i=0; i<n2; ++i)
    {
      fft_in1_[i][0] = fft_h_ikx_[i].real();
      fft_in1_[i][1] = fft_h_ikx_[i].imag();

      fft_in2_[i][0] = fft_h_iky_[i].real();
      fft_in2_[i][1] = fft_h_iky_[i].imag();
    }

    // Run the FFTs
    fftw_execute(fft_plan1_);
    fftw_execute(fft_plan2_);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   dhdx[i] = fft_out1_[i][0];
    //   dhdy[i] = fft_out2_[i][0];
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // z(i,j) => z(x, y): x = j, y = i
    // dz(i,j)/di => dz(x, y)/dy: x = j, y = i
    // dz(i,j)/dj => dz(x, y)/dx: x = j, y = i
    for (size_t ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny_; ++iky)
      {
        int ij = ikx * this->ny_ + iky;
        int xy = iky * this->nx_ + ikx;
        dhdy(xy, 0) = fft_out1_[ij][0];
        dhdx(xy, 0) = fft_out2_[ij][0];
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy)
  {
    size_t n2 = this->nx_ * this->ny_;

    // Populate input array
    for (size_t i=0; i<n2; ++i)
    {
      fft_in3_[i][0] = fft_sx_[i].real();
      fft_in3_[i][1] = fft_sx_[i].imag();

      fft_in4_[i][0] = fft_sy_[i].real();
      fft_in4_[i][1] = fft_sy_[i].imag();
    }

    // Run the FFTs
    fftw_execute(fft_plan3_);
    fftw_execute(fft_plan4_);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   sx[i] = fft_out3_[i][0] * lambda_;
    //   sy[i] = fft_out4_[i][0] * lambda_;
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // sy(i,j) => si(x, y): x = j, y = i
    // sx(i,j) => sj(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny_; ++iky)
      {
        int ij = ikx * this->ny_ + iky;
        int xy = iky * this->nx_ + ikx;
        sy(xy, 0) = fft_out3_[ij][0] * lambda_ * -1.0;
        sx(xy, 0) = fft_out4_[ij][0] * lambda_ * -1.0;
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    size_t n2 = this->nx_ * this->ny_;

    // Populate input array
    for (size_t i=0; i<n2; ++i)
    {
      fft_in5_[i][0] = fft_h_kxkx_[i].real();
      fft_in5_[i][1] = fft_h_kxkx_[i].imag();

      fft_in6_[i][0] = fft_h_kyky_[i].real();
      fft_in6_[i][1] = fft_h_kyky_[i].imag();

      fft_in7_[i][0] = fft_h_kxky_[i].real();
      fft_in7_[i][1] = fft_h_kxky_[i].imag();
    }

    // Run the FFTs
    fftw_execute(fft_plan5_);
    fftw_execute(fft_plan6_);
    fftw_execute(fft_plan7_);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   _dsxdx[i] = fft_out5_[i][0] * lambda_;
    //   _dsydy[i] = fft_out6_[i][0] * lambda_;
    //   _dsxdy[i] = fft_out7_[i][0] * lambda_;
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // sy(i,j) => si(x, y): x = j, y = i
    // sx(i,j) => sj(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny_; ++iky)
      {
        int ij = ikx * this->ny_ + iky;
        int xy = iky * this->nx_ + ikx;
        dsydy(xy, 0) = fft_out5_[ij][0] * lambda_ * -1.0;
        dsxdx(xy, 0) = fft_out6_[ij][0] * lambda_ * -1.0;
        dsxdy(xy, 0) = fft_out7_[ij][0] * lambda_ *  1.0;
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeBaseAmplitudes()
  {
    // gravity acceleration [m/s^2] 
    const double g = 9.81;

    size_t n2 = this->nx_ * this->ny_;

    // storage for Fourier coefficients
    fft_h_ = Eigen::VectorXcd::Zero(n2);
    fft_h_ikx_ = Eigen::VectorXcd::Zero(n2);
    fft_h_iky_ = Eigen::VectorXcd::Zero(n2);
    fft_sx_ = Eigen::VectorXcd::Zero(n2);
    fft_sy_ = Eigen::VectorXcd::Zero(n2);
    fft_h_kxkx_ = Eigen::VectorXcd::Zero(n2);
    fft_h_kyky_ = Eigen::VectorXcd::Zero(n2);
    fft_h_kxky_ = Eigen::VectorXcd::Zero(n2);

    // continuous two-sided elevation variance spectrum
    Eigen::VectorXd cap_psi_2s_math =
        Eigen::VectorXd::Zero(this->nx_ * this->ny_);

    // calculate spectrum in math-order (not vectorised)
    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      // kx: fftfreq and ifftshift
      const double kx = (ikx - this->nx_/2) * this->kx_f_;
      const double kx2 = kx*kx;
      this->kx_math_[ikx] = kx;
      this->kx_fft_[(ikx + nx_/2) % nx_] = kx;

      for (int iky = 0; iky < this->ny_; ++iky)
      {
        // ky: fftfreq and ifftshift
        const double ky = (iky - this->ny_/2) * this->ky_f_;
        const double ky2 = ky*ky;
        this->ky_math_[iky] = ky;
        this->ky_fft_[(iky + ny_/2) % ny_] = ky;
        
        const double k = sqrt(kx2 + ky2);
        const double phi = atan2(ky, kx);

        // index for flattened array
        int idx = ikx * this->ny_ + iky;

        if (k == 0.0)
        {
          cap_psi_2s_math[idx] = 0.0;
        }
        else
        {
          double cap_psi = 0.0;
          if (this->use_symmetric_spreading_fn_)
          {
            // standing waves - symmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::ECKVSpreadingFunction(
                k, phi - this->phi10_, this->u10_, this->cap_omega_c_);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::Cos2SSpreadingFunction(
                this->s_param_, phi - this->phi10_, this->u10_, this->cap_omega_c_);
          }
          const double cap_s =
              WaveSimulationFFT2Impl::ECKVOmniDirectionalSpectrum(
                  k, this->u10_, this->cap_omega_c_);
          cap_psi_2s_math[idx] = cap_s * cap_psi / k;
        }
      }
    }

    // convert to fft-order
    Eigen::VectorXd cap_psi_2s_fft = Eigen::VectorXd::Zero(this->nx_ * this->ny_);
    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      int ikx_fft = (ikx + nx_/2) % nx_;
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        int iky_fft = (iky + ny_/2) % ny_;

        // index for flattened array
        int idx = ikx * this->ny_ + iky;
        int idx_fft = ikx_fft * this->ny_ + iky_fft;

        cap_psi_2s_fft[idx_fft] = cap_psi_2s_math[idx];
      }
    }

    // square-root of two-sided discrete elevation variance spectrum
    double cap_psi_norm = 0.5;
    double delta_kx = this->kx_f_;
    double delta_ky = this->ky_f_;
    // double c1 = cap_psi_norm * sqrt(delta_kx * delta_ky);

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (int i = 0; i < n2; ++i)
    {
      // this->cap_psi_2s_root[i] = c1 * sqrt(cap_psi_2s_fft[i]);
      this->cap_psi_2s_root_[i] =
          cap_psi_norm * sqrt(cap_psi_2s_fft[i] * delta_kx * delta_ky);

      this->rho_[i] = distribution(generator);
      this->sigma_[i] = distribution(generator);
    }


    // angular temporal frequency for time-dependent (from dispersion)
    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      double kx = this->kx_fft_[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        double ky = this->ky_fft_[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);

        // index for flattened array
        int idx = ikx * this->ny_ + iky;
        this->omega_k_[idx] = sqrt(g * k);
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudes(double time)
  {
    // alias
    auto& nx_ = this->nx_;
    auto& ny_ = this->ny_;
    auto& r = this->rho_;
    auto& s = this->sigma_;
    auto& psi_root = this->cap_psi_2s_root_;

    // time update
    Eigen::VectorXd cos_omega_k = Eigen::VectorXd::Zero(nx_ * ny_);
    Eigen::VectorXd sin_omega_k = Eigen::VectorXd::Zero(nx_ * ny_);
    for (int ikx = 0; ikx < nx_; ++ikx)
    {
      for (int iky = 0; iky < ny_; ++iky)
      {
        // index for flattened array
        int idx = ikx * this->ny_ + iky;

        double omega_t = this->omega_k_[idx] * time;
        cos_omega_k(idx) = cos(omega_t);
        sin_omega_k(idx) = sin(omega_t);
      }
    }

    // non-vectorised reference version
    Eigen::VectorXcd zhat = Eigen::VectorXcd::Zero(nx_ * ny_);
    for (int ikx = 1; ikx < nx_; ++ikx)
    {
      for (int iky = 1; iky < ny_; ++iky)
      {
        // index for flattened array (ikx, iky)
        int idx = ikx * this->ny_ + iky;

        // index for conjugate (nx_-ikx, ny_-iky)
        int cdx = (nx_-ikx) * this->ny_ + (ny_-iky);

        zhat[idx] = complex(
            + ( r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx) ) * cos_omega_k(idx)
            + ( s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx) ) * sin_omega_k(idx),
            - ( r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx) ) * sin_omega_k(idx)
            + ( s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx) ) * cos_omega_k(idx));
      }
    }

    for (int iky = 1; iky < ny_/2+1; ++iky)
    {
      int ikx = 0;

      // index for flattened array (ikx, iky)
      int idx = ikx * this->ny_ + iky;

      // index for conjugate (ikx, ny_-iky)
      int cdx = ikx * this->ny_ + (ny_-iky);

      zhat[idx] = complex(
          + ( r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx) ) * cos_omega_k(idx)
          + ( s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx) ) * sin_omega_k(idx),
          - ( r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx) ) * sin_omega_k(idx)
          + ( s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx) ) * cos_omega_k(idx));
      zhat[cdx] = std::conj(zhat[idx]);
    }

    for (int ikx = 1; ikx < nx_/2+1; ++ikx)
    {
      int iky = 0;

      // index for flattened array (ikx, iky)
      int idx = ikx * this->ny_ + iky;

      // index for conjugate (nx_-ikx, iky)
      int cdx = (nx_-ikx) * this->ny_ + iky;

      zhat[idx] = complex(
          + ( r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx) ) * cos_omega_k(idx)
          + ( s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx) ) * sin_omega_k(idx),
          - ( r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx) ) * sin_omega_k(idx)
          + ( s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx) ) * cos_omega_k(idx));
      zhat[cdx] = std::conj(zhat[idx]);
    }

    zhat[0] = complex(0.0, 0.0);

    // write into fft_h_, fft_h_ikx_, fft_h_iky_, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (int ikx = 0; ikx < nx_; ++ikx)
    {
      double kx = this->kx_fft_[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < ny_; ++iky)
      {
        double ky = this->ky_fft_[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        // index for flattened arrays
        int idx = ikx * ny_ + iky;

        complex h  = zhat[idx];
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        this->fft_h_[idx] = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        this->fft_h_ikx_[idx] = hi * kx;
        this->fft_h_iky_[idx] = hi * ky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          fft_sx_[idx]    = czero;
          fft_sy_[idx]    = czero;
          fft_h_kxkx_[idx] = czero;
          fft_h_kyky_[idx] = czero;
          fft_h_kxky_[idx] = czero;
        }
        else
        {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;
          
          fft_sx_[idx]    = dx;
          fft_sy_[idx]    = dy;
          fft_h_kxkx_[idx] = hkxkx;
          fft_h_kyky_[idx] = hkyky;
          fft_h_kxky_[idx] = hkxky;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeBaseAmplitudesReference()
  {
    size_t n2 = this->nx_ * this->ny_;

    // storage for Fourier coefficients
    fft_h_ = Eigen::VectorXcd::Zero(n2);
    fft_h_ikx_ = Eigen::VectorXcd::Zero(n2);
    fft_h_iky_ = Eigen::VectorXcd::Zero(n2);
    fft_sx_ = Eigen::VectorXcd::Zero(n2);
    fft_sy_ = Eigen::VectorXcd::Zero(n2);
    fft_h_kxkx_ = Eigen::VectorXcd::Zero(n2);
    fft_h_kyky_ = Eigen::VectorXcd::Zero(n2);
    fft_h_kxky_ = Eigen::VectorXcd::Zero(n2);

    // arrays for reference version
    if (this->cap_psi_2s_root_ref_.size() == 0 || 
        this->cap_psi_2s_root_ref_.rows() != this->nx_ ||
        this->cap_psi_2s_root_ref_.cols() != this->ny_)
    {
      this->cap_psi_2s_root_ref_ = Eigen::MatrixXd::Zero(this->nx_, this->ny_);
      this->rho_ref_ = Eigen::MatrixXd::Zero(this->nx_, this->ny_);
      this->sigma_ref_ = Eigen::MatrixXd::Zero(this->nx_, this->ny_);
      this->omega_k_ref_ = Eigen::MatrixXd::Zero(this->nx_, this->ny_);
    }

    // Guide to indexing conventions:  1. index, 2. math-order, 3. fft-order
    // 
    // 1. [ 0,  1,  2,  3,  4,  5,  6,  7]
    // 2. [-4, -3, -2, -1,  0,  1,  2,  3]
    // 3.                 [ 0,  1,  2,  3, -4, -3, -2, -3]
    // 

    // fftfreq and ifftshift
    for(int ikx = 0; ikx < this->nx_; ++ikx)
    {
      const double kx = (ikx - this->nx_/2) * this->kx_f_;
      kx_math_[ikx] = kx;
      kx_fft_[(ikx + nx_/2) % nx_] = kx;
    }

    for(int iky = 0; iky < this->ny_; ++iky)
    {
      const double ky = (iky - this->ny_/2) * this->ky_f_;
      ky_math_[iky] = ky;
      ky_fft_[(iky + ny_/2) % ny_] = ky;
    }

    // debug
    gzmsg << "WaveSimulationFFT2" << "\n";
    gzmsg << "lx:          " << this->lx_ << "\n";
    gzmsg << "ly:          " << this->ly_ << "\n";
    gzmsg << "nx:          " << this->nx_ << "\n";
    gzmsg << "ny:          " << this->ny_ << "\n";
    gzmsg << "delta_x:     " << this->delta_x_ << "\n";
    gzmsg << "delta_y:     " << this->delta_y_ << "\n";
    gzmsg << "lambda_x_f:  " << this->lambda_x_f_ << "\n";
    gzmsg << "lambda_y_f:  " << this->lambda_y_f_ << "\n";
    gzmsg << "nu_x_f:      " << this->nu_x_f_ << "\n";
    gzmsg << "nu_y_f:      " << this->nu_y_f_ << "\n";
    gzmsg << "nu_x_ny:     " << this->nu_x_ny_ << "\n";
    gzmsg << "nu_y_ny:     " << this->nu_y_ny_ << "\n";
    gzmsg << "kx_f:        " << this->kx_f_ << "\n";
    gzmsg << "ky_f:        " << this->ky_f_ << "\n";
    gzmsg << "kx_ny:       " << this->kx_ny_ << "\n";
    gzmsg << "ky_ny:       " << this->ky_ny_ << "\n";
    #if 0
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->kx_fft_) os << v << " "; os << "]\n";
      gzmsg << "kx_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->ky_fft_) os << v << " "; os << "]\n";
      gzmsg << "ky_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->kx_math_) os << v << " "; os << "]\n";
      gzmsg << "kx_math:     " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->ky_math_) os << v << " "; os << "]\n";
      gzmsg << "ky_math:     " << os.str();
    }
    #endif

    // continuous two-sided elevation variance spectrum
    Eigen::MatrixXd cap_psi_2s_math = Eigen::MatrixXd::Zero(this->nx_, this->ny_);

    // calculate spectrum in math-order (not vectorised)
    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        double k = sqrt(this->kx_math_[ikx]*this->kx_math_[ikx]
            + this->ky_math_[iky]*this->ky_math_[iky]);
        double phi = atan2(this->ky_math_[iky], this->kx_math_[ikx]);

        if (k == 0.0)
        {
          cap_psi_2s_math(ikx, iky) = 0.0;
        }
        else
        {
          double cap_psi = 0.0;
          if (this->use_symmetric_spreading_fn_)
          {
            // standing waves - symmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::ECKVSpreadingFunction(
                k, phi - this->phi10_, this->u10_, this->cap_omega_c_);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::Cos2SSpreadingFunction(
                this->s_param_, phi - this->phi10_, this->u10_, this->cap_omega_c_);
          }
          double cap_s = WaveSimulationFFT2Impl::ECKVOmniDirectionalSpectrum(
              k, this->u10_, this->cap_omega_c_);
          cap_psi_2s_math(ikx, iky) = cap_s * cap_psi / k;
        }
      }
    }

    // debug
    #if 0
    {
      std::ostringstream os;
      os << "[\n";
      for (int ikx = 0; ikx < this->nx_; ++ikx)
      {
        os << " [ ";
        for (auto& v : cap_psi_2s_math[ikx])
        {
          os << v << " ";
        }
        os << "]\n";
      }
      os << "]\n";

      gzmsg << "cap_psi_2s:  " << os.str();
    }
    #endif

    // convert to fft-order
    Eigen::MatrixXd cap_psi_2s_fft = Eigen::MatrixXd::Zero(this->nx_, this->ny_);
    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      int ikx_fft = (ikx + nx_/2) % nx_;
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        int iky_fft = (iky + ny_/2) % ny_;
        cap_psi_2s_fft(ikx_fft, iky_fft) = cap_psi_2s_math(ikx, iky);
      }
    }

    // square-root of two-sided discrete elevation variance spectrum
    double cap_psi_norm = 0.5;
    double delta_kx = this->kx_f_;
    double delta_ky = this->ky_f_;

    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        this->cap_psi_2s_root_ref_(ikx, iky) =
            cap_psi_norm * sqrt(cap_psi_2s_fft(ikx, iky) * delta_kx * delta_ky);
      }
    }

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        this->rho_ref_(ikx, iky) = distribution(generator);
        this->sigma_ref_(ikx, iky) = distribution(generator);
      }
    }

    // gravity acceleration [m/s^2] 
    double g = 9.81;

    // angular temporal frequency for time-dependent (from dispersion)
    for (int ikx = 0; ikx < this->nx_; ++ikx)
    {
      double kx = this->kx_fft_[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < this->ny_; ++iky)
      {
        double ky = this->ky_fft_[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        this->omega_k_ref_(ikx, iky) = sqrt(g * k);
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudesReference(double time)
  {
    // alias
    auto& nx_ = this->nx_;
    auto& ny_ = this->ny_;
    auto& r = this->rho_ref_;
    auto& s = this->sigma_ref_;
    auto& psi_root = this->cap_psi_2s_root_ref_;

    // time update
    Eigen::MatrixXd cos_omega_k = Eigen::MatrixXd::Zero(nx_, ny_);
    Eigen::MatrixXd sin_omega_k = Eigen::MatrixXd::Zero(nx_, ny_);
    for (int ikx = 0; ikx < nx_; ++ikx)
    {
      for (int iky = 0; iky < ny_; ++iky)
      {
        cos_omega_k(ikx, iky) = cos(this->omega_k_ref_(ikx, iky) * time);
        sin_omega_k(ikx, iky) = sin(this->omega_k_ref_(ikx, iky) * time);
      }
    }

    // non-vectorised reference version
    Eigen::MatrixXcd zhat = Eigen::MatrixXcd::Zero(nx_, ny_);
    for (int ikx = 1; ikx < nx_; ++ikx)
    {
      for (int iky = 1; iky < ny_; ++iky)
      {
        zhat(ikx, iky) = complex(
            + ( r(ikx, iky) * psi_root(ikx, iky) + r(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky) ) * cos_omega_k(ikx, iky)
            + ( s(ikx, iky) * psi_root(ikx, iky) + s(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky) ) * sin_omega_k(ikx, iky),
            - ( r(ikx, iky) * psi_root(ikx, iky) - r(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky) ) * sin_omega_k(ikx, iky)
            + ( s(ikx, iky) * psi_root(ikx, iky) - s(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky) ) * cos_omega_k(ikx, iky));
      }
    }

    for (int iky = 1; iky < ny_/2+1; ++iky)
    {
      int ikx = 0;
      zhat(ikx, iky) = complex(
          + ( r(ikx, iky) * psi_root(ikx, iky) + r(ikx, ny_-iky) * psi_root(ikx, ny_-iky) ) * cos_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) + s(ikx, ny_-iky) * psi_root(ikx, ny_-iky) ) * sin_omega_k(ikx, iky),
          - ( r(ikx, iky) * psi_root(ikx, iky) - r(ikx, ny_-iky) * psi_root(ikx, ny_-iky) ) * sin_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) - s(ikx, ny_-iky) * psi_root(ikx, ny_-iky) ) * cos_omega_k(ikx, iky));
      zhat(ikx, ny_-iky) = std::conj(zhat(ikx, iky));
    }

    for (int ikx = 1; ikx < nx_/2+1; ++ikx)
    {
      int iky = 0;
      zhat(ikx, iky) = complex(
          + ( r(ikx, iky) * psi_root(ikx, iky) + r(nx_-ikx, iky) * psi_root(nx_-ikx, iky) ) * cos_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) + s(nx_-ikx, iky) * psi_root(nx_-ikx, iky) ) * sin_omega_k(ikx, iky),
          - ( r(ikx, iky) * psi_root(ikx, iky) - r(nx_-ikx, iky) * psi_root(nx_-ikx, iky) ) * sin_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) - s(nx_-ikx, iky) * psi_root(nx_-ikx, iky) ) * cos_omega_k(ikx, iky));
      zhat(nx_-ikx, iky) = std::conj(zhat(ikx, iky));
    }

    zhat(0, 0) = complex(0.0, 0.0);

    /// \todo: change zhat to 1D array and use directly

    // write into fft_h_, fft_h_ikx_, fft_h_iky_, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (int ikx = 0; ikx < nx_; ++ikx)
    {
      double kx = this->kx_fft_[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < ny_; ++iky)
      {
        double ky = this->ky_fft_[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        // index for flattened arrays
        int idx = ikx * ny_ + iky;

        complex h  = zhat(ikx, iky);
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        this->fft_h_[idx] = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        this->fft_h_ikx_[idx] = hi * kx;
        this->fft_h_iky_[idx] = hi * ky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          fft_sx_[idx]    = czero;
          fft_sy_[idx]    = czero;
          fft_h_kxkx_[idx] = czero;
          fft_h_kyky_[idx] = czero;
          fft_h_kxky_[idx] = czero;
        }
        else
        {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;
          
          fft_sx_[idx]    = dx;
          fft_sy_[idx]    = dy;
          fft_h_kxkx_[idx] = hkxkx;
          fft_h_kyky_[idx] = hkyky;
          fft_h_kxky_[idx] = hkxky;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  double WaveSimulationFFT2Impl::ECKVOmniDirectionalSpectrum(
      double k, double u10, double cap_omega_c)
  {
    if (std::abs(k) < 1.0E-8 || std::abs(u10) < 1.0E-8)
    {
      return 0.0;
    }

    double alpha = 0.0081;
    double beta = 1.25;
    double g = 9.81;
    double Cd_10N = 0.00144;
    double u_star = sqrt(Cd_10N) * u10;
    double ao = 0.1733;
    double ap = 4.0;
    double km = 370.0;
    double cm = 0.23;
    double am = 0.13 * u_star / cm;
    
    double gamma = 1.7;
    if (cap_omega_c < 1.0)
    {
      gamma = 1.7;
    }
    else
    {
      gamma = 1.7 + 6.0 * log10(cap_omega_c);
    }

    double sigma = 0.08 * (1.0 + 4.0 * pow(cap_omega_c, -3.0));
    double alpha_p = 0.006 * pow(cap_omega_c, 0.55);

    double alpha_m; 
    if (u_star <= cm)
    {
      alpha_m = 0.01 * (1.0 + log(u_star / cm));
    }
    else
    {
      alpha_m = 0.01 * (1.0 + 3.0 * log(u_star / cm));
    }

    double ko = g / u10 / u10;
    double kp = ko * cap_omega_c * cap_omega_c;

    double cp = sqrt(g / kp);
    double c  = sqrt((g / k) * (1.0 + pow(k / km, 2.0)));
        
    double L_PM = exp(-1.25 * pow(kp / k, 2.0));
    
    double Gamma = exp(-1.0/(2.0 * pow(sigma, 2.0)) * pow(sqrt(k / kp) - 1.0, 2.0));
    
    double J_p = pow(gamma, Gamma);
    
    double F_p = L_PM * J_p * exp(-0.3162 * cap_omega_c * (sqrt(k / kp) - 1.0));

    double F_m = L_PM * J_p * exp(-0.25 * pow(k / km - 1.0, 2.0));

    double B_l = 0.5 * alpha_p * (cp / c) * F_p;
    double B_h = 0.5 * alpha_m * (cm / c) * F_m;
    
    double k3 = k * k * k;
    double S = (B_l + B_h) / k3;

    // debug
    // gzmsg << "g:       " << g << "\n";
    // gzmsg << "Omega_c: " << cap_omega_c << "\n";
    // gzmsg << "Cd10N:   " << Cd_10N << "\n";
    // gzmsg << "ustar:   " << u_star << "\n";
    // gzmsg << "ao:      " << ao << "\n";
    // gzmsg << "ap:      " << ap << "\n";
    // gzmsg << "cm:      " << cm << "\n";
    // gzmsg << "am:      " << am << "\n";
    // gzmsg << "km:      " << km << "\n";
    // gzmsg << "gamma:   " << gamma << "\n";
    // gzmsg << "sigma:   " << sigma << "\n";
    // gzmsg << "alphap:  " << alpha_p << "\n";
    // gzmsg << "alpham:  " << alpha_m << "\n";
    // gzmsg << "ko:      " << ko << "\n";
    // gzmsg << "kp:      " << kp << "\n";
    // gzmsg << "cp:      " << cp << "\n";
    // gzmsg << "c:       " << c << "\n";
    // gzmsg << "Gamma:   " << Gamma << "\n";
    // gzmsg << "fJp:     " << J_p << "\n";
    // gzmsg << "fLpm:    " << L_PM << "\n";
    // gzmsg << "Fp:      " << F_p << "\n";
    // gzmsg << "Bl:      " << B_l << "\n";
    // gzmsg << "Fm:      " << F_m << "\n";
    // gzmsg << "Bh:      " << B_h << "\n";
    
    return S;
  }

  //////////////////////////////////////////////////
  double WaveSimulationFFT2Impl::ECKVSpreadingFunction(
      double k, double phi, double u10, double cap_omega_c)
  {
    double g = 9.81;
    double Cd_10N = 0.00144;
    double u_star = sqrt(Cd_10N) * u10;
    double ao = 0.1733;
    double ap = 4.0;
    double km = 370.0;
    double cm = 0.23;
    double am = 0.13 * u_star / cm;
    double ko = g / u10 / u10;
    double kp = ko * cap_omega_c * cap_omega_c;
    double cp = sqrt(g / kp);
    double c  = sqrt((g / k) * (1 + pow(k / km, 2.0)));
    double cap_phi = (1 + tanh(ao + ap * pow(c / cp, 2.5)
        + am * pow(cm / c, 2.5)) * cos(2.0 * phi))/ 2.0 / M_PI;
    return cap_phi;
  }

  //////////////////////////////////////////////////
  double WaveSimulationFFT2Impl::Cos2SSpreadingFunction(
      double s, double phi, double u10, double cap_omega_c)
  {
    // Longuet-Higgins et al. 'cosine-2S' spreading function
    //
    // Eq. (B.18) from Ocean Optics


    // Note: s is the spreading parameter and in general
    // will depend on k, u10, cap_omega_c
    // s = 2
    double cap_c_s = std::tgamma(s + 1) / std::tgamma(s + 0.5) / 2 / sqrt(M_PI);
    double cap_phi = cap_c_s * pow(cos(phi / 2), 2 * s);
    return cap_phi;
  }

  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  WaveSimulationFFT2::~WaveSimulationFFT2()
  {
  }

  //////////////////////////////////////////////////
  WaveSimulationFFT2::WaveSimulationFFT2(
    double lx, double ly, int nx, int ny) :
    impl_(new WaveSimulationFFT2Impl(lx, ly, nx, ny))
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetUseVectorised(bool value)
  {
    impl_->SetUseVectorised(value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetLambda(double value)
  {
    impl_->SetLambda(value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetWindVelocity(double ux, double uy)
  {
    impl_->SetWindVelocity(ux, uy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetTime(double value)
  {
    impl_->SetTime(value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> h)
  {
    impl_->ComputeElevation(h);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dhdx,
    Eigen::Ref<Eigen::MatrixXd> dhdy)
  {
    impl_->ComputeElevationDerivatives(dhdx, dhdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> sx,
    Eigen::Ref<Eigen::MatrixXd> sy)
  {
    impl_->ComputeDisplacements(sx, sy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> dsxdx,
    Eigen::Ref<Eigen::MatrixXd> dsydy,
    Eigen::Ref<Eigen::MatrixXd> dsxdy)
  {
    impl_->ComputeDisplacementsDerivatives(dsxdx, dsxdy, dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacementsAndDerivatives(
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
