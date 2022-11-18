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
#include "gz/waves/WaveSpectrum.hh"

#include "WaveSimulationFFT2Impl.hh"

#include <Eigen/Dense>

#include <gz/common.hh>

#include <fftw3.h>

#include <complex>
#include <random>

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
    fftw_destroy_plan(fft_plan0);
    fftw_destroy_plan(fft_plan1);
    fftw_destroy_plan(fft_plan2);
    fftw_destroy_plan(fft_plan3);
    fftw_destroy_plan(fft_plan4);
    fftw_destroy_plan(fft_plan5);
    fftw_destroy_plan(fft_plan6);
    fftw_destroy_plan(fft_plan7);

    fftw_free(fft_out0);
    fftw_free(fft_out1);
    fftw_free(fft_out2);
    fftw_free(fft_out3);
    fftw_free(fft_out4);
    fftw_free(fft_out5);
    fftw_free(fft_out6);
    fftw_free(fft_out7);

    fftw_free(fft_in0);
    fftw_free(fft_in1);
    fftw_free(fft_in2);
    fftw_free(fft_in3);
    fftw_free(fft_in4);
    fftw_free(fft_in5);
    fftw_free(fft_in6);
    fftw_free(fft_in7);
  }

  //////////////////////////////////////////////////
  WaveSimulationFFT2Impl::WaveSimulationFFT2Impl(
    double _lx, double _ly, int _nx, int _ny) :
    nx(_nx),
    ny(_ny),
    lx(_lx),
    ly(_ly),
    lambda(0.6)
  {
    ComputeBaseAmplitudes();

    // FFT 2D.
    size_t n2 = this->nx * this->ny;

    // For height
    fft_in0  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in1  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in2  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));

    // For xy-displacements
    fft_in3  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in4  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in5  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in6  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));
    fft_in7  = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));

    // For height
    fft_out0 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out1 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out2 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  

    // For xy-displacements
    fft_out3 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out4 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out5 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out6 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  
    fft_out7 = (fftw_complex*)fftw_malloc(n2 * sizeof(fftw_complex));  

    // For height
    fft_plan0 = fftw_plan_dft_2d(nx, ny, fft_in0, fft_out0, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan1 = fftw_plan_dft_2d(nx, ny, fft_in1, fft_out1, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan2 = fftw_plan_dft_2d(nx, ny, fft_in2, fft_out2, FFTW_BACKWARD, FFTW_ESTIMATE);

    // For xy-displacements
    fft_plan3 = fftw_plan_dft_2d(nx, ny, fft_in3, fft_out3, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan4 = fftw_plan_dft_2d(nx, ny, fft_in4, fft_out4, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan5 = fftw_plan_dft_2d(nx, ny, fft_in5, fft_out5, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan6 = fftw_plan_dft_2d(nx, ny, fft_in6, fft_out6, FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan7 = fftw_plan_dft_2d(nx, ny, fft_in7, fft_out7, FFTW_BACKWARD, FFTW_ESTIMATE);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetUseVectorised(bool _value)
  {
    this->use_vectorised = _value;
    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetLambda(double _value)
  {
    lambda = _value;
    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetWindVelocity(double _ux, double _uy)
  {
    // Update wind velocity and recompute base amplitudes.
    this->u10 = sqrt(_ux*_ux + _uy *_uy);
    this->phi10 = atan2(_uy, _ux);

    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetTime(double _time)
  {
    ComputeCurrentAmplitudes(_time);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> _h)
  {
    // Populate input array
    for (size_t i=0; i<this->nx * this->ny; ++i)
    {
      fft_in0[i][0] = fft_h[i].real();
      fft_in0[i][1] = fft_h[i].imag();
    }

    // Run the FFT
    fftw_execute(fft_plan0);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   _heights[i] = fft_out0[i][0];
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // z(i,j) => z(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny; ++iky)
      {
        int ij = ikx * this->ny + iky;
        int xy = iky * this->nx + ikx;
        _h(xy, 0) = fft_out0[ij][0];
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    size_t n2 = this->nx * this->ny;

    // Populate input array
    for (size_t i=0; i<n2; ++i)
    {
      fft_in1[i][0] = fft_h_ikx[i].real();
      fft_in1[i][1] = fft_h_ikx[i].imag();

      fft_in2[i][0] = fft_h_iky[i].real();
      fft_in2[i][1] = fft_h_iky[i].imag();
    }

    // Run the FFTs
    fftw_execute(fft_plan1);
    fftw_execute(fft_plan2);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   _dhdx[i] = fft_out1[i][0];
    //   _dhdy[i] = fft_out2[i][0];
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // z(i,j) => z(x, y): x = j, y = i
    // dz(i,j)/di => dz(x, y)/dy: x = j, y = i
    // dz(i,j)/dj => dz(x, y)/dx: x = j, y = i
    for (size_t ikx = 0; ikx < this->nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny; ++iky)
      {
        int ij = ikx * this->ny + iky;
        int xy = iky * this->nx + ikx;
        _dhdy(xy, 0) = fft_out1[ij][0];
        _dhdx(xy, 0) = fft_out2[ij][0];
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    size_t n2 = this->nx * this->ny;

    // Populate input array
    for (size_t i=0; i<n2; ++i)
    {
      fft_in3[i][0] = fft_sx[i].real();
      fft_in3[i][1] = fft_sx[i].imag();

      fft_in4[i][0] = fft_sy[i].real();
      fft_in4[i][1] = fft_sy[i].imag();
    }

    // Run the FFTs
    fftw_execute(fft_plan3);
    fftw_execute(fft_plan4);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   _sx[i] = fft_out3[i][0] * lambda;
    //   _sy[i] = fft_out4[i][0] * lambda;
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // sy(i,j) => si(x, y): x = j, y = i
    // sx(i,j) => sj(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny; ++iky)
      {
        int ij = ikx * this->ny + iky;
        int xy = iky * this->nx + ikx;
        _sy(xy, 0) = fft_out3[ij][0] * lambda * -1.0;
        _sx(xy, 0) = fft_out4[ij][0] * lambda * -1.0;
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    size_t n2 = this->nx * this->ny;

    // Populate input array
    for (size_t i=0; i<n2; ++i)
    {
      fft_in5[i][0] = fft_h_kxkx[i].real();
      fft_in5[i][1] = fft_h_kxkx[i].imag();

      fft_in6[i][0] = fft_h_kyky[i].real();
      fft_in6[i][1] = fft_h_kyky[i].imag();

      fft_in7[i][0] = fft_h_kxky[i].real();
      fft_in7[i][1] = fft_h_kxky[i].imag();
    }

    // Run the FFTs
    fftw_execute(fft_plan5);
    fftw_execute(fft_plan6);
    fftw_execute(fft_plan7);

    // for (size_t i=0; i<n2; ++i)
    // {
    //   _dsxdx[i] = fft_out5[i][0] * lambda;
    //   _dsydy[i] = fft_out6[i][0] * lambda;
    //   _dsxdy[i] = fft_out7[i][0] * lambda;
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // sy(i,j) => si(x, y): x = j, y = i
    // sx(i,j) => sj(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->ny; ++iky)
      {
        int ij = ikx * this->ny + iky;
        int xy = iky * this->nx + ikx;
        _dsydy(xy, 0) = fft_out5[ij][0] * lambda * -1.0;
        _dsxdx(xy, 0) = fft_out6[ij][0] * lambda * -1.0;
        _dsxdy(xy, 0) = fft_out7[ij][0] * lambda *  1.0;
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeBaseAmplitudes()
  {
    // gravity acceleration [m/s^2] 
    const double g = 9.81;

    size_t n2 = this->nx * this->ny;

    // storage for Fourier coefficients
    fft_h = Eigen::VectorXcd::Zero(n2);
    fft_h_ikx = Eigen::VectorXcd::Zero(n2);
    fft_h_iky = Eigen::VectorXcd::Zero(n2);
    fft_sx = Eigen::VectorXcd::Zero(n2);
    fft_sy = Eigen::VectorXcd::Zero(n2);
    fft_h_kxkx = Eigen::VectorXcd::Zero(n2);
    fft_h_kyky = Eigen::VectorXcd::Zero(n2);
    fft_h_kxky = Eigen::VectorXcd::Zero(n2);

    // continuous two-sided elevation variance spectrum
    Eigen::VectorXd cap_psi_2s_math =
        Eigen::VectorXd::Zero(this->nx * this->ny);

    // calculate spectrum in math-order (not vectorised)
    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      // kx: fftfreq and ifftshift
      const double kx = (ikx - this->nx/2) * this->kx_f;
      const double kx2 = kx*kx;
      this->kx_math[ikx] = kx;
      this->kx_fft[(ikx + nx/2) % nx] = kx;

      for (int iky = 0; iky < this->ny; ++iky)
      {
        // ky: fftfreq and ifftshift
        const double ky = (iky - this->ny/2) * this->ky_f;
        const double ky2 = ky*ky;
        this->ky_math[iky] = ky;
        this->ky_fft[(iky + ny/2) % ny] = ky;
        
        const double k = sqrt(kx2 + ky2);
        const double phi = atan2(ky, kx);

        // index for flattened array
        int idx = ikx * this->ny + iky;

        if (k == 0.0)
        {
          cap_psi_2s_math[idx] = 0.0;
        }
        else
        {
          double cap_psi = 0.0;
          if (this->use_symmetric_spreading_fn)
          {
            // standing waves - symmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::ECKVSpreadingFunction(
                k, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::Cos2SSpreadingFunction(
                this->s_param, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          const double cap_s =
              WaveSimulationFFT2Impl::ECKVOmniDirectionalSpectrum(
                  k, this->u10, this->cap_omega_c);
          cap_psi_2s_math[idx] = cap_s * cap_psi / k;
        }
      }
    }

    // convert to fft-order
    Eigen::VectorXd cap_psi_2s_fft = Eigen::VectorXd::Zero(this->nx * this->ny);
    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      int ikx_fft = (ikx + nx/2) % nx;
      for (int iky = 0; iky < this->ny; ++iky)
      {
        int iky_fft = (iky + ny/2) % ny;

        // index for flattened array
        int idx = ikx * this->ny + iky;
        int idx_fft = ikx_fft * this->ny + iky_fft;

        cap_psi_2s_fft[idx_fft] = cap_psi_2s_math[idx];
      }
    }

    // square-root of two-sided discrete elevation variance spectrum
    double cap_psi_norm = 0.5;
    double delta_kx = this->kx_f;
    double delta_ky = this->ky_f;
    // double c1 = cap_psi_norm * sqrt(delta_kx * delta_ky);

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (int i = 0; i < n2; ++i)
    {
      // this->cap_psi_2s_root[i] = c1 * sqrt(cap_psi_2s_fft[i]);
      this->cap_psi_2s_root[i] =
          cap_psi_norm * sqrt(cap_psi_2s_fft[i] * delta_kx * delta_ky);

      this->rho[i] = distribution(generator);
      this->sigma[i] = distribution(generator);
    }


    // angular temporal frequency for time-dependent (from dispersion)
    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < this->ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);

        // index for flattened array
        int idx = ikx * this->ny + iky;
        this->omega_k[idx] = sqrt(g * k);
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudes(double _time)
  {
    // alias
    auto& nx = this->nx;
    auto& ny = this->ny;
    auto& r = this->rho;
    auto& s = this->sigma;
    auto& psi_root = this->cap_psi_2s_root;

    // time update
    Eigen::VectorXd cos_omega_k = Eigen::VectorXd::Zero(nx * ny);
    Eigen::VectorXd sin_omega_k = Eigen::VectorXd::Zero(nx * ny);
    for (int ikx = 0; ikx < nx; ++ikx)
    {
      for (int iky = 0; iky < ny; ++iky)
      {
        // index for flattened array
        int idx = ikx * this->ny + iky;

        double omega_t = this->omega_k[idx] * _time;
        cos_omega_k(idx) = cos(omega_t);
        sin_omega_k(idx) = sin(omega_t);
      }
    }

    // non-vectorised reference version
    Eigen::VectorXcd zhat = Eigen::VectorXcd::Zero(nx * ny);
    for (int ikx = 1; ikx < nx; ++ikx)
    {
      for (int iky = 1; iky < ny; ++iky)
      {
        // index for flattened array (ikx, iky)
        int idx = ikx * this->ny + iky;

        // index for conjugate (nx-ikx, ny-iky)
        int cdx = (nx-ikx) * this->ny + (ny-iky);

        zhat[idx] = complex(
            + ( r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx) ) * cos_omega_k(idx)
            + ( s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx) ) * sin_omega_k(idx),
            - ( r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx) ) * sin_omega_k(idx)
            + ( s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx) ) * cos_omega_k(idx));
      }
    }

    for (int iky = 1; iky < ny/2+1; ++iky)
    {
      int ikx = 0;

      // index for flattened array (ikx, iky)
      int idx = ikx * this->ny + iky;

      // index for conjugate (ikx, ny-iky)
      int cdx = ikx * this->ny + (ny-iky);

      zhat[idx] = complex(
          + ( r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx) ) * cos_omega_k(idx)
          + ( s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx) ) * sin_omega_k(idx),
          - ( r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx) ) * sin_omega_k(idx)
          + ( s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx) ) * cos_omega_k(idx));
      zhat[cdx] = std::conj(zhat[idx]);
    }

    for (int ikx = 1; ikx < nx/2+1; ++ikx)
    {
      int iky = 0;

      // index for flattened array (ikx, iky)
      int idx = ikx * this->ny + iky;

      // index for conjugate (nx-ikx, iky)
      int cdx = (nx-ikx) * this->ny + iky;

      zhat[idx] = complex(
          + ( r(idx) * psi_root(idx) + r(cdx) * psi_root(cdx) ) * cos_omega_k(idx)
          + ( s(idx) * psi_root(idx) + s(cdx) * psi_root(cdx) ) * sin_omega_k(idx),
          - ( r(idx) * psi_root(idx) - r(cdx) * psi_root(cdx) ) * sin_omega_k(idx)
          + ( s(idx) * psi_root(idx) - s(cdx) * psi_root(cdx) ) * cos_omega_k(idx));
      zhat[cdx] = std::conj(zhat[idx]);
    }

    zhat[0] = complex(0.0, 0.0);

    // write into fft_h, fft_h_ikx, fft_h_iky, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (int ikx = 0; ikx < nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        // index for flattened arrays
        int idx = ikx * ny + iky;

        complex h  = zhat[idx];
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        this->fft_h[idx] = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        this->fft_h_ikx[idx] = hi * kx;
        this->fft_h_iky[idx] = hi * ky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          fft_sx[idx]    = czero;
          fft_sy[idx]    = czero;
          fft_h_kxkx[idx] = czero;
          fft_h_kyky[idx] = czero;
          fft_h_kxky[idx] = czero;
        }
        else
        {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;
          
          fft_sx[idx]    = dx;
          fft_sy[idx]    = dy;
          fft_h_kxkx[idx] = hkxkx;
          fft_h_kyky[idx] = hkyky;
          fft_h_kxky[idx] = hkxky;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeBaseAmplitudesReference()
  {
    size_t n2 = this->nx * this->ny;

    // storage for Fourier coefficients
    fft_h = Eigen::VectorXcd::Zero(n2);
    fft_h_ikx = Eigen::VectorXcd::Zero(n2);
    fft_h_iky = Eigen::VectorXcd::Zero(n2);
    fft_sx = Eigen::VectorXcd::Zero(n2);
    fft_sy = Eigen::VectorXcd::Zero(n2);
    fft_h_kxkx = Eigen::VectorXcd::Zero(n2);
    fft_h_kyky = Eigen::VectorXcd::Zero(n2);
    fft_h_kxky = Eigen::VectorXcd::Zero(n2);

    // arrays for reference version
    if (this->cap_psi_2s_root_ref.size() == 0 || 
        this->cap_psi_2s_root_ref.rows() != this->nx ||
        this->cap_psi_2s_root_ref.cols() != this->ny)
    {
      this->cap_psi_2s_root_ref = Eigen::MatrixXd::Zero(this->nx, this->ny);
      this->rho_ref = Eigen::MatrixXd::Zero(this->nx, this->ny);
      this->sigma_ref = Eigen::MatrixXd::Zero(this->nx, this->ny);
      this->omega_k_ref = Eigen::MatrixXd::Zero(this->nx, this->ny);
    }

    // Guide to indexing conventions:  1. index, 2. math-order, 3. fft-order
    // 
    // 1. [ 0,  1,  2,  3,  4,  5,  6,  7]
    // 2. [-4, -3, -2, -1,  0,  1,  2,  3]
    // 3.                 [ 0,  1,  2,  3, -4, -3, -2, -3]
    // 

    // fftfreq and ifftshift
    for(int ikx = 0; ikx < this->nx; ++ikx)
    {
      const double kx = (ikx - this->nx/2) * this->kx_f;
      kx_math[ikx] = kx;
      kx_fft[(ikx + nx/2) % nx] = kx;
    }

    for(int iky = 0; iky < this->ny; ++iky)
    {
      const double ky = (iky - this->ny/2) * this->ky_f;
      ky_math[iky] = ky;
      ky_fft[(iky + ny/2) % ny] = ky;
    }

    // debug
    gzmsg << "WaveSimulationFFT2" << "\n";
    gzmsg << "Lx:          " << this->lx << "\n";
    gzmsg << "Ly:          " << this->ly << "\n";
    gzmsg << "nx:          " << this->nx << "\n";
    gzmsg << "ny:          " << this->ny << "\n";
    gzmsg << "delta_x:     " << this->delta_x << "\n";
    gzmsg << "delta_x:     " << this->delta_y << "\n";
    gzmsg << "lambda_x_f:  " << this->lambda_x_f << "\n";
    gzmsg << "lambda_y_f:  " << this->lambda_y_f << "\n";
    gzmsg << "nu_x_f:      " << this->nu_x_f << "\n";
    gzmsg << "nu_y_f:      " << this->nu_y_f << "\n";
    gzmsg << "nu_x_ny:     " << this->nu_x_ny << "\n";
    gzmsg << "nu_y_ny:     " << this->nu_y_ny << "\n";
    gzmsg << "kx_f:        " << this->kx_f << "\n";
    gzmsg << "ky_f:        " << this->ky_f << "\n";
    gzmsg << "kx_ny:       " << this->kx_ny << "\n";
    gzmsg << "ky_ny:       " << this->ky_ny << "\n";
    #if 0
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->kx_fft) os << v << " "; os << "]\n";
      gzmsg << "kx_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->ky_fft) os << v << " "; os << "]\n";
      gzmsg << "ky_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->kx_math) os << v << " "; os << "]\n";
      gzmsg << "kx_math:     " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->ky_math) os << v << " "; os << "]\n";
      gzmsg << "ky_math:     " << os.str();
    }
    #endif

    // continuous two-sided elevation variance spectrum
    Eigen::MatrixXd cap_psi_2s_math = Eigen::MatrixXd::Zero(this->nx, this->ny);

    // calculate spectrum in math-order (not vectorised)
    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      for (int iky = 0; iky < this->ny; ++iky)
      {
        double k = sqrt(this->kx_math[ikx]*this->kx_math[ikx]
            + this->ky_math[iky]*this->ky_math[iky]);
        double phi = atan2(this->ky_math[iky], this->kx_math[ikx]);

        if (k == 0.0)
        {
          cap_psi_2s_math(ikx, iky) = 0.0;
        }
        else
        {
          double cap_psi = 0.0;
          if (this->use_symmetric_spreading_fn)
          {
            // standing waves - symmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::ECKVSpreadingFunction(
                k, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = WaveSimulationFFT2Impl::Cos2SSpreadingFunction(
                this->s_param, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          double cap_s = WaveSimulationFFT2Impl::ECKVOmniDirectionalSpectrum(
              k, this->u10, this->cap_omega_c);
          cap_psi_2s_math(ikx, iky) = cap_s * cap_psi / k;
        }
      }
    }

    // debug
    #if 0
    {
      std::ostringstream os;
      os << "[\n";
      for (int ikx = 0; ikx < this->nx; ++ikx)
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
    Eigen::MatrixXd cap_psi_2s_fft = Eigen::MatrixXd::Zero(this->nx, this->ny);
    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      int ikx_fft = (ikx + nx/2) % nx;
      for (int iky = 0; iky < this->ny; ++iky)
      {
        int iky_fft = (iky + ny/2) % ny;
        cap_psi_2s_fft(ikx_fft, iky_fft) = cap_psi_2s_math(ikx, iky);
      }
    }

    // square-root of two-sided discrete elevation variance spectrum
    double cap_psi_norm = 0.5;
    double delta_kx = this->kx_f;
    double delta_ky = this->ky_f;

    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      for (int iky = 0; iky < this->ny; ++iky)
      {
        this->cap_psi_2s_root_ref(ikx, iky) =
            cap_psi_norm * sqrt(cap_psi_2s_fft(ikx, iky) * delta_kx * delta_ky);
      }
    }

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      for (int iky = 0; iky < this->ny; ++iky)
      {
        this->rho_ref(ikx, iky) = distribution(generator);
        this->sigma_ref(ikx, iky) = distribution(generator);
      }
    }

    // gravity acceleration [m/s^2] 
    double g = 9.81;

    // angular temporal frequency for time-dependent (from dispersion)
    for (int ikx = 0; ikx < this->nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < this->ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        this->omega_k_ref(ikx, iky) = sqrt(g * k);
      }
    }
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudesReference(double _time)
  {
    // alias
    auto& nx = this->nx;
    auto& ny = this->ny;
    auto& r = this->rho_ref;
    auto& s = this->sigma_ref;
    auto& psi_root = this->cap_psi_2s_root_ref;

    // time update
    Eigen::MatrixXd cos_omega_k = Eigen::MatrixXd::Zero(nx, ny);
    Eigen::MatrixXd sin_omega_k = Eigen::MatrixXd::Zero(nx, ny);
    for (int ikx = 0; ikx < nx; ++ikx)
    {
      for (int iky = 0; iky < ny; ++iky)
      {
        cos_omega_k(ikx, iky) = cos(this->omega_k_ref(ikx, iky) * _time);
        sin_omega_k(ikx, iky) = sin(this->omega_k_ref(ikx, iky) * _time);
      }
    }

    // non-vectorised reference version
    Eigen::MatrixXcd zhat = Eigen::MatrixXcd::Zero(nx, ny);
    for (int ikx = 1; ikx < nx; ++ikx)
    {
      for (int iky = 1; iky < ny; ++iky)
      {
        zhat(ikx, iky) = complex(
            + ( r(ikx, iky) * psi_root(ikx, iky) + r(nx-ikx, ny-iky) * psi_root(nx-ikx, ny-iky) ) * cos_omega_k(ikx, iky)
            + ( s(ikx, iky) * psi_root(ikx, iky) + s(nx-ikx, ny-iky) * psi_root(nx-ikx, ny-iky) ) * sin_omega_k(ikx, iky),
            - ( r(ikx, iky) * psi_root(ikx, iky) - r(nx-ikx, ny-iky) * psi_root(nx-ikx, ny-iky) ) * sin_omega_k(ikx, iky)
            + ( s(ikx, iky) * psi_root(ikx, iky) - s(nx-ikx, ny-iky) * psi_root(nx-ikx, ny-iky) ) * cos_omega_k(ikx, iky));
      }
    }

    for (int iky = 1; iky < ny/2+1; ++iky)
    {
      int ikx = 0;
      zhat(ikx, iky) = complex(
          + ( r(ikx, iky) * psi_root(ikx, iky) + r(ikx, ny-iky) * psi_root(ikx, ny-iky) ) * cos_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) + s(ikx, ny-iky) * psi_root(ikx, ny-iky) ) * sin_omega_k(ikx, iky),
          - ( r(ikx, iky) * psi_root(ikx, iky) - r(ikx, ny-iky) * psi_root(ikx, ny-iky) ) * sin_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) - s(ikx, ny-iky) * psi_root(ikx, ny-iky) ) * cos_omega_k(ikx, iky));
      zhat(ikx, ny-iky) = std::conj(zhat(ikx, iky));
    }

    for (int ikx = 1; ikx < nx/2+1; ++ikx)
    {
      int iky = 0;
      zhat(ikx, iky) = complex(
          + ( r(ikx, iky) * psi_root(ikx, iky) + r(nx-ikx, iky) * psi_root(nx-ikx, iky) ) * cos_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) + s(nx-ikx, iky) * psi_root(nx-ikx, iky) ) * sin_omega_k(ikx, iky),
          - ( r(ikx, iky) * psi_root(ikx, iky) - r(nx-ikx, iky) * psi_root(nx-ikx, iky) ) * sin_omega_k(ikx, iky)
          + ( s(ikx, iky) * psi_root(ikx, iky) - s(nx-ikx, iky) * psi_root(nx-ikx, iky) ) * cos_omega_k(ikx, iky));
      zhat(nx-ikx, iky) = std::conj(zhat(ikx, iky));
    }

    zhat(0, 0) = complex(0.0, 0.0);

    /// \todo: change zhat to 1D array and use directly

    // write into fft_h, fft_h_ikx, fft_h_iky, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (int ikx = 0; ikx < nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        // index for flattened arrays
        int idx = ikx * ny + iky;

        complex h  = zhat(ikx, iky);
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        this->fft_h[idx] = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        this->fft_h_ikx[idx] = hi * kx;
        this->fft_h_iky[idx] = hi * ky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          fft_sx[idx]    = czero;
          fft_sy[idx]    = czero;
          fft_h_kxkx[idx] = czero;
          fft_h_kyky[idx] = czero;
          fft_h_kxky[idx] = czero;
        }
        else
        {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;
          
          fft_sx[idx]    = dx;
          fft_sy[idx]    = dy;
          fft_h_kxkx[idx] = hkxkx;
          fft_h_kyky[idx] = hkyky;
          fft_h_kxky[idx] = hkxky;
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
    double _lx, double _ly, int _nx, int _ny) :
    impl(new WaveSimulationFFT2Impl(_lx, _ly, _nx, _ny))
  {
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetUseVectorised(bool _value)
  {
    impl->SetUseVectorised(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetLambda(double _value)
  {
    impl->SetLambda(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::SetTime(double _value)
  {
    impl->SetTime(_value);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeElevation(
    Eigen::Ref<Eigen::MatrixXd> _h)
  {
    impl->ComputeElevation(_h);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeElevationDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy)
  {
    impl->ComputeElevationDerivatives(_dhdx, _dhdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacements(
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacementsDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    impl->ComputeDisplacementsDerivatives(_dsxdx, _dsxdy, _dsxdy);
  }

  //////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacementsAndDerivatives(
    Eigen::Ref<Eigen::MatrixXd> _h,
    Eigen::Ref<Eigen::MatrixXd> _sx,
    Eigen::Ref<Eigen::MatrixXd> _sy,
    Eigen::Ref<Eigen::MatrixXd> _dhdx,
    Eigen::Ref<Eigen::MatrixXd> _dhdy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdx,
    Eigen::Ref<Eigen::MatrixXd> _dsydy,
    Eigen::Ref<Eigen::MatrixXd> _dsxdy)
  {
    impl->ComputeElevation(_h);
    impl->ComputeElevationDerivatives(_dhdx, _dhdy);
    impl->ComputeDisplacements(_sx, _sy);
    impl->ComputeDisplacementsDerivatives(_dsxdx, _dsydy, _dsxdy);
  }
}
}
