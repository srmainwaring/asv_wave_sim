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

#include "gz/marine/WaveSimulationFFT2.hh"
#include "gz/marine/WaveSpectrum.hh"

#include "WaveSimulationFFT2Impl.hh"

#include <gz/common.hh>

#include <fftw3.h>

#include <complex>
#include <random>
#include <vector>

namespace ignition
{
namespace marine
{
  /////////////////////////////////////////////////
  WaveSimulationFFT2Impl::~WaveSimulationFFT2Impl()
  {
    fftw_destroy_plan(mFFTPlan0);
    fftw_destroy_plan(mFFTPlan1);
    fftw_destroy_plan(mFFTPlan2);
    fftw_destroy_plan(mFFTPlan3);
    fftw_destroy_plan(mFFTPlan4);
    fftw_destroy_plan(mFFTPlan5);
    fftw_destroy_plan(mFFTPlan6);
    fftw_destroy_plan(mFFTPlan7);

    fftw_free(mOut0);
    fftw_free(mOut1);
    fftw_free(mOut2);
    fftw_free(mOut3);
    fftw_free(mOut4);
    fftw_free(mOut5);
    fftw_free(mOut6);
    fftw_free(mOut7);

    fftw_free(mIn0);
    fftw_free(mIn1);
    fftw_free(mIn2);
    fftw_free(mIn3);
    fftw_free(mIn4);
    fftw_free(mIn5);
    fftw_free(mIn6);
    fftw_free(mIn7);
  }

  /////////////////////////////////////////////////
  WaveSimulationFFT2Impl::WaveSimulationFFT2Impl(int _N, double _L) :
    mN(_N),
    mN2(_N * _N),
    mL(_L),
    mLambda(0.6)
  {
    ComputeBaseAmplitudes();

    // FFT 2D.

    // For height
    mIn0  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn1  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn2  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));

    // For xy-displacements
    mIn3  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn4  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn5  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn6  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn7  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));

    // For height
    mOut0 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut1 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut2 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  

    // For xy-displacements
    mOut3 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut4 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut5 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut6 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut7 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  

    // For height
    mFFTPlan0 = fftw_plan_dft_2d(mN, mN, mIn0, mOut0, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan1 = fftw_plan_dft_2d(mN, mN, mIn1, mOut1, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan2 = fftw_plan_dft_2d(mN, mN, mIn2, mOut2, FFTW_BACKWARD, FFTW_ESTIMATE);

    // For xy-displacements
    mFFTPlan3 = fftw_plan_dft_2d(mN, mN, mIn3, mOut3, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan4 = fftw_plan_dft_2d(mN, mN, mIn4, mOut4, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan5 = fftw_plan_dft_2d(mN, mN, mIn5, mOut5, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan6 = fftw_plan_dft_2d(mN, mN, mIn6, mOut6, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan7 = fftw_plan_dft_2d(mN, mN, mIn7, mOut7, FFTW_BACKWARD, FFTW_ESTIMATE);
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetWindVelocity(double _ux, double _uy)
  {
    // Update wind velocity and recompute base amplitudes.
    this->u10 = sqrt(_ux*_ux + _uy *_uy);
    this->phi10 = atan2(_uy, _ux);

    ComputeBaseAmplitudes();
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetTime(double _time)
  {
    ComputeCurrentAmplitudes(_time);
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::SetLambda(double _lambda)
  {
    mLambda = _lambda;
    ComputeBaseAmplitudes();
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeHeights(
    std::vector<double>& _heights)
  {
    // Populate input array
    for (size_t i=0; i<mN2; ++i)
    {
      mIn0[i][0] = mH[i].real();
      mIn0[i][1] = mH[i].imag();
    }

    // Run the FFT
    fftw_execute(mFFTPlan0);

    // Resize output if necessary
    if (_heights.size() != mN2)
    {
      _heights.resize(mN2, 0.0);
    }

    // for (size_t i=0; i<mN2; ++i)
    // {
    //   _heights[i] = mOut0[i][0];
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // z(i,j) => z(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->Ny; ++iky)
      {
        int ij = ikx * this->Ny + iky;
        int xy = iky * this->Nx + ikx;
        _heights[xy] = mOut0[ij][0];
      }
    }
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    // Populate input array
    for (size_t i=0; i<mN2; ++i)
    {
      mIn1[i][0] = mHikx[i].real();
      mIn1[i][1] = mHikx[i].imag();

      mIn2[i][0] = mHiky[i].real();
      mIn2[i][1] = mHiky[i].imag();
    }

    // Run the FFTs
    fftw_execute(mFFTPlan1);
    fftw_execute(mFFTPlan2);

    // Resize output if necessary
    if (_dhdx.size() != mN2)
    {
      _dhdx.resize(mN2, 0.0);
    }
    if (_dhdy.size() != mN2)
    {
      _dhdy.resize(mN2, 0.0);
    }

    // for (size_t i=0; i<mN2; ++i)
    // {
    //   _dhdx[i] = mOut1[i][0];
    //   _dhdy[i] = mOut2[i][0];
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // z(i,j) => z(x, y): x = j, y = i
    // dz(i,j)/di => dz(x, y)/dy: x = j, y = i
    // dz(i,j)/dj => dz(x, y)/dx: x = j, y = i
    for (size_t ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->Ny; ++iky)
      {
        int ij = ikx * this->Ny + iky;
        int xy = iky * this->Nx + ikx;
        _dhdy[xy] = mOut1[ij][0];
        _dhdx[xy] = mOut2[ij][0];
      }
    }
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    // Populate input array
    for (size_t i=0; i<mN2; ++i)
    {
      mIn3[i][0] = mDx[i].real();
      mIn3[i][1] = mDx[i].imag();

      mIn4[i][0] = mDy[i].real();
      mIn4[i][1] = mDy[i].imag();
    }

    // Run the FFTs
    fftw_execute(mFFTPlan3);
    fftw_execute(mFFTPlan4);

    // Resize output if necessary
    if (_sx.size() != mN2)
    {
      _sx.resize(mN2, 0.0);
    }
    if (_sy.size() != mN2)
    {
      _sy.resize(mN2, 0.0);
    }

    // for (size_t i=0; i<mN2; ++i)
    // {
    //   _sx[i] = mOut3[i][0] * mLambda;
    //   _sy[i] = mOut4[i][0] * mLambda;
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // sy(i,j) => si(x, y): x = j, y = i
    // sx(i,j) => sj(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->Ny; ++iky)
      {
        int ij = ikx * this->Ny + iky;
        int xy = iky * this->Nx + ikx;
        _sy[xy] = mOut3[ij][0] * mLambda;
        _sx[xy] = mOut4[ij][0] * mLambda;
      }
    }
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    // Populate input array
    for (size_t i=0; i<mN2; ++i)
    {
      mIn5[i][0] = mHkxkx[i].real();
      mIn5[i][1] = mHkxkx[i].imag();

      mIn6[i][0] = mHkyky[i].real();
      mIn6[i][1] = mHkyky[i].imag();

      mIn7[i][0] = mHkxky[i].real();
      mIn7[i][1] = mHkxky[i].imag();
    }

    // Run the FFTs
    fftw_execute(mFFTPlan5);
    fftw_execute(mFFTPlan6);
    fftw_execute(mFFTPlan7);

    // Resize output if necessary
    if (_dsxdx.size() != mN2)
    {
      _dsxdx.resize(mN2, 0.0);
    }
    if (_dsydy.size() != mN2)
    {
      _dsydy.resize(mN2, 0.0);
    }
    if (_dsxdy.size() != mN2)
    {
      _dsxdy.resize(mN2, 0.0);
    }

    // for (size_t i=0; i<mN2; ++i)
    // {
    //   _dsxdx[i] = mOut5[i][0] * mLambda;
    //   _dsydy[i] = mOut6[i][0] * mLambda;
    //   _dsxdy[i] = mOut7[i][0] * mLambda;
    // }
    // change from matrix 'ij' to cartesian 'xy' coordinates
    // sy(i,j) => si(x, y): x = j, y = i
    // sx(i,j) => sj(x, y): x = j, y = i
    for (size_t ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (size_t iky = 0; iky < this->Ny; ++iky)
      {
        int ij = ikx * this->Ny + iky;
        int xy = iky * this->Nx + ikx;
        _dsydy[xy] = mOut5[ij][0] * mLambda;
        _dsxdx[xy] = mOut6[ij][0] * mLambda;
        _dsxdy[xy] = mOut7[ij][0] * mLambda;
      }
    }
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeBaseAmplitudes()
  {
    // gravity acceleration [m/s^2] 
    const double g = 9.82;

    // storage for Fourier coefficients
    mH.resize(mN2, complex(0.0, 0.0));
    mHikx.resize(mN2, complex(0.0, 0.0));
    mHiky.resize(mN2, complex(0.0, 0.0));
    mDx.resize(mN2, complex(0.0, 0.0));
    mDy.resize(mN2, complex(0.0, 0.0));
    mHkxkx.resize(mN2, complex(0.0, 0.0));
    mHkyky.resize(mN2, complex(0.0, 0.0));
    mHkxky.resize(mN2, complex(0.0, 0.0));

    // continuous two-sided elevation variance spectrum
    std::vector<double> cap_psi_2s_math(this->Nx * this->Ny, 0.0);

    // calculate spectrum in math-order (not vectorised)
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      // kx: fftfreq and ifftshift
      const double kx = (ikx - this->Nx/2) * this->kx_f;
      const double kx2 = kx*kx;
      this->kx_math[ikx] = kx;
      this->kx_fft[(ikx + Nx/2) % Nx] = kx;

      for (int iky = 0; iky < this->Ny; ++iky)
      {
        // ky: fftfreq and ifftshift
        const double ky = (iky - this->Ny/2) * this->ky_f;
        const double ky2 = ky*ky;
        this->ky_math[iky] = ky;
        this->ky_fft[(iky + Ny/2) % Ny] = ky;
        
        const double k = sqrt(kx2 + ky2);
        const double phi = atan2(ky, kx);

        // index for flattened array
        int idx = ikx * this->Ny + iky;

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
            cap_psi = this->ECKVSpreadingFunction(
                k, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = this->Cos2SSpreadingFunction(
                this->s_param, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          const double cap_s = this->ECKVOmniDirectionalSpectrum(
              k, this->u10, this->cap_omega_c);
          cap_psi_2s_math[idx] = cap_s * cap_psi / k;
        }
      }
    }

    // convert to fft-order
    std::vector<double> cap_psi_2s_fft(this->Nx * this->Ny, 0.0);
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      int ikx_fft = (ikx + Nx/2) % Nx;
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        int iky_fft = (iky + Ny/2) % Ny;

        // index for flattened array
        int idx = ikx * this->Ny + iky;
        int idx_fft = ikx_fft * this->Ny + iky_fft;

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

    for (int i = 0; i < mN2; ++i)
    {
      // this->cap_psi_2s_root[i] = c1 * sqrt(cap_psi_2s_fft[i]);
      this->cap_psi_2s_root[i] =
          cap_psi_norm * sqrt(cap_psi_2s_fft[i] * delta_kx * delta_ky);

      this->rho[i] = distribution(generator);
      this->sigma[i] = distribution(generator);
    }


    // angular temporal frequency for time-dependent (from dispersion)
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);

        // index for flattened array
        int idx = ikx * this->Ny + iky;
        this->omega_k[idx] = sqrt(g * k);
      }
    }
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudes(double _time)
  {
    // alias
    auto& Nx = this->Nx;
    auto& Ny = this->Ny;
    auto& r = this->rho;
    auto& s = this->sigma;
    auto& psi_root = this->cap_psi_2s_root;

    // time update
    std::vector<double> cos_omega_k(Nx * Ny, 0.0);
    std::vector<double> sin_omega_k(Nx * Ny, 0.0);
    for (int ikx = 0; ikx < Nx; ++ikx)
    {
      for (int iky = 0; iky < Ny; ++iky)
      {
        // index for flattened array
        int idx = ikx * this->Ny + iky;

        double omega_t = this->omega_k[idx] * _time;
        cos_omega_k[idx] = cos(omega_t);
        sin_omega_k[idx] = sin(omega_t);
      }
    }

    // non-vectorised reference version
    std::vector<complex> zhat(Nx * Ny, complex(0.0, 0.0));
    for (int ikx = 1; ikx < Nx; ++ikx)
    {
      for (int iky = 1; iky < Ny; ++iky)
      {
        // index for flattened array [ikx][iky]
        int idx = ikx * this->Ny + iky;

        // index for conjugate [Nx-ikx][Ny-iky]
        int cdx = (Nx-ikx) * this->Ny + (Ny-iky);

        zhat[idx] = complex(
            + ( r[idx] * psi_root[idx] + r[cdx] * psi_root[cdx] ) * cos_omega_k[idx]
            + ( s[idx] * psi_root[idx] + s[cdx] * psi_root[cdx] ) * sin_omega_k[idx],
            - ( r[idx] * psi_root[idx] - r[cdx] * psi_root[cdx] ) * sin_omega_k[idx]
            + ( s[idx] * psi_root[idx] - s[cdx] * psi_root[cdx] ) * cos_omega_k[idx]);
      }
    }

    for (int iky = 1; iky < Ny/2+1; ++iky)
    {
      int ikx = 0;

      // index for flattened array [ikx][iky]
      int idx = ikx * this->Ny + iky;

      // index for conjugate [ikx][Ny-iky]
      int cdx = ikx * this->Ny + (Ny-iky);

      zhat[idx] = complex(
          + ( r[idx] * psi_root[idx] + r[cdx] * psi_root[cdx] ) * cos_omega_k[idx]
          + ( s[idx] * psi_root[idx] + s[cdx] * psi_root[cdx] ) * sin_omega_k[idx],
          - ( r[idx] * psi_root[idx] - r[cdx] * psi_root[cdx] ) * sin_omega_k[idx]
          + ( s[idx] * psi_root[idx] - s[cdx] * psi_root[cdx] ) * cos_omega_k[idx]);
      zhat[cdx] = std::conj(zhat[idx]);
    }

    for (int ikx = 1; ikx < Nx/2+1; ++ikx)
    {
      int iky = 0;

      // index for flattened array [ikx][iky]
      int idx = ikx * this->Ny + iky;

      // index for conjugate [Nx-ikx][iky]
      int cdx = (Nx-ikx) * this->Ny + iky;

      zhat[idx] = complex(
          + ( r[idx] * psi_root[idx] + r[cdx] * psi_root[cdx] ) * cos_omega_k[idx]
          + ( s[idx] * psi_root[idx] + s[cdx] * psi_root[cdx] ) * sin_omega_k[idx],
          - ( r[idx] * psi_root[idx] - r[cdx] * psi_root[cdx] ) * sin_omega_k[idx]
          + ( s[idx] * psi_root[idx] - s[cdx] * psi_root[cdx] ) * cos_omega_k[idx]);
      zhat[cdx] = std::conj(zhat[idx]);
    }

    zhat[0] = complex(0.0, 0.0);

    // write into mH, mHikx, mHiky, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (int ikx = 0; ikx < Nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < Ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        // index for flattened arrays
        int idx = ikx * Ny + iky;

        complex h  = zhat[idx];
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        this->mH[idx] = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        this->mHikx[idx] = hi * kx;
        this->mHiky[idx] = hi * ky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          mDx[idx]    = czero;
          mDy[idx]    = czero;
          mHkxkx[idx] = czero;
          mHkyky[idx] = czero;
          mHkxky[idx] = czero;
        }
        else
        {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;
          
          mDx[idx]    = dx;
          mDy[idx]    = dy;
          mHkxkx[idx] = hkxkx;
          mHkyky[idx] = hkyky;
          mHkxky[idx] = hkxky;
        }
      }
    }
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeBaseAmplitudesReference()
  {
    // storage for Fourier coefficients
    mH.resize(mN2, complex(0.0, 0.0));
    mHikx.resize(mN2, complex(0.0, 0.0));
    mHiky.resize(mN2, complex(0.0, 0.0));
    mDx.resize(mN2, complex(0.0, 0.0));
    mDy.resize(mN2, complex(0.0, 0.0));
    mHkxkx.resize(mN2, complex(0.0, 0.0));
    mHkyky.resize(mN2, complex(0.0, 0.0));
    mHkxky.resize(mN2, complex(0.0, 0.0));

    // arrays for reference version
    if (cap_psi_2s_root_ref.empty() || 
        this->cap_psi_2s_root_ref.size() != this->Nx ||
        this->cap_psi_2s_root_ref[0].size() != this->Ny)
    {
      this->cap_psi_2s_root_ref = std::vector<std::vector<double>>(
          this->Nx, std::vector<double>(this->Ny, 0.0));
      this->rho_ref = std::vector<std::vector<double>>(
          this->Nx, std::vector<double>(this->Ny, 0.0));
      this->sigma_ref = std::vector<std::vector<double>>(
          this->Nx, std::vector<double>(this->Ny, 0.0));
      this->omega_k_ref = std::vector<std::vector<double>>(
          this->Nx, std::vector<double>(this->Ny, 0.0));
    }

    // Guide to indexing conventions:  1. index, 2. math-order, 3. fft-order
    // 
    // 1. [ 0,  1,  2,  3,  4,  5,  6,  7]
    // 2. [-4, -3, -2, -1,  0,  1,  2,  3]
    // 3.                 [ 0,  1,  2,  3, -4, -3, -2, -3]
    // 

    // fftfreq and ifftshift
    for(int ikx = 0; ikx < this->Nx; ++ikx)
    {
      const double kx = (ikx - this->Nx/2) * this->kx_f;
      kx_math[ikx] = kx;
      kx_fft[(ikx + Nx/2) % Nx] = kx;
    }

    for(int iky = 0; iky < this->Ny; ++iky)
    {
      const double ky = (iky - this->Ny/2) * this->ky_f;
      ky_math[iky] = ky;
      ky_fft[(iky + Ny/2) % Ny] = ky;
    }

    // debug
    ignmsg << "WaveSimulationFFT2" << "\n";
    ignmsg << "Lx:          " << this->Lx << "\n";
    ignmsg << "Ly:          " << this->Ly << "\n";
    ignmsg << "Nx:          " << this->Nx << "\n";
    ignmsg << "Ny:          " << this->Ny << "\n";
    ignmsg << "delta_x:     " << this->delta_x << "\n";
    ignmsg << "delta_x:     " << this->delta_y << "\n";
    ignmsg << "lambda_x_f:  " << this->lambda_x_f << "\n";
    ignmsg << "lambda_y_f:  " << this->lambda_y_f << "\n";
    ignmsg << "nu_x_f:      " << this->nu_x_f << "\n";
    ignmsg << "nu_y_f:      " << this->nu_y_f << "\n";
    ignmsg << "nu_x_Ny:     " << this->nu_x_Ny << "\n";
    ignmsg << "nu_y_Ny:     " << this->nu_y_Ny << "\n";
    ignmsg << "kx_f:        " << this->kx_f << "\n";
    ignmsg << "ky_f:        " << this->ky_f << "\n";
    ignmsg << "kx_Ny:       " << this->kx_Ny << "\n";
    ignmsg << "ky_Ny:       " << this->ky_Ny << "\n";
    #if 0
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->kx_fft) os << v << " "; os << "]\n";
      ignmsg << "kx_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->ky_fft) os << v << " "; os << "]\n";
      ignmsg << "ky_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->kx_math) os << v << " "; os << "]\n";
      ignmsg << "kx_math:     " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : this->ky_math) os << v << " "; os << "]\n";
      ignmsg << "ky_math:     " << os.str();
    }
    #endif

    // continuous two-sided elevation variance spectrum
    std::vector<std::vector<double>> cap_psi_2s_math(
        this->Nx, std::vector<double>(this->Ny, 0.0));

    // calculate spectrum in math-order (not vectorised)
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        double k = sqrt(this->kx_math[ikx]*this->kx_math[ikx]
            + this->ky_math[iky]*this->ky_math[iky]);
        double phi = atan2(this->ky_math[iky], this->kx_math[ikx]);

        if (k == 0.0)
        {
          cap_psi_2s_math[ikx][iky] = 0.0;
        }
        else
        {
          double cap_psi = 0.0;
          if (this->use_symmetric_spreading_fn)
          {
            // standing waves - symmetric spreading function
            cap_psi = this->ECKVSpreadingFunction(
                k, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = this->Cos2SSpreadingFunction(
                this->s_param, phi - this->phi10, this->u10, this->cap_omega_c);
          }
          double cap_s = this->ECKVOmniDirectionalSpectrum(
              k, this->u10, this->cap_omega_c);
          cap_psi_2s_math[ikx][iky] = cap_s * cap_psi / k;
        }
      }
    }

    // debug
    #if 0
    {
      std::ostringstream os;
      os << "[\n";
      for (int ikx = 0; ikx < this->Nx; ++ikx)
      {
        os << " [ ";
        for (auto& v : cap_psi_2s_math[ikx])
        {
          os << v << " ";
        }
        os << "]\n";
      }
      os << "]\n";

      ignmsg << "cap_psi_2s:  " << os.str();
    }
    #endif

    // convert to fft-order
    std::vector<std::vector<double>> cap_psi_2s_fft(
        this->Nx, std::vector<double>(this->Ny, 0.0));
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      int ikx_fft = (ikx + Nx/2) % Nx;
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        int iky_fft = (iky + Ny/2) % Ny;
        cap_psi_2s_fft[ikx_fft][iky_fft] = cap_psi_2s_math[ikx][iky];
      }
    }

    // square-root of two-sided discrete elevation variance spectrum
    double cap_psi_norm = 0.5;
    double delta_kx = this->kx_f;
    double delta_ky = this->ky_f;

    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        this->cap_psi_2s_root_ref[ikx][iky] =
            cap_psi_norm * sqrt(cap_psi_2s_fft[ikx][iky] * delta_kx * delta_ky);
      }
    }

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        this->rho_ref[ikx][iky] = distribution(generator);
        this->sigma_ref[ikx][iky] = distribution(generator);
      }
    }

    // gravity acceleration [m/s^2] 
    double g = 9.82;

    // angular temporal frequency for time-dependent (from dispersion)
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        this->omega_k_ref[ikx][iky] = sqrt(g * k);
      }
    }

  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudesReference(double _time)
  {
    // alias
    auto& Nx = this->Nx;
    auto& Ny = this->Ny;
    auto& r = this->rho_ref;
    auto& s = this->sigma_ref;
    auto& psi_root = this->cap_psi_2s_root_ref;

    // time update
    std::vector<std::vector<double>> cos_omega_k(
        Nx, std::vector<double>(Ny, 0.0));
    std::vector<std::vector<double>> sin_omega_k(
        Nx, std::vector<double>(Ny, 0.0));
    for (int ikx = 0; ikx < Nx; ++ikx)
    {
      for (int iky = 0; iky < Ny; ++iky)
      {
        cos_omega_k[ikx][iky] = cos(this->omega_k_ref[ikx][iky] * _time);
        sin_omega_k[ikx][iky] = sin(this->omega_k_ref[ikx][iky] * _time);
      }
    }

    // non-vectorised reference version
    std::vector<std::vector<complex>> zhat(Nx, std::vector<complex>(Ny, complex(0.0, 0.0)));
    for (int ikx = 1; ikx < Nx; ++ikx)
    {
      for (int iky = 1; iky < Ny; ++iky)
      {
        zhat[ikx][iky] = complex(
            + ( r[ikx][iky] * psi_root[ikx][iky] + r[Nx-ikx][Ny-iky] * psi_root[Nx-ikx][Ny-iky] ) * cos_omega_k[ikx][iky]
            + ( s[ikx][iky] * psi_root[ikx][iky] + s[Nx-ikx][Ny-iky] * psi_root[Nx-ikx][Ny-iky] ) * sin_omega_k[ikx][iky],
            - ( r[ikx][iky] * psi_root[ikx][iky] - r[Nx-ikx][Ny-iky] * psi_root[Nx-ikx][Ny-iky] ) * sin_omega_k[ikx][iky]
            + ( s[ikx][iky] * psi_root[ikx][iky] - s[Nx-ikx][Ny-iky] * psi_root[Nx-ikx][Ny-iky] ) * cos_omega_k[ikx][iky]);
      }
    }

    for (int iky = 1; iky < Ny/2+1; ++iky)
    {
      int ikx = 0;
      zhat[ikx][iky] = complex(
          + ( r[ikx][iky] * psi_root[ikx][iky] + r[ikx][Ny-iky] * psi_root[ikx][Ny-iky] ) * cos_omega_k[ikx][iky]
          + ( s[ikx][iky] * psi_root[ikx][iky] + s[ikx][Ny-iky] * psi_root[ikx][Ny-iky] ) * sin_omega_k[ikx][iky],
          - ( r[ikx][iky] * psi_root[ikx][iky] - r[ikx][Ny-iky] * psi_root[ikx][Ny-iky] ) * sin_omega_k[ikx][iky]
          + ( s[ikx][iky] * psi_root[ikx][iky] - s[ikx][Ny-iky] * psi_root[ikx][Ny-iky] ) * cos_omega_k[ikx][iky]);
      zhat[ikx][Ny-iky] = std::conj(zhat[ikx][iky]);
    }

    for (int ikx = 1; ikx < Nx/2+1; ++ikx)
    {
      int iky = 0;
      zhat[ikx][iky] = complex(
          + ( r[ikx][iky] * psi_root[ikx][iky] + r[Nx-ikx][iky] * psi_root[Nx-ikx][iky] ) * cos_omega_k[ikx][iky]
          + ( s[ikx][iky] * psi_root[ikx][iky] + s[Nx-ikx][iky] * psi_root[Nx-ikx][iky] ) * sin_omega_k[ikx][iky],
          - ( r[ikx][iky] * psi_root[ikx][iky] - r[Nx-ikx][iky] * psi_root[Nx-ikx][iky] ) * sin_omega_k[ikx][iky]
          + ( s[ikx][iky] * psi_root[ikx][iky] - s[Nx-ikx][iky] * psi_root[Nx-ikx][iky] ) * cos_omega_k[ikx][iky]);
      zhat[Nx-ikx][iky] = std::conj(zhat[ikx][iky]);
    }

    zhat[0][0] = complex(0.0, 0.0);

    /// \todo: change zhat to 1D array and use directly

    // write into mH, mHikx, mHiky, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (int ikx = 0; ikx < Nx; ++ikx)
    {
      double kx = this->kx_fft[ikx];
      double kx2 = kx*kx;
      for (int iky = 0; iky < Ny; ++iky)
      {
        double ky = this->ky_fft[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        // index for flattened arrays
        int idx = ikx * Ny + iky;

        complex h  = zhat[ikx][iky];
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        this->mH[idx] = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        this->mHikx[idx] = hi * kx;
        this->mHiky[idx] = hi * ky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          mDx[idx]    = czero;
          mDy[idx]    = czero;
          mHkxkx[idx] = czero;
          mHkyky[idx] = czero;
          mHkxky[idx] = czero;
        }
        else
        {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;
          
          mDx[idx]    = dx;
          mDy[idx]    = dy;
          mHkxkx[idx] = hkxkx;
          mHkyky[idx] = hkyky;
          mHkxky[idx] = hkxky;
        }
      }
    }

  }

  /////////////////////////////////////////////////
  double WaveSimulationFFT2Impl::ECKVOmniDirectionalSpectrum(
      double k, double u10, double cap_omega_c)
  {
    double alpha = 0.0081;
    double beta = 1.25;
    double g = 9.82;
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
    // ignmsg << "g:       " << g << "\n";
    // ignmsg << "Omega_c: " << cap_omega_c << "\n";
    // ignmsg << "Cd10N:   " << Cd_10N << "\n";
    // ignmsg << "ustar:   " << u_star << "\n";
    // ignmsg << "ao:      " << ao << "\n";
    // ignmsg << "ap:      " << ap << "\n";
    // ignmsg << "cm:      " << cm << "\n";
    // ignmsg << "am:      " << am << "\n";
    // ignmsg << "km:      " << km << "\n";
    // ignmsg << "gamma:   " << gamma << "\n";
    // ignmsg << "sigma:   " << sigma << "\n";
    // ignmsg << "alphap:  " << alpha_p << "\n";
    // ignmsg << "alpham:  " << alpha_m << "\n";
    // ignmsg << "ko:      " << ko << "\n";
    // ignmsg << "kp:      " << kp << "\n";
    // ignmsg << "cp:      " << cp << "\n";
    // ignmsg << "c:       " << c << "\n";
    // ignmsg << "Gamma:   " << Gamma << "\n";
    // ignmsg << "fJp:     " << J_p << "\n";
    // ignmsg << "fLpm:    " << L_PM << "\n";
    // ignmsg << "Fp:      " << F_p << "\n";
    // ignmsg << "Bl:      " << B_l << "\n";
    // ignmsg << "Fm:      " << F_m << "\n";
    // ignmsg << "Bh:      " << B_h << "\n";
    
    return S;
  }

  /////////////////////////////////////////////////
  double WaveSimulationFFT2Impl::ECKVSpreadingFunction(
      double k, double phi, double u10, double cap_omega_c)
  {
    double g = 9.82;
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

  /////////////////////////////////////////////////
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

  /////////////////////////////////////////////////
  // WaveSimulationFFT2

  /////////////////////////////////////////////////
  WaveSimulationFFT2::~WaveSimulationFFT2()
  {
  }

  /////////////////////////////////////////////////
  WaveSimulationFFT2::WaveSimulationFFT2(int _N, double _L) :
    impl(new WaveSimulationFFT2Impl(_N, _L))
  {
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::SetTime(double _time)
  {
    impl->SetTime(_time);
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeHeights(
    std::vector<double>& _h)
  {
    impl->ComputeHeights(_h);    
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);  
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);    
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);      
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::ComputeDisplacementsAndDerivatives(
    std::vector<double>& _h,
    std::vector<double>& _sx,
    std::vector<double>& _sy,
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy,
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeHeights(_h);
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);
    impl->ComputeDisplacements(_sx, _sy);
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);    
  }

  /////////////////////////////////////////////////
  void WaveSimulationFFT2::SetLambda(double _lambda)
  {
    impl->SetLambda(_lambda);
  }

}
}
