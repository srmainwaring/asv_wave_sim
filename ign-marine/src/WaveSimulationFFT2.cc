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

#include "ignition/marine/WaveSimulationFFT2.hh"
#include "ignition/marine/WaveSpectrum.hh"

#include <fftw3.h>

#include <complex>
#include <random>
#include <vector>

namespace ignition
{
namespace marine
{

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationFFT2Impl

  typedef double fftw_data_type;
  typedef std::complex<fftw_data_type> complex;

  class WaveSimulationFFT2Impl
  {
    public: ~WaveSimulationFFT2Impl();

    public: WaveSimulationFFT2Impl(int _N, double _L);

    public: void SetWindVelocity(double _ux, double _uy);

    public: void SetTime(double _time);

    public: void SetScale(double _scale);

    public: void SetLambda(double _lambda);

    public: void ComputeHeights(
      std::vector<double>& _heights);

    public: void ComputeHeightDerivatives(
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy);

    public: void ComputeDisplacements(
      std::vector<double>& _sx,
      std::vector<double>& _sy);

    public: void ComputeDisplacementDerivatives(
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy);

    private: void ComputeBaseAmplitudes();

    private: void ComputeCurrentAmplitudes(double _time);
    
    private: complex Htilde0(double _k, double _kx, double _ky, double _u, double _ux, double _uy, complex _gz);

    int mN;
    int mN2;
    int mNOver2;
    double mL;
    double mUx;
    double mUy;
    double mScale;
    double mLambda;

    std::vector<double> mX;
    std::vector<double> mK;
    std::vector<double> mOmega;

    std::vector<complex> mGz;
    std::vector<complex> mH0;
    std::vector<complex> mH;      // FFT0 - height
    std::vector<complex> mHikx;   // FFT1 - d height / dx
    std::vector<complex> mHiky;   // FFT1 - d height / dy
    std::vector<complex> mDx;     // FFT3 - displacement x
    std::vector<complex> mDy;     // FFT4 - displacement y
    std::vector<complex> mHkxkx;  // FFT5 - d displacement x / dx
    std::vector<complex> mHkyky;  // FFT6 - d displacement y / dy
    std::vector<complex> mHkxky;  // FFT7 - d displacement x / dy = d displacement y / dx

    fftw_complex* mIn0;
    fftw_complex* mIn1;
    fftw_complex* mIn2;
    fftw_complex* mIn3;
    fftw_complex* mIn4;
    fftw_complex* mIn5;
    fftw_complex* mIn6;
    fftw_complex* mIn7;

    fftw_complex* mOut0;
    fftw_complex* mOut1;
    fftw_complex* mOut2;
    fftw_complex* mOut3;
    fftw_complex* mOut4;
    fftw_complex* mOut5;
    fftw_complex* mOut6;
    fftw_complex* mOut7;

    fftw_plan mFFTPlan0, mFFTPlan1, mFFTPlan2;
    fftw_plan mFFTPlan3, mFFTPlan4, mFFTPlan5, mFFTPlan6, mFFTPlan7;
  };

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

  WaveSimulationFFT2Impl::WaveSimulationFFT2Impl(int _N, double _L) :
    mN(_N),
    mN2(_N * _N),
    mNOver2(_N / 2),
    mL(_L),
    mUx(0.0),
    mUy(0.0),
    mScale(4.0 / _N),
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

  void WaveSimulationFFT2Impl::SetWindVelocity(double _ux, double _uy)
  {
    // Update wind velocity and recompute base amplitudes.
    mUx = _ux;
    mUy = _uy;
    ComputeBaseAmplitudes();
  }

  void WaveSimulationFFT2Impl::SetTime(double _time)
  {
    ComputeCurrentAmplitudes(_time);
  }

  void WaveSimulationFFT2Impl::SetScale(double _scale)
  {
    mScale = _scale;
    ComputeBaseAmplitudes();
  }

  void WaveSimulationFFT2Impl::SetLambda(double _lambda)
  {
    mLambda = _lambda;
    ComputeBaseAmplitudes();
  }

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

    for (size_t i=0; i<mN2; ++i)
    {
      _heights[i] = mOut0[i][0] * mScale;
    }
  }

  void WaveSimulationFFT2Impl::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    // Populate input array
    for (size_t i=0; i<mN2; ++i)
    {
      mIn1[i][0] = mHikx[i].real();
      mIn1[i][1] = mHikx[i].imag();

      mIn2[i][0]   = mHiky[i].real();
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

    for (size_t i=0; i<mN2; ++i)
    {
      _dhdx[i] = mOut1[i][0] * mScale;
      _dhdy[i] = mOut2[i][0] * mScale;
    }
  }

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

    for (size_t i=0; i<mN2; ++i)
    {
      _sx[i] = - mOut3[i][0] * mScale * mLambda;
      _sy[i] = - mOut4[i][0] * mScale * mLambda;
    }
  }

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

    for (size_t i=0; i<mN2; ++i)
    {
      _dsxdx[i] = - mOut5[i][0] * mScale * mLambda;
      _dsydy[i] = - mOut6[i][0] * mScale * mLambda;
      _dsxdy[i] = - mOut7[i][0] * mScale * mLambda;
    }
  }

  void WaveSimulationFFT2Impl::ComputeBaseAmplitudes()
  {
    // 1D axes
    mX.resize(mN, 0.0);
    mK.resize(mN, 0.0);
    mOmega.resize(mN, 0.0);

    // 2D grids
    mGz.resize(mN2, complex(0.0, 0.0));
    mH0.resize(mN2, complex(0.0, 0.0));
    mH.resize(mN2, complex(0.0, 0.0));
    mHikx.resize(mN2, complex(0.0, 0.0));
    mHiky.resize(mN2, complex(0.0, 0.0));
    
    mDx.resize(mN2, complex(0.0, 0.0));
    mDy.resize(mN2, complex(0.0, 0.0));
    mHkxkx.resize(mN2, complex(0.0, 0.0));
    mHkyky.resize(mN2, complex(0.0, 0.0));
    mHkxky.resize(mN2, complex(0.0, 0.0));

    // Populate wavenumber and radian frequency arrays. 
    for (size_t i=1; i<mN/2; ++i)
    {
      mK[i] = i * 2.0 * M_PI/ mL;
      mK[mN - i] = mK[i];
    }
    for (size_t i=0; i<mN; ++i)
    {
      mK[i] = i * 2.0 * M_PI/ mL;
    }
    for (size_t i=0; i<mN; ++i)
    {
      mX[i] = i * mL / mN;
      mOmega[i] = WaveSpectrum::Dispersion(mK[i]);
    }    

    // Compute Gaussian random variables.
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);

    std::normal_distribution<double> distribution(0.0, 1.0);
    for (size_t i=0; i<mN2; ++i)
    {
      mGz[i].real(distribution(generator));
      mGz[i].imag(distribution(generator));
    }

    // Compute Fourier amplitudes.
    double u  = std::sqrt(mUx*mUx + mUy*mUy);
    double kx = mK[0];
    double ky = mK[0];
    double k  = std::sqrt(kx*kx + ky*ky);
    complex gz = mGz[0];

    mH0[0] = Htilde0(k, kx, ky, u, mUx, mUy, gz);
    for (size_t ix=1; ix<mN/2; ++ix)
    {
      for (size_t iy=1; iy<mN/2; ++iy)
      {
        size_t idx = ix * mN + iy;
        kx = mK[ix];
        ky = mK[iy];
        k  = std::sqrt(kx*kx + ky*ky);
        gz = mGz[idx];
        
        mH0[idx] = Htilde0(k, kx, ky, u, mUx, mUy, gz);

        size_t cdx = (mN - ix) * mN + (mN - iy);
        mH0[cdx] = std::conj(mH0[idx]);
      }
    }
  }

  void WaveSimulationFFT2Impl::ComputeCurrentAmplitudes(double _time)
  {
    double kx0 = mK[0];
    double ky0 = mK[0];
    double k0  = std::sqrt(kx0*kx0 + ky0*ky0);
    double omega0 = WaveSpectrum::Dispersion(k0);
    complex phase0 = std::exp(complex(0.0, -omega0 * _time));
    mH[0] = mH0[0] * phase0;

    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 1.0);
    for (size_t ix=1; ix<mN/2; ++ix)
    {
      double kx = mK[ix];
      double kx2 = kx*kx;
      for (size_t iy=1; iy<mN/2; ++iy)
      {
        // Wavenumber index and conjugate index 
        size_t idx = ix * mN + iy;
        size_t cdx = (mN - ix) * mN + (mN - iy);

        // Time dependence
        double ky = mK[iy];
        double ky2 = ky*ky;
        double k  = std::sqrt(kx2 + ky2);
        double ook = 1.0 / k;
        double omega = WaveSpectrum::Dispersion(k);
        double wt = omega * _time;
        // complex phase = std::exp(complex(0.0, -omega * _time));
        double c = std::cos(wt);
        double s = std::sin(wt);
        complex phase(c, -s);

        // Height
        complex h0 = mH0[idx];
        complex h  = h0 * phase;
        complex hi = h * iunit;
        complex hok = h / k;
        complex hiok = hi / k;
        mH[idx] = h;
        mH[cdx] = std::conj(h);

        // Height derivative
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        mHikx[idx] = hikx;
        mHiky[idx] = hiky;
        mHikx[cdx] = std::conj(hikx);
        mHiky[cdx] = std::conj(hiky);

        // Displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {          
          mDx[idx]    = czero;
          mDy[idx]    = czero;
          mHkxkx[idx] = czero;
          mHkyky[idx] = czero;
          mHkxky[idx] = czero;
          mDx[cdx]    = czero;
          mDy[cdx]    = czero;
          mHkxkx[cdx] = czero;
          mHkyky[cdx] = czero;
          mHkxky[cdx] = czero;
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
          
          mDx[cdx]    = std::conj(dx);
          mDy[cdx]    = std::conj(dy);
          mHkxkx[cdx] = std::conj(hkxkx);
          mHkyky[cdx] = std::conj(hkyky);
          mHkxky[cdx] = std::conj(hkxky);
        }
      }
    }
  }

  complex WaveSimulationFFT2Impl::Htilde0(double _k, double _kx, double _ky, double _u, double _ux, double _uy, complex _gz)
  {
    double rp = std::sqrt(0.5 * WaveSpectrum::Spectrum(_k, _kx, _ky, _u, _ux, _uy));
    complex h = _gz * rp;
    return h;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationFFT2

  WaveSimulationFFT2::~WaveSimulationFFT2()
  {
  }

  WaveSimulationFFT2::WaveSimulationFFT2(int _N, double _L) :
    impl(new WaveSimulationFFT2Impl(_N, _L))
  {
    impl->SetScale(4.0/_N);
    impl->SetLambda(1.0);
  }

  void WaveSimulationFFT2::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  void WaveSimulationFFT2::SetTime(double _time)
  {
    impl->SetTime(_time);
  }

  void WaveSimulationFFT2::ComputeHeights(
    std::vector<double>& _h)
  {
    impl->ComputeHeights(_h);    
  }

  void WaveSimulationFFT2::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);  
  }

  void WaveSimulationFFT2::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);    
  }

  void WaveSimulationFFT2::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);      
  }

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

}
}
