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

#include <ignition/common.hh>

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

    ////////////////////////////////////////////////////////////
    /// \note: reworked version

    // parameters
    double  Lx = this->mL;
    double  Ly = this->mL;
    int     Nx = this->mN;
    int     Ny = this->mN;
    double  u10 = 5.0;
    double  phi0 = 0.0;
    double  s_param = 10.0;
    double  cap_omega_c = 0.84;

    // derived quantities

    // sample spacing [m]
    double  delta_x = this->Lx / this->Nx;
    double  delta_y = this->Ly / this->Ny;

    // fundamental wavelength [m]
    double  lambda_x_f = this->Lx;
    double  lambda_y_f = this->Ly;

    // Nyquist wavelength [m]
    double  lambda_x_Ny = 2.0 * this->delta_x;
    double  lambda_y_Ny = 2.0 * this->delta_y;

    // fundamental spatial frequency [1/m]
    double  nu_x_f = 1.0 / this->Lx;
    double  nu_y_f = 1.0 / this->Ly;

    // Nyquist spatial frequency [1/m]
    double  nu_x_Ny = 1.0 / (2.0 * this->delta_x);
    double  nu_y_Ny = 1.0 / (2.0 * this->delta_y);

    // fundamental angular spatial frequency [rad/m]
    double  kx_f = 2.0 * M_PI / this->Lx;
    double  ky_f = 2.0 * M_PI / this->Ly;

    // Nyquist angular spatial frequency [rad/m]
    double  kx_Ny = this->kx_f * this->Nx / 2.0;
    double  ky_Ny = this->ky_f * this->Ny / 2.0;

    // angular spatial frequencies in fft and math order
    std::vector<double> kx_fft  = std::vector<double>(this->Nx, 0.0);
    std::vector<double> ky_fft  = std::vector<double>(this->Ny, 0.0);
    std::vector<double> kx_math = std::vector<double>(this->Nx, 0.0);
    std::vector<double> ky_math = std::vector<double>(this->Ny, 0.0);

    // set to 1 to use a symmetric spreading function (=> standing waves)
    bool use_symmetric_spreading_fn = false;

    // square-root of two-sided discrete elevation variance spectrum
    std::vector<std::vector<double>> cap_psi_2s_root = std::vector<std::vector<double>>(
        this->Nx, std::vector<double>(this->Ny, 0.0));

    // iid random normals for real and imaginary parts of the amplitudes
    std::vector<std::vector<double>> rho = std::vector<std::vector<double>>(
        this->Nx, std::vector<double>(this->Ny, 0.0));
    std::vector<std::vector<double>> sigma = std::vector<std::vector<double>>(
        this->Nx, std::vector<double>(this->Ny, 0.0));

    // angular temporal frequency
    std::vector<std::vector<double>> omega_k = std::vector<std::vector<double>>(
        this->Nx, std::vector<double>(this->Ny, 0.0));

    double ECKVOmniDirectionalSpectrum(double k, double u10, double cap_omega_c=0.84);
    double ECKVSpreadingFunction(double k, double phi, double u10, double cap_omega_c=0.84);
    double Cos2SSpreadingFunction(double s_param, double phi, double u10, double cap_omega_c=0.84);
  };

  WaveSimulationFFT2Impl::~WaveSimulationFFT2Impl()
  {
    fftw_destroy_plan(mFFTPlan0);
    fftw_destroy_plan(mFFTPlan1);
    fftw_destroy_plan(mFFTPlan2);
    // fftw_destroy_plan(mFFTPlan3);
    // fftw_destroy_plan(mFFTPlan4);
    // fftw_destroy_plan(mFFTPlan5);
    // fftw_destroy_plan(mFFTPlan6);
    // fftw_destroy_plan(mFFTPlan7);

    fftw_free(mOut0);
    fftw_free(mOut1);
    fftw_free(mOut2);
    // fftw_free(mOut3);
    // fftw_free(mOut4);
    // fftw_free(mOut5);
    // fftw_free(mOut6);
    // fftw_free(mOut7);

    fftw_free(mIn0);
    fftw_free(mIn1);
    fftw_free(mIn2);
    // fftw_free(mIn3);
    // fftw_free(mIn4);
    // fftw_free(mIn5);
    // fftw_free(mIn6);
    // fftw_free(mIn7);
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
    /*
    mIn3  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn4  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn5  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn6  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    mIn7  = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));
    */

    // For height
    mOut0 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut1 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut2 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  

    // For xy-displacements
    /*
    mOut3 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut4 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut5 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut6 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    mOut7 = (fftw_complex*)fftw_malloc(mN2 * sizeof(fftw_complex));  
    */

    // For height
    mFFTPlan0 = fftw_plan_dft_2d(mN, mN, mIn0, mOut0, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan1 = fftw_plan_dft_2d(mN, mN, mIn1, mOut1, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan2 = fftw_plan_dft_2d(mN, mN, mIn2, mOut2, FFTW_BACKWARD, FFTW_ESTIMATE);

    // For xy-displacements
    /*
    mFFTPlan3 = fftw_plan_dft_2d(mN, mN, mIn3, mOut3, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan4 = fftw_plan_dft_2d(mN, mN, mIn4, mOut4, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan5 = fftw_plan_dft_2d(mN, mN, mIn5, mOut5, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan6 = fftw_plan_dft_2d(mN, mN, mIn6, mOut6, FFTW_BACKWARD, FFTW_ESTIMATE);
    mFFTPlan7 = fftw_plan_dft_2d(mN, mN, mIn7, mOut7, FFTW_BACKWARD, FFTW_ESTIMATE);
    */

    ////////////////////////////////////////////////////////////
    /// \note: reworked version

    // index, math-order, fft-order
    // [ 0,  1,  2,  3,  4,  5,  6,  7]
    // [-4, -3, -2, -1,  0,  1,  2,  3]
    //                 [ 0,  1,  2,  3, -4, -3, -2, -3]

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
                k, phi - this->phi0, this->u10, this->cap_omega_c);
          }
          else
          {
            // travelling waves - asymmetric spreading function
            cap_psi = this->Cos2SSpreadingFunction(
                this->s_param, phi - this->phi0, this->u10, this->cap_omega_c);
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
    // std::vector<std::vector<double>> cap_psi_2s_root(
    //     this->Nx, std::vector<double>(this->Ny, 0.0));
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        this->cap_psi_2s_root[ikx][iky] =
            cap_psi_norm * sqrt(cap_psi_2s_fft[ikx][iky] * delta_kx * delta_ky);
      }
    }

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    // std::vector<std::vector<double>> rho(
    //     this->Nx, std::vector<double>(this->Ny, 0.0));
    // std::vector<std::vector<double>> sigma(
    //     this->Nx, std::vector<double>(this->Ny, 0.0));
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        this->rho[ikx][iky] = distribution(generator);
        this->sigma[ikx][iky] = distribution(generator);
      }
    }

    // gravity acceleration [m/s^2] 
    double g = 9.82;

    // angular temporal frequency for time-dependent (from dispersion)
    // std::vector<std::vector<double>> omega_k(
    //     this->Nx, std::vector<double>(this->Ny, 0.0));
    for (int ikx = 0; ikx < this->Nx; ++ikx)
    {
      for (int iky = 0; iky < this->Ny; ++iky)
      {
        double k = sqrt(this->kx_math[ikx]*this->kx_math[ikx]
            + this->ky_math[iky]*this->ky_math[iky]);
        this->omega_k[ikx][iky] = sqrt(g * k);
      }
    }

  }

  void WaveSimulationFFT2Impl::SetWindVelocity(double _ux, double _uy)
  {
    // Update wind velocity and recompute base amplitudes.
    this->mUx = _ux;
    this->mUy = _uy;

    this->u10 = sqrt(_ux*_ux + _uy *_uy);
    this->phi0 = atan2(_uy, _ux);

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
    /*
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
    */
    // Resize output if necessary
    const int NxNy = this->Nx * this->Ny;
    if (_sx.size() != NxNy)
    {
      _sx.resize(NxNy, 0.0);
    }
    if (_sy.size() != NxNy)
    {
      _sy.resize(NxNy, 0.0);
    }

    /// \todo: add horizontal displacements
    for (int i = 0; i < NxNy; ++i)
    {
      _sx[i] = 0.0;
      _sy[i] = 0.0;
    }
  }

  void WaveSimulationFFT2Impl::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    /*
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
    */
    // Resize output if necessary
    const int NxNy = this->Nx * this->Ny;
    
    if (_dsxdx.size() != NxNy)
    {
      _dsxdx.resize(NxNy, 0.0);
    }
    if (_dsydy.size() != NxNy)
    {
      _dsydy.resize(NxNy, 0.0);
    }
    if (_dsxdy.size() != NxNy)
    {
      _dsxdy.resize(NxNy, 0.0);
    }

    /// \todo: add horizontal displacements
    for (int i = 0; i < NxNy; ++i)
    {
      _dsxdx[i] = 0.0;
      _dsydy[i] = 0.0;
      _dsxdy[i] = 0.0;
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
  
    ////////////////////////////////////////////////////////////
    /// \note: reworked version

    // alias
    auto& Nx = this->Nx;
    auto& Ny = this->Ny;
    auto& r = this->rho;
    auto& s = this->sigma;
    auto& psi_root = this->cap_psi_2s_root;


    // time update
    std::vector<std::vector<double>> cos_omega_k(
        Nx, std::vector<double>(Ny, 0.0));
    std::vector<std::vector<double>> sin_omega_k(
        Nx, std::vector<double>(Ny, 0.0));
    for (int ikx = 0; ikx < Nx; ++ikx)
    {
      for (int iky = 0; iky < Ny; ++iky)
      {
        cos_omega_k[ikx][iky] = cos(this->omega_k[ikx][iky] * _time);
        sin_omega_k[ikx][iky] = sin(this->omega_k[ikx][iky] * _time);
      }
    }

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
    /// \todo: calculate the derivatives as well (for tangent space)

    // write into mH
    for (int ikx = 0; ikx < Nx; ++ikx)
    {
      for (int iky = 0; iky < Ny; ++iky)
      {
        int idx = ikx * Nx + iky;
        this->mH[idx] = zhat[ikx][iky];
      }
    }

  }

  complex WaveSimulationFFT2Impl::Htilde0(double _k, double _kx, double _ky, double _u, double _ux, double _uy, complex _gz)
  {
    double rp = std::sqrt(0.5 * WaveSpectrum::Spectrum(_k, _kx, _ky, _u, _ux, _uy));
    complex h = _gz * rp;
    return h;
  }

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

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationFFT2

  WaveSimulationFFT2::~WaveSimulationFFT2()
  {
  }

  WaveSimulationFFT2::WaveSimulationFFT2(int _N, double _L) :
    impl(new WaveSimulationFFT2Impl(_N, _L))
  {
    impl->SetScale(1.0);
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
