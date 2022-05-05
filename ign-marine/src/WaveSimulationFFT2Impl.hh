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

#ifndef IGNITION_MARINE_WAVESIMULATIONFFT2_IMPL_HH_
#define IGNITION_MARINE_WAVESIMULATIONFFT2_IMPL_HH_

#include "ignition/marine/WaveSimulation.hh"

#include <fftw3.h>

#include <complex>
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
  public:
    ~WaveSimulationFFT2Impl();

    WaveSimulationFFT2Impl(int _N, double _L);

    void SetWindVelocity(double _ux, double _uy);

    void SetTime(double _time);

    void SetLambda(double _lambda);

    void ComputeHeights(
      std::vector<double>& _heights);

    void ComputeHeightDerivatives(
      std::vector<double>& _dhdx,
      std::vector<double>& _dhdy);

    void ComputeDisplacements(
      std::vector<double>& _sx,
      std::vector<double>& _sy);

    void ComputeDisplacementDerivatives(
      std::vector<double>& _dsxdx,
      std::vector<double>& _dsydy,
      std::vector<double>& _dsxdy);

    void ComputeBaseAmplitudes();

    void ComputeCurrentAmplitudes(double _time);
    
    int mN;
    int mN2;
    double mL;
    double mLambda;

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

    // parameters
    double  Lx = this->mL;
    double  Ly = this->mL;
    int     Nx = this->mN;
    int     Ny = this->mN;
    double  u10 = 5.0;
    double  phi10 = 0.0;
    double  s_param = 5.0;
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

    /// \brief For testing
    friend class TestFixtureWaveSimulationFFT2;
  };

}
}

#endif
