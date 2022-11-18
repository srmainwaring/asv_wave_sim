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

#ifndef GZ_WAVES_WAVESIMULATIONFFT2_IMPL_HH_
#define GZ_WAVES_WAVESIMULATIONFFT2_IMPL_HH_

#include "gz/waves/WaveSimulation.hh"

#include <Eigen/Dense>

#include <fftw3.h>

#include <complex>

using Eigen::MatrixXcd;
using Eigen::MatrixXd;
using Eigen::VectorXcd;
using Eigen::VectorXd;

namespace gz
{
namespace waves
{
  //////////////////////////////////////////////////
  // WaveSimulationFFT2Impl

  typedef double fftw_data_type;
  typedef std::complex<fftw_data_type> complex;

  /// \brief Implementation of a FFT based wave simulation model
  class WaveSimulationFFT2Impl
  {
  public:
    /// \brief Destructor 
    ~WaveSimulationFFT2Impl();

    /// \brief Construct a wave simulation model
    WaveSimulationFFT2Impl(double _lx, double _ly, int _nx, int _ny);

    void SetUseVectorised(bool _value);

    /// \brief Set the components of the wind velocity (U10) in [m/s]
    void SetWindVelocity(double _ux, double _uy);

    /// \brief Set the current time in seconds
    void SetTime(double _value);

    /// \brief Set the horizontal displacement scaling factor
    void SetLambda(double _value);

    /// \brief Calculate the sea surface elevation
    void ComputeElevation(
      Eigen::Ref<Eigen::MatrixXd> _h);

    /// \brief Calculate the derivative of the elevation wrt x and y
    void ComputeElevationDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dhdx,
      Eigen::Ref<Eigen::MatrixXd> _dhdy);

    /// \brief Calculate the sea surface horizontal displacements
    void ComputeDisplacements(
      Eigen::Ref<Eigen::MatrixXd> _sx,
      Eigen::Ref<Eigen::MatrixXd> _sy);

    /// \brief Calculate the derivative of the horizontal displacements wrt x and y
    void ComputeDisplacementsDerivatives(
      Eigen::Ref<Eigen::MatrixXd> _dsxdx,
      Eigen::Ref<Eigen::MatrixXd> _dsydy,
      Eigen::Ref<Eigen::MatrixXd> _dsxdy);

    /// \brief Calculate the base (time-independent) Fourier amplitudes
    void ComputeBaseAmplitudes();

    /// \brief Calculate the time-independent Fourier amplitudes
    void ComputeCurrentAmplitudes(double _time);
    
    /// \brief Reference implementation of base amplitude calculation
    void ComputeBaseAmplitudesReference();

    /// \brief Reference implementation of time-dependent amplitude calculation
    void ComputeCurrentAmplitudesReference(double _time);

    bool use_vectorised{false};

    /// \brief Horizontal displacement scaling factor. Zero for no displacement
    double lambda;

    Eigen::VectorXcd fft_h;       // FFT0 - height
    Eigen::VectorXcd fft_h_ikx;   // FFT1 - d height / dx
    Eigen::VectorXcd fft_h_iky;   // FFT1 - d height / dy
    Eigen::VectorXcd fft_sx;      // FFT3 - displacement x
    Eigen::VectorXcd fft_sy;      // FFT4 - displacement y
    Eigen::VectorXcd fft_h_kxkx;  // FFT5 - d displacement x / dx
    Eigen::VectorXcd fft_h_kyky;  // FFT6 - d displacement y / dy
    Eigen::VectorXcd fft_h_kxky;  // FFT7 - d displacement x / dy = d displacement y / dx

    fftw_complex* fft_in0;
    fftw_complex* fft_in1;
    fftw_complex* fft_in2;
    fftw_complex* fft_in3;
    fftw_complex* fft_in4;
    fftw_complex* fft_in5;
    fftw_complex* fft_in6;
    fftw_complex* fft_in7;

    fftw_complex* fft_out0;
    fftw_complex* fft_out1;
    fftw_complex* fft_out2;
    fftw_complex* fft_out3;
    fftw_complex* fft_out4;
    fftw_complex* fft_out5;
    fftw_complex* fft_out6;
    fftw_complex* fft_out7;

    fftw_plan fft_plan0, fft_plan1, fft_plan2;
    fftw_plan fft_plan3, fft_plan4, fft_plan5, fft_plan6, fft_plan7;

    // parameters
    double  lx{1.0};
    double  ly{1.0};
    int     nx{2};
    int     ny{2};

    /// \brief Wind speed at 10m above mean sea level [m]
    double  u10{5.0};

    /// \brief Direction of u10. Counter clockwise angle from x-axis [rad]
    double  phi10{0.0};

    /// \brief Spreading parameter for the cosine-2S model
    double  s_param{5.0};

    /// \brief Parameter controlling the maturity of the sea state.
    double  cap_omega_c{0.84};

    // derived quantities

    // sample spacing [m]
    double  delta_x{this->lx / this->nx};
    double  delta_y{this->ly / this->ny};

    // fundamental wavelength [m]
    double  lambda_x_f{this->lx};
    double  lambda_y_f{this->ly};

    // nyquist wavelength [m]
    double  lambda_x_ny{2.0 * this->delta_x};
    double  lambda_y_ny{2.0 * this->delta_y};

    // fundamental spatial frequency [1/m]
    double  nu_x_f{1.0 / this->lx};
    double  nu_y_f{1.0 / this->ly};

    // nyquist spatial frequency [1/m]
    double  nu_x_ny{1.0 / (2.0 * this->delta_x)};
    double  nu_y_ny{1.0 / (2.0 * this->delta_y)};

    // fundamental angular spatial frequency [rad/m]
    double  kx_f{2.0 * M_PI / this->lx};
    double  ky_f{2.0 * M_PI / this->ly};

    // nyquist angular spatial frequency [rad/m]
    double  kx_ny{this->kx_f * this->nx / 2.0};
    double  ky_ny{this->ky_f * this->ny / 2.0};

    // angular spatial frequencies in fft and math order
    Eigen::VectorXd kx_fft{Eigen::VectorXd::Zero(this->nx)};
    Eigen::VectorXd ky_fft{Eigen::VectorXd::Zero(this->ny)};
    Eigen::VectorXd kx_math{Eigen::VectorXd::Zero(this->nx)};
    Eigen::VectorXd ky_math{Eigen::VectorXd::Zero(this->ny)};

    // set to 1 to use a symmetric spreading function (=> standing waves)
    bool use_symmetric_spreading_fn{false};

    //////////////////////////////////////////////////
    /// \note: use flattened array storage for optimised version

    // square-root of two-sided discrete elevation variance spectrum
    Eigen::VectorXd cap_psi_2s_root{Eigen::VectorXd::Zero(this->nx * this->ny)};

    // iid random normals for real and imaginary parts of the amplitudes
    Eigen::VectorXd rho{Eigen::VectorXd::Zero(this->nx * this->ny)};
    Eigen::VectorXd sigma{Eigen::VectorXd::Zero(this->nx * this->ny)};

    // angular temporal frequency
    Eigen::VectorXd omega_k{Eigen::VectorXd::Zero(this->nx * this->ny)};

    //////////////////////////////////////////////////
    /// \note: use 2d array storage for reference version, resized if required

    // square-root of two-sided discrete elevation variance spectrum
    Eigen::MatrixXd cap_psi_2s_root_ref;

    // iid random normals for real and imaginary parts of the amplitudes
    Eigen::MatrixXd rho_ref;
    Eigen::MatrixXd sigma_ref;

    // angular temporal frequency
    Eigen::MatrixXd omega_k_ref;

    static double ECKVOmniDirectionalSpectrum(
        double k, double u10, double cap_omega_c=0.84);
    static double ECKVSpreadingFunction(
        double k, double phi, double u10, double cap_omega_c=0.84);
    static double Cos2SSpreadingFunction(
        double s_param, double phi, double u10, double cap_omega_c=0.84);

    /// \brief For testing
    friend class TestFixtureWaveSimulationFFT2;
  };
}
}

#endif
