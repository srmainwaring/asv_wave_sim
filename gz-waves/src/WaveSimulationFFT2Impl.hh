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

#include <complex>

#include <Eigen/Dense>

#include <fftw3.h>

#include "gz/waves/WaveSimulation.hh"

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
  ///
  /// \note The FFT wave simulation mixes storage ordering which
  ///       must be made consistent and should be column major
  ///       which is the Eigen default..
  ///
  class WaveSimulationFFT2Impl
  {
  public:
    /// \brief Destructor 
    ~WaveSimulationFFT2Impl();

    /// \brief Construct a wave simulation model
    WaveSimulationFFT2Impl(double lx, double ly, int nx, int ny);

    void SetUseVectorised(bool value);

    /// \brief Set the components of the wind velocity (U10) in [m/s]
    void SetWindVelocity(double ux, double uy);

    /// \brief Set the current time in seconds
    void SetTime(double value);

    /// \brief Set the horizontal displacement scaling factor
    void SetLambda(double value);

    /// \brief Calculate the sea surface elevation
    void ComputeElevation(
      Eigen::Ref<Eigen::MatrixXd> h);

    /// \brief Calculate the derivative of the elevation wrt x and y
    void ComputeElevationDerivatives(
      Eigen::Ref<Eigen::MatrixXd> dhdx,
      Eigen::Ref<Eigen::MatrixXd> dhdy);

    /// \brief Calculate the sea surface horizontal displacements
    void ComputeDisplacements(
      Eigen::Ref<Eigen::MatrixXd> sx,
      Eigen::Ref<Eigen::MatrixXd> sy);

    /// \brief Calculate the derivative of the horizontal displacements wrt x and y
    void ComputeDisplacementsDerivatives(
      Eigen::Ref<Eigen::MatrixXd> dsxdx,
      Eigen::Ref<Eigen::MatrixXd> dsydy,
      Eigen::Ref<Eigen::MatrixXd> dsxdy);

    /// \brief Calculate the base (time-independent) Fourier amplitudes
    void ComputeBaseAmplitudes();

    /// \brief Calculate the time-independent Fourier amplitudes
    void ComputeCurrentAmplitudes(double time);
    
    /// \brief Reference implementation of base amplitude calculation
    void ComputeBaseAmplitudesReference();

    /// \brief Reference implementation of time-dependent amplitude calculation
    void ComputeCurrentAmplitudesReference(double time);

    bool use_vectorised_{false};

    /// \brief Horizontal displacement scaling factor. Zero for no displacement
    double lambda_;

    Eigen::VectorXcd fft_h_;       // FFT0 - height
    Eigen::VectorXcd fft_h_ikx_;   // FFT1 - d height / dx
    Eigen::VectorXcd fft_h_iky_;   // FFT1 - d height / dy
    Eigen::VectorXcd fft_sx_;      // FFT3 - displacement x
    Eigen::VectorXcd fft_sy_;      // FFT4 - displacement y
    Eigen::VectorXcd fft_h_kxkx_;  // FFT5 - d displacement x / dx
    Eigen::VectorXcd fft_h_kyky_;  // FFT6 - d displacement y / dy
    Eigen::VectorXcd fft_h_kxky_;  // FFT7 - d displacement x / dy = d displacement y / dx

    fftw_complex* fft_in0_;
    fftw_complex* fft_in1_;
    fftw_complex* fft_in2_;
    fftw_complex* fft_in3_;
    fftw_complex* fft_in4_;
    fftw_complex* fft_in5_;
    fftw_complex* fft_in6_;
    fftw_complex* fft_in7_;

    fftw_complex* fft_out0_;
    fftw_complex* fft_out1_;
    fftw_complex* fft_out2_;
    fftw_complex* fft_out3_;
    fftw_complex* fft_out4_;
    fftw_complex* fft_out5_;
    fftw_complex* fft_out6_;
    fftw_complex* fft_out7_;

    fftw_plan fft_plan0_, fft_plan1_, fft_plan2_, fft_plan3_;
    fftw_plan fft_plan4_, fft_plan5_, fft_plan6_, fft_plan7_;

    // parameters
    double  lx_{1.0};
    double  ly_{1.0};
    int     nx_{2};
    int     ny_{2};

    /// \brief Wind speed at 10m above mean sea level [m]
    double  u10_{5.0};

    /// \brief Direction of u10. Counter clockwise angle from x-axis [rad]
    double  phi10_{0.0};

    /// \brief Spreading parameter for the cosine-2S model
    double  s_param_{5.0};

    /// \brief Parameter controlling the maturity of the sea state.
    double  cap_omega_c_{0.84};

    // derived quantities

    // sample spacing [m]
    double  delta_x_{lx_ / nx_};
    double  delta_y_{ly_ / ny_};

    // fundamental wavelength [m]
    double  lambda_x_f_{lx_};
    double  lambda_y_f_{ly_};

    // nyquist wavelength [m]
    double  lambda_x_ny_{2.0 * delta_x_};
    double  lambda_y_ny_{2.0 * delta_y_};

    // fundamental spatial frequency [1/m]
    double  nu_x_f_{1.0 / lx_};
    double  nu_y_f_{1.0 / ly_};

    // nyquist spatial frequency [1/m]
    double  nu_x_ny_{1.0 / (2.0 * delta_x_)};
    double  nu_y_ny_{1.0 / (2.0 * delta_y_)};

    // fundamental angular spatial frequency [rad/m]
    double  kx_f_{2.0 * M_PI / lx_};
    double  ky_f_{2.0 * M_PI / ly_};

    // nyquist angular spatial frequency [rad/m]
    double  kx_ny_{kx_f_ * nx_ / 2.0};
    double  ky_ny_{ky_f_ * ny_ / 2.0};

    // angular spatial frequencies in fft and math order
    Eigen::VectorXd kx_fft_{Eigen::VectorXd::Zero(nx_)};
    Eigen::VectorXd ky_fft_{Eigen::VectorXd::Zero(ny_)};
    Eigen::VectorXd kx_math_{Eigen::VectorXd::Zero(nx_)};
    Eigen::VectorXd ky_math_{Eigen::VectorXd::Zero(ny_)};

    // set to 1 to use a symmetric spreading function (=> standing waves)
    bool use_symmetric_spreading_fn_{false};

    //////////////////////////////////////////////////
    /// \note: use flattened array storage for optimised version

    // square-root of two-sided discrete elevation variance spectrum
    Eigen::VectorXd cap_psi_2s_root_{Eigen::VectorXd::Zero(nx_ * ny_)};

    // iid random normals for real and imaginary parts of the amplitudes
    Eigen::VectorXd rho_{Eigen::VectorXd::Zero(nx_ * ny_)};
    Eigen::VectorXd sigma_{Eigen::VectorXd::Zero(nx_ * ny_)};

    // angular temporal frequency
    Eigen::VectorXd omega_k_{Eigen::VectorXd::Zero(nx_ * ny_)};

    //////////////////////////////////////////////////
    /// \note: use 2d array storage for reference version, resized if required

    // square-root of two-sided discrete elevation variance spectrum
    Eigen::MatrixXd cap_psi_2s_root_ref_;

    // iid random normals for real and imaginary parts of the amplitudes
    Eigen::MatrixXd rho_ref_;
    Eigen::MatrixXd sigma_ref_;

    // angular temporal frequency
    Eigen::MatrixXd omega_k_ref_;

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
