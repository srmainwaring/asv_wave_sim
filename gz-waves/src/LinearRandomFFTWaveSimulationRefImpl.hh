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

#ifndef GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONREFIMPL_HH_
#define GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONREFIMPL_HH_

#include <Eigen/Dense>

#include <fftw3.h>

#include <complex>

#include "gz/waves/WaveSimulation.hh"
#include "LinearRandomFFTWaveSimulationRef.hh"

namespace Eigen
{
  typedef Eigen::Array<
    std::complex<double>,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > ArrayXXcdRowMajor;
}  // namespace Eigen

namespace gz
{
namespace waves
{
//////////////////////////////////////////////////
// LinearRandomFFTWaveSimulationRef::Impl

typedef double fftw_data_type;
typedef std::complex<fftw_data_type> complex;

/// \brief Implementation of a FFT based wave simulation model
///
/// \note The FFT wave simulation mixes storage ordering which
///       must be made consistent and should be column major
///       which is the Eigen default..
///
class LinearRandomFFTWaveSimulationRef::Impl
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Destructor
  ~Impl();

  /// \brief Construct a wave simulation model
  Impl(double lx, double ly, Index nx, Index ny);

  /// \brief Set the components of the wind velocity (U10) in [m/s]
  void SetWindVelocity(double ux, double uy);

  void SetSteepness(double value);

  /// \brief Set the current time in seconds
  void SetTime(double value);

  /// \brief Set the horizontal displacement scaling factor
  void SetLambda(double value);

  /// \brief Calculate the sea surface elevation
  void ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h);

  /// \brief Calculate the derivative of the elevation wrt x and y
  void ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy);

  /// \brief Calculate the sea surface horizontal displacements
  void DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy);

  /// \brief Calculate the derivative of the horizontal displacements
  ///        wrt x and y
  void DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy);

  /// \brief Base amplitude calculation
  void ComputeBaseAmplitudes();

  /// \brief Time-dependent amplitude calculation
  void ComputeCurrentAmplitudes(double time);

  void InitFFTCoeffStorage();
  void InitWaveNumbers();

  void CreateFFTWPlans();
  void DestroyFFTWPlans();

  /// \note FFTW expects the multi-dimensional arrays to be in row-major
  ///       format. Eigen::ArrayXXcd is column-major, so here we
  ///       explicity set the storage type.
  ///
  /// https://www.fftw.org/fftw3_doc/Row_002dmajor-Format.html
  ///
  Eigen::ArrayXXcdRowMajor fft_h_;       // FFT0 - height
  Eigen::ArrayXXcdRowMajor fft_h_ikx_;   // FFT1 - d height / dx
  Eigen::ArrayXXcdRowMajor fft_h_iky_;   // FFT1 - d height / dy
  Eigen::ArrayXXcdRowMajor fft_sx_;      // FFT3 - displacement x
  Eigen::ArrayXXcdRowMajor fft_sy_;      // FFT4 - displacement y
  Eigen::ArrayXXcdRowMajor fft_h_kxkx_;  // FFT5 - d displacement x / dx
  Eigen::ArrayXXcdRowMajor fft_h_kyky_;  // FFT6 - d displacement y / dy
  Eigen::ArrayXXcdRowMajor fft_h_kxky_;  // FFT7 - d displacement x / dy

  Eigen::ArrayXXcdRowMajor fft_out0_;
  Eigen::ArrayXXcdRowMajor fft_out1_;
  Eigen::ArrayXXcdRowMajor fft_out2_;
  Eigen::ArrayXXcdRowMajor fft_out3_;
  Eigen::ArrayXXcdRowMajor fft_out4_;
  Eigen::ArrayXXcdRowMajor fft_out5_;
  Eigen::ArrayXXcdRowMajor fft_out6_;
  Eigen::ArrayXXcdRowMajor fft_out7_;

  fftw_plan fft_plan0_, fft_plan1_, fft_plan2_, fft_plan3_;
  fftw_plan fft_plan4_, fft_plan5_, fft_plan6_, fft_plan7_;

  /// \brief Gravity acceleration [m/s^2]
  double gravity_{9.81};

  /// \brief Horizontal displacement scaling factor. Zero for no displacement
  double lambda_{0.0};

  // grid parameters
  double  lx_{1.0};
  double  ly_{1.0};
  Index   nx_{2};
  Index   ny_{2};

  /// \brief Wind speed at 10m above mean sea level [m]
  double  u10_{5.0};

  /// \brief Direction of u10. Counter clockwise angle from x-axis [rad]
  double  phi10_{0.0};

  /// \brief Spreading parameter for the cosine-2S model
  double  s_param_{5.0};

  /// \brief Parameter controlling the maturity of the sea state.
  double  cap_omega_c_{0.84};

  // fundamental angular spatial frequency [rad/m]
  double  kx_f_{2.0 * M_PI / lx_};
  double  ky_f_{2.0 * M_PI / ly_};

  // angular spatial frequencies in fft and math order
  Eigen::ArrayXd kx_fft_;
  Eigen::ArrayXd ky_fft_;
  Eigen::ArrayXd kx_math_;
  Eigen::ArrayXd ky_math_;

  /// \brief Set to 1 to use a symmetric spreading function (standing waves).
  bool use_symmetric_spreading_fn_{false};

  //////////////////////////////////////////////////
  /// \note: use 2d array storage for reference version, resized if required

  // square-root of two-sided discrete elevation variance spectrum
  Eigen::ArrayXXd cap_psi_2s_root_;

  // iid random normals for real and imaginary parts of the amplitudes
  Eigen::ArrayXXd rho_;
  Eigen::ArrayXXd sigma_;

  // angular temporal frequency
  Eigen::ArrayXXd omega_k_;

  static double ECKVOmniDirectionalSpectrum(
      double k, double u10, double cap_omega_c = 0.84,
      double gravity = 9.81);
  static double ECKVSpreadingFunction(
      double k, double phi, double u10, double cap_omega_c = 0.84,
      double gravity = 9.81);
  static double Cos2sSpreadingFunction(
      double s_param, double phi, double u10, double cap_omega_c = 0.84,
      double gravity = 9.81);

  /// \brief For testing
  friend class LinearRandomFFTWaveSimFixture;
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONREFIMPL_HH_
