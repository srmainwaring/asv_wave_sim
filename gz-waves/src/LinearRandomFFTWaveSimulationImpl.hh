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

#ifndef GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONIMPL_HH_
#define GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONIMPL_HH_

#include <Eigen/Dense>

#include <fftw3.h>

#include <complex>
#include <vector>

#include "gz/waves/WaveSimulation.hh"
#include "gz/waves/LinearRandomFFTWaveSimulation.hh"

namespace Eigen
{
  typedef Eigen::Array<
    std::complex<double>,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > ArrayXXcdRowMajor;

  typedef Eigen::Array<
    double,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor
  > ArrayXXdRowMajor;
}  // namespace Eigen

namespace gz
{
namespace waves
{
//////////////////////////////////////////////////
// LinearRandomFFTWaveSimulation::Impl

typedef double fftw_data_type;
typedef std::complex<fftw_data_type> complex;

/// \brief Implementation of a FFT based wave simulation model
///
/// \note The FFT wave simulation mixes storage ordering which
///       must be made consistent and should be column major
///       which is the Eigen default..
///
class LinearRandomFFTWaveSimulation::Impl
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Destructor
  ~Impl();

  /// \brief Construct a wave simulation model
  Impl(double lx, double ly, Index nx, Index ny);

  /// \brief Construct a wave simulation model
  Impl(double lx, double ly, double lz, Index nx, Index ny, Index nz);

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

  void PressureAt(
      Index iz,
      Eigen::Ref<Eigen::ArrayXXd> pressure);

  void ElevationAt(
      Index ix, Index iy,
      double& eta);

  void DisplacementAt(
      Index ix, Index iy,
      double& sx, double& sy);

  void PressureAt(
      Index ix, Index iy, Index iz,
      double& pressure);

  /// \brief Calculate the base (time-independent) Fourier amplitudes
  void ComputeBaseAmplitudes();

  /// \brief Calculate the time-independent Fourier amplitudes
  void ComputeCurrentAmplitudes(double time);

  void InitWaveNumbers();
  void InitPressureGrid();

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

  /// \note if using fftw_plan_dft_c2r_2d:
  ///       complex input array has size: nx * ny / 2 + 1
  ///       real output array has size:   nx * ny
  ///
  Eigen::ArrayXXdRowMajor  fft_out0_;
  Eigen::ArrayXXdRowMajor  fft_out1_;
  Eigen::ArrayXXdRowMajor  fft_out2_;
  Eigen::ArrayXXdRowMajor  fft_out3_;
  Eigen::ArrayXXdRowMajor  fft_out4_;
  Eigen::ArrayXXdRowMajor  fft_out5_;
  Eigen::ArrayXXdRowMajor  fft_out6_;
  Eigen::ArrayXXdRowMajor  fft_out7_;

  fftw_plan fft_plan0_, fft_plan1_, fft_plan2_, fft_plan3_;
  fftw_plan fft_plan4_, fft_plan5_, fft_plan6_, fft_plan7_;

  /// FFT storage and plans for pressure calculations
  std::vector<Eigen::ArrayXXcdRowMajor>   fft_in_p_;
  std::vector<Eigen::ArrayXXdRowMajor>    fft_out_p_;
  std::vector<fftw_plan>                  fft_plan_p_;

  /// \brief lazy evaluation flags
  std::vector<bool> fft_needs_update_;

  /// \brief Gravity acceleration [m/s^2]
  double gravity_{9.81};

  /// \brief Horizontal displacement scaling factor. Zero for no displacement
  double lambda_{0.6};

  // elevation and pressure grid parameters
  double  lx_{1.0};
  double  ly_{1.0};
  double  lz_{0.0};
  Index   nx_{2};
  Index   ny_{2};
  Index   nz_{1};

  // points along z-axis at which pressure is sampled
  Eigen::ArrayXd z_;

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

  // angular spatial frequencies in fft order (nx, ny)
  Eigen::ArrayXd kx_fft_;
  Eigen::ArrayXd ky_fft_;

  /// \brief Set to 1 to use a symmetric spreading function (standing waves).
  bool use_symmetric_spreading_fn_{false};

  //////////////////////////////////////////////////
  // storage for current amplitudes (nx * ny)
  Eigen::ArrayXcd zhat_;

  //////////////////////////////////////////////////
  // storage for base amplitudes (fft order)

  // square-root of two-sided discrete elevation variance spectrum
  Eigen::ArrayXd cap_psi_2s_root_;

  // iid random normals for real and imaginary parts of the amplitudes
  Eigen::ArrayXd rho_;
  Eigen::ArrayXd sigma_;

  // unique angular temporal frequency, index and reverse lookup
  std::vector<double> uomega_k_;
  std::vector<Index>  uindex_;
  std::vector<Index>  uinverse_;
  Eigen::ArrayXd      ucos_wt_;
  Eigen::ArrayXd      usin_wt_;

  /// \brief For testing
  friend class LinearRandomFFTWaveSimFixture;
};

}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_SRC_LINEARRANDOMFFTWAVESIMULATIONIMPL_HH_
