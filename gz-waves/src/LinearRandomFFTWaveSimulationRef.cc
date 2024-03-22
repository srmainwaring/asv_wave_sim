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

// Attribution:
// The time-dependent update step in ComputeCurrentAmplitudes is based
// on the Curtis Mobley's IDL code cgAnimate_2D_SeaSurface.py

//*****************************************************************************
//* This code is copyright (c) 2016 by Curtis D. Mobley.
//* Permission is hereby given to reproduce and use this code for
//* non-commercial academic research, provided that the user suitably
//* acknowledges Curtis D. Mobley in any presentations, reports, publications,
//* or other works that make use of the code or its output.  Depending on the
//* extent of use of the code or its outputs, suitable acknowledgement can
//* range from a footnote to offer of coauthorship.  Further questions can be
//* directed to curtis.mobley@sequoiasci.com.
//*****************************************************************************

#include "LinearRandomFFTWaveSimulationRef.hh"

#include <Eigen/Dense>

#include <fftw3.h>

#include <complex>
#include <random>

#include <gz/common/Console.hh>

#include "gz/waves/Types.hh"
#include "LinearRandomFFTWaveSimulationRefImpl.hh"

namespace gz
{
namespace waves
{
  //////////////////////////////////////////////////
  LinearRandomFFTWaveSimulationRef::Impl::~Impl()
  {
    DestroyFFTWPlans();
  }

  //////////////////////////////////////////////////
  LinearRandomFFTWaveSimulationRef::Impl::Impl(
    double lx, double ly, Index nx, Index ny) :
    lambda_(0.6),
    lx_(lx),
    ly_(ly),
    nx_(nx),
    ny_(ny)
  {
    ComputeBaseAmplitudes();
    CreateFFTWPlans();
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::SetLambda(double value)
  {
    lambda_ = value;
    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::SetWindVelocity(
      double ux, double uy)
  {
    // Update wind velocity and recompute base amplitudes.
    u10_ = sqrt(ux*ux + uy *uy);
    phi10_ = atan2(uy, ux);

    ComputeBaseAmplitudes();
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::SetSteepness(double value)
  {
    lambda_ = value;
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::SetTime(double time)
  {
    ComputeCurrentAmplitudes(time);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h)
  {
    // run the FFT
    fftw_execute(fft_plan0_);

    // change from row to column major storage
    Index n2 = nx_ * ny_;
    h = fft_out0_.reshaped<Eigen::ColMajor>(n2, 1).real();
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy)
  {
    // run the FFTs
    fftw_execute(fft_plan1_);
    fftw_execute(fft_plan2_);

    // change from row to column major storage
    Index n2 = nx_ * ny_;
    dhdy = fft_out1_.reshaped<Eigen::ColMajor>(n2, 1).real();
    dhdx = fft_out2_.reshaped<Eigen::ColMajor>(n2, 1).real();
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy)
  {
    // run the FFTs
    fftw_execute(fft_plan3_);
    fftw_execute(fft_plan4_);

    // change from row to column major storage
    Index n2 = nx_ * ny_;
    sy = fft_out3_.reshaped<Eigen::ColMajor>(n2, 1).real() * lambda_ * -1.0;
    sx = fft_out4_.reshaped<Eigen::ColMajor>(n2, 1).real() * lambda_ * -1.0;
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy)
  {
    // run the FFTs
    fftw_execute(fft_plan5_);
    fftw_execute(fft_plan6_);
    fftw_execute(fft_plan7_);

    // change from row to column major storage
    Index n2 = nx_ * ny_;
    dsydy = fft_out5_.reshaped<Eigen::ColMajor>(n2, 1).real() * lambda_ * -1.0;
    dsxdx = fft_out6_.reshaped<Eigen::ColMajor>(n2, 1).real() * lambda_ * -1.0;
    dsxdy = fft_out7_.reshaped<Eigen::ColMajor>(n2, 1).real() * lambda_ *  1.0;
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::ComputeBaseAmplitudes()
  {
    InitFFTCoeffStorage();
    InitWaveNumbers();

    // Index n2 = nx_ * ny_;

    // arrays for reference version
    if (cap_psi_2s_root_.size() == 0)
    {
      cap_psi_2s_root_ = Eigen::ArrayXXd::Zero(nx_, ny_);
      rho_             = Eigen::ArrayXXd::Zero(nx_, ny_);
      sigma_           = Eigen::ArrayXXd::Zero(nx_, ny_);
      omega_k_         = Eigen::ArrayXXd::Zero(nx_, ny_);
    }

    // Guide to indexing conventions:  1. index, 2. math-order, 3. fft-order
    //
    // 1. [ 0,  1,  2,  3,  4,  5,  6,  7]
    // 2. [-4, -3, -2, -1,  0,  1,  2,  3]
    // 3.                 [ 0,  1,  2,  3, -4, -3, -2, -3]
    //

    // sample spacing [m]
    double  delta_x{lx_ / nx_};
    double  delta_y{ly_ / ny_};

    // fundamental wavelength [m]
    double  lambda_x_f{lx_};
    double  lambda_y_f{ly_};

    // nyquist wavelength [m]
    // double  lambda_x_nyquist{2.0 * delta_x};
    // double  lambda_y_nyquist{2.0 * delta_y};

    // fundamental spatial frequency [1/m]
    double  nu_x_f{1.0 / lx_};
    double  nu_y_f{1.0 / ly_};

    // nyquist spatial frequency [1/m]
    double  nu_x_nyquist{1.0 / (2.0 * delta_x)};
    double  nu_y_nyquist{1.0 / (2.0 * delta_y)};

    // nyquist angular spatial frequency [rad/m]
    double  kx_nyquist{kx_f_ * nx_ / 2.0};
    double  ky_nyquist{ky_f_ * ny_ / 2.0};

    // debug
    gzmsg << "LinearRandomFFTWaveSimulationRef" << "\n";
    gzmsg << "lx:           " << lx_ << "\n";
    gzmsg << "ly:           " << ly_ << "\n";
    gzmsg << "nx:           " << nx_ << "\n";
    gzmsg << "ny:           " << ny_ << "\n";
    gzmsg << "delta_x:      " << delta_x << "\n";
    gzmsg << "delta_y:      " << delta_y << "\n";
    gzmsg << "lambda_x_f:   " << lambda_x_f << "\n";
    gzmsg << "lambda_y_f:   " << lambda_y_f << "\n";
    gzmsg << "nu_x_f:       " << nu_x_f << "\n";
    gzmsg << "nu_y_f:       " << nu_y_f << "\n";
    gzmsg << "nu_x_nyquist: " << nu_x_nyquist << "\n";
    gzmsg << "nu_y_nyquist: " << nu_y_nyquist << "\n";
    gzmsg << "kx_f:         " << kx_f_ << "\n";
    gzmsg << "ky_f:         " << ky_f_ << "\n";
    gzmsg << "kx_nyquist:   " << kx_nyquist << "\n";
    gzmsg << "ky_nyquist:   " << ky_nyquist << "\n";

#if 0
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : kx_fft_) os << v << " "; os << "]\n";
      gzmsg << "kx_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : ky_fft_) os << v << " "; os << "]\n";
      gzmsg << "ky_fft:      " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : kx_math_) os << v << " "; os << "]\n";
      gzmsg << "kx_math:     " << os.str();
    }
    {
      std::ostringstream os;
      os << "[ "; for (auto& v : ky_math_) os << v << " "; os << "]\n";
      gzmsg << "ky_math:     " << os.str();
    }
#endif

    // continuous two-sided elevation variance spectrum
    Eigen::ArrayXXd cap_psi_2s_math = Eigen::ArrayXXd::Zero(nx_, ny_);

    // calculate spectrum in math-order (not vectorised)
    for (Index ikx=0; ikx < nx_; ++ikx)
    {
      for (Index iky=0; iky < ny_; ++iky)
      {
        double k = sqrt(kx_math_[ikx]*kx_math_[ikx]
            + ky_math_[iky]*ky_math_[iky]);
        double phi = atan2(ky_math_[iky], kx_math_[ikx]);

        if (std::abs(k) < 1.0E-8)
        {
          cap_psi_2s_math(ikx, iky) = 0.0;
        } else {
          double cap_psi = 0.0;
          if (use_symmetric_spreading_fn_)
          {
            // standing waves - symmetric spreading function
            cap_psi = Impl::ECKVSpreadingFunction(
                k, phi - phi10_, u10_, cap_omega_c_, gravity_);
          } else {
            // travelling waves - asymmetric spreading function
            cap_psi = Impl::Cos2sSpreadingFunction(
                s_param_, phi - phi10_, u10_, cap_omega_c_, gravity_);
          }
          double cap_s = Impl::ECKVOmniDirectionalSpectrum(
              k, u10_, cap_omega_c_, gravity_);
          cap_psi_2s_math(ikx, iky) = cap_s * cap_psi / k;
        }
      }
    }

    // debug
#if 0
    {
      std::ostringstream os;
      os << "[\n";
      for (Index ikx = 0; ikx < nx_; ++ikx)
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
    Eigen::ArrayXXd cap_psi_2s_fft = Eigen::ArrayXXd::Zero(nx_, ny_);
    for (Index ikx=0; ikx < nx_; ++ikx)
    {
      Index ikx_fft = (ikx + nx_/2) % nx_;
      for (Index iky=0; iky < ny_; ++iky)
      {
        Index iky_fft = (iky + ny_/2) % ny_;
        cap_psi_2s_fft(ikx_fft, iky_fft) = cap_psi_2s_math(ikx, iky);
      }
    }

    // square-root of two-sided discrete elevation variance spectrum
    double cap_psi_norm = 0.5;
    double delta_kx = kx_f_;
    double delta_ky = ky_f_;

    for (Index ikx=0; ikx < nx_; ++ikx)
    {
      for (Index iky=0; iky < ny_; ++iky)
      {
        cap_psi_2s_root_(ikx, iky) =
            cap_psi_norm * sqrt(cap_psi_2s_fft(ikx, iky) * delta_kx * delta_ky);
      }
    }

    // iid random normals for real and imaginary parts of the amplitudes
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (Index ikx=0; ikx < nx_; ++ikx)
    {
      for (Index iky=0; iky < ny_; ++iky)
      {
        rho_(ikx, iky) = distribution(generator);
        sigma_(ikx, iky) = distribution(generator);
      }
    }

    // angular temporal frequency for time-dependent (from dispersion)
    for (Index ikx = 0; ikx < nx_; ++ikx)
    {
      double kx = kx_fft_[ikx];
      double kx2 = kx*kx;
      for (Index iky=0; iky < ny_; ++iky)
      {
        double ky = ky_fft_[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        omega_k_(ikx, iky) = sqrt(gravity_ * k);
      }
    }
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::ComputeCurrentAmplitudes(
      double time)
  {
    // alias
    const Eigen::Ref<const Eigen::ArrayXXd>& r = rho_;
    const Eigen::Ref<const Eigen::ArrayXXd>& s = sigma_;
    const Eigen::Ref<const Eigen::ArrayXXd>& psi_root = cap_psi_2s_root_;

    // time update
    Eigen::ArrayXXd cos_omega_k = Eigen::ArrayXXd::Zero(nx_, ny_);
    Eigen::ArrayXXd sin_omega_k = Eigen::ArrayXXd::Zero(nx_, ny_);
    for (Index ikx=0; ikx < nx_; ++ikx)
    {
      for (Index iky=0; iky < ny_; ++iky)
      {
        cos_omega_k(ikx, iky) = cos(omega_k_(ikx, iky) * time);
        sin_omega_k(ikx, iky) = sin(omega_k_(ikx, iky) * time);
      }
    }

    // non-vectorised reference version
    Eigen::ArrayXXcd zhat = Eigen::ArrayXXcd::Zero(nx_, ny_);
    for (Index ikx=1; ikx < nx_; ++ikx)
    {
      for (Index iky=1; iky < ny_; ++iky)
      {
        zhat(ikx, iky) = complex(
            + (r(ikx, iky) * psi_root(ikx, iky)
            + r(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky))
            * cos_omega_k(ikx, iky)
            + (s(ikx, iky) * psi_root(ikx, iky)
            + s(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky))
            * sin_omega_k(ikx, iky),
            - (r(ikx, iky) * psi_root(ikx, iky)
            - r(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky))
            * sin_omega_k(ikx, iky)
            + (s(ikx, iky) * psi_root(ikx, iky)
            - s(nx_-ikx, ny_-iky) * psi_root(nx_-ikx, ny_-iky))
            * cos_omega_k(ikx, iky));
      }
    }

    for (Index iky = 1; iky < ny_/2+1; ++iky)
    {
      Index ikx = 0;
      zhat(ikx, iky) = complex(
          + (r(ikx, iky) * psi_root(ikx, iky)
          + r(ikx, ny_-iky) * psi_root(ikx, ny_-iky)) * cos_omega_k(ikx, iky)
          + (s(ikx, iky) * psi_root(ikx, iky)
          + s(ikx, ny_-iky) * psi_root(ikx, ny_-iky)) * sin_omega_k(ikx, iky),
          - (r(ikx, iky) * psi_root(ikx, iky)
          - r(ikx, ny_-iky) * psi_root(ikx, ny_-iky)) * sin_omega_k(ikx, iky)
          + (s(ikx, iky) * psi_root(ikx, iky)
          - s(ikx, ny_-iky) * psi_root(ikx, ny_-iky)) * cos_omega_k(ikx, iky));
      zhat(ikx, ny_-iky) = std::conj(zhat(ikx, iky));
    }

    for (Index ikx = 1; ikx < nx_/2+1; ++ikx)
    {
      Index iky = 0;
      zhat(ikx, iky) = complex(
          + (r(ikx, iky) * psi_root(ikx, iky)
          + r(nx_-ikx, iky) * psi_root(nx_-ikx, iky)) * cos_omega_k(ikx, iky)
          + (s(ikx, iky) * psi_root(ikx, iky)
          + s(nx_-ikx, iky) * psi_root(nx_-ikx, iky)) * sin_omega_k(ikx, iky),
          - (r(ikx, iky) * psi_root(ikx, iky)
          - r(nx_-ikx, iky) * psi_root(nx_-ikx, iky)) * sin_omega_k(ikx, iky)
          + (s(ikx, iky) * psi_root(ikx, iky)
          - s(nx_-ikx, iky) * psi_root(nx_-ikx, iky)) * cos_omega_k(ikx, iky));
      zhat(nx_-ikx, iky) = std::conj(zhat(ikx, iky));
    }

    zhat(0, 0) = complex(0.0, 0.0);

    // write into fft_h_, fft_h_ikx_, fft_h_iky_, etc.
    const complex iunit(0.0, 1.0);
    const complex czero(0.0, 0.0);
    for (Index ikx=0; ikx < nx_; ++ikx)
    {
      double kx = kx_fft_[ikx];
      double kx2 = kx*kx;
      for (Index iky=0; iky < ny_; ++iky)
      {
        double ky = ky_fft_[iky];
        double ky2 = ky*ky;
        double k = sqrt(kx2 + ky2);
        double ook = 1.0 / k;

        complex h  = zhat(ikx, iky);
        complex hi = h * iunit;
        complex hok = h * ook;
        complex hiok = hi * ook;

        // height (amplitude)
        fft_h_(ikx, iky) = h;

        // height derivatives
        complex hikx = hi * kx;
        complex hiky = hi * ky;

        // Nyquist terms for derivatives must be zero.
        // For an explanation see:
        // https://math.mit.edu/~stevenj/fft-deriv.pdf
        if (ikx == nx_ / 2)
          hikx = czero;
        if (iky == ny_ / 2)
          hiky = czero;

        fft_h_ikx_(ikx, iky) = hikx;
        fft_h_iky_(ikx, iky) = hiky;

        // displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {
          fft_sx_(ikx, iky)     = czero;
          fft_sy_(ikx, iky)     = czero;
          fft_h_kxkx_(ikx, iky) = czero;
          fft_h_kyky_(ikx, iky) = czero;
          fft_h_kxky_(ikx, iky) = czero;
        } else {
          complex dx  = - hiok * kx;
          complex dy  = - hiok * ky;
          complex hkxkx = hok * kx2;
          complex hkyky = hok * ky2;
          complex hkxky = hok * kx * ky;

          if (ikx == nx_ / 2)
          {
            dx = czero;
            hkxkx = complex(hkxkx.real(), 0.0);
            hkxky = czero;
          }
          if (iky == ny_ / 2)
          {
            dy = czero;
            hkyky = complex(hkyky.real(), 0.0);
            hkxky = czero;
          }

          fft_sx_(ikx, iky)     = dx;
          fft_sy_(ikx, iky)     = dy;
          fft_h_kxkx_(ikx, iky) = hkxkx;
          fft_h_kyky_(ikx, iky) = hkyky;
          fft_h_kxky_(ikx, iky) = hkxky;
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::InitFFTCoeffStorage()
  {
    // initialise storage for Fourier coefficients
    fft_h_      = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_h_ikx_  = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_h_iky_  = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_sx_     = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_sy_     = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_h_kxkx_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_h_kyky_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_h_kxky_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::InitWaveNumbers()
  {
    kx_fft_  = Eigen::ArrayXd::Zero(nx_);
    ky_fft_  = Eigen::ArrayXd::Zero(ny_);
    kx_math_ = Eigen::ArrayXd::Zero(nx_);
    ky_math_ = Eigen::ArrayXd::Zero(ny_);

    // wavenumbers in fft and math ordering
    for (Index ikx = 0; ikx < nx_; ++ikx)
    {
      double kx = (ikx - nx_/2) * kx_f_;
      kx_math_(ikx) = kx;
      kx_fft_((ikx + nx_/2) % nx_) = kx;
    }

    for (Index iky = 0; iky < ny_; ++iky)
    {
      double ky = (iky - ny_/2) * ky_f_;
      ky_math_(iky) = ky;
      ky_fft_((iky + ny_/2) % ny_) = ky;
    }
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::CreateFFTWPlans()
  {
    // elevation
    fft_out0_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_out1_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_out2_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);

    // xy-displacements
    fft_out3_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_out4_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_out5_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_out6_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);
    fft_out7_ = Eigen::ArrayXXcdRowMajor::Zero(nx_, ny_);

    // elevation
    fft_plan0_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_h_.data()),
        reinterpret_cast<fftw_complex*>(fft_out0_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan1_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_h_ikx_.data()),
        reinterpret_cast<fftw_complex*>(fft_out1_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan2_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_h_iky_.data()),
        reinterpret_cast<fftw_complex*>(fft_out2_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);

    // xy-displacements
    fft_plan3_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_sx_.data()),
        reinterpret_cast<fftw_complex*>(fft_out3_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan4_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_sy_.data()),
        reinterpret_cast<fftw_complex*>(fft_out4_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan5_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_h_kxkx_.data()),
        reinterpret_cast<fftw_complex*>(fft_out5_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan6_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_h_kyky_.data()),
        reinterpret_cast<fftw_complex*>(fft_out6_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
    fft_plan7_ = fftw_plan_dft_2d(nx_, ny_,
        reinterpret_cast<fftw_complex*>(fft_h_kxky_.data()),
        reinterpret_cast<fftw_complex*>(fft_out7_.data()),
        FFTW_BACKWARD, FFTW_ESTIMATE);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::Impl::DestroyFFTWPlans()
  {
    fftw_destroy_plan(fft_plan0_);
    fftw_destroy_plan(fft_plan1_);
    fftw_destroy_plan(fft_plan2_);
    fftw_destroy_plan(fft_plan3_);
    fftw_destroy_plan(fft_plan4_);
    fftw_destroy_plan(fft_plan5_);
    fftw_destroy_plan(fft_plan6_);
    fftw_destroy_plan(fft_plan7_);
  }

  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  double LinearRandomFFTWaveSimulationRef::Impl::ECKVOmniDirectionalSpectrum(
      double k, double u10, double cap_omega_c, double gravity)
  {
    if (std::abs(k) < 1.0E-8 || std::abs(u10) < 1.0E-8)
    {
      return 0.0;
    }

    // double alpha = 0.0081;
    // double beta = 1.25;
    double g = gravity;
    double Cd_10N = 0.00144;
    double u_star = sqrt(Cd_10N) * u10;
    // double ao = 0.1733;
    // double ap = 4.0;
    double km = 370.0;
    double cm = 0.23;
    // double am = 0.13 * u_star / cm;

    double gamma = 1.7;
    if (cap_omega_c < 1.0)
    {
      gamma = 1.7;
    } else {
      gamma = 1.7 + 6.0 * log10(cap_omega_c);
    }

    double sigma = 0.08 * (1.0 + 4.0 * pow(cap_omega_c, -3.0));
    double alpha_p = 0.006 * pow(cap_omega_c, 0.55);

    double alpha_m;
    if (u_star <= cm)
    {
      alpha_m = 0.01 * (1.0 + log(u_star / cm));
    } else {
      alpha_m = 0.01 * (1.0 + 3.0 * log(u_star / cm));
    }

    double ko = g / u10 / u10;
    double kp = ko * cap_omega_c * cap_omega_c;

    double cp = sqrt(g / kp);
    double c  = sqrt((g / k) * (1.0 + pow(k / km, 2.0)));

    double L_PM = exp(-1.25 * pow(kp / k, 2.0));

    double Gamma = exp(-1.0/(2.0 * pow(sigma, 2.0))
        * pow(sqrt(k / kp) - 1.0, 2.0));

    double J_p = pow(gamma, Gamma);

    double F_p = L_PM * J_p * exp(-0.3162 * cap_omega_c
        * (sqrt(k / kp) - 1.0));

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
  double LinearRandomFFTWaveSimulationRef::Impl::ECKVSpreadingFunction(
      double k, double phi, double u10, double cap_omega_c, double gravity)
  {
    double g = gravity;
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
  double LinearRandomFFTWaveSimulationRef::Impl::Cos2sSpreadingFunction(
      double s, double phi, double /*u10*/,
      double /*cap_omega_c*/, double /*gravity*/)
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
  LinearRandomFFTWaveSimulationRef::~LinearRandomFFTWaveSimulationRef()
  {
  }

  //////////////////////////////////////////////////
  LinearRandomFFTWaveSimulationRef::LinearRandomFFTWaveSimulationRef(
    double lx, double ly, Index nx, Index ny) :
    impl_(new LinearRandomFFTWaveSimulationRef::Impl(lx, ly, nx, ny))
  {
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::SetLambda(double value)
  {
    impl_->SetLambda(value);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::SetWindVelocity(double ux, double uy)
  {
    impl_->SetWindVelocity(ux, uy);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::SetSteepness(double value)
  {
    impl_->SetSteepness(value);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::SetTime(double value)
  {
    impl_->SetTime(value);
  }

  //////////////////////////////////////////////////
  Index LinearRandomFFTWaveSimulationRef::SizeX() const
  {
    return impl_->nx_;
  }

  //////////////////////////////////////////////////
  Index LinearRandomFFTWaveSimulationRef::SizeY() const
  {
    return impl_->ny_;
  }

  //////////////////////////////////////////////////
  Index LinearRandomFFTWaveSimulationRef::SizeZ() const
  {
    return 0;
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::ElevationAt(
    Eigen::Ref<Eigen::ArrayXXd> h) const
  {
    impl_->ElevationAt(h);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::ElevationDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy) const
  {
    impl_->ElevationDerivAt(dhdx, dhdy);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::DisplacementAt(
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy) const
  {
    impl_->DisplacementAt(sx, sy);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::DisplacementDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
  {
    impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
  }

  //////////////////////////////////////////////////
  void LinearRandomFFTWaveSimulationRef::DisplacementAndDerivAt(
    Eigen::Ref<Eigen::ArrayXXd> h,
    Eigen::Ref<Eigen::ArrayXXd> sx,
    Eigen::Ref<Eigen::ArrayXXd> sy,
    Eigen::Ref<Eigen::ArrayXXd> dhdx,
    Eigen::Ref<Eigen::ArrayXXd> dhdy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdx,
    Eigen::Ref<Eigen::ArrayXXd> dsydy,
    Eigen::Ref<Eigen::ArrayXXd> dsxdy) const
  {
    impl_->ElevationAt(h);
    impl_->ElevationDerivAt(dhdx, dhdy);
    impl_->DisplacementAt(sx, sy);
    impl_->DisplacementDerivAt(dsxdx, dsydy, dsxdy);
  }
}  // namespace waves
}  // namespace gz
