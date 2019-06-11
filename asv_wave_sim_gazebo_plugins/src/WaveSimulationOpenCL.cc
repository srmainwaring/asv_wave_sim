// Copyright (C) 2019  Rhys Mainwaring
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

#include "asv_wave_sim_gazebo_plugins/WaveSimulationOpenCL.hh"
#include "asv_wave_sim_gazebo_plugins/WaveSpectrum.hh"

#include <clFFT.h>

#include <complex>
#include <random>
#include <vector>

namespace asv
{

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationOpenCLImpl

  typedef std::complex<float> complex;

  class WaveSimulationOpenCLImpl
  {
    public: ~WaveSimulationOpenCLImpl();

    public: WaveSimulationOpenCLImpl(int _N, double _L);

    public: void SetWindVelocity(double _ux, double _uy);

    public: void SetTime(double _time);

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

    // OpenCL parameters
    cl_platform_id mPlatform;
    cl_device_id mDevice;
    cl_context_properties mProps[3];
    cl_context mContext;
    cl_command_queue mCmdQueue;
    cl_mem mBufIn0, mBufIn1, mBufIn2;
    cl_mem mBufIn3, mBufIn4, mBufIn5, mBufIn6, mBufIn7;

    char mPlatformName[128];
    char mDeviceName[128];

    // FFT library related declarations
    clfftDim mDim;
    size_t mClLengths[2];

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

    // Carry out FFT in-place
    float* mIn0;
    float* mIn1;
    float* mIn2;
    float* mIn3;
    float* mIn4;
    float* mIn5;
    float* mIn6;
    float* mIn7;

    clfftPlanHandle mFFTPlan0, mFFTPlan1, mFFTPlan2;
    clfftPlanHandle mFFTPlan3, mFFTPlan4, mFFTPlan5, mFFTPlan6, mFFTPlan7;
  };

  WaveSimulationOpenCLImpl::~WaveSimulationOpenCLImpl()
  {
    free(mIn0);
    free(mIn1);
    free(mIn2);
    free(mIn3);
    free(mIn4);
    free(mIn5);
    free(mIn6);
    free(mIn7);

    cl_int err = 0;
    err = clfftDestroyPlan(&mFFTPlan0);
    err = clfftDestroyPlan(&mFFTPlan1);
    err = clfftDestroyPlan(&mFFTPlan2);
    err = clfftDestroyPlan(&mFFTPlan3);
    err = clfftDestroyPlan(&mFFTPlan4);
    err = clfftDestroyPlan(&mFFTPlan5);
    err = clfftDestroyPlan(&mFFTPlan6);
    err = clfftDestroyPlan(&mFFTPlan7);

    // Release clFFT library.
    clfftTeardown();

    // Release OpenCL working objects.
    clReleaseCommandQueue(mCmdQueue);
    clReleaseContext(mContext);
  }

  WaveSimulationOpenCLImpl::WaveSimulationOpenCLImpl(int _N, double _L) :
    mN(_N),
    mN2(_N * _N),
    mNOver2(_N / 2),
    mL(_L),
    mUx(0.0),
    mUy(0.0),
    mScale(1 * _N),
    // @TODO Add accessors to control displacements...
    mLambda(0.9),
    mPlatform(0),
    mDevice(0),
    mContext(0),
    mCmdQueue(0),
    mDim(CLFFT_2D)
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Constructing WaveSimulationCLFFT...");

    // OpenCL initialisation
    mProps[0] = CL_CONTEXT_PLATFORM;
    mProps[1] = 0;
    mProps[2] = 0;
    mClLengths[0] = mN;
    mClLengths[1] = mN;

    // Setup OpenCL environment.
    // logManager.logMessage("Setup OpenCL environment.");
    cl_int err = clGetPlatformIDs( 1, &mPlatform, NULL );

    size_t ret_param_size = 0;
    err = clGetPlatformInfo(mPlatform, CL_PLATFORM_NAME, sizeof(mPlatformName), mPlatformName, &ret_param_size);
    // logManager.logMessage(Ogre::String("Platform found: ") + mPlatformName);

    err = clGetDeviceIDs( mPlatform, CL_DEVICE_TYPE_DEFAULT, 1, &mDevice, NULL );
    err = clGetDeviceInfo(mDevice, CL_DEVICE_NAME, sizeof(mDeviceName), mDeviceName, &ret_param_size);
    // logManager.logMessage(Ogre::String("Device found on the above platform: ") + mDeviceName);

    mProps[1] = (cl_context_properties)mPlatform;
    mContext  = clCreateContext( mProps, 1, &mDevice, NULL, NULL, &err );
    mCmdQueue = clCreateCommandQueue( mContext, mDevice, 0, &err );

    // Setup clFFT
    clfftSetupData fftSetup;
    err = clfftInitSetupData(&fftSetup);
    err = clfftSetup(&fftSetup);

    // logManager.logMessage("Computing base amplitudes.");
    ComputeBaseAmplitudes();

    // FFT 2D.
    // logManager.logMessage("Creating FFTW plans.");

    // For height
    mIn0  = (float*)malloc(2 * mN2 * sizeof(float));
    mIn1  = (float*)malloc(2 * mN2 * sizeof(float));
    mIn2  = (float*)malloc(2 * mN2 * sizeof(float));

    // For xy-displacements
    mIn3  = (float*)malloc(2 * mN2 * sizeof(float));
    mIn4  = (float*)malloc(2 * mN2 * sizeof(float));
    mIn5  = (float*)malloc(2 * mN2 * sizeof(float));
    mIn6  = (float*)malloc(2 * mN2 * sizeof(float));
    mIn7  = (float*)malloc(2 * mN2 * sizeof(float));

    // For height
    err = clfftCreateDefaultPlan(&mFFTPlan0, mContext, mDim, mClLengths);
    err = clfftSetPlanPrecision(mFFTPlan0, CLFFT_SINGLE);
    err = clfftSetLayout(mFFTPlan0, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED);
    err = clfftSetResultLocation(mFFTPlan0, CLFFT_INPLACE);
    err = clfftBakePlan(mFFTPlan0, 1, &mCmdQueue, NULL, NULL);

    err = clfftCopyPlan(&mFFTPlan1, mContext, mFFTPlan0);
    err = clfftCopyPlan(&mFFTPlan2, mContext, mFFTPlan0);

    // For xy-displacements
    err = clfftCopyPlan(&mFFTPlan3, mContext, mFFTPlan0);
    err = clfftCopyPlan(&mFFTPlan4, mContext, mFFTPlan0);
    err = clfftCopyPlan(&mFFTPlan5, mContext, mFFTPlan0);
    err = clfftCopyPlan(&mFFTPlan6, mContext, mFFTPlan0);
    err = clfftCopyPlan(&mFFTPlan7, mContext, mFFTPlan0);

    // logManager.logMessage("Done constructing WaveSimulationCLFFT.");
  }

  void WaveSimulationOpenCLImpl::SetWindVelocity(double _ux, double _uy)
  {
    // Update wind velocity and recompute base amplitudes.
    mUx = _ux;
    mUy = _uy;
    ComputeBaseAmplitudes();
  }

  void WaveSimulationOpenCLImpl::SetTime(double _time)
  {
    ComputeCurrentAmplitudes(_time);
  }

  void WaveSimulationOpenCLImpl::ComputeHeights(
    std::vector<double>& _heights)
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Computing heights...");

    // Populate input array
    // logManager.logMessage("Populate input array.");
    for (size_t i=0; i<mN2; ++i)
    {
      mIn0[2*i]   = mH[i].real();
      mIn0[2*i+1] = mH[i].imag();
    }

    // Run the FFT
    // logManager.logMessage("Execute FFT.");

    // Prepare OpenCL memory objects and place data inside them.
    cl_int err = 0;
    size_t buffer_size  = 2 * mN2 * sizeof(float);
    mBufIn0 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn0, CL_TRUE, 0, buffer_size, mIn0, 0, NULL, NULL );

    // Execute the plan.
    err = clfftEnqueueTransform(mFFTPlan0, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn0, NULL, NULL);

    // Wait for calculations to be finished.
    err = clFinish(mCmdQueue);

    // Fetch results of calculations.
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn0, CL_TRUE, 0, buffer_size, mIn0, 0, NULL, NULL );

    // Resize output if necessary
    if (_heights.size() != mN2)
    {
      _heights.resize(mN2, 0.0);
    }

    // logManager.logMessage("Populate output array.");
    for (size_t i=0; i<mN2; ++i)
    {
      _heights[i] = mIn0[2*i] * mScale;
    }

    // Release OpenCL memory objects
    clReleaseMemObject(mBufIn0);

    // logManager.logMessage("Done computing heights.");
  }

  void WaveSimulationOpenCLImpl::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Computing derivatives...");

    // Populate input array
    // logManager.logMessage("Populate input array.");
    for (size_t i=0; i<mN2; ++i)
    {
      mIn1[2*i]   = mHikx[i].real();
      mIn1[2*i+1] = mHikx[i].imag();

      mIn2[2*i]   = mHiky[i].real();
      mIn2[2*i+1] = mHiky[i].imag();
    }

    // Run the FFTs
    // logManager.logMessage("Execute FFT.");
    cl_int err = 0;
    size_t buffer_size  = 2 * mN2 * sizeof(float);

    mBufIn1 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn1, CL_TRUE, 0, buffer_size, mIn1, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan1, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn1, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn1, CL_TRUE, 0, buffer_size, mIn1, 0, NULL, NULL );

    mBufIn2 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn2, CL_TRUE, 0, buffer_size, mIn2, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan2, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn2, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn2, CL_TRUE, 0, buffer_size, mIn2, 0, NULL, NULL );

    // Resize output if necessary
    if (_dhdx.size() != mN2)
    {
      _dhdx.resize(mN2, 0.0);
    }
    if (_dhdy.size() != mN2)
    {
      _dhdy.resize(mN2, 0.0);
    }

    // logManager.logMessage("Populate output array.");
    for (size_t i=0; i<mN2; ++i)
    {
      _dhdx[i] = mIn1[2*i] * mScale;
      _dhdy[i] = mIn2[2*i] * mScale;
    }

    // Release OpenCL memory objects
    clReleaseMemObject(mBufIn1);
    clReleaseMemObject(mBufIn2);

    // logManager.logMessage("Done computing derivatives.");  
  }

  void WaveSimulationOpenCLImpl::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Computing displacements...");

    // Populate input array
    // logManager.logMessage("Populate input array.");
    for (size_t i=0; i<mN2; ++i)
    {
      mIn3[2*i]   = mDx[i].real();
      mIn3[2*i+1] = mDx[i].imag();

      mIn4[2*i]   = mDy[i].real();
      mIn4[2*i+1] = mDy[i].imag();
    }

    // Run the FFTs
    // logManager.logMessage("Execute FFT.");
    cl_int err = 0;
    size_t buffer_size  = 2 * mN2 * sizeof(float);

    mBufIn3 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn3, CL_TRUE, 0, buffer_size, mIn3, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan3, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn3, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn3, CL_TRUE, 0, buffer_size, mIn3, 0, NULL, NULL );

    mBufIn4 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn4, CL_TRUE, 0, buffer_size, mIn4, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan4, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn4, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn4, CL_TRUE, 0, buffer_size, mIn4, 0, NULL, NULL );

    // Resize output if necessary
    if (_sx.size() != mN2)
    {
      _sx.resize(mN2, 0.0);
    }
    if (_sy.size() != mN2)
    {
      _sy.resize(mN2, 0.0);
    }

    // logManager.logMessage("Populate output array.");
    for (size_t i=0; i<mN2; ++i)
    {
      _sx[i] = - mIn3[2*i] * mScale * mLambda;
      _sy[i] = - mIn4[2*i] * mScale * mLambda;
    }

    // Release OpenCL memory objects
    clReleaseMemObject(mBufIn3);
    clReleaseMemObject(mBufIn4);

    // logManager.logMessage("Done computing displacements.");  
  }

  void WaveSimulationOpenCLImpl::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Computing derivatives...");

    // Populate input array
    // logManager.logMessage("Populate input array.");
    for (size_t i=0; i<mN2; ++i)
    {
      mIn5[2*i]   = mHkxkx[i].real();
      mIn5[2*i+1] = mHkxkx[i].imag();

      mIn6[2*i]   = mHkyky[i].real();
      mIn6[2*i+1] = mHkyky[i].imag();

      mIn7[2*i]   = mHkxky[i].real();
      mIn7[2*i+1] = mHkxky[i].imag();
    }

    // Run the FFTs
    // logManager.logMessage("Execute FFT.");
    cl_int err = 0;
    size_t buffer_size  = 2 * mN2 * sizeof(float);

    mBufIn5 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn5, CL_TRUE, 0, buffer_size, mIn5, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan5, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn5, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn5, CL_TRUE, 0, buffer_size, mIn5, 0, NULL, NULL );

    mBufIn6 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn6, CL_TRUE, 0, buffer_size, mIn6, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan6, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn6, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn6, CL_TRUE, 0, buffer_size, mIn6, 0, NULL, NULL );

    mBufIn7 = clCreateBuffer( mContext, CL_MEM_READ_WRITE, buffer_size, NULL, &err );
    err = clEnqueueWriteBuffer( mCmdQueue, mBufIn7, CL_TRUE, 0, buffer_size, mIn7, 0, NULL, NULL );
    err = clfftEnqueueTransform(mFFTPlan7, CLFFT_BACKWARD, 1, &mCmdQueue, 0, NULL, NULL, &mBufIn7, NULL, NULL);
    err = clFinish(mCmdQueue);
    err = clEnqueueReadBuffer( mCmdQueue, mBufIn7, CL_TRUE, 0, buffer_size, mIn7, 0, NULL, NULL );

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

    // logManager.logMessage("Populate output array.");
    for (size_t i=0; i<mN2; ++i)
    {
      _dsxdx[i] = - mIn5[2*i] * mScale * mLambda;
      _dsydy[i] = - mIn6[2*i] * mScale * mLambda;
      _dsxdy[i] = - mIn7[2*i] * mScale * mLambda;
    }

    // Release OpenCL memory objects
    clReleaseMemObject(mBufIn5);
    clReleaseMemObject(mBufIn6);
    clReleaseMemObject(mBufIn7);

    // logManager.logMessage("Done computing derivatives.");  
  }

  void WaveSimulationOpenCLImpl::ComputeBaseAmplitudes()
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Computing base amplitudes...");

    // 1D axes
    // logManager.logMessage("Reserve 1D workspace");
    mX.resize(mN, 0.0);
    mK.resize(mN, 0.0);
    mOmega.resize(mN, 0.0);

    // 2D grids
    // logManager.logMessage("Reserve 2D workspace");
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
    // logManager.logMessage("Populate wavenumbers.");
    for (size_t i=1; i<mN/2; ++i)
    {
      mK[i] = i * 2.0 * M_PI/ mL;
      mK[mN - i] = mK[i];
    }
    for (size_t i=0; i<mN; ++i)
    {
      mK[i] = i * 2.0 * M_PI/ mL;
    }
    // logManager.logMessage("Populate radian frequency.");
    for (size_t i=0; i<mN; ++i)
    {
      mX[i] = i * mL / mN;
      mOmega[i] = WaveSpectrum::Dispersion(mK[i]);
    }    

    // Compute Gaussian random variables.
    // logManager.logMessage("Compute Gaussian random variables.");
    auto seed = std::default_random_engine::default_seed;
    std::default_random_engine generator(seed);
    // logManager.logMessage(Ogre::String("Seed: ") + Ogre::StringConverter::toString(seed));

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
    // logManager.logMessage("Compute Fourier amplitudes.");
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

    // logManager.logMessage("Done computing base amplitudes.");
  }

  void WaveSimulationOpenCLImpl::ComputeCurrentAmplitudes(double _time)
  {
    // Logging
    // auto& logManager = Ogre::LogManager::getSingleton();
    // logManager.logMessage("Computing current amplitudes...");

    float kx = mK[0];
    float ky = mK[0];
    float k  = std::sqrt(kx*kx + ky*ky);
    float omega = WaveSpectrum::Dispersion(k);
    complex phase = std::exp(complex(0.0, -omega * _time));

    complex rm1(0.0, 1.0);

    mH[0] = mH0[0] * phase;
    for (size_t ix=1; ix<mN/2; ++ix)
    {
      for (size_t iy=1; iy<mN/2; ++iy)
      {
        // Wavenumber index and conjugate index 
        size_t idx = ix * mN + iy;
        size_t cdx = (mN - ix) * mN + (mN - iy);

        // Time dependence
        kx = mK[ix];
        ky = mK[iy];
        k  = std::sqrt(kx*kx + ky*ky);
        omega = WaveSpectrum::Dispersion(k);
        phase = std::exp(complex(0.0, -omega * _time));

        // Height
        mH[idx] = mH0[idx] * phase;
        mH[cdx] = std::conj(mH[idx]);

        // Height derivative
        mHikx[idx] = mH[idx] * kx * rm1;
        mHiky[idx] = mH[idx] * ky * rm1;
        mHikx[cdx] = std::conj(mHikx[idx]);
        mHiky[cdx] = std::conj(mHiky[idx]);

        // Displacement and derivatives
        if (std::abs(k) < 1.0E-8)
        {
          mDx[idx] = - complex(0.0, 0.0);
          mDy[idx] = - complex(0.0, 0.0);
          mHkxkx[idx] = complex(0.0, 0.0);
          mHkyky[idx] = complex(0.0, 0.0);
          mHkxky[idx] = complex(0.0, 0.0);
        }
        else
        {
          mDx[idx] = - mH[idx] * kx * rm1 / k;
          mDy[idx] = - mH[idx] * ky * rm1 / k;
          mHkxkx[idx] = mH[idx] * kx * kx / k;
          mHkyky[idx] = mH[idx] * ky * ky / k;
          mHkxky[idx] = mH[idx] * kx * ky / k;
        }
        mDx[cdx] = std::conj(mDx[idx]);
        mDy[cdx] = std::conj(mDy[idx]);
        mHkxkx[cdx] = std::conj(mHkxkx[idx]);
        mHkyky[cdx] = std::conj(mHkyky[idx]);
        mHkxky[cdx] = std::conj(mHkxky[idx]);
      }
    }
    // logManager.logMessage("Done computing current amplitudes.");
  }

  complex WaveSimulationOpenCLImpl::Htilde0(double _k, double _kx, double _ky, double _u, double _ux, double _uy, complex _gz)
  {
    float rp = std::sqrt(0.5 * WaveSpectrum::Spectrum(_k, _kx, _ky, _u, _ux, _uy));
    complex h = _gz * rp;
    return h;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // WaveSimulationOpenCL

  WaveSimulationOpenCL::~WaveSimulationOpenCL()
  {
  }

  WaveSimulationOpenCL::WaveSimulationOpenCL(int _N, double _L) :
    impl(new WaveSimulationOpenCLImpl(_N, _L))
  {
  }

  void WaveSimulationOpenCL::SetWindVelocity(double _ux, double _uy)
  {
    impl->SetWindVelocity(_ux, _uy);
  }

  void WaveSimulationOpenCL::SetTime(double _time)
  {
    impl->SetTime(_time);
  }

  void WaveSimulationOpenCL::ComputeHeights(
    std::vector<double>& _h)
  {
    impl->ComputeHeights(_h);    
  }

  void WaveSimulationOpenCL::ComputeHeightDerivatives(
    std::vector<double>& _dhdx,
    std::vector<double>& _dhdy)
  {
    impl->ComputeHeightDerivatives(_dhdx, _dhdy);  
  }

  void WaveSimulationOpenCL::ComputeDisplacements(
    std::vector<double>& _sx,
    std::vector<double>& _sy)
  {
    impl->ComputeDisplacements(_sx, _sy);    
  }

  void WaveSimulationOpenCL::ComputeDisplacementDerivatives(
    std::vector<double>& _dsxdx,
    std::vector<double>& _dsydy,
    std::vector<double>& _dsxdy)
  {
    impl->ComputeDisplacementDerivatives(_dsxdx, _dsydy, _dsxdy);      
  }

  void WaveSimulationOpenCL::ComputeDisplacementsAndDerivatives(
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
