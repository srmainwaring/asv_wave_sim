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

#include "asv_wave_sim_gazebo_plugins/WavefieldSampler.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WaveParameters.hh"

#include <Eigen/Dense>

#include <gazebo/gazebo.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <array>
#include <iostream>
#include <cmath>
#include <string>

namespace asv 
{
///////////////////////////////////////////////////////////////////////////////    
// WavefieldSamplerPrivate

  /// \internal
  /// \brief Private data for the WavefieldSampler.
  class WavefieldSamplerPrivate
  {
    /// \brief The wavefield grid.
    public: std::shared_ptr<const Wavefield> wavefield;

    /// \brief A sample of the wavefield. This copy is located at the initial pose. 
    public: std::shared_ptr<const Grid> initWaterPatch;

    /// \brief A sample of the wavefield. This copy is updated.
    public: std::shared_ptr<Grid> waterPatch;    
  };

///////////////////////////////////////////////////////////////////////////////    
// WavefieldSampler

  WavefieldSampler::~WavefieldSampler()
  {        
  }

  WavefieldSampler::WavefieldSampler(
    std::shared_ptr<const Wavefield> _wavefield,
    std::shared_ptr<const Grid> _waterPatch
  ) : data(new WavefieldSamplerPrivate())
  {
    this->data->wavefield = _wavefield;
    this->data->initWaterPatch = _waterPatch;
    this->data->waterPatch.reset(new Grid(*_waterPatch));
  }

  std::shared_ptr<const Grid> WavefieldSampler::GetWaterPatch() const
  {
    return this->data->waterPatch;
  }

  void WavefieldSampler::ApplyPose(const ignition::math::Pose3d& _pose)
  {
    // @TODO_FRAGILE - Move to Grid as changing internal state 
    // Apply pose to center
    const Point3& c0 = this->data->initWaterPatch->GetCenter();
    Point3 c1(c0.x() + _pose.Pos().X(), c0.y() + _pose.Pos().Y(), c0.z());
    this->data->waterPatch->SetCenter(c1);

    // Iterate over vertices
    auto& source = *this->data->initWaterPatch->GetMesh();
    auto& target = *this->data->waterPatch->GetMesh();
    for (
      auto&& it = std::make_pair(std::begin(source.vertices()), std::begin(target.vertices()));
      it.first != std::end(source.vertices()) && it.second != std::end(target.vertices());
      ++it.first, ++it.second)
    {
      auto& v0 = *it.first;
      auto& v1 = *it.second;
      const Point3& p0 = source.point(v0);

      // Transformation: slide the patch in the xy - plane only
      Point3 p1(p0.x() + _pose.Pos().X(), p0.y() + _pose.Pos().Y(), p0.z());
      target.point(v1) = p1;
    }
  }

  void WavefieldSampler::UpdatePatch()
  {
#if 0
    // Direction of the line search (i.e. positive z-axis)
    Direction3 direction(0, 0, 1);

    // Update the water patch Mesh
    const auto& target = this->data->waterPatch->GetMesh();
    for (
      auto&& vb = std::begin(target->vertices()); 
      vb != std::end(target->vertices());
      ++vb
    )
    {
      auto& v1 = *vb;

      Point3& origin = target->point(v1);
      Point3 point = CGAL::ORIGIN;

      auto& wavefieldGrid = *this->data->wavefield->GetGrid();
      std::array<size_t, 3> cellIndex = { 0, 0, 0 };
      bool isFound = GridTools::FindIntersectionIndex(
        wavefieldGrid, origin.x(), origin.y(), cellIndex);
      if (!isFound)
      {
        // @DEBUG_INFO
        gzmsg << "origin:   " << origin << std::endl;
        gzerr << "Wavefield is too small" << std::endl;
        return;
      }
      isFound = GridTools::FindIntersectionGrid(
        wavefieldGrid, origin, direction, cellIndex, point);

      if (!isFound)
      {
        // @DEBUG_INFO
        gzmsg << "origin:   " << origin << std::endl;
        gzerr << "Wavefield is too small" << std::endl;
        return;
      }

      target->point(v1) = point;
    }
#else

    // @NOTE: alternative approach using CGAL triangulation.

    // Update the water patch Mesh
    // gzmsg << "Update water patch..." << std::endl;
    const auto& target = this->data->waterPatch->GetMesh();
    for (
      auto&& vb = std::begin(target->vertices()); 
      vb != std::end(target->vertices());
      ++vb
    )
    {
      const auto& vertex = *vb;
      const auto& p0 = target->point(vertex);
      double height = 0.0;
      this->data->wavefield->Height(p0, height);
      Point3 p1(p0.x(), p0.y(), height);
      target->point(vertex) = p1;
      // gzmsg << target->point(vertex) << std::endl;
    }

#endif
  }

  double WavefieldSampler::ComputeDepth(const Point3& _point) const
  {
    auto& grid = *this->data->waterPatch;
    return WavefieldSampler::ComputeDepth(grid, _point);

    // @TODO_EXPERIMENTAL - direct calculation (currently static only)
    // auto& waveParams = *this->data->wavefield->GetParameters();
    // return WavefieldSampler::ComputeDepthDirectly(waveParams, _point, 0.0);
  }
  
  double WavefieldSampler::ComputeDepth(  
    const Grid& _patch,
    const Point3& _point
  )
  {
    // Calculate the depth
    Direction3 direction(0, 0, 1);
    Point3 wavePoint = CGAL::ORIGIN;
    std::array<size_t, 3> index;
    bool isFound = GridTools::FindIntersectionIndex(
      _patch, _point.x(), _point.y(), index);
    if (!isFound)
    {
      // @DEBUG_INFO
      gzerr << "point:  " << _point << std::endl;
      // _patch.DebugPrint();
      gzerr << "Water patch is too small" << std::endl;
      return 0;
    }
    isFound = GridTools::FindIntersectionGrid(
      _patch, _point, direction, index, wavePoint);
    if (!isFound)
    {
      // @DEBUG_INFO
      gzerr << "point:  " << _point << std::endl;
      // _patch.DebugPrint();
      gzerr << "Water patch is too small" << std::endl;
      return 0;
    }
    double h = wavePoint.z() - _point.z();
    return h;
  }

  double WavefieldSampler::ComputeDepthDirectly(  
    const WaveParameters& _waveParams,
    const Point3& _point,
    double time
  )
  {
    // Struture for passing wave parameters to lambdas
    struct WaveParams
    {
      WaveParams(
        const std::vector<double>& _a,
        const std::vector<double>& _k,
        const std::vector<double>& _omega,
        const std::vector<double>& _phi,
        const std::vector<double>& _q,
        const std::vector<Vector2>& _dir) :
        a(_a), k(_k), omega(_omega), phi(_phi), q(_q), dir(_dir) {}

      const std::vector<double>& a;
      const std::vector<double>& k;
      const std::vector<double>& omega;
      const std::vector<double>& phi;
      const std::vector<double>& q;
      const std::vector<Vector2>& dir;
    };

    // Compute the target function and Jacobian. Also calculate pz,
    // the z-componen of the Gerstner wave, which we essentially get for free.
    auto wave_fdf = [=](auto x, auto p, auto t, auto& wp, auto& F, auto& J)
    {
      double pz = 0;
      F(0) = p.x() - x.x();
      F(1) = p.y() - x.y();
      J(0, 0) = -1;
      J(0, 1) =  0;
      J(1, 0) =  0;
      J(1, 1) = -1;
      const size_t n = wp.a.size();
      for (auto&& i=0; i<n; ++i)
      {
        const double dx = wp.dir[i].x();
        const double dy = wp.dir[i].y();
        const double q = wp.q[i];
        const double a = wp.a[i];
        const double k = wp.k[i];
        const double dot = x.x() * dx + x.y() * dy;
        const double theta = k * dot - wp.omega[i] * t;
        const double s = std::sin(theta);
        const double c = std::cos(theta);
        const double qakc = q * a * k * c;
        const double df1x = qakc * dx * dx;
        const double df1y = qakc * dx * dy;
        const double df2x = df1y;
        const double df2y = qakc * dy * dy;
        pz += a * c;
        F(0) += a * dx * s;
        F(1) += a * dy * s;
        J(0, 0) += df1x;
        J(0, 1) += df1y;
        J(1, 0) += df2x;
        J(1, 1) += df2y;
      }
      return pz;
    };

    // Simple multi-variate Newton solver - this version returns the z-component of the
    // wave field at the desired point p.
    auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, auto& wp, auto tol, auto nmax)
    {
      int n = 0;
      double err = 1;
      double pz = 0;
      auto xn = x0;
      Eigen::Vector2d F;
      Eigen::Matrix2d J;
      while (std::abs(err) > tol && n < nmax)
      {
        pz = fdfunc(x0, p, t, wp, F, J);
        xn = x0 - J.inverse() * F;
        x0 = xn;
        err = F.norm();
        n++;
      }
      return pz;
    };

    // Set up parameter references
    WaveParams wp(
      _waveParams.Amplitude_V(),
      _waveParams.Wavenumber_V(),
      _waveParams.AngularFrequency_V(),
      _waveParams.Phase_V(),
      _waveParams.Steepness_V(),
      _waveParams.Direction_V()
    );

    // Tolerances etc.
    const double tol = 1.0E-10;
    const double nmax = 30;

    // Use the target point as the initial guess (this is within sum{amplitudes} of the solution)
    Eigen::Vector2d p2(_point.x(), _point.y());
    const double pz = solver(wave_fdf, p2, p2, time, wp, tol, nmax);
    const double h = pz - _point.z();
    return h;
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv
