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

#include "gz/waves/WavefieldSampler.hh"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include "gz/waves/CGALTypes.hh"
#include "gz/waves/Grid.hh"
#include "gz/waves/Utilities.hh"
#include "gz/waves/Wavefield.hh"
#include "gz/waves/WaveParameters.hh"

namespace gz
{
namespace waves
{
//////////////////////////////////////////////////
/// \internal
/// \brief Private data for the WavefieldSampler.
class WavefieldSamplerPrivate
{
 public:
  /// \brief The wavefield grid.
  std::shared_ptr<const Wavefield> wavefield_;

  /// \brief A sample of the wavefield located at the initial pose.
  std::shared_ptr<const Grid> init_patch_;

  /// \brief A sample of the wavefield. This copy is updated.
  std::shared_ptr<Grid> patch_;
};

//////////////////////////////////////////////////
WavefieldSampler::~WavefieldSampler()
{
}

//////////////////////////////////////////////////
WavefieldSampler::WavefieldSampler(
  std::shared_ptr<const Wavefield> wavefield,
  std::shared_ptr<const Grid> patch
) : impl_(new WavefieldSamplerPrivate())
{
  impl_->wavefield_ = wavefield;
  impl_->init_patch_ = patch;
  impl_->patch_.reset(new Grid(*patch));
}

//////////////////////////////////////////////////
std::shared_ptr<const Grid> WavefieldSampler::GetWaterPatch() const
{
  return impl_->patch_;
}

//////////////////////////////////////////////////
void WavefieldSampler::ApplyPose(const gz::math::Pose3d& pose)
{
  // @TODO_FRAGILE - Move to Grid as changing internal state
  // Apply pose to center
  const cgal::Point3& c0 = impl_->init_patch_->GetCenter();
  cgal::Point3 c1(c0.x() + pose.Pos().X(), c0.y() + pose.Pos().Y(), c0.z());
  impl_->patch_->SetCenter(c1);

  // Iterate over vertices
  auto& source = *impl_->init_patch_->GetMesh();
  auto& target = *impl_->patch_->GetMesh();
  for (
    auto&& it = std::make_pair(std::begin(source.vertices()),
        std::begin(target.vertices()));
    it.first != std::end(source.vertices()) &&
        it.second != std::end(target.vertices());
    ++it.first, ++it.second)
  {
    const auto& v0 = *it.first;
    const auto& v1 = *it.second;
    const cgal::Point3& p0 = source.point(v0);

    // Transformation: slide the patch in the xy - plane only
    cgal::Point3 p1(p0.x() + pose.Pos().X(), p0.y() + pose.Pos().Y(), p0.z());
    target.point(v1) = p1;
  }
}

//////////////////////////////////////////////////
void WavefieldSampler::UpdatePatch()
{
  // Update the water patch Mesh
  // gzmsg << "Update water patch..." << std::endl;
  const auto& target = impl_->patch_->GetMesh();
  for (
    auto&& vb = std::begin(target->vertices());
    vb != std::end(target->vertices());
    ++vb
  )
  {
    const auto& vertex = *vb;
    const auto& p0 = target->point(vertex);
    double height = 0.0;
    impl_->wavefield_->Height(
        Eigen::Vector3d(p0.x(), p0.y(), p0.z()), height);
    cgal::Point3 p1(p0.x(), p0.y(), height);
    target->point(vertex) = p1;
    // gzmsg << target->point(vertex) << std::endl;
  }
}

//////////////////////////////////////////////////
double WavefieldSampler::ComputeDepth(const cgal::Point3& point) const
{
  auto& grid = *impl_->patch_;
  return WavefieldSampler::ComputeDepth(grid, point);

  // @TODO_EXPERIMENTAL - direct calculation (currently static only)
  // auto& waveParams = *impl_->wavefield_->GetParameters();
  // return WavefieldSampler::ComputeDepthDirectly(waveParams, point, 0.0);
}

//////////////////////////////////////////////////
double WavefieldSampler::ComputeDepth(
  const Grid& patch,
  const cgal::Point3& point
)
{
  // Calculate the depth
  cgal::Direction3 direction(0, 0, 1);
  cgal::Point3 wave_point = CGAL::ORIGIN;
  std::array<Index, 3> index;
  bool is_found = GridTools::FindIntersectionIndex(
    patch, point.x(), point.y(), index);
  if (!is_found)
  {
    // @DEBUG_INFO
    gzerr << "point:  " << point << std::endl;
    // patch.DebugPrint();
    gzerr << "Water patch is too small" << std::endl;
    return 0;
  }
  is_found = GridTools::FindIntersectionGrid(
    patch, point, direction, index, wave_point);
  if (!is_found)
  {
    // @DEBUG_INFO
    gzerr << "point:  " << point << std::endl;
    // patch.DebugPrint();
    gzerr << "Water patch is too small" << std::endl;
    return 0;
  }
  double h = wave_point.z() - point.z();
  return h;
}

//////////////////////////////////////////////////
double WavefieldSampler::ComputeDepthDirectly(
  const WaveParameters& wave_params,
  const cgal::Point3& point,
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
      const std::vector<gz::math::Vector2d>& _dir) :
      a(_a), k(_k), omega(_omega), phi(_phi), q(_q), dir(_dir) {}

    const std::vector<double>& a;
    const std::vector<double>& k;
    const std::vector<double>& omega;
    const std::vector<double>& phi;
    const std::vector<double>& q;
    const std::vector<gz::math::Vector2d>& dir;
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
    Index n = wp.a.size();
    for (Index i=0; i < n; ++i)
    {
      const double dx = wp.dir[i].X();
      const double dy = wp.dir[i].Y();
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

  // Simple multi-variate Newton solver - this version returns the
  // z-component of the wave field at the desired point p.
  auto solver = [=](auto& fdfunc, auto x0, auto p, auto t,
      auto& wp, auto tol, auto nmax)
  {
    Index n = 0;
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
    wave_params.Amplitude_V(),
    wave_params.Wavenumber_V(),
    wave_params.AngularFrequency_V(),
    wave_params.Phase_V(),
    wave_params.Steepness_V(),
    wave_params.Direction_V());

  // Tolerances etc.
  const double tol = 1.0E-10;
  const double nmax = 30;

  // Use the target point as the initial guess (this is within
  // sum{amplitudes} of the solution)
  Eigen::Vector2d p2(point.x(), point.y());
  const double pz = solver(wave_fdf, p2, p2, time, wp, tol, nmax);
  const double h = pz - point.z();
  return h;
}

}  // namespace waves
}  // namespace gz
