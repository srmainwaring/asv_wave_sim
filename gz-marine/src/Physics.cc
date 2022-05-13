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

#include "gz/marine/Physics.hh"
#include "gz/marine/Algorithm.hh"
#include "gz/marine/Convert.hh"
#include "gz/marine/Geometry.hh"
#include "gz/marine/Grid.hh"
#include "gz/marine/PhysicalConstants.hh"
#include "gz/marine/Utilities.hh"
#include "gz/marine/Wavefield.hh"
#include "gz/marine/WavefieldSampler.hh"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/common.hh>

#include <sdf/sdf.hh>

#include <array>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>

namespace ignition
{
namespace marine
{

///////////////////////////////////////////////////////////////////////////////    
// Utilities
  void DebugPrint(const cgal::Triangle& triangle)
  {
    ignmsg << "Vertex[0]:   " << triangle[0] << std::endl;
    ignmsg << "Vertex[1]:   " << triangle[1] << std::endl;
    ignmsg << "Vertex[2]:   " << triangle[2] << std::endl;
    ignmsg << "Normal:      " << Geometry::Normal(triangle) << std::endl;
  }

///////////////////////////////////////////////////////////////////////////////    
// Physics

  cgal::Point3 Physics::CenterOfForce(
    double _fA, double _fB,
    const cgal::Point3& _A,
    const cgal::Point3& _B
  )
  { 
    double div = _fA + _fB;
    if (div != 0)
    {
      double t = _fA / div;
      return _B + (_A - _B) * t;  
    }
    else
    {
      return _B;
    }
  }

  double Physics::DeepWaterDispersionToOmega(double _wavenumber)
  {
    const double g = std::fabs(PhysicalConstants::Gravity());
    return std::sqrt(g * _wavenumber);
  }

  double Physics::DeepWaterDispersionToWavenumber(double _omega)
  {
    const double g = std::fabs(PhysicalConstants::Gravity());
    return _omega * _omega / g; 
  }

  double Physics::ViscousDragCoefficient(double Rn)
  {
    // Set a lower limit on Rn to 1.0E+3 since the 1957 ITTC formula
    // has a pole at Rn = 1.0E+2. For a 10m boat in salt water this 
    // corresponds to a velocity ~ 1.0E-5 
    double r = std::max(1.0E3, Rn);
    double d = std::log10(r) - 2.0;
    double d2 = d*d;
    double CF = 0.075 / d2;
    return CF;
  }

  cgal::Point3 Physics::CenterOfPressureApexUp(
    double _z0,
    const cgal::Point3& _H,
    const cgal::Point3& _M,
    const cgal::Point3& _B
  )
  {
    cgal::Vector3 alt = _B - _H;
    double h = _H.z() - _M.z();
    double tc = 2.0/3.0;    
    double div = 6.0 * _z0 + 4.0 * h;
    if (div != 0)
    {
      tc = (4.0 * _z0 + 3.0 * h) / div;
    }
    return _H + alt * tc;
  }

  cgal::Point3 Physics::CenterOfPressureApexDn(
    double _z0,
    const cgal::Point3& _L,
    const cgal::Point3& _M,
    const cgal::Point3& _B
  )
  {
    cgal::Vector3 alt = _L - _B;
    double h = _M.z() - _L.z();
    double tc = 1.0/3.0;
    double div = 6.0 * _z0 + 2.0 * h;
    if (div != 0)
    {
      tc = (2.0 * _z0 + h) / div;  
    }
    return _B + alt * tc;
  }

  void Physics::BuoyancyForceAtCenterOfPressure(
    double _depthC,
    const cgal::Point3& _C,
    const cgal::Point3& _H,
    const cgal::Point3& _M,
    const cgal::Point3& _L,
    const cgal::Vector3& _normal,
    cgal::Point3& _center, 
    cgal::Vector3& _force
  )
  {
    double fluidDensity = PhysicalConstants::WaterDensity(); // kg m^-3 
    double gravity = PhysicalConstants::Gravity();           // m s^-1
    
    // Split the triangle into upper and lower triangles bisected by a line normal to the z-axis
    cgal::Point3 D = Geometry::HorizontalIntercept(_H, _M, _L);
    cgal::Point3 B = Geometry::MidPoint(_M, D);

    // Initialise to the base midpoint (correct force calcuation for triangles with a horizontal base)    
    double fU=0, fL=0;
    cgal::Point3 CpU = B;   
    cgal::Point3 CpL = B;

    // Upper triangle H > M
    if (_H.z() >= _M.z())
    {
      // Center of pressure
      double z0 = _depthC - (_H.z() - _C.z());
      CpU = CenterOfPressureApexUp(z0, _H, _M, B);
      
      // Force at centroid
      cgal::Point3 CU = Geometry::TriangleCentroid(_H, _M, D);
      double hCU = _depthC + (_C.z() - CU.z());
      double area = Geometry::TriangleArea(_H, _M, D);
      fU = fluidDensity * gravity * area * hCU;    

      // @DEBUG_INFO
      // ignmsg << "_depthC: " << _depthC << std::endl; 
      // ignmsg << "_C:      " << _C << std::endl; 
      // ignmsg << "_H:      " << _H << std::endl; 
      // ignmsg << "_M:      " << _M << std::endl; 
      // ignmsg << "_L:      " << _L << std::endl; 
      // ignmsg << "_normal: " << _normal << std::endl; 
      // ignmsg << "D:       " << D << std::endl; 
      // ignmsg << "B:       " << B << std::endl; 
      // ignmsg << "z0:      " << z0 << std::endl; 
      // ignmsg << "CpU:     " << CpU << std::endl; 
      // ignmsg << "CU:      " << CU << std::endl; 
      // ignmsg << "hCU:     " << hCU << std::endl; 
      // ignmsg << "area:    " << area << std::endl; 
      // ignmsg << "fU:      " << fU << std::endl; 
    }

    // Lower triangle L < M
    if (_M.z() > _L.z())
    { 
      // Center of preseesure
      double z0 = _depthC + (_C.z() - _M.z());
      CpL = CenterOfPressureApexDn(z0, _L, _M, B);
      
      // Force at centroid
      cgal::Point3 CL = Geometry::TriangleCentroid(_L, _M, D);
      double hCL = _depthC + (_C.z() - CL.z());
      double area = Geometry::TriangleArea(_L, _M, D);
      fL = fluidDensity * gravity * area * hCL;

      // @DEBUG_INFO
      // ignmsg << "_depthC: " << _depthC << std::endl; 
      // ignmsg << "_C:      " << _C << std::endl; 
      // ignmsg << "_H:      " << _H << std::endl; 
      // ignmsg << "_M:      " << _M << std::endl; 
      // ignmsg << "_L:      " << _L << std::endl; 
      // ignmsg << "_normal: " << _normal << std::endl; 
      // ignmsg << "D:       " << D << std::endl; 
      // ignmsg << "B:       " << B << std::endl; 
      // ignmsg << "z0:      " << z0 << std::endl; 
      // ignmsg << "CpL:     " << CpL << std::endl; 
      // ignmsg << "CL:      " << CL << std::endl; 
      // ignmsg << "hCL:     " << hCL << std::endl; 
      // ignmsg << "area:    " << area << std::endl;
      // ignmsg << "fL:      " << fL << std::endl; 
    }
        
    // Calculate the force and centre of application
    _force = _normal * (fU + fL);
    _center = CenterOfForce(fU, fL, CpU, CpL);

    // @DEBUG_INFO
    // ignmsg << "_depthC: " << _depthC << std::endl;
    // ignmsg << "_C:      " << _C << std::endl;
    // ignmsg << "_H:      " << _H << std::endl;
    // ignmsg << "_M:      " << _M << std::endl;
    // ignmsg << "_L:      " << _L << std::endl;
    // ignmsg << "_normal: " << _normal << std::endl;
    // ignmsg << "_force:  " << _force << std::endl;
    // ignmsg << "_center: " << _center << std::endl;
  }

  void Physics::BuoyancyForceAtCentroid(
    const WavefieldSampler& _wavefieldSampler,
    const cgal::Triangle& _triangle,
    cgal::Point3& _center,
    cgal::Vector3& _force
  )
  {
    // Physical constants
    double density = PhysicalConstants::WaterDensity();   // kg m^-3 
    double gravity = PhysicalConstants::Gravity();        // m s^-1
    
    // Calculate the triangles centroid
    _center = Geometry::TriangleCentroid(_triangle);

    // Calculate the depth
    double h = _wavefieldSampler.ComputeDepth(_center);

    // Calculate the force
    cgal::Vector3 normal = Geometry::Normal(_triangle);
    double area = Geometry::TriangleArea(_triangle);
    _force = normal * (density * gravity * area * h);
  }

  void Physics::BuoyancyForceAtCenterOfPressure(
    const WavefieldSampler& _wavefieldSampler,
    const cgal::Triangle& _triangle,
    cgal::Point3& _center,
    cgal::Vector3& _force
  )
  {
    // Sort triangle vertices by height.
    std::array<cgal::Point3, 3> v {
      _triangle[0],
      _triangle[1],
      _triangle[2]
    };
    std::array<double, 3> vz { v[0].z(), v[1].z(), v[2].z() };
    auto index = algorithm::sort_indexes(vz); 
    
    cgal::Point3 H = v[index[0]];
    cgal::Point3 M = v[index[1]];
    cgal::Point3 L = v[index[2]];

    // @DEBUG_INFO
    // DebugPrint(_triangle);
    // ignmsg << "vz:          "; for (auto z: vz)    { ignmsg << z << " "; }; ignmsg << std::endl;
    // ignmsg << "index:       "; for (auto i: index) { ignmsg << i << " "; }; ignmsg << std::endl;
    // ignmsg << "H:           " << H << std::endl;
    // ignmsg << "M:           " << M << std::endl;
    // ignmsg << "L:           " << L << std::endl;

    // Calculate the depth at the centroid
    cgal::Point3 C = Geometry::TriangleCentroid(_triangle);

    // Calculate the depth    
    double depthC = _wavefieldSampler.ComputeDepth(C);

    cgal::Vector3 normal = Geometry::Normal(_triangle);

    // Calculate buoyancy
    BuoyancyForceAtCenterOfPressure(depthC, C, H, M, L, normal, _center, _force);
  }

  std::array<double, 3> Physics::ComputeHeightMap(
    const WavefieldSampler& _wavefieldSampler,
    const cgal::Triangle& _triangle
  )
  {
    // Heightmap for the triangle vertices
    cgal::Direction3 direction(0, 0, -1);
    std::array<double, 3> heightMap;

    // Calculate the height above the surface (-depth)
    for (int i=0; i<3; ++i)
    {
      cgal::Point3 vertex = _triangle[i];
      heightMap[i] = -_wavefieldSampler.ComputeDepth(vertex);
    }
    return heightMap;
  }    

///////////////////////////////////////////////////////////////////////////////
// HydrodynamicsParameters

  class HydrodynamicsParametersPrivate
  {
    public: HydrodynamicsParametersPrivate() :
      dampingOn(true),
      cDampL1(1.0E-6),
      cDampL2(1.0E-6),
      cDampR1(1.0E-6),
      cDampR2(1.0E-6),
      viscousDragOn(true),
      pressureDragOn(true),
      cPDrag1(1.0E+2),
      cPDrag2(1.0E+2),
      fPDrag(0.4),
      cSDrag1(1.0E+2),
      cSDrag2(1.0E+2),
      fSDrag(0.4),
      vRDrag(1.0)
    {}

    // Linear and rotational damping
    public: bool dampingOn;

    // Linear drag coefficients
    public: double cDampL1;
    public: double cDampL2;

    // Rotation drag coefficients
    public: double cDampR1;
    public: double cDampR2;

    /// Viscous drag
    public: bool viscousDragOn;

    /// Pressure drag
    public: bool pressureDragOn;

    /// Positive pressure (lift)
    public: double cPDrag1;
    public: double cPDrag2;
    public: double fPDrag;
    /// Negative pressure (suction)
    public: double cSDrag1;
    public: double cSDrag2;
    public: double fSDrag;
    
    // Reference speed for the pressure drag calculation
    public: double vRDrag;

  };

  HydrodynamicsParameters::~HydrodynamicsParameters()
  {
  }

  HydrodynamicsParameters::HydrodynamicsParameters() :
    data(new HydrodynamicsParametersPrivate())
  {
  }

  bool HydrodynamicsParameters::DampingOn() const
  {
    return this->data->dampingOn;
  }

  bool HydrodynamicsParameters::ViscousDragOn() const
  {
    return this->data->viscousDragOn;
  }

  bool HydrodynamicsParameters::PressureDragOn() const
  {
    return this->data->pressureDragOn;
  }

  double HydrodynamicsParameters::CDampL1() const
  {
    return this->data->cDampL1;
  }

  double HydrodynamicsParameters::CDampL2() const
  {
    return this->data->cDampL2;
  }

  double HydrodynamicsParameters::CDampR1() const
  {
    return this->data->cDampR1;
  }

  double HydrodynamicsParameters::CDampR2() const
  {
    return this->data->cDampR2;
  }

  double HydrodynamicsParameters::CPDrag1() const
  {
    return this->data->cPDrag1;
  }

  double HydrodynamicsParameters::CPDrag2() const
  {
    return this->data->cPDrag2;
  }

  double HydrodynamicsParameters::FPDrag() const
  {
    return this->data->fPDrag;
  }

  double HydrodynamicsParameters::CSDrag1() const
  {
    return this->data->cSDrag1;
  }

  double HydrodynamicsParameters::CSDrag2() const
  {
    return this->data->cSDrag2;
  }

  double HydrodynamicsParameters::FSDrag() const
  {
    return this->data->fSDrag;
  }
  
  double HydrodynamicsParameters::VRDrag() const
  {
    return this->data->vRDrag;
  }

  void HydrodynamicsParameters::SetFromMsg(const ignition::msgs::Param_V& _msg)
  {
    this->data->dampingOn      = Utilities::MsgParamBool(_msg,  "damping_on",       this->data->dampingOn);
    this->data->viscousDragOn  = Utilities::MsgParamBool(_msg,  "viscous_drag_on",  this->data->viscousDragOn);
    this->data->pressureDragOn = Utilities::MsgParamBool(_msg,  "pressure_drag_on", this->data->pressureDragOn);

    this->data->cDampL1 = Utilities::MsgParamDouble(_msg, "cDampL1",  this->data->cDampL1);
    this->data->cDampL2 = Utilities::MsgParamDouble(_msg, "cDampL2",  this->data->cDampL2);
    this->data->cDampR1 = Utilities::MsgParamDouble(_msg, "cDampR1",  this->data->cDampR1);
    this->data->cDampR2 = Utilities::MsgParamDouble(_msg, "cDampR2",  this->data->cDampR2);
    this->data->cPDrag1 = Utilities::MsgParamDouble(_msg, "cPDrag1",  this->data->cPDrag1);
    this->data->cPDrag2 = Utilities::MsgParamDouble(_msg, "cPDrag2",  this->data->cPDrag2);
    this->data->fPDrag  = Utilities::MsgParamDouble(_msg, "fPDrag",   this->data->fPDrag);
    this->data->cSDrag1 = Utilities::MsgParamDouble(_msg, "cSDrag1",  this->data->cSDrag1);
    this->data->cSDrag2 = Utilities::MsgParamDouble(_msg, "cSDrag2",  this->data->cSDrag2);
    this->data->fSDrag  = Utilities::MsgParamDouble(_msg, "fSDrag",   this->data->fSDrag);
    this->data->vRDrag  = Utilities::MsgParamDouble(_msg, "vRDrag",   this->data->vRDrag);
  }

  void HydrodynamicsParameters::SetFromSDF(sdf::Element& _sdf)
  {
    this->data->dampingOn      = Utilities::SdfParamBool(_sdf,  "damping_on",       this->data->dampingOn);
    this->data->viscousDragOn  = Utilities::SdfParamBool(_sdf,  "viscous_drag_on",  this->data->viscousDragOn);
    this->data->pressureDragOn = Utilities::SdfParamBool(_sdf,  "pressure_drag_on", this->data->pressureDragOn);

    this->data->cDampL1 = Utilities::SdfParamDouble(_sdf, "cDampL1",  this->data->cDampL1);
    this->data->cDampL2 = Utilities::SdfParamDouble(_sdf, "cDampL2",  this->data->cDampL2);
    this->data->cDampR1 = Utilities::SdfParamDouble(_sdf, "cDampR1",  this->data->cDampR1);
    this->data->cDampR2 = Utilities::SdfParamDouble(_sdf, "cDampR2",  this->data->cDampR2);
    this->data->cPDrag1 = Utilities::SdfParamDouble(_sdf, "cPDrag1",  this->data->cPDrag1);
    this->data->cPDrag2 = Utilities::SdfParamDouble(_sdf, "cPDrag2",  this->data->cPDrag2);
    this->data->fPDrag  = Utilities::SdfParamDouble(_sdf, "fPDrag",   this->data->fPDrag);
    this->data->cSDrag1 = Utilities::SdfParamDouble(_sdf, "cSDrag1",  this->data->cSDrag1);
    this->data->cSDrag2 = Utilities::SdfParamDouble(_sdf, "cSDrag2",  this->data->cSDrag2);
    this->data->fSDrag  = Utilities::SdfParamDouble(_sdf, "fSDrag",   this->data->fSDrag);
    this->data->vRDrag  = Utilities::SdfParamDouble(_sdf, "vRDrag",   this->data->vRDrag);
  }

  void HydrodynamicsParameters::DebugPrint() const
  {
    ignmsg << "damping_on:       " << this->data->dampingOn << std::endl;
    ignmsg << "viscous_drag_on:  " << this->data->viscousDragOn << std::endl;
    ignmsg << "pressure_drag_on: " << this->data->pressureDragOn << std::endl;
    ignmsg << "cDampL1:          " << this->data->cDampL1 << std::endl;
    ignmsg << "cDampL2:          " << this->data->cDampL2 << std::endl;
    ignmsg << "cDampR1:          " << this->data->cDampR1 << std::endl;
    ignmsg << "cDampR2:          " << this->data->cDampR2 << std::endl;
    ignmsg << "cPDrag1:          " << this->data->cPDrag1 << std::endl;
    ignmsg << "cPDrag2:          " << this->data->cPDrag2 << std::endl;
    ignmsg << "fPDrag:           " << this->data->fPDrag << std::endl;
    ignmsg << "cSDrag1:          " << this->data->cSDrag1 << std::endl;
    ignmsg << "cSDrag2:          " << this->data->cSDrag2 << std::endl;
    ignmsg << "fSDrag:           " << this->data->fSDrag << std::endl;
    ignmsg << "vRDrag:           " << this->data->vRDrag << std::endl;
  }

///////////////////////////////////////////////////////////////////////////////    
// TriangleProperties / SubmergedTriangleProperties

  class TriangleProperties
  {
    public: TriangleProperties() :
      index(0),
      normal(CGAL::NULL_VECTOR),
      area(std::numeric_limits<double>::signaling_NaN()),
      subArea(std::numeric_limits<double>::signaling_NaN()),
      vh(CGAL::ORIGIN),
      vm(CGAL::ORIGIN),
      vl(CGAL::ORIGIN),
      hh(std::numeric_limits<double>::signaling_NaN()),
      hm(std::numeric_limits<double>::signaling_NaN()),
      hl(std::numeric_limits<double>::signaling_NaN())
    {
    }

    public: size_t index;                       // index to the original triangle  
    public: cgal::Vector3 normal;                     // triangle normal
    public: double area;                        // area
    public: double subArea;                     // submerged area
    public: std::array<double, 3> heightMap;    // heightmap[3] - unsorted      
    public: cgal::Point3 vh;                          // high vertex
    public: cgal::Point3 vm;                          // mid vertex
    public: cgal::Point3 vl;                          // low vertex
    public: double hh;                          // high vertex height
    public: double hm;                          // mid vertex height
    public: double hl;                          // low vertex height
  };

  void DebugPrint(const TriangleProperties& props)
  {
    ignmsg << "index:        " << props.index << std::endl;
    ignmsg << "normal:       " << props.normal << std::endl;
    ignmsg << "area:         " << props.area << std::endl;
    ignmsg << "subArea:      " << props.subArea << std::endl;
    ignmsg << "vh:           " << props.vh << std::endl;
    ignmsg << "vm:           " << props.vm << std::endl;
    ignmsg << "vl:           " << props.vl << std::endl;
    ignmsg << "hh:           " << props.hh << std::endl;
    ignmsg << "hm:           " << props.hm << std::endl;
    ignmsg << "hl:           " << props.hl << std::endl;
  }

  class SubmergedTriangleProperties
  {
    public: SubmergedTriangleProperties() :
      index(0),
      normal(CGAL::NULL_VECTOR),
      centroid(CGAL::ORIGIN),
      xr(CGAL::NULL_VECTOR),
      area(std::numeric_limits<double>::signaling_NaN()),
      vp(CGAL::NULL_VECTOR),
      up(CGAL::NULL_VECTOR),
      cosTheta(std::numeric_limits<double>::signaling_NaN()),
      vn(CGAL::NULL_VECTOR),
      vt(CGAL::NULL_VECTOR),
      ut(CGAL::NULL_VECTOR),
      uf(CGAL::NULL_VECTOR),
      vf(CGAL::NULL_VECTOR)
    {          
    }

    public: int index;              // index to the original triangle  
    public: cgal::Vector3 normal;   // triangle normal
    public: cgal::Point3 centroid;  // triangle centroid = r
    public: cgal::Vector3 xr;       // xr = centroid - CoM = (r - x)
    public: double area;            // area
    public: cgal::Vector3 vp;       // point velocity vp = v + omega x r, where r = centroid - CoM
    public: cgal::Vector3 up;       // normalized point velocity. 
    public: double cosTheta;        // cos[theta] = up . normal  
    public: cgal::Vector3 vn;       // point velocity normal to surface vn = (vp . normal) normal
    public: cgal::Vector3 vt;       // point velocity tangential to surface vt = vp - vn
    public: cgal::Vector3 ut;       // normalized tangential point velocity. 
    public: cgal::Vector3 uf;       // direction of tangential flow. uf = - vt / ||vt|| = - ut 
    public: cgal::Vector3 vf;       // tangential flow vf = ||vp|| uf
  };

  void DebugPrint(const SubmergedTriangleProperties& props)
  {
    ignmsg << "index:        " << props.index << std::endl;
    ignmsg << "normal:       " << props.normal << std::endl;
    ignmsg << "centroid:     " << props.centroid << std::endl;
    ignmsg << "xr:           " << props.xr << std::endl;
    ignmsg << "area:         " << props.area << std::endl;
    ignmsg << "vp:           " << props.vp << std::endl;
    ignmsg << "up:           " << props.up << std::endl;
    ignmsg << "cosTheta:     " << props.cosTheta << std::endl;
    ignmsg << "vn:           " << props.vn << std::endl;
    ignmsg << "vt:           " << props.vt << std::endl;
    ignmsg << "ut:           " << props.ut << std::endl;
    ignmsg << "uf:           " << props.uf << std::endl;
    ignmsg << "vf:           " << props.vf << std::endl;
  }

///////////////////////////////////////////////////////////////////////////////    
// HydrodynamicsPrivate

  class HydrodynamicsPrivate
  {
    /// \brief The hydrodynamics parameters. 
    public: std::shared_ptr<const HydrodynamicsParameters> params;

    /// \brief The mesh of the rigid body described by this model link.
    public: std::shared_ptr<const cgal::Mesh> linkMesh;

    /// \brief The wavefield sampler for this rigid body (linkMesh).
    public: std::shared_ptr<const WavefieldSampler>  wavefieldSampler;

    /// \brief Pose of the centre of mass.
    public: ignition::math::Pose3d pose;

    /// \brief Position of the centre of mass (CGAL types).
    public: cgal::Point3 position;

    // \brief Linear velocity of the centre of mass.
    public: cgal::Vector3 linVelocity;

    /// \brief Angular velocity of the centre of mass.
    public: cgal::Vector3 angVelocity;

    /// \brief The calculated waterline length.
    public: double waterlineLength;

    /// \brief The depth at each vertex point.
    public: cgal::Mesh::Property_map<cgal::Mesh::Vertex_index, double> depths;
    public: std::vector<cgal::Triangle> submergedTriangles;
    public: std::vector<TriangleProperties> triangleProperties;
    public: std::vector<SubmergedTriangleProperties> submergedTriangleProperties;
    public: std::vector<cgal::Line> waterline;

    public: double area;

    public: double submergedArea;

    // Keep buoyance force and center of pressure for debugging...
    public: std::vector<cgal::Vector3> fBuoyancy;
    public: std::vector<cgal::Point3>  cBuoyancy;
  
    /// \brief The computed force
    public: cgal::Vector3 force;

    /// \brief The computed torque
    public: cgal::Vector3 torque;
  };

///////////////////////////////////////////////////////////////////////////////    
// Hydrodynamics

  Hydrodynamics::Hydrodynamics(
    std::shared_ptr<const HydrodynamicsParameters> _params,
    std::shared_ptr<const cgal::Mesh> _linkMesh,
    std::shared_ptr<const WavefieldSampler> _wavefieldSampler
  ) : data(new HydrodynamicsPrivate())
  {
    this->data->params = _params;
    this->data->linkMesh = _linkMesh;
    this->data->wavefieldSampler = _wavefieldSampler;
    this->data->position = CGAL::ORIGIN;
    this->data->linVelocity = CGAL::NULL_VECTOR;
    this->data->angVelocity = CGAL::NULL_VECTOR;
    this->data->waterlineLength = 0.0;
  }

  void Hydrodynamics::Update(
    std::shared_ptr<const WavefieldSampler> _wavefieldSampler,
    const ignition::math::Pose3d& _pose,
    const cgal::Vector3& _linVelocity,
    const cgal::Vector3& _angVelocity
  )
  {
    // Set rigid body props.
    this->data->wavefieldSampler = _wavefieldSampler;
    this->data->pose = _pose;
    this->data->position = ToPoint3(_pose.Pos());
    this->data->linVelocity = _linVelocity;
    this->data->angVelocity = _angVelocity;

    // Reset
    this->data->force = CGAL::NULL_VECTOR;
    this->data->torque = CGAL::NULL_VECTOR;

    // Update physics
    this->UpdateSubmergedTriangles();
    this->ComputeAreas();
    this->ComputeWaterlineLength();
    this->ComputePointVelocities();
    this->ComputeBuoyancyForce();

    if (this->data->params->ViscousDragOn())
      this->ComputeViscousDragForce();
    
    if (this->data->params->PressureDragOn())
      this->ComputePressureDragForce();
    
    if (this->data->params->DampingOn())
      this->ComputeDampingForce();
  }

  const cgal::Vector3& Hydrodynamics::Force() const
  {
    return this->data->force;
  }

  const cgal::Vector3& Hydrodynamics::Torque() const
  {
    return this->data->torque;
  }

  const std::vector<cgal::Line>& Hydrodynamics::GetWaterline() const
  {
    return this->data->waterline;
  }

  const std::vector<cgal::Triangle>& Hydrodynamics::GetSubmergedTriangles() const
  {
    return this->data->submergedTriangles;
  }

  void Hydrodynamics::UpdateSubmergedTriangles()
  {
    this->data->submergedTriangles.clear();
    this->data->triangleProperties.clear();
    this->data->submergedTriangleProperties.clear();
    this->data->waterline.clear();
    
    auto& linkMesh = *this->data->linkMesh;
    auto& wavefieldSampler = *this->data->wavefieldSampler;

    // @TODO_FRAGILE - prefer not to const_cast... assign prop map at creation.
    // Compute depths
    auto& ncLinkMesh = const_cast<cgal::Mesh&>(*this->data->linkMesh);
    auto pair = ncLinkMesh.add_property_map<cgal::Mesh::Vertex_index, double>("v:depth", 0);
    this->data->depths = pair.first;
    for (auto&& v : linkMesh.vertices())
    {
      this->data->depths[v] = wavefieldSampler.ComputeDepth(linkMesh.point(v));
    }

    // Get a list of the meshes exterior triangles
    for (auto&& face : linkMesh.faces())
    {
      cgal::Triangle triangle = Geometry::MakeTriangle(linkMesh, face);

      TriangleProperties triProps;
      // triProps.index = i;
      triProps.normal = Geometry::Normal(triangle);
      triProps.area = Geometry::TriangleArea(triangle);

      // @TODO_OPTIMISE - compute depth once for each vertex then assign height to each triangle
      // triProps.heightMap =  Physics::ComputeHeightMap(wavefieldSampler, triangle);
     
      // Note sign change for height.
      const auto& rng = CGAL::vertices_around_face(linkMesh.halfedge(face), linkMesh);
      for (
        auto&& it = std::make_pair(std::begin(rng), 0); 
        it.first != std::end(rng);
        ++it.first, ++it.second)
      {
        triProps.heightMap[it.second] = -this->data->depths[*it.first];
      }

      this->data->triangleProperties.push_back(triProps);

      // Populate the submerged sub-triangles
      this->PopulateSubmergedTriangle(triangle, triProps);

      // @DEBUG_INFO
      // DebugPrint(triangle);
      // DebugPrint(triProps);
    }

    // @DEBUG_INFO
    // ignmsg << "TriCount: " << this->data->triangleProperties.size() << std::endl;
    // for (auto triProps : this->data->triangleProperties)
    // {
    //   DebugPrint(triProps);
    // }
    // ignmsg << "SubTriCount: " << this->data->submergedTriangles.size() << std::endl;
    // for (auto subTriProps : this->data->submergedTriangleProperties)
    // {
    //   DebugPrint(subTriProps);
    // }
  }

  void Hydrodynamics::PopulateSubmergedTriangle(
    const cgal::Triangle& _triangle,
    TriangleProperties& _triProps)
  {
    // Calculations
    const size_t H=0, M=1, L=2;
    std::array<size_t, 3> idx = algorithm::sort_indexes(_triProps.heightMap);
  
    _triProps.hh = _triProps.heightMap[idx[H]];
    _triProps.hm = _triProps.heightMap[idx[M]];
    _triProps.hl = _triProps.heightMap[idx[L]];
  
    _triProps.vh = _triangle[idx[H]];
    _triProps.vm = _triangle[idx[M]];
    _triProps.vl = _triangle[idx[L]];
      
    if (_triProps.hh > 0)
    {
      if (_triProps.hm > 0)
      {
        if (_triProps.hl > 0)
          ; // no-op
        else
          this->SplitPartiallySubmergedTriangle1(_triProps);        
      } 
      else
        this->SplitPartiallySubmergedTriangle2(_triProps);        
    }
    else
      this->AddFullySubmergedTriangle(_triProps);      
  }

  void Hydrodynamics::SplitPartiallySubmergedTriangle1(TriangleProperties& _triProps)
  {
    cgal::Vector3& n = _triProps.normal;
    cgal::Point3& vh = _triProps.vh;
    cgal::Point3& vm = _triProps.vm;
    cgal::Point3& vl = _triProps.vl;
    double hh = _triProps.hh;
    double hm = _triProps.hm;
    double hl = _triProps.hl;
    
    double tm = -hl/(hm - hl);
    double th = -hl/(hh - hl);
    
    cgal::Point3 vmi = vl + (vm - vl) * tm;
    cgal::Point3 vhi = vl + (vh - vl) * th; 

    // @DEBUG_INFO
    // ignmsg << "index:         " << _triProps.index << std::endl;
    // ignmsg << "vmi:           " << vmi << std::endl;
    // ignmsg << "vhi:           " << vhi << std::endl;
            
    // Create the new submerged triangle
    cgal::Triangle tri0(vl, vmi, vhi);
    if (CGAL::scalar_product(n, Geometry::Normal(tri0)) < 0.0)
    {
      // Change orientation
      tri0 = cgal::Triangle(vl, vhi, vmi);
    }
    this->data->submergedTriangles.push_back(tri0);

    // Properties of the submerged tri0
    SubmergedTriangleProperties subTriProps0;
    subTriProps0.index = _triProps.index;
    subTriProps0.normal = Geometry::Normal(tri0);
    subTriProps0.centroid = Geometry::TriangleCentroid(tri0);    
    subTriProps0.area = Geometry::TriangleArea(tri0);
    this->data->submergedTriangleProperties.push_back(subTriProps0);

    // Fraction of original triangle submerged.
    _triProps.subArea = subTriProps0.area;

    // Create a new line (for the water line)
    cgal::Line line(vmi, vhi);
    this->data->waterline.push_back(line);
  }

  void Hydrodynamics::SplitPartiallySubmergedTriangle2(TriangleProperties& _triProps)
  {
    cgal::Vector3& n = _triProps.normal;
    cgal::Point3& vh = _triProps.vh;
    cgal::Point3& vm = _triProps.vm;
    cgal::Point3& vl = _triProps.vl;
    double hh = _triProps.hh;
    double hm = _triProps.hm;
    double hl = _triProps.hl;

    double tm = -hm/(hh - hm);
    double tl = -hl/(hh - hl);
    
    cgal::Point3 vmi =  vm + (vh - vm) * tm;
    cgal::Point3 vli =  vl + (vh - vl) * tl;
          
    // Create the new submerged triangles
    cgal::Triangle tri0(vm, vmi, vl);
    cgal::Triangle tri1(vmi, vli, vl);

    if (CGAL::scalar_product(n, Geometry::Normal(tri0)) < 0.0)
    {
      tri0 = cgal::Triangle(vmi, vm, vl);
    }
    if (CGAL::scalar_product(n, Geometry::Normal(tri1)) < 0.0)
    {
      tri1 = cgal::Triangle(vli, vmi, vl);
    }
    
    this->data->submergedTriangles.push_back(tri0);
    this->data->submergedTriangles.push_back(tri1);

    // Properties of the submerged tri0
    SubmergedTriangleProperties subTriProps0;
    subTriProps0.index = _triProps.index;
    subTriProps0.normal = Geometry::Normal(tri0);
    subTriProps0.centroid = Geometry::TriangleCentroid(tri0);
    subTriProps0.area = Geometry::TriangleArea(tri0);
    this->data->submergedTriangleProperties.push_back(subTriProps0);

    // Properties of the submerged tri1
    SubmergedTriangleProperties subTriProps1;
    subTriProps1.index = _triProps.index;
    subTriProps1.normal = Geometry::Normal(tri1);
    subTriProps1.centroid = Geometry::TriangleCentroid(tri1);
    subTriProps1.area = Geometry::TriangleArea(tri1);
    this->data->submergedTriangleProperties.push_back(subTriProps1);

    // Fraction of original triangle submerged.
    _triProps.subArea = subTriProps0.area + subTriProps1.area;

    // Create a new line (for the water line)
    cgal::Line line(vmi, vli);
    this->data->waterline.push_back(line);
  }

  void Hydrodynamics::AddFullySubmergedTriangle(TriangleProperties& _triProps)
  {
    // Add the full triangle
    cgal::Vector3& n = _triProps.normal;
    cgal::Point3& vh = _triProps.vh;
    cgal::Point3& vm = _triProps.vm;
    cgal::Point3& vl = _triProps.vl;
    
    // Create the new submerged triangle
    cgal::Triangle tri(vh, vm, vl);
    if (CGAL::scalar_product(n, Geometry::Normal(tri)) < 0.0)
    {
      tri = cgal::Triangle(vm, vh, vl);
    }
    this->data->submergedTriangles.push_back(tri);

    // Properties of the submerged triangle
    SubmergedTriangleProperties subTriProps;
    subTriProps.index = _triProps.index;
    subTriProps.normal = _triProps.normal;
    subTriProps.centroid = Geometry::TriangleCentroid(tri);    
    subTriProps.area = _triProps.area;
    this->data->submergedTriangleProperties.push_back(subTriProps);

    // Fraction of original triangle submerged.
    _triProps.subArea = subTriProps.area;
  }

  void Hydrodynamics::ComputeAreas()
  {
    double area=0.0;
    for (auto&& props : this->data->triangleProperties)
    {
      area += props.area;
    }
    this->data->area = area;

    double subArea=0.0;
    for (auto&& props : this->data->submergedTriangleProperties)
    {
      subArea += props.area;
    }
    this->data->submergedArea = subArea;
  }

  void Hydrodynamics::ComputeWaterlineLength()
  {
    // Calculate the direction of the x-axis
    cgal::Vector3 xaxis = ToVector3(this->data->pose.Rot().RotateVector(
      ignition::math::Vector3d(1, 0, 0)));

    // Project the waterline onto the x-axis
    double length = 0.0;
    for (auto&& line : this->data->waterline)
    {
      length += std::abs(CGAL::scalar_product(line.to_vector(), xaxis));
    }
    length *= 0.5;
    this->data->waterlineLength = length;
    // @DEBUG_INFO
    // ignmsg << "waterline length: " << length << std::endl;
  }

  // Compute the point velocity at a triangles centroid
  void Hydrodynamics::ComputePointVelocities()
  {
    auto& position = this->data->position;
    auto& v = this->data->linVelocity;
    auto& omega = this->data->angVelocity;

    for (auto&& subTriProps : this->data->submergedTriangleProperties)
    {
      // relative position of the centroid wrt CoM
      subTriProps.xr = subTriProps.centroid - position;
      
      // vp = v + omega x xr
      subTriProps.vp = v + CGAL::cross_product(omega, subTriProps.xr);

      // up = vp / ||vp||
      subTriProps.up = Geometry::Normalize(subTriProps.vp);
      
      // cos(theta) = up . n
      subTriProps.cosTheta = CGAL::scalar_product(subTriProps.up, subTriProps.normal);

      // vn = (up . n) n
      subTriProps.vn = subTriProps.normal * subTriProps.cosTheta;
      
      // vt = vp - vn
      subTriProps.vt = subTriProps.vp - subTriProps.vn;

      // un = vn / ||vn||
      // subTriProps.un = Geometry::Normalize(subTriProps.vn);

      // ut = vt / ||vt||
      subTriProps.ut = Geometry::Normalize(subTriProps.vt);

      // uf = - vt / ||vt|| = - ut
      subTriProps.uf = - subTriProps.ut;

      // vf = ||vp|| uf
      subTriProps.vf = subTriProps.uf * std::sqrt(subTriProps.vp.squared_length());
    }
  }

  // Compute the Reynolds number
  double Hydrodynamics::ComputeReynoldsNumber() const 
  {
    // fluid speed
    auto& v = this->data->linVelocity;
    double u = std::sqrt(v.squared_length());

    // characteristic length
    double L = this->data->waterlineLength;

    // Reynolds number
    double kv = PhysicalConstants::WaterKinematicViscosity();
    double Rn = u * L / kv;

    return Rn;
  }

  void Hydrodynamics::ComputeBuoyancyForce()
  {
    cgal::Vector3 sumForce  = CGAL::NULL_VECTOR;
    cgal::Vector3 sumTorque = CGAL::NULL_VECTOR;

    this->data->fBuoyancy.clear();
    this->data->cBuoyancy.clear();

    // Calculate the buoyancy force for the submerged triangles
    auto& position  =  this->data->position;
    auto& wavefieldSampler = *this->data->wavefieldSampler;
    for (auto&& subTri : this->data->submergedTriangles)
    {
      // Force and center of pressure.
      cgal::Point3 center = CGAL::ORIGIN;
      cgal::Vector3 force = CGAL::NULL_VECTOR;
      Physics::BuoyancyForceAtCenterOfPressure(
        wavefieldSampler, subTri, center, force);
      this->data->fBuoyancy.push_back(force);
      this->data->cBuoyancy.push_back(center);

      // Torque    
      cgal::Vector3 xr = center - position;
      cgal::Vector3 torque = CGAL::cross_product(xr, force);
      sumForce += force;
      sumTorque += torque;    
    }

    this->data->force  += sumForce;
    this->data->torque += sumTorque;
  }

  void Hydrodynamics::ComputeDampingForce()
  {
    auto& params = *this->data->params;

     // Linear drag coefficients
    double cDampL1 = params.CDampL1();
    double cDampR1 = params.CDampR1();

    // Quadratic drag coefficients
    double cDampL2 = params.CDampL2();
    double cDampR2 = params.CDampR2();

    double area = this->data->area;
    double subArea = this->data->submergedArea;
    double rs = subArea / area;
     
    // Force
    auto& v = this->data->linVelocity;
    double linSpeed = std::sqrt(v.squared_length());
    double cL = - rs * (cDampL1 + cDampL2 * linSpeed);
    cgal::Vector3 force = v * cL;
 
    auto& omega = this->data->angVelocity;
    double angSpeed = std::sqrt(omega.squared_length());
    double cR = - rs * (cDampR1 + cDampR2 * angSpeed);    
    cgal::Vector3 torque = omega * cR;

    this->data->force  += force;
    this->data->torque += torque;

    // @DEBUG_INF0
    // ignmsg << "area:       " << area << std::endl; 
    // ignmsg << "subArea:    " << subArea << std::endl; 
    // ignmsg << "v:          " << v << std::endl; 
    // ignmsg << "omega:      " << omega << std::endl; 
    // ignmsg << "linSpeed:   " << linSpeed << std::endl; 
    // ignmsg << "angSpeed:   " << angSpeed << std::endl; 
    // ignmsg << "cR:         " << cR << std::endl; 
    // ignmsg << "cL:         " << cL << std::endl; 
    // ignmsg << "force:      " << force << std::endl; 
    // ignmsg << "torque:     " << torque << std::endl; 

  }

  // Viscous drag force - applied at triangle centroid.
  void Hydrodynamics::ComputeViscousDragForce()
  {    
    double rho = PhysicalConstants::WaterDensity();
    double Rn = this->ComputeReynoldsNumber();
    double cF = Physics::ViscousDragCoefficient(Rn);
 
    cgal::Vector3 sumForce  = CGAL::NULL_VECTOR;
    cgal::Vector3 sumTorque = CGAL::NULL_VECTOR;
    for (auto&& subTriProps : this->data->submergedTriangleProperties)
    {
      // Force
      double fDrag = 0.5 * rho * cF * subTriProps.area 
        * std::sqrt(subTriProps.vf.squared_length());
      cgal::Vector3 force = subTriProps.vf * fDrag;  
      sumForce += force;

      // Torque;
      cgal::Vector3 torque = CGAL::cross_product(subTriProps.xr, force);
      sumTorque += torque;
    }

    this->data->force  += sumForce;
    this->data->torque += sumTorque;
  }

  // Pressure drag force - applied at triangle centroid.
  void Hydrodynamics::ComputePressureDragForce()
  {
    auto& params = *this->data->params;

    // Positive pressure
    double cPDrag1 = params.CPDrag1();
    double cPDrag2 = params.CPDrag2();
    double fPDrag  = params.FPDrag();
    // Negative pressure (suction)
    double cSDrag1 = params.CSDrag1();
    double cSDrag2 = params.CSDrag2();
    double fSDrag  = params.FSDrag();

    // Reference speed
    double vRDrag  = params.VRDrag();

    cgal::Vector3 sumForce  = CGAL::NULL_VECTOR;
    cgal::Vector3 sumTorque = CGAL::NULL_VECTOR;
    for (auto&& subTriProps : this->data->submergedTriangleProperties)
    {
      // General
      double S    = subTriProps.area;
      double vp   = std::sqrt(subTriProps.vp.squared_length());
      double cosTheta = subTriProps.cosTheta;

      double v    = vp / vRDrag;
      double drag = 0.0;
      if (cosTheta >= 0.0)
      {
        drag = -(cPDrag1 * v + cPDrag2 * v * v) * S * std::pow( cosTheta, fPDrag);
      }
      else
      {
        drag =  (cSDrag1 * v + cSDrag2 * v * v) * S * std::pow(-cosTheta, fSDrag);
      }
      cgal::Vector3 force = subTriProps.normal * drag;
      sumForce += force;

      // Torque;
      cgal::Vector3 torque = CGAL::cross_product(subTriProps.xr, force);
      sumTorque += torque;
    }

    this->data->force  += sumForce;
    this->data->torque += sumTorque;

    // @DEBUG_INFO
    // if (std::abs(sumForce.z()) > 1.0E+10)
    // {
    //   ignmsg << "Overflow in ComputePressureDragForce..."    << std::endl;
    //   ignmsg << "position:     " << this->data->position     << std::endl;
    //   ignmsg << "linVelocity:  " << this->data->linVelocity  << std::endl;
    //   ignmsg << "angVelocity:  " << this->data->angVelocity  << std::endl;
    //   ignmsg << "force:        " << sumForce                 << std::endl;
    //   ignmsg << "torque:       " << sumTorque                << std::endl;
    //   for (auto&& subTriProps : this->data->submergedTriangleProperties)
    //   {
    //     DebugPrint(subTriProps);
    //   }
    // }
  }

}
}
