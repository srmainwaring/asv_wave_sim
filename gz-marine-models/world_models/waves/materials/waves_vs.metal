// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.s


// Copyright (c) 2019 Rhys Mainwaring.
//
// Modified to accept vector parameters and use the form
// for Gerstner waves published in:
//
// Jerry Tessendorf, "Simulating Ocean Water", 1999-2004
//
// theta = k * dir . x - omega * t
//
// px = x - dir.x * a * k * sin(theta)
// py = y - dir.y * a * k * sin(theta)
// pz =         a * k * cos(theta)
//
// k is the wavenumber
// omega is the angular frequency
//
// The derivative terms (Tangent, Binormal, Normal) have been
// updated to be consistent with this convention.


// original concept:
// https://developer.nvidia.com/gpugems/gpugems/part-i-natural-effects/chapter-1-effective-water-simulation-physical-models

#include <metal_stdlib>
using namespace metal;

struct VS_INPUT
{
  float4 position [[attribute(VES_POSITION)]];
  float2 uv0      [[attribute(VES_TEXTURE_COORDINATES0)]];
};

struct PS_INPUT
{
  float4 gl_Position  [[position]];
  float2 uv0;
  float3 T;
  float3 B;
  float3 N;
  float3 eyeVec;
  float2 bumpCoord;
};

struct Params
{
  float4x4 world_matrix;
  float4x4 worldviewproj_matrix;
  float3 camera_position;
  float t;
  float rescale;
  float2 bumpScale;
  float2 bumpSpeed;
};

vertex PS_INPUT main_metal
(
  VS_INPUT input [[stage_in]]
  , texture2d<float, access::sample>  heightMap   [[texture(0)]]
  , texture2d<float, access::sample>  normalMap   [[texture(1)]]
  , texture2d<float, access::sample>  tangentMap  [[texture(2)]]
  , sampler heightMapSampler  [[sampler(0)]]
  , sampler normalMapSampler  [[sampler(1)]]
  , sampler tangentMapSampler [[sampler(2)]]
  , constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  PS_INPUT outVs;

  float2 resolution = float2(1.0, 1.0) * 16.0;

  // uncomment to use a custom sampler
  // constexpr sampler s(coord::normalized, address::repeat, filter::linear);

  // DEBUG - switch TBN between object and world space
  float4x4 identity(
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1);

  // float4x4 worldM = identity;
  float4x4 worldM = p.world_matrix;

  float4 P = input.position.xyzw;

  float3 T(0.0, 0.0, 0.0);
  float3 N(0.0, 0.0, 0.0);

  // debug check to establish which vertex quadrant the uv0 maps to
  // P.y += ((1 - input.uv0.x) > 0.5 && input.uv0.y > 0.5) ? 10.0 : 0.0;

  float2 texcoord(/*1.0 - */ input.uv0.x, 1.0 - input.uv0.y);

  // Resampling at different scales
  float sampleScale = 1.0;
  float sampleWeight = 1.0;
  for (int i=0; i<1; ++i)
  {
    // float4 displacements = heightMap.read(ushort2(texcoord * resolution));
    float4 displacements = heightMap.sample(heightMapSampler, sampleScale * texcoord);
    P.x += displacements.x * sampleWeight;
    P.y += displacements.y * sampleWeight;
    P.z += displacements.z * sampleWeight;

    // float4 tangent = tangentMap.read(ushort2(texcoord * resolution));
    float4 tangent = tangentMap.sample(tangentMapSampler, sampleScale * texcoord);
    T += (worldM * tangent).xyz * sampleWeight;

    // float4 normal = normalMap.read(ushort2(texcoord * resolution));
    float4 normal = normalMap.sample(tangentMapSampler, sampleScale * texcoord);
    N += (worldM * normal).xyz * sampleWeight;

    // update sample weight and scale
    sampleScale *= 3;
    sampleWeight *= 0.5;
  }
 
  T = normalize(T);
  N = normalize(N);

  float3 B = cross(N, T);
  B = normalize(B);

  outVs.gl_Position = p.worldviewproj_matrix * P.xyzw;
  outVs.uv0 = input.uv0.xy;

  // Rescale tangent vectors
  outVs.T = T * p.rescale;
  outVs.B = B * p.rescale;
  outVs.N = N;

  // Compute texture coordinates for bump map
  outVs.bumpCoord = input.uv0.xy * p.bumpScale * resolution + p.t * p.bumpSpeed;

  // Eye position in world space
  outVs.eyeVec = (worldM * P).xyz - p.camera_position;

  return outVs;
}
