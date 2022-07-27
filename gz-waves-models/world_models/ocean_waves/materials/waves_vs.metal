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

// Adapted from glsl trochoid wave shaders developed
// in https://github.com/uuvsimulator/uuv_simulator

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

float m_det(float3x3 m)
{
  float a = m[0][0];
  float b = m[0][1];
  float c = m[0][2];
  float d = m[1][0];
  float e = m[1][1];
  float f = m[1][2];
  float g = m[2][0];
  float h = m[2][1];
  float i = m[2][2];
  float A =  (e*i - f*h);
  float B = -(d*i - f*g);
  float C =  (d*h - e*g);
  float det = a*A + b*B + c*C;
  return det;
 }

float3x3 m_inverse(float3x3 m)
{
  float a = m[0][0];
  float b = m[0][1];
  float c = m[0][2];
  float d = m[1][0];
  float e = m[1][1];
  float f = m[1][2];
  float g = m[2][0];
  float h = m[2][1];
  float i = m[2][2];
  float A =  (e*i - f*h);
  float B = -(d*i - f*g);
  float C =  (d*h - e*g);
  float D = -(b*i - c*h);
  float E =  (a*i - c*g);
  float F = -(a*h - b*g);
  float G =  (b*f - c*e);
  float H = -(a*f - c*d);
  float I =  (a*e - b*d);
  float det = a*A + b*B + c*C;
  float inv_det = 1.0/det;

  float3x3 inv = float3x3(
      A, D, G, B, E, H, C, F, I);
  inv = inv * inv_det;

  return inv;
}

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

  float4x4 worldM = p.world_matrix;

  // compute normal matrix
  float3 model0 = worldM[0].xyz;
  float3 model1 = worldM[1].xyz;
  float3 model2 = worldM[2].xyz;
  float3x3 model = float3x3(
    model0.x, model0.y, model0.z,  
    model1.x, model1.y, model1.z,  
    model2.x, model2.y, model2.z);
  float3x3 inv_model = m_inverse(model);
  float3x3 normal_matrix = transpose(inv_model);

  float4 P = input.position.xyzw;

  // debug check to establish which vertex quadrant the uv0 maps to
  // P.y += ((1 - input.uv0.x) > 0.5 && input.uv0.y > 0.5) ? 10.0 : 0.0;

  float2 texcoord(/*1.0 - */ input.uv0.x, 1.0 - input.uv0.y);

  float4 displacements = heightMap.sample(heightMapSampler, texcoord);
  P.x += displacements.x;
  P.y += displacements.y;
  P.z += displacements.z;

  float4 tangent = tangentMap.sample(tangentMapSampler, texcoord);
  float3 T = tangent.xyz;

  float4 normal = normalMap.sample(normalMapSampler, texcoord);
  float3 N = normal.xyz;
 
  T = normalize(normal_matrix * T);
  N = normalize(normal_matrix * N);

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
