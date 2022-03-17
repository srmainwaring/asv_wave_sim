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


// Copyright (c) 2021 Rhys Mainwaring.
// All rights reserved.
//
// Ported to Metal and modified to remove trochoid wave generation
// and pass uv0, BTN, and bump coords to the fragment shader.
//

#include <metal_stdlib>
using namespace metal;

struct VS_INPUT
{
  float4 position     [[attribute(VES_POSITION)]];
  float3 normal       [[attribute(VES_NORMAL)]];
  float2 uv0          [[attribute(VES_TEXTURE_COORDINATES0)]];
};

struct PS_INPUT
{
  float4 gl_Position  [[position]];
  float2 uv0;
  float3 B;
  float3 T;
  float3 N;
  float3 eyeVec;
  float2 bumpCoord;
};

struct Params
{
  float4x4 worldviewproj_matrix;
  float3 camera_position_object_space;
  float rescale;
  float2 bumpScale;
  float2 bumpSpeed;
  float t;
};

vertex PS_INPUT main_metal
(
  VS_INPUT input [[stage_in]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  PS_INPUT outVs;

  float4 P = input.position;

  // Binormal, tangent, and normal vectors:
  float3 B = float3(1.0, 0.0, 0.0);
  float3 T = float3(0.0, 1.0, 0.0);
  float3 N = float3(0.0, 0.0, 1.0);

  // VES_NORMAL does not appear to be populated correctly by Ignition?
  // float3 N = input.normal;

  // Compute (Surf2World * Rescale) matrix
  B = normalize(B) * p.rescale;
  T = normalize(T) * p.rescale;
  N = normalize(N);

  // outVs won't accept float3x3, so pass components 
  outVs.B = B;
  outVs.T = T;
  outVs.N = N;
 
  outVs.gl_Position = p.worldviewproj_matrix * P;

  // Pass texture coordinates to frag shader
  outVs.uv0 = input.uv0.xy;

  // Eye position in vertex space
  outVs.eyeVec = P.xyz - p.camera_position_object_space;

  // Compute texture coordinates for bump map
  outVs.bumpCoord = input.uv0.xy * p.bumpScale + p.t * p.bumpSpeed;

  return outVs;
}
