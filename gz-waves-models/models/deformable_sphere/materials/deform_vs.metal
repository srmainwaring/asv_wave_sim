/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <metal_stdlib>
using namespace metal;

struct VS_INPUT
{
  float4 position [[attribute(VES_POSITION)]];
};

struct PS_INPUT
{
  float4 gl_Position  [[position]];
  float3 pos;
  float3 norm;
};

struct Params
{
  float4x4 worldviewproj_matrix;
  float4x4 worldview_matrix;
  float4x4 inverse_transpose_world_matrix;
  float t;
};

// deform along vertex normal dir
float3 deform(float3 v, float3 n, float t)
{
  float z = 0.05 * sin((t+3.0*v.y)*4.0) * cos((t+3.0*v.x)*4.0);
  return v + n * z;
}

vertex PS_INPUT main_metal
(
  VS_INPUT input [[stage_in]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  PS_INPUT outVs;

  float3 norm = normalize(input.position.xyz);
  float3 v = deform(input.position.xyz, norm, p.t);
  outVs.gl_Position = p.worldviewproj_matrix * float4(v, 1.0);
  float4 position = p.worldview_matrix * float4(v, 1.0);
  outVs.pos = position.xyz / position.w;
  outVs.norm = float3((p.inverse_transpose_world_matrix) * float4(norm, 1.0));

  return outVs;
}

