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
// limitations under the License.

// Copyright (c) 2021 Rhys Mainwaring.
// All rights reserved.
//
// Ported to Metal

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
  float3 B;
  float3 T;
  float3 N;
  float3 eyeVec;
  float2 bumpCoord;
};

struct Params
{
  float4 deepColor;
  float4 shallowColor;
  float fresnelPower;
  float hdrMultiplier;
};

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texturecube<float>  cubeMap         [[texture(1)]],
  texture2d<float>    bumpMap         [[texture(0)]],
  sampler             cubeMapSampler  [[sampler(1)]],
  sampler             bumpMapSampler  [[sampler(0)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  // Apply bump mapping to normal vector to make waves look more detailed:
  float4 bump = bumpMap.sample(bumpMapSampler, inPs.bumpCoord)*2.0 - 1.0;
  float3x3 rotMatrix(inPs.B, inPs.T, inPs.N);
  float3 N = normalize(rotMatrix * bump.xyz);

  // Reflected ray:
  float3 E = normalize(inPs.eyeVec);
  float3 R = reflect(E, N);

  // Negate z for use with the skybox texture that comes with ign-rendering
  R = float3(R.x, R.y, -R.z);

  // Get environment color of reflected ray:
  float4 envColor = cubeMap.sample(cubeMapSampler, R);

  // Cheap hdr effect:
  envColor.rgb *= (envColor.r+envColor.g+envColor.b)*p.hdrMultiplier;

  // Compute refraction ratio (Fresnel):
  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, p.fresnelPower), 0.0, 1.0);

  // Refracted ray only considers deep and shallow water colors:
  float4 waterColor = mix(p.shallowColor, p.deepColor, facing);

  // Perform linear interpolation between reflection and refraction.
  float4 color = mix(waterColor, envColor, waterEnvRatio);

  return float4(color.xyz, 0.7);

  // Debugging
  // FAIL - VES_NORMAL from vertex shader is not valid (all components are zero)
  // return float4(N.xyz, 1.0);
}
