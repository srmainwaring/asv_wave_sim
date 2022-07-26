#version 330

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
// limitations under the License.

in block
{
  vec2 uv0;
  vec3 T;
  vec3 B;
  vec3 N;
  vec3 eyeVec;
  vec2 bumpCoord;
} inPs;

uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

uniform samplerCube cubeMap;
uniform sampler2D bumpMap;

out vec4 fragColor;

void main()
{
  // Apply bump mapping to normal vector to make waves look more detailed:
  vec4 bump = texture(bumpMap, inPs.bumpCoord)*2.0 - 1.0;
  mat3 rotMatrix = mat3(inPs.T, inPs.B, inPs.N);
  vec3 N = normalize(rotMatrix * bump.xyz);

  // Reflected ray:
  vec3 E = normalize(inPs.eyeVec);
  vec3 R = reflect(E, N);

  // Negate z for use with the skybox texture that comes with gz-rendering
  R = vec3(R.x, R.y, -R.z);

  // Get environment color of reflected ray:
  vec4 envColor = texture(cubeMap, R, 0.0);

  // Cheap hdr effect:
  envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

  // Compute refraction ratio (Fresnel):
  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

  // Refracted ray only considers deep and shallow water colors:
  vec4 waterColor = mix(shallowColor, deepColor, facing);

  // Perform linear interpolation between reflection and refraction.
  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  fragColor = vec4(color.xyz, 0.9);

  // debug
  // fragColor = vec4(inPs.T, 1.0);
}
