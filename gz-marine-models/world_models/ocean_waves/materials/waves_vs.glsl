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
// limitations under the License.s

in vec4 position;
in vec2 uv0;

out block
{
  vec2 uv0;
  vec3 T;
  vec3 B;
  vec3 N;
  vec3 eyeVec;
  vec2 bumpCoord;
} outVs;

out gl_PerVertex
{
  vec4 gl_Position;
};

uniform mat4 world_matrix;
uniform mat4 worldviewproj_matrix;
uniform vec3 camera_position;
uniform float t;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;

uniform sampler2D heightMap;
uniform sampler2D normalMap;
uniform sampler2D tangentMap;

void main()
{
  vec2 resolution = vec2(1.0, 1.0) * 16.0;

  mat4 model = world_matrix;

  // compute normal matrix
  // vec3 model0 = model[0].xyz;
  // vec3 model1 = model[1].xyz;
  // vec3 model2 = model[2].xyz;
  // mat3 model_inv = inverse(mat3(
  //     model0.x, model0.y, model0.z,
  //     model1.x, model1.y, model1.z,
  //     model2.x, model2.y, model2.z));
  // mat3 normal_matrix = transpose(model_inv);

  vec4 P = position.xyzw;

  // debug check to establish which vertex quadrant the uv0 maps to
//  P.z += ((1 - uv0.x) > 0.5 && uv0.y > 0.5) ? 10.0 : 0.0;
  //P.z += (uv0.x > 0.5 && uv0.y > 0.5) ? 10.0 : 0.0;

  vec2 texcoord = vec2(/*1.0 - */ uv0.x, /*1.0 -*/ uv0.y);

  // Displacement map
  vec4 displacements = texture(heightMap, texcoord);
  P.x += displacements.x;
  P.y += displacements.y;
  P.z += displacements.z;

  vec4 tangent = texture(tangentMap, texcoord);
  vec3 T = (model * tangent).xyz;
  T = normalize(T);

  vec4 normal = texture(normalMap, texcoord);
  vec3 N = (model * normal).xyz;
  N = normalize(N);

  vec3 B = cross(N, T);
  B = normalize(B);
  
  gl_Position = worldviewproj_matrix * P.xyzw;
  outVs.uv0 = uv0.xy;

  // Rescale tangent vectors
  outVs.T = T * rescale;
  outVs.B = B * rescale;
  outVs.N = N;

  // Compute texture coordinates for bump map
  outVs.bumpCoord = uv0.xy * bumpScale * resolution + t * bumpSpeed;

  // Eye position in world space
  outVs.eyeVec = (model * P).xyz - camera_position;
}
