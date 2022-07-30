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

#version 330

in vec4 vertex;
uniform mat4 worldviewproj_matrix;
uniform mat4 worldview_matrix;
uniform mat4 inverse_transpose_world_matrix;
uniform float t;

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec3 pos;
  vec3 norm;
} outVs;

// deform along vertex normal dir
vec3 deform(vec3 v, vec3 n)
{
  float z = 0.05 * sin((t+3.0*v.y)*4.0) * cos((t+3.0*v.x)*4.0);
  return v + n * z;
}

void main()
{
  vec3 norm = normalize(vertex.xyz);
  vec3 v = deform(vertex.xyz, norm);
  gl_Position = worldviewproj_matrix * vec4(v, 1.0);
  vec4 p = worldview_matrix * vec4(v, 1.0);
  outVs.pos = p.xyz / p.w;
  outVs.norm = vec3((inverse_transpose_world_matrix) * vec4(norm, 1.0));
}

