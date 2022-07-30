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

out vec4 fragColor;
uniform vec3 color;

in block
{
  vec3 pos;
  vec3 norm;
} inPs;

void main()
{
  vec3 lightAmbient = vec3(0.1f, 0.1f, 0.1f);
  vec3 lightDiffuse = vec3(0.8f, 0.7f, 0.2f);
  vec3 lightSpecular = vec3(0.1f, 0.1f, 0.1f);

  vec3 ambientColor = color;
  vec3 diffuseColor = color;
  vec3 specularColor = color;

  vec3 lightPos = vec3(0, 0, 1);
  vec3 lightDir = normalize(lightPos - inPs.pos);

  // normalize both input vectors
  vec3 n = normalize(inPs.norm);
  vec3 e = normalize(-inPs.pos);

  float specular = 0.0f;
  float NdotL = max(dot(lightDir, inPs.norm), 0.0);
  // if the vertex is lit compute the specular color
  if (NdotL> 0.0)
  {
    // compute the half vector
    vec3 halfVector = normalize(lightDir + e);

    // add specular
    float NdotH = max(dot(halfVector, n), 0.0);

    float shininess = 1.0;
    specular = pow(NdotH, shininess);
  }

  vec3 finalColor = lightAmbient * ambientColor;
  finalColor += lightDiffuse * diffuseColor * NdotL;
  finalColor += lightSpecular * specularColor * specular;

  fragColor = vec4(finalColor, 1.0);
}
