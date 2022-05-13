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

// Copyright (c) 2019 Rhys Mainwaring.
//
// Modified for use with FFT generated waves:
// 1. Remove inputs for trochoid waves.
// 2. The vertex shader forwards: vNormal, vBumpCoord, vEyeVec to the fragment shader. 
// 3. Replace the rotation matrix with the input normal. 

// Input parameters
uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

// Input computed in vertex shader
varying vec3 vNormal;
varying vec3 vEyeVec;
varying vec2 vBumpCoordxy;
varying vec2 vBumpCoordzw;
varying vec4 vVertex;

void main(void)
{
    // osgOcean noise normal calculation...
    vec3 noiseNormal = vec3(texture2D(bumpMap, vBumpCoordxy) * 2.0 - 1.0);
    noiseNormal += vec3(texture2D(bumpMap, vBumpCoordzw) * 2.0 - 1.0);

    // Apply bump mapping to normal vector to make waves look more detailed:
    // vec4 bump = texture2D(bumpMap, vBumpCoord) * 2.0 - 1.0;
    // vec3 N = vNormal;    
    vec3 N = normalize(vNormal + noiseNormal);

    // Reflected ray:
    vec3 E = normalize(vEyeVec);
    vec3 R = reflect(E, N);
    // Gazebo requires rotated cube map lookup.
    R = vec3(R.x, R.z, R.y);

    // Get environment color of reflected ray:
    vec4 envColor = textureCube(cubeMap, R, 0.0);

    // Cheap hdr effect:
    envColor.rgb *= (envColor.r + envColor.g + envColor.b) * hdrMultiplier;

    // Compute refraction ratio (Fresnel):
    float facing = 1.0 - dot(-E, N);
    float refractionRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

    // Refracted ray only considers deep and shallow water colors:
    vec4 waterColor = mix(shallowColor, deepColor, facing);

    // Perform linear interpolation between reflection and refraction.
    vec4 color = mix(waterColor, envColor, refractionRatio);
    vec4 final_color = vec4(color.xyz, 0.9);

    gl_FragColor = final_color;
}
