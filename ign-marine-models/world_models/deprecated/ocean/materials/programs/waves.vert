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
// Modified for use with FFT generated waves:
// 1. Remove inputs for trochoid waves.
// 2. The vertex shader forwards: normal, bumpCoord, eyeVec to the fragment shader. 
// 3. Replace the rotation matrix with the input normal. 

// Attributes
// vec4 gl_Vertex;          position
// vec3 gl_Normal           normal
// vec4 gl_MultiTexCoord0   texture coordinate of texture unit 0

// Input parameters
uniform vec3 eyePos;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform float time;

// Output variables
varying vec3 vNormal;
varying vec3 vEyeVec;
varying vec2 vBumpCoordxy;
varying vec2 vBumpCoordzw;
varying vec4 vVertex;

void main(void)
{
    // [out] gl_Position vertex position
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    // [out] normal
    vNormal = gl_Normal;

    // [out] texture coordinates for bump map
    vBumpCoordxy = gl_MultiTexCoord0.xy * bumpScale + time * bumpSpeed;
    vBumpCoordzw = gl_MultiTexCoord0.zw * bumpScale + time * bumpSpeed;

    ////////////////////////////////////////////////////////////////////////////////
    // See http://fabiensanglard.net/bumpMapping/index.php:
    // gl_TexCoord[0] = gl_MultiTexCoord0;

    // // Build the matrix to map Eye Space -> Tangent Space
    // vec3 n = normalize(gl_NormalMatrix * gl_Normal);
    // vec3 t = normalize(gl_NormalMatrix * tangent);
    // vec3 b = cross(n, t);
    ////////////////////////////////////////////////////////////////////////////////


    // [out] eye position in object space
    vEyeVec = gl_Vertex.xyz - eyePos; 

    // [out] vertex in object space
    vVertex = gl_Vertex;
}
