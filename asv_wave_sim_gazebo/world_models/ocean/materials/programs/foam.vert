// Copyright (c) 2019 Rhys Mainwaring.
//
// Code excerpt from osgOcean_ocean_surface.vert

// Input parameters
uniform vec3 eyePos;
uniform float osgOcean_FoamScale;

// Output variables
varying vec3 vNormal;
varying vec3 vEyeVec;
varying vec4 vVertex;
varying vec2 vFoamTexCoord;

void main(void)
{
    // [out] gl_Position vertex position
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    // [out] normal
    vNormal = gl_Normal;

    // [out] eye position in vertex space
    vEyeVec = gl_Vertex.xyz - eyePos; 

    // [out] foam effect
    vVertex = gl_Vertex;
    vFoamTexCoord = gl_Vertex.xy * osgOcean_FoamScale;
}
