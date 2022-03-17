// Copyright (c) 2019 Rhys Mainwaring.
//
// Code excerpt from osgOcean_ocean_surface.frag

// Input parameters
uniform float osgOcean_FresnelMul;
uniform float osgOcean_FoamCapBottom;
uniform float osgOcean_FoamCapTop;
uniform sampler2D osgOcean_FoamMap;

// Input computed in vertex shader
varying vec3 vNormal;
varying vec3 vEyeVec;
varying vec4 vVertex;
varying vec2 vFoamTexCoord;

float alphaHeight( float min, float max, float val)
{
    if(max-min == 0.0)
        return 1.0;

    return (val - min) / (max - min);
}

float calcFresnel( float dotEN, float mul )
{
    float fresnel = clamp( dotEN, 0.0, 1.0 ) + 1.0;
    return pow(fresnel, -8.0) * mul;
}

void main(void)
{
    vec3 N = vNormal;    
    vec3 E = normalize(vEyeVec);

    vec4 final_color = vec4(0.0);

    // Foam - from osgOcean_ocean_surface.frag
    float dotEN = dot(E, N);
    float fresnel = calcFresnel(dotEN, osgOcean_FresnelMul);

    if( vVertex.z > osgOcean_FoamCapBottom)
    {
        vec4 foam_color = texture2D(osgOcean_FoamMap, vFoamTexCoord/10.0);
        float alpha = alphaHeight(osgOcean_FoamCapBottom, osgOcean_FoamCapTop, vVertex.z) * (fresnel * 2.0);
        // final_color = final_color + (foam_color * alpha);
        final_color = (foam_color * alpha);
    }

    gl_FragColor = final_color;
}
