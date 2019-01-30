// #version 110 core

#define USE_NORMAL_MAP 1

// In
varying vec3 vPosition;
varying vec3 vNormal;
varying mat3 vTBN;
varying vec2 vBumpCoord;

// Uniform
uniform vec3 uViewPos;
uniform vec4 uDeepColor;
uniform vec4 uShallowColor;
uniform float uFresnelPower;
uniform float uHdrMultiplier;
uniform sampler2D uBumpMap;
uniform samplerCube uEnvMap;

void main(void)
{
    // Normal mapping
    vec3 N;
    #if USE_NORMAL_MAP
        N = texture2D(uBumpMap, vBumpCoord).rgb;
        N = normalize(N * 2.0 - 1.0);
        N = normalize(vTBN * N);
    #else
        N = normalize(vTBN[2]);
    #endif

    // Reflected ray.
    vec3 E = normalize(vPosition - uViewPos);
    vec3 R = reflect(E, N);

    // Gazebo requires rotated cube map lookup.
    // R = vec3(R.x, R.z, R.y);

    // Lookup texel in the direction of the reflection ray.
    vec4 envColor = textureCube(uEnvMap, R, 0.0);

	// Cheap hdr effect:
    envColor.rgb *= (envColor.r + envColor.g + envColor.b) * uHdrMultiplier;

	// Compute refraction ratio (Fresnel):
    float facing = 1.0 - dot(-E, N);
    float refractionRatio = clamp(pow(facing, uFresnelPower), 0.0, 1.0);
    // float refractionRatio = 0.0;

    // Refracted ray only considers deep and shallow water colors:
    vec4 waterColor = mix(uShallowColor, uDeepColor, facing);

    // Perform linear interpolation between reflection and refraction.
    vec4 color = mix(waterColor, envColor, refractionRatio);
    gl_FragColor = vec4(color.xyz, 0.9);

    // @DEBUG single color
    // gl_FragColor = vec4(uShallowColor.xyz, 1.0);

    // @DEBUG normals etc.
    // gl_FragColor = vec4(vNormal, 0.9);
}
