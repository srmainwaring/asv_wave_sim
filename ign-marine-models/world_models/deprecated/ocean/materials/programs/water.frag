// Set to 1 to use normal maps.
#define USE_NORMAL_MAP 1

// In
varying vec3 vWorldPosition;
varying vec3 vWorldNormal;
varying mat3 vWorldTBN;

varying vec2 vTexCoord0;
varying vec2 vTexCoord1;
varying vec2 vTexCoord2;

varying vec3 vWorldToCameraDir;

// Uniform
uniform vec4 uDeepColor;
uniform vec4 uShallowColor;
uniform vec4 uReflectColor;
uniform float uReflectAmount;
uniform float uReflectBlur;
uniform float uWaterAmount;
uniform float uFresnelPower;
uniform float uFresnelBias;
uniform float uHdrMultiplier;
uniform sampler2D uNormalMap;
uniform samplerCube uEnvMap;

void main(void)
{
    // Normal mapping
    vec3 unitNormal;
    #if USE_NORMAL_MAP
        vec4 normalTex0 = texture2D(uNormalMap, vTexCoord0) * 2.0 - 1.0;
        vec4 normalTex1 = texture2D(uNormalMap, vTexCoord1) * 2.0 - 1.0;
        vec4 normalTex2 = texture2D(uNormalMap, vTexCoord2) * 2.0 - 1.0;
        vec3 normalObjSpace = normalTex0.xyz + normalTex1.xyz + normalTex2.xyz;
        vec3 normal = vWorldTBN * normalObjSpace;
        unitNormal = normalize(normal);
    #else
        unitNormal = normalize(vWorldNormal);
    #endif

    vec3 unitToCameraDir = normalize(vWorldToCameraDir);

    // Ray from camera reflected about normal.
    vec3 reflectDir = reflect(unitToCameraDir, unitNormal);

    // Gazebo coordinates are rotated, rotate R pi/2 about x...
    reflectDir = vec3(reflectDir.x, -reflectDir.z, reflectDir.y);

    // Lookup texel in the direction of the reflection ray.
    vec4 reflectEnvColor = textureCube(uEnvMap, reflectDir, 0.0);

    // HDR effect
    reflectEnvColor.rgb *= (reflectEnvColor.r + reflectEnvColor.g + reflectEnvColor.b) * uHdrMultiplier;

    // Fresnel
    float cosTheta = dot(unitNormal, unitToCameraDir);
    float reflectionFactor = clamp(uFresnelBias + pow(1.0 - cosTheta, uFresnelPower), 0.0, 1.0);

    // Refracted ray only considers deep and shallow water colors:
    vec4 waterColor = mix(uShallowColor, uDeepColor, cosTheta) * uWaterAmount;

    // Perform linear interpolation between reflection and refraction.
    reflectEnvColor = mix(waterColor, reflectEnvColor * uReflectColor, reflectionFactor) * uReflectAmount;

    vec4 finalColor = waterColor + reflectEnvColor;

    gl_FragColor = vec4(finalColor.rgb, 0.9);

    // @DEBUG single color
    // gl_FragColor = vec4(uShallowColor.xyz, 1.0);

    // @DEBUG normals etc.
    // gl_FragColor = vec4(unitNormal, 1.0);
    // gl_FragColor = vec4(vWorldTBN[0], 1.0); // tangent (R)
    // gl_FragColor = vec4(vWorldTBN[1], 1.0); // bitangent (G)
    // gl_FragColor = vec4(vWorldTBN[2], 1.0); // normal (N)
}
