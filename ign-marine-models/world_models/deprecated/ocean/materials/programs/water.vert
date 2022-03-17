// Attributes
attribute vec4 vertex;
attribute vec4 normal;
attribute vec4 uv0;     // tex coord
attribute vec4 uv6;     // tangent
attribute vec4 uv7;     // bitangent

// Out
varying vec3 vWorldPosition;
varying vec3 vWorldNormal;
varying mat3 vWorldTBN;
varying vec2 vTexCoord0;
varying vec2 vTexCoord1;
varying vec2 vTexCoord2;
varying vec3 vWorldToCameraDir;

// Uniform
uniform mat4 uModelViewProjection;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uITModelMatrix;
uniform vec3 uCameraPosition;

uniform vec2 uBumpScale;
uniform vec2 uBumpSpeed;
uniform float uTime;

// glslify/glsl-inverse
// https://github.com/glslify/glsl-inverse/blob/master/index.glsl
// License: MIT
mat3 inverse(mat3 m)
{
  float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
  float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2];
  float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2];

  float b01 = a22 * a11 - a12 * a21;
  float b11 = -a22 * a10 + a12 * a20;
  float b21 = a21 * a10 - a11 * a20;

  float det = a00 * b01 + a01 * b11 + a02 * b21;

  return mat3(b01, (-a22 * a01 + a02 * a21), (a12 * a01 - a02 * a11),
              b11, (a22 * a00 - a02 * a20), (-a12 * a00 + a02 * a10),
              b21, (-a21 * a00 + a01 * a20), (a11 * a00 - a01 * a10)) / det;
}

mat3 transpose(mat3 m)
{
  float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
  float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2];
  float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2];

  return mat3(
    a00, a10, a20,
    a01, a11, a21,
    a02, a12, a22
  );
} 

void main(void)
{
	vec3 tangent   = vec3(uv6);
	vec3 bitangent = vec3(uv7);

  // World space
  vec3 worldPosition = vec3(uModel * vertex);

  // Compute normal.
  vec3 normal_ = normalize(cross(tangent, bitangent));

  // Tangent, bitangent and normal vectors.
  mat3 model = mat3(uModel[0].xyz, uModel[1].xyz, uModel[2].xyz);
  mat3 transpose_inverse_model = transpose(inverse(model));
  vec3 T = normalize(transpose_inverse_model * tangent);
  vec3 B = normalize(transpose_inverse_model * bitangent);
  vec3 N = normalize(transpose_inverse_model * normal_);
  mat3 TBN = mat3(T, B, N);

  // Perturb texture coordinates for bump map.
	vec2 texCoord0 = uv0.xy * uBumpScale * 1.0 + uTime * uBumpSpeed * 1.0;
	vec2 texCoord1 = uv0.xy * uBumpScale * 1.4142135 + uTime * uBumpSpeed * 3.0;
	vec2 texCoord2 = uv0.xy * uBumpScale * 1.7320508 + uTime * uBumpSpeed * 5.0;

  // Out OpenGL
  gl_Position = uModelViewProjection * vertex;

  // Out
  vWorldPosition = worldPosition;
  vWorldNormal = N;
  vWorldTBN = TBN;
  vTexCoord0 = texCoord0;
  vTexCoord1 = texCoord1;
  vTexCoord2 = texCoord2;
  vWorldToCameraDir = uCameraPosition - vWorldPosition;
}
