#version 130

vec4 expand(vec4 v)
{
  return v * 2 - 1;
}

vec4 lit(float NdotL, float NdotH, float m)
{
  float specular = (NdotL > 0) ? pow(max(0.0, NdotH), m) : 0.0;
  return vec4(1.0, max(0.0, NdotL), specular, 1.0);
}
// Simple PCF
// Number of samples in one dimension (square for total samples)
#define NUM_SHADOW_SAMPLES_1D 2.0
#define SHADOW_FILTER_SCALE 1.0
#define SHADOW_SAMPLES NUM_SHADOW_SAMPLES_1D*NUM_SHADOW_SAMPLES_1D
vec4 offsetSample(vec4 uv, vec2 offset, float invMapSize)
{
  return vec4(uv.xy + offset * invMapSize * uv.w, uv.z, uv.w);
}
float calcDepthShadow(sampler2D shadowMap, vec4 uv, float invShadowMapSize)
{
  // 4-sample PCF
  float shadow = 0.0;
  float offset = (NUM_SHADOW_SAMPLES_1D/2.0 - 0.5) *SHADOW_FILTER_SCALE;
  for (float y = -offset; y <= offset; y += SHADOW_FILTER_SCALE)
    for (float x = -offset; x <= offset; x += SHADOW_FILTER_SCALE)
    {
      vec4 newUV = offsetSample(uv, vec2(x, y), invShadowMapSize);
      // manually project and assign derivatives
      // to avoid gradient issues inside loops
      newUV = newUV / newUV.w;
      float depth = textureGrad(shadowMap, newUV.xy,  vec2(1.0, 1.0),
          vec2(1.0, 1.0)).x;
      if (depth >= 1.0 || depth >= newUV.z)
        shadow += 1.0;
    }
  shadow /= (SHADOW_SAMPLES);
  return shadow;
}
float calcPSSMDepthShadow(
  sampler2D shadowMap0, sampler2D shadowMap1, sampler2D shadowMap2,
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2,
  float invShadowmapSize0, float invShadowmapSize1, float invShadowmapSize2,
  vec4 pssmSplitPoints, float camDepth)
{
  float shadow = 1.0;
  // calculate shadow
  if (camDepth <= pssmSplitPoints.x)
  {
    shadow = calcDepthShadow(shadowMap0, lsPos0, invShadowmapSize0);
  }
  else if (camDepth <= pssmSplitPoints.y)
  {
    shadow = calcDepthShadow(shadowMap1, lsPos1, invShadowmapSize1);
  }
  else
  {
    shadow = calcDepthShadow(shadowMap2, lsPos2, invShadowmapSize2);
  }
  return shadow;
}

in vec4 uvMisc;

uniform mat4 uvTransform;

uniform vec4 pssmSplitPoints;
uniform sampler2D shadowMap0;
uniform sampler2D shadowMap1;
uniform sampler2D shadowMap2;

uniform sampler2D texMap;

uniform float inverseShadowmapSize0;
uniform float inverseShadowmapSize1;
uniform float inverseShadowmapSize2;

in vec4 lightSpacePos0;
in vec4 lightSpacePos1;
in vec4 lightSpacePos2;

out vec4 outputCol;

void main()
{
  float shadow = 1.0;

 // transform uv coordinates in case the terrain's been partitioned
  vec2 newUV = (uvTransform * vec4(uvMisc.xy, 0.0f, 1.0f)).xy;

  // sample the texture map
  vec4 texSample  = texture2D(texMap, newUV);

  // real time shadows
  float camDepth = uvMisc.z;
  float rtshadow = calcPSSMDepthShadow(shadowMap0, shadowMap1, shadowMap2,
    lightSpacePos0, lightSpacePos1, lightSpacePos2,
    inverseShadowmapSize0, inverseShadowmapSize1, inverseShadowmapSize2,
    pssmSplitPoints, camDepth);

  // red terrain color
  vec3 diffuse = vec3(1.0, 0.0, 0.0) * texSample.xyz;

  // apply shadows
  outputCol.xyz = diffuse * rtshadow;
}
