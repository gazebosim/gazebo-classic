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
      float depth = textureGrad(shadowMap, newUV.xy,  vec2(1.0, 1.0), vec2(1.0, 1.0)).x;
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

in vec3 vsPos;
in vec4 uvMisc;

// not used for now
uniform vec3 ambient;
uniform vec3 lightDiffuseColour;
uniform vec3 lightSpecularColour;

uniform vec4 pssmSplitPoints;
uniform sampler2D shadowMap0;
uniform sampler2D shadowMap1;
uniform sampler2D shadowMap2;

uniform float inverseShadowmapSize0;
uniform float inverseShadowmapSize1;
uniform float inverseShadowmapSize2;

in vec4 lightSpacePos0;
in vec4 lightSpacePos1;
in vec4 lightSpacePos2;

// spot light params
uniform vec4 vsSpotLightPos0;
uniform vec4 vsSpotLightPos1;
uniform vec4 vsSpotLightPos2;
uniform vec4 vsSpotLightPos3;
uniform vec4 vsSpotLightPos4;
uniform vec4 vsSpotLightPos5;
uniform vec4 vsSpotLightPos6;
uniform vec4 vsSpotLightPos7;
uniform vec4 vsSpotLightPos8;
uniform vec4 vsSpotLightPos9;
uniform vec4 vsSpotLightPos10;
uniform vec4 vsSpotLightDir0;
uniform vec4 vsSpotLightDir1;
uniform vec4 vsSpotLightDir2;
uniform vec4 vsSpotLightDir3;
uniform vec4 vsSpotLightDir4;
uniform vec4 vsSpotLightDir5;
uniform vec4 vsSpotLightDir6;
uniform vec4 vsSpotLightDir7;
uniform vec4 vsSpotLightDir8;
uniform vec4 vsSpotLightDir9;
uniform vec4 vsSpotLightDir10;
uniform vec4 spotLightColor0;
uniform vec4 spotLightAtten0;
uniform vec4 spotLightParams0;

uniform sampler2D shadowMap3;
uniform sampler2D shadowMap4;
uniform sampler2D shadowMap5;
uniform sampler2D shadowMap6;
uniform sampler2D shadowMap7;
uniform sampler2D shadowMap8;
uniform sampler2D shadowMap9;
uniform sampler2D shadowMap10;
uniform sampler2D shadowMap11;
uniform sampler2D shadowMap12;
uniform sampler2D shadowMap13;

in vec4 lightSpacePos3;
in vec4 lightSpacePos4;
in vec4 lightSpacePos5;
in vec4 lightSpacePos6;
in vec4 lightSpacePos7;
in vec4 lightSpacePos8;
in vec4 lightSpacePos9;
in vec4 lightSpacePos10;
in vec4 lightSpacePos11;
in vec4 lightSpacePos12;
in vec4 lightSpacePos13;

out vec4 outputCol;

//------------------------------------------------------------------------------
float ShadowSimple(in sampler2D shadowMap, in vec4 shadowMapPos)
{
  // perform perspective divide
  vec3 shadowMapUV = shadowMapPos.xyz / shadowMapPos.w;

  if (shadowMapUV.z < 0.0 || shadowMapUV.z > 1.0)
    return 0.0;

  // get closest depth value from light's perspective
  float closestDepth = texture2D(shadowMap, shadowMapUV.xy).r;

  // get depth of current fragment from light's perspective
  float currentDepth = shadowMapUV.z;

  // check whether current frag pos is in shadow
  float shadow = currentDepth < closestDepth  ? 1.0 : 0.0;

  return shadow;
}

/**
 * vsVecToLight must not be normalized.
 * vsNegLightDir must be normalized.
 */
vec3 spotlight(in vec3 vsVecToLight,
               in vec3 vsNegLightDir,
               in vec4 attenParams,
               in vec3 spotParams,
               in vec3 color,
               in sampler2D shadowMap, in vec4 shadowMapPos)
{
  float lightD = length(vsVecToLight);
  vec3 vsVecToLightNorm = vsVecToLight / lightD;
  vec3 vsNegLightDirNorm = normalize(vsNegLightDir);

  // For realism, we are only using squared component in attenuation.
  float atten = 1.0 / (/*attenParams.y + attenParams.z * lightD +*/ attenParams.w * lightD * lightD);

  // Even though we are projecting textures, we use this spot cone calculation
  // to avoid artifacts to the side of the light
  float rho = dot(vsNegLightDirNorm, vsVecToLightNorm);
  float spotT = clamp((rho - spotParams.y) / (spotParams.x - spotParams.y), 0.0, 1.0);
  // We don't need a falloff exponent for this simulation
  //float spotT = pow(spotT, spotParams.z);

  float shadow = ShadowSimple(shadowMap, shadowMapPos);

  // Attenuation and spot cone get baked into final light color. This is how
  // spotlights get generalized so they can be stored in lights array.
  return max(color * /*atten * */spotT * shadow, vec3(0.0, 0.0, 0.0));
}

void main()
{
  float camDepth = uvMisc.z;
  float rtshadow = calcPSSMDepthShadow(shadowMap0, shadowMap1, shadowMap2,
    lightSpacePos0, lightSpacePos1, lightSpacePos2,
    inverseShadowmapSize0, inverseShadowmapSize1, inverseShadowmapSize2,
    pssmSplitPoints, camDepth);

  // light
  vec3 diffuse = vec3(0.5 * rtshadow);

  //f += ShadowSimple(shadowMap0, lightSpacePos3);
  diffuse += spotlight(vsSpotLightPos0.xyz - vsPos, -vsSpotLightDir0.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap3, lightSpacePos3);
  diffuse += spotlight(vsSpotLightPos1.xyz - vsPos, -vsSpotLightDir1.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap4, lightSpacePos4);
  diffuse += spotlight(vsSpotLightPos2.xyz - vsPos, -vsSpotLightDir2.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap5, lightSpacePos5);
  diffuse += spotlight(vsSpotLightPos3.xyz - vsPos, -vsSpotLightDir3.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap6, lightSpacePos6);
  diffuse += spotlight(vsSpotLightPos4.xyz - vsPos, -vsSpotLightDir4.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap7, lightSpacePos7);
  diffuse += spotlight(vsSpotLightPos5.xyz - vsPos, -vsSpotLightDir5.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap8, lightSpacePos8);
  diffuse += spotlight(vsSpotLightPos6.xyz - vsPos, -vsSpotLightDir6.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap9, lightSpacePos9);
  diffuse += spotlight(vsSpotLightPos7.xyz - vsPos, -vsSpotLightDir7.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap10, lightSpacePos10);
  diffuse += spotlight(vsSpotLightPos8.xyz - vsPos, -vsSpotLightDir8.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap11, lightSpacePos11);
  diffuse += spotlight(vsSpotLightPos9.xyz - vsPos, -vsSpotLightDir9.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap12, lightSpacePos12);
  diffuse += spotlight(vsSpotLightPos10.xyz - vsPos, -vsSpotLightDir10.xyz, spotLightAtten0,
                    spotLightParams0.xyz, spotLightColor0.rgb, shadowMap13, lightSpacePos13);

  outputCol.xyz = diffuse;
}
