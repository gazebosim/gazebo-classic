#version 130

uniform sampler2D shadowMap0;
uniform sampler2D shadowMap1;
uniform sampler2D shadowMap2;

uniform float inverseShadowmapSize0;
uniform float inverseShadowmapSize1;
uniform float inverseShadowmapSize2;

in vec4 lightSpacePos0;
in vec4 lightSpacePos1;
in vec4 lightSpacePos2;

uniform sampler2D shadowMap3;
uniform float inverseShadowmapSize3;
in vec4 lightSpacePos3;

uniform sampler2D shadowMap4;
uniform float inverseShadowmapSize4;
in vec4 lightSpacePos4;

uniform sampler2D shadowMap5;
uniform float inverseShadowmapSize5;
in vec4 lightSpacePos5;

uniform sampler2D shadowMap6;
uniform float inverseShadowmapSize6;
in vec4 lightSpacePos6;

uniform sampler2D shadowMap7;
uniform float inverseShadowmapSize7;
in vec4 lightSpacePos7;

uniform sampler2D shadowMap8;
uniform float inverseShadowmapSize8;
in vec4 lightSpacePos8;


in vec4 worldPos;
in vec4 worldViewPos;

out vec4 outputCol;


//------------------------------------------------------------------------------
float ShadowSimple(in sampler2D shadowMap, in vec4 shadowMapPos,
                const in vec2 offset)
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
  float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

  return shadow;
}


void main()
{
  // plane color - red
  vec4 color = vec4(1.0, 0.0, 0.0, 1.0);

  float f = 0.0f;
//  f += ShadowSimple(shadowMap3, lightSpacePos3, vec2(inverseShadowmapSize3));
//  f = clamp(f, 0.0f, 1.0f);
//  f += ShadowSimple(shadowMap4, lightSpacePos4, vec2(inverseShadowmapSize4));
//  f = clamp(f, 0.0f, 1.0f);
  f += ShadowSimple(shadowMap5, lightSpacePos5, vec2(inverseShadowmapSize5));
//  f = clamp(f, 0.0f, 1.0f);
//  f += ShadowSimple(shadowMap6, lightSpacePos6, vec2(inverseShadowmapSize6));
//  f = clamp(f, 0.0f, 1.0f);
//  f += ShadowSimple(shadowMap7, lightSpacePos7, vec2(inverseShadowmapSize7));
//  f = clamp(f, 0.0f, 1.0f);
//  f += ShadowSimple(shadowMap8, lightSpacePos8, vec2(inverseShadowmapSize8));
//  f = clamp(f, 0.0f, 1.0f);

  float shadowFactor = f;
//  outputCol = color * (1.0-shadowFactor);

  if (abs(length(lightSpacePos3 - lightSpacePos7)) < 0.0001)
    outputCol = vec4(1.0, 0.0, 0.0, 1.0);
  else
    outputCol = vec4(0.0, 1.0, 0.0, 1.0);
}
