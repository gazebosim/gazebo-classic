#version 120

uniform sampler2D shadowMap0;
uniform sampler2D shadowMap1;
uniform sampler2D shadowMap2;
uniform sampler2D shadowMap3;
uniform sampler2D shadowMap4;
uniform sampler2D shadowMap5;
uniform sampler2D shadowMap6;
uniform sampler2D shadowMap7;
uniform sampler2D shadowMap8;
uniform sampler2D shadowMap9;
uniform sampler2D shadowMap10;

varying vec4 lightSpacePos0;
varying vec4 lightSpacePos1;
varying vec4 lightSpacePos2;
varying vec4 lightSpacePos3;
varying vec4 lightSpacePos4;
varying vec4 lightSpacePos5;
varying vec4 lightSpacePos6;
varying vec4 lightSpacePos7;
varying vec4 lightSpacePos8;
varying vec4 lightSpacePos9;
varying vec4 lightSpacePos10;

varying vec4 worldPos;
varying vec4 worldViewPos;

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
  float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

  return shadow;
}

void main()
{
  float f = 0.0f;

  // flat red color - no lighting
  vec4 outputCol = vec4(1.0, 0.0, 0.0, 1.0);

  // grey shadows

  // shadowMap0-2 and lightSpacePos0-2 are reserved for directional light

  // render spot light shadows
  f += ShadowSimple(shadowMap3, lightSpacePos3);
  f += ShadowSimple(shadowMap4, lightSpacePos4);
  f += ShadowSimple(shadowMap5, lightSpacePos5);
  f += ShadowSimple(shadowMap6, lightSpacePos6);
  f += ShadowSimple(shadowMap7, lightSpacePos7);
  f += ShadowSimple(shadowMap8, lightSpacePos8);
  f += ShadowSimple(shadowMap9, lightSpacePos9);
  f += ShadowSimple(shadowMap10, lightSpacePos10);

  f = clamp(f, 0.0f, 1.0f);
  if (f > 0.0f)
    outputCol = vec4(0.2, 0.2, 0.2, 1.0);

  gl_FragColor = outputCol;
}
