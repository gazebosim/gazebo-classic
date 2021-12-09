#version 130

uniform sampler2DShadow shadowMap0;
uniform sampler2DShadow shadowMap1;
uniform sampler2DShadow shadowMap2;
uniform sampler2DShadow shadowMap3;
uniform sampler2DShadow shadowMap4;
uniform sampler2DShadow shadowMap5;

varying vec4 lightSpacePos0;
varying vec4 lightSpacePos1;
varying vec4 lightSpacePos2;
varying vec4 lightSpacePos3;
varying vec4 lightSpacePos4;
varying vec4 lightSpacePos5;

varying vec4 worldPos;

//------------------------------------------------------------------------------
float ShadowSimple(in sampler2DShadow shadowMap, in vec4 shadowMapPos)
{
  // perform perspective divide
  vec3 shadowMapUV = shadowMapPos.xyz / shadowMapPos.w;

  if (shadowMapUV.z < 0.0 || shadowMapUV.z > 1.0)
    return 0.0;
  if (shadowMapUV.x < 0.0 || shadowMapUV.x > 1.0)
    return 0.0;
  if (shadowMapUV.y < 0.0 || shadowMapUV.y > 1.0)
    return 0.0;

  // get closest depth value from light's perspective
  float closestDepth = texture(shadowMap, shadowMapUV);

  // get depth of current fragment from light's perspective
  float currentDepth = shadowMapUV.z;

  // check whether current frag pos is in shadow
  float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

  return shadow;
}

void main()
{
  float f = 0.0f;

  vec4 outColor = vec4(1.0, 0.0, 0.0, 1.0);

  // grey shadows
  f += ShadowSimple(shadowMap0, lightSpacePos0);
  f += ShadowSimple(shadowMap1, lightSpacePos1);
  f += ShadowSimple(shadowMap2, lightSpacePos2);
  f += ShadowSimple(shadowMap3, lightSpacePos3);
  f += ShadowSimple(shadowMap4, lightSpacePos4);
  f += ShadowSimple(shadowMap5, lightSpacePos5);
  f = clamp(f, 0.0f, 1.0f);
  if (f > 0.0f)
    outColor = vec4(0.0, 0.0, 0.0, 1.0);

  gl_FragColor = outColor;
}
