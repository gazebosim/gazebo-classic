#version 130

uniform sampler2D shadowMap0;
in vec4 lightSpacePos0;

in vec4 worldPos;
in vec4 worldViewPos;

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
  float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

  return shadow;
}

void main()
{
  float f = 0.0f;

  // flat red color - no lighting
  outputCol = vec4(1.0, 0.0, 0.0, 1.0);

  // grey shadows
  f += ShadowSimple(shadowMap0, lightSpacePos0);
  f = clamp(f, 0.0f, 1.0f);
  if (f > 0.0f)
    outputCol = vec4(0.2, 0.2, 0.2, 1.0);
}
