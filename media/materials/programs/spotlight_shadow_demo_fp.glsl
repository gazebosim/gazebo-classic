#version 130

uniform sampler2DShadow shadowMap0;
varying vec4 lightSpacePos0;

varying vec4 worldPos;
varying vec4 worldViewPos;

//------------------------------------------------------------------------------
float ShadowSimple(in sampler2DShadow shadowMap, in vec4 shadowMapPos)
{
  // perform perspective divide
  vec3 shadowMapUV = shadowMapPos.xyz / shadowMapPos.w;

  if (shadowMapUV.z < 0.0 || shadowMapUV.z > 1.0)
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

  // flat red color - no lighting
  vec4 outputCol = vec4(1.0, 0.0, 0.0, 1.0);

  // grey shadows
  f += ShadowSimple(shadowMap0, lightSpacePos0);
  f = clamp(f, 0.0f, 1.0f);
  if (f > 0.0f)
    outputCol = vec4(0.2, 0.2, 0.2, 1.0);

  gl_FragColor = outputCol;
}
