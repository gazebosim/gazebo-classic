#version 130

uniform sampler2D shadowMap0;
uniform sampler2D shadowMap1;
uniform sampler2D shadowMap2;
uniform sampler2D shadowMap3;

uniform float inverseShadowmapSize0;
uniform float inverseShadowmapSize1;
uniform float inverseShadowmapSize2;
uniform float inverseShadowmapSize3;

in vec4 lightSpacePos0;
in vec4 lightSpacePos1;
in vec4 lightSpacePos2;
in vec4 lightSpacePos3;

in vec4 worldPos;
in vec4 worldViewPos;

out vec4 outputCol;


//------------------------------------------------------------------------------
// A Simple blur function
vec4 Blur(sampler2D map, vec2 uv, const in vec2 offset, float steps, float adjust)
{
  float stepSize = offset.x;
  uv.xy -= vec2(stepSize * steps);

  vec4 total = vec4(0.0, 0.0, 0.0, 0.0);
  for (float x = 0.0; x < steps; x+=1.0)
    for (float y = 0.0; y < steps; y+=1.0)
      total +=
        texture2D(map, vec2(uv.xy + vec2(x * stepSize, y * stepSize))) + adjust;

  return total / (steps * steps);
}

//------------------------------------------------------------------------------
// Calculate the shadow factor
float ShadowPCF(in sampler2D shadow_map, in vec4 shadow_map_pos,
                const in vec2 offset)
{
  // Old depth calc, using linear distance
  //float depth = (vertex_light_pos.z - shadow_depth_range.x) *
  //              shadow_depth_range.w;

  // Get the shadow map position
  shadow_map_pos = shadow_map_pos / shadow_map_pos.w;
  vec2 uv = shadow_map_pos.xy;

  float depth_adjust = 0.000001;
  vec2 c = Blur(shadow_map, uv, offset, 2.0, depth_adjust).xy;

  // standard variance shadow mapping code
  float variance = min(max( c.y - (c.x * c.x), 0.0), 1.0);
  float m_d = c.x - shadow_map_pos.z;
  float p = variance / (variance + m_d * m_d);

  return smoothstep(0.4, 1.0, shadow_map_pos.z <= c.x ? 1.0 : p);
}



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

  vec4 color = vec4(1.0, 0.0, 0.0, 1.0);
  // float shadowFactor = ShadowPCF(shadowMap3, lightSpacePos3, vec2(inverseShadowmapSize3));


  float shadowFactor = ShadowSimple(shadowMap3, lightSpacePos3, vec2(inverseShadowmapSize3));
  // float shadowFactor = ShadowSimple(shadowMap0, lightSpacePos0, vec2(inverseShadowmapSize0));

  outputCol = color * (1.0-shadowFactor);


//   vec4 color = ambient * gl_LightSource[0].ambient;
// 
//   // normalize both input vectors
//   vec3 n = normalize(normal);
//   vec3 e = normalize(-position);
// 
//   vec3 lightDir = normalize(vec3(gl_LightSource[0].position));
//   float NdotL = max(dot(normal, lightDir), 0.0);
// 
//   // if the vertex is lit compute the specular color
//   if (NdotL> 0.0) {
//       color += gl_LightSource[0].diffuse * diffuse * NdotL;
//       // compute the half vector
//       // vec3 halfVector = normalize(lightDir + e);
//       vec3 halfVector = normalize(gl_LightSource[0].halfVector.xyz);
//       // add specular
//       float NdotH = max(dot(n, halfVector), 0.0);
//       float shininess = 1.0;
//       color += gl_LightSource[0].specular * specular * pow(NdotH, shininess);
//   }

}
