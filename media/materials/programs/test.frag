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






vec4 offsetSample(vec4 uv, vec2 offset, float invMapSize) 
{ 
  return vec4(uv.xy + offset * invMapSize * uv.w, uv.z, uv.w); 
} 
float calcDepthShadow(sampler2D shadowMap, vec4 uv, float invShadowMapSize)
{
  
  float shadow = 0.0;
  float offset = (2.0 /2 - 0.5) * 1 ;
  for (float y = -offset; y <= offset; y += 1 ) 
    for (float x = -offset; x <= offset; x += 1 ) 
    { 
      vec4 newUV = offsetSample(uv, vec2(x, y), invShadowMapSize);
      
      
      newUV = newUV / newUV.w; 
      float depth = texture(shadowMap, newUV.xy, 1.0).x; 
      if (depth >= 1 || depth >= uv.z)
        shadow += 1.0;
    } 
  shadow /= 2.0 *2.0  ; 
  return shadow; 
} 
float calcPSSMDepthShadow(
  sampler2D shadowMap0, sampler2D shadowMap1, sampler2D shadowMap2, 
  vec4 lsPos0, vec4 lsPos1, vec4 lsPos2, 
  float invShadowmapSize0, float invShadowmapSize1, float invShadowmapSize2, 
  vec4 pssmSplitPoints, float camDepth) 
{ 
  float shadow; 
  
  if (camDepth <= pssmSplitPoints.r) 
  { 
    shadow = calcDepthShadow(shadowMap0, lsPos0, invShadowmapSize0); 
  } 
  else if (camDepth <= pssmSplitPoints.g) 
  { 
    shadow = calcDepthShadow(shadowMap1, lsPos1, invShadowmapSize1); 
  } 
  else 
  { 
    shadow = calcDepthShadow(shadowMap2, lsPos2, invShadowmapSize2); 
  } 
  return shadow; 
} 


vec4 vertexPos;
vec4 position;
vec4 uvMisc;
vec4 layerUV0;
vec4 layerUV1;
uniform vec3 ambient;
uniform vec4 lightPosObjSpace;
uniform vec3 lightDiffuseColour;
uniform vec3 lightSpecularColour;
uniform vec3 eyePosObjSpace;
uniform vec4 scaleBiasSpecular;
uniform sampler2D globalNormal;
uniform sampler2D lightMap;
uniform sampler2D blendTex0;
uniform sampler2D difftex0;
uniform sampler2D normtex0;
uniform sampler2D difftex1;
uniform sampler2D normtex1;
uniform sampler2D difftex2;
uniform sampler2D normtex2;
uniform vec4 pssmSplitPoints;
vec4 lightSpacePos0;
uniform sampler2D shadowMap0;
uniform float inverseShadowmapSize0;
vec4 lightSpacePos1;
uniform sampler2D shadowMap1;
uniform float inverseShadowmapSize1;
vec4 lightSpacePos2;
uniform sampler2D shadowMap2;
uniform float inverseShadowmapSize2;
void main(){
  vec4 outputCol;
  float shadow = 1.0;
  vec2 uv = uvMisc.xy;
  outputCol = vec4(0,0,0,1);
  vec3 normal = expand(texture(globalNormal, uv)).rgb;
  vec3 lightDir = 
    lightPosObjSpace.xyz -  (position.xyz * lightPosObjSpace.w);
  vec3 eyeDir = eyePosObjSpace - position.xyz;
  vec3 diffuse = vec3(0,0,0);
  float specular = 0;
  vec4 blendTexVal0 = texture(blendTex0, uv);
  vec3 tangent = vec3(1, 0, 0);
  vec3 binormal = normalize(cross(tangent, normal));
  tangent = normalize(cross(normal, binormal));
  mat3 TBN = mat3(tangent, binormal, normal);
  vec4 litRes, litResLayer;
  vec3 TSlightDir, TSeyeDir, TShalfAngle, TSnormal;
  float displacement;
  TSlightDir = normalize(TBN * lightDir);
  TSeyeDir = normalize(TBN * eyeDir);
  vec2 uv0 = layerUV0.xy;
  displacement = texture(normtex0, uv0).a
    * scaleBiasSpecular.x + scaleBiasSpecular.y;
  uv0 += TSeyeDir.xy * displacement;
  TSnormal = expand(texture(normtex0, uv0)).rgb;
  TShalfAngle = normalize(TSlightDir + TSeyeDir);
  litResLayer = lit(dot(TSlightDir, TSnormal), dot(TShalfAngle, TSnormal), scaleBiasSpecular.z);
  litRes = litResLayer;
  vec4 diffuseSpecTex0 = texture(difftex0, uv0);
  diffuse = diffuseSpecTex0.rgb;
  specular = diffuseSpecTex0.a;
  vec2 uv1 = layerUV0.zw;
  displacement = texture(normtex1, uv1).a
    * scaleBiasSpecular.x + scaleBiasSpecular.y;
  uv1 += TSeyeDir.xy * displacement;
  TSnormal = expand(texture(normtex1, uv1)).rgb;
  TShalfAngle = normalize(TSlightDir + TSeyeDir);
  litResLayer = lit(dot(TSlightDir, TSnormal), dot(TShalfAngle, TSnormal), scaleBiasSpecular.z);
litRes = litRes + blendTexVal0.r * (litResLayer-litRes);
  vec4 diffuseSpecTex1 = texture(difftex1, uv1);
  diffuse = diffuse + blendTexVal0.r * (diffuseSpecTex1.rgb - diffuse);
  specular = specular + blendTexVal0.r * (diffuseSpecTex1.a - diffuse);
  vec2 uv2 = layerUV1.xy;
  displacement = texture(normtex2, uv2).a
    * scaleBiasSpecular.x + scaleBiasSpecular.y;
  uv2 += TSeyeDir.xy * displacement;
  TSnormal = expand(texture(normtex2, uv2)).rgb;
  TShalfAngle = normalize(TSlightDir + TSeyeDir);
  litResLayer = lit(dot(TSlightDir, TSnormal), dot(TShalfAngle, TSnormal), scaleBiasSpecular.z);
litRes = litRes + blendTexVal0.g * (litResLayer-litRes);
  vec4 diffuseSpecTex2 = texture(difftex2, uv2);
  diffuse = diffuse + blendTexVal0.g * (diffuseSpecTex2.rgb - diffuse);
  specular = specular + blendTexVal0.g * (diffuseSpecTex2.a - diffuse);
  shadow = texture(lightMap, uv).r;
  float camDepth = uvMisc.z;
  float rtshadow = calcPSSMDepthShadow(shadowMap0, shadowMap1, shadowMap2, 
    lightSpacePos0, lightSpacePos1, lightSpacePos2, 
    inverseShadowmapSize0, inverseShadowmapSize1, inverseShadowmapSize2, 
    pssmSplitPoints, camDepth);
  shadow = min(shadow, rtshadow);
  outputCol.rgb += ambient * diffuse + litRes.y * lightDiffuseColour * diffuse * shadow;
  outputCol.rgb += litRes.z * lightSpecularColour * specular * shadow;
  gl_FragColor = outputCol;
}
