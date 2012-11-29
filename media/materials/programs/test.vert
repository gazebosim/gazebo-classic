#version 130

vec2 posIndex;
float height;
vec2 delta;
uniform mat4 worldMatrix;
uniform mat4 viewProjMatrix;
uniform vec2 lodMorph;
uniform mat4  posIndexToObjectSpace;
uniform float baseUVScale;
uniform vec4 uvMul_0;
out vec4 oPos;
out vec4 oPosObj;
out vec4 oUVMisc;
out vec4 oUV0;
out vec4 oUV1;
out vec4 oLightSpacePos0;
uniform mat4 texViewProjMatrix0;
out vec4 oLightSpacePos1;
uniform mat4 texViewProjMatrix1;
out vec4 oLightSpacePos2;
uniform mat4 texViewProjMatrix2;
void main()
{
   vec4 pos;
   pos = posIndexToObjectSpace * vec4(posIndex, height, 1);
   vec2 uv = vec2(posIndex.x * baseUVScale, 1.0 - (posIndex.y * baseUVScale));
   vec4 worldPos = worldMatrix * pos;
   oPosObj = pos;
   float toMorph = -min(0, sign(delta.y - lodMorph.y));
   worldPos.z += delta.x * toMorph * lodMorph.x;
   oUV0.xy =  uv.xy * uvMul_0.r;
   oUV0.zw =  uv.xy * uvMul_0.g;
   oUV1.xy =  uv.xy * uvMul_0.b;
   oUV1.zw =  uv.xy * uvMul_0.a;
   oPos = viewProjMatrix * worldPos;
   oUVMisc.xy = uv.xy;
   oLightSpacePos0 = texViewProjMatrix0 * worldPos; 
   oLightSpacePos1 = texViewProjMatrix1 * worldPos; 
   oLightSpacePos2 = texViewProjMatrix2 * worldPos; 
   
   oUVMisc.z = oPos.z;
}

