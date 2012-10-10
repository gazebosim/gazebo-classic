#version 130

uniform mat4 worldViewProj;
uniform mat4 worldView;
uniform mat4 worldMatrix;

in vec4 iPosition;
in vec3 iNormal;
in vec2 iTexCoord;

out vec3 oViewPos;
out vec3 oNormal;
out vec3 oWorldPos;
out vec3 oProjPos;
out vec2 oTexcoord;

void main()
{
  gl_Position = worldViewProj * iPosition;

  oViewPos = (worldView * iPosition).xyz;
  oNormal = (worldMatrix * vec4(iNormal, 0.0)).xyz;
  oWorldPos = (worldMatrix * iPosition).xyz;

  //vec4 prPos = worldViewProj * iPosition;
  //prPos.xyz /= prPos.w;
  //oProjPos = prPos.xyz;
  oProjPos = (gl_Position.xyz / gl_Position.w).xyz;
  oTexcoord = iTexCoord;
}
