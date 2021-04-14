#version 120

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;

varying vec4 worldPos;
varying vec4 worldViewPos;

uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;
uniform mat4 texViewProjMatrix3;
uniform mat4 texViewProjMatrix4;
uniform mat4 texViewProjMatrix5;
uniform mat4 texViewProjMatrix6;
uniform mat4 texViewProjMatrix7;
uniform mat4 texViewProjMatrix8;
uniform mat4 texViewProjMatrix9;
uniform mat4 texViewProjMatrix10;

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

void main()
{
  worldPos = worldMatrix * gl_Vertex;
  gl_Position = viewProjMatrix * worldPos;

  worldViewPos = worldViewMatrix * gl_Vertex;

  lightSpacePos0 = texViewProjMatrix0 * worldPos;
  lightSpacePos1 = texViewProjMatrix1 * worldPos;
  lightSpacePos2 = texViewProjMatrix2 * worldPos;
  lightSpacePos3 = texViewProjMatrix3 * worldPos;
  lightSpacePos4 = texViewProjMatrix4 * worldPos;
  lightSpacePos5 = texViewProjMatrix5 * worldPos;
  lightSpacePos6 = texViewProjMatrix6 * worldPos;
  lightSpacePos7 = texViewProjMatrix7 * worldPos;
  lightSpacePos8 = texViewProjMatrix8 * worldPos;
  lightSpacePos9 = texViewProjMatrix9 * worldPos;
  lightSpacePos10 = texViewProjMatrix10 * worldPos;
}
