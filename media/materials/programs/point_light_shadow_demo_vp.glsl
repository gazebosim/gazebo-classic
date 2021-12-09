#version 120

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;

uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;
uniform mat4 texViewProjMatrix3;
uniform mat4 texViewProjMatrix4;
uniform mat4 texViewProjMatrix5;
varying vec4 lightSpacePos0;
varying vec4 lightSpacePos1;
varying vec4 lightSpacePos2;
varying vec4 lightSpacePos3;
varying vec4 lightSpacePos4;
varying vec4 lightSpacePos5;

varying vec4 worldPos;

void main()
{
  worldPos = worldMatrix * gl_Vertex;
  gl_Position = viewProjMatrix * worldPos;

  lightSpacePos0 = texViewProjMatrix0 * worldPos;
  lightSpacePos1 = texViewProjMatrix1 * worldPos;
  lightSpacePos2 = texViewProjMatrix2 * worldPos;
  lightSpacePos3 = texViewProjMatrix3 * worldPos;
  lightSpacePos4 = texViewProjMatrix4 * worldPos;
  lightSpacePos5 = texViewProjMatrix5 * worldPos;
}

