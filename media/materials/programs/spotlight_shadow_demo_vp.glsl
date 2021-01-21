#version 120

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;

uniform mat4 texViewProjMatrix0;
varying vec4 lightSpacePos0;

varying vec4 worldPos;
varying vec4 worldViewPos;

void main()
{
  worldPos = worldMatrix * gl_Vertex;
  gl_Position = viewProjMatrix * worldPos;

  worldViewPos = worldViewMatrix * gl_Vertex;

  lightSpacePos0 = texViewProjMatrix0 * worldPos;
}

