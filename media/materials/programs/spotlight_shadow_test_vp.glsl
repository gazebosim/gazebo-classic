#version 130


in vec4 position;

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;

uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;
uniform mat4 texViewProjMatrix3;

out vec4 lightSpacePos0;
out vec4 lightSpacePos1;
out vec4 lightSpacePos2;
out vec4 lightSpacePos3;

out vec4 worldPos;
out vec4 worldViewPos;


void main()
{
  worldPos = worldMatrix * position;
  gl_Position = viewProjMatrix * worldPos;

  worldViewPos = worldViewMatrix * position;

  lightSpacePos0 = texViewProjMatrix0 * worldPos;
  lightSpacePos1 = texViewProjMatrix1 * worldPos;
  lightSpacePos2 = texViewProjMatrix2 * worldPos;
  lightSpacePos3 = texViewProjMatrix3 * worldPos;
}

