#version 130

in vec4 position;

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;

uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;

out vec4 lightSpacePos0;
out vec4 lightSpacePos1;
out vec4 lightSpacePos2;

uniform mat4 texViewProjMatrix3;
out vec4 lightSpacePos3;

uniform mat4 texViewProjMatrix4;
out vec4 lightSpacePos4;

uniform mat4 texViewProjMatrix5;
out vec4 lightSpacePos5;

uniform mat4 texViewProjMatrix6;
out vec4 lightSpacePos6;

uniform mat4 texViewProjMatrix7;
out vec4 lightSpacePos7;

uniform mat4 texViewProjMatrix8;
out vec4 lightSpacePos8;


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
  lightSpacePos4 = texViewProjMatrix4 * worldPos;
  lightSpacePos5 = texViewProjMatrix5 * worldPos;
  lightSpacePos6 = texViewProjMatrix6 * worldPos;
  lightSpacePos7 = texViewProjMatrix7 * worldPos;
  lightSpacePos8 = texViewProjMatrix8 * worldPos;
}

