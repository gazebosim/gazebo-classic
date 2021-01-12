#version 130

in vec4 position;

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;

uniform mat4 texViewProjMatrix0;
out vec4 lightSpacePos0;

out vec4 worldPos;
out vec4 worldViewPos;

void main()
{
  worldPos = worldMatrix * position;
  gl_Position = viewProjMatrix * worldPos;

  worldViewPos = worldViewMatrix * position;

  lightSpacePos0 = texViewProjMatrix0 * worldPos;
}

