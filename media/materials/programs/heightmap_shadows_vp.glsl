#version 130

// params bound by ogre
in vec4 position;
in vec2 uv0;
in vec2 uv1;

uniform mat4 worldMatrix;
uniform mat4 worldViewMatrix;
uniform mat4 viewProjMatrix;
uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;

out vec3 vsPos;
out vec4 uvMisc;
out vec4 lightSpacePos0;
out vec4 lightSpacePos1;
out vec4 lightSpacePos2;

// spot light params
uniform mat4 texViewProjMatrix3;
uniform mat4 texViewProjMatrix4;
uniform mat4 texViewProjMatrix5;
uniform mat4 texViewProjMatrix6;
uniform mat4 texViewProjMatrix7;
uniform mat4 texViewProjMatrix8;
uniform mat4 texViewProjMatrix9;
uniform mat4 texViewProjMatrix10;
uniform mat4 texViewProjMatrix11;
uniform mat4 texViewProjMatrix12;
uniform mat4 texViewProjMatrix13;

out vec4 lightSpacePos3;
out vec4 lightSpacePos4;
out vec4 lightSpacePos5;
out vec4 lightSpacePos6;
out vec4 lightSpacePos7;
out vec4 lightSpacePos8;
out vec4 lightSpacePos9;
out vec4 lightSpacePos10;
out vec4 lightSpacePos11;
out vec4 lightSpacePos12;
out vec4 lightSpacePos13;

void main()
{
  vsPos = vec3(worldViewMatrix * position);

  vec2 uv = vec2(uv0.x, uv0.y);
  vec4 worldPos = worldMatrix * position;
  gl_Position = viewProjMatrix * worldPos;
  uvMisc.xy = uv.xy;
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
  lightSpacePos11 = texViewProjMatrix11 * worldPos;
  lightSpacePos12 = texViewProjMatrix12 * worldPos;
  lightSpacePos13 = texViewProjMatrix13 * worldPos;

  // pass cam depth
  uvMisc.z = gl_Position.z;
}
