#version 130

// params bound by ogre
in vec4 position;
in vec2 uv0;
in vec2 uv1;

uniform mat4 worldMatrix;
uniform mat4 viewProjMatrix;
uniform vec2 lodMorph;
uniform mat4 texViewProjMatrix0;
uniform mat4 texViewProjMatrix1;
uniform mat4 texViewProjMatrix2;

out vec4 uvMisc;
out vec4 lightSpacePos0;
out vec4 lightSpacePos1;
out vec4 lightSpacePos2;

void main()
{
  // calc pos after terrain lod morphing
  vec2 uv = vec2(uv0.x, uv0.y);
  vec4 worldPos = worldMatrix * position;
  float toMorph = -min(0.0, sign(uv1.y - lodMorph.y));
  worldPos.z += uv1.x * toMorph * lodMorph.x;
  gl_Position = viewProjMatrix * worldPos;

  // pass the uv coord
  uvMisc.xy = uv.xy;

  // shadows light space pos.
  lightSpacePos0 = texViewProjMatrix0 * worldPos;
  lightSpacePos1 = texViewProjMatrix1 * worldPos;
  lightSpacePos2 = texViewProjMatrix2 * worldPos;

  // pass cam depth
  uvMisc.z = gl_Position.z;
}
