#version 120

attribute vec4 vertex;
attribute vec3 normal;

uniform mat4 cWorldViewProj;
uniform mat4 cWorldView;

varying vec3 oViewPos;
varying vec3 oNormal;

void main()
{
  // transform the vertex position to the view space
  oViewPos = (cWorldView * vertex).xyz;
  // transform the vertex normal to view space
  oNormal = (cWorldView * vec4(normal, 0)).xyz;
  gl_Position = cWorldViewProj * vertex;
}
