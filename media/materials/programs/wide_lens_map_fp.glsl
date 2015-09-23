#version 130

uniform samplerCube envMap;

uniform float cutOffAngle;

uniform float f;
uniform float c1;
uniform float c2;
uniform float c3;
uniform vec3 fun;

varying vec2 frag_pos;

float pi = 3.141592653;
float r = length(frag_pos);

vec3 map(float th)
{
  return vec3(-sin(th)*frag_pos.x/r, sin(th)*frag_pos.y/r, cos(th));
}

void main()
{
  float param = r/(c1*f);
  float theta = fun.x*asin(param)+fun.y*atan(param)+fun.z*param;

  theta = (theta-c3)*c2;

  vec3 tc = map(theta);
  gl_FragColor = vec4(textureCube(envMap, tc).rgb, 1);

  //TODO: move to vertex shader
  float param2 = cutOffAngle/c2+c3;
  float cutRadius = c1*f*(fun.x*sin(param2)+fun.y*tan(param2)+fun.z*param2);

  // gl_FragColor.rgb *= 1.0-step(cutRadius,r);
  gl_FragColor.rgb *= 1.0-smoothstep(cutRadius-0.02,cutRadius,r);
}
