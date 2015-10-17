#version 130

uniform samplerCube envMap;

uniform float cutOffAngle;

// focal length
uniform float f;

// linear scaling constant
uniform float c1;

// angle scaling constant
uniform float c2;

// angle offset constant
uniform float c3;

// unit axis
// depends on the type of math function sin (X), tan (Y), or identity (Z)
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
  // calculate angle from optical axis based on the mapping function specified
  float param = r/(c1*f);
  float theta = fun.x*asin(param)+fun.y*atan(param)+fun.z*param;
  theta = (theta-c3)*c2;

  // compute the direction vector that will be used to sample from the cubemap
  vec3 tc = map(theta);

  // sample and set resulting color
  gl_FragColor = vec4(textureCube(envMap, tc).rgb, 1);

  // limit to visible fov
  //TODO: move to vertex shader
  float param2 = cutOffAngle/c2+c3;
  float cutRadius = c1*f*(fun.x*sin(param2)+fun.y*tan(param2)+fun.z*param2);

  // smooth edges
  // gl_FragColor.rgb *= 1.0-step(cutRadius,r);
  gl_FragColor.rgb *= 1.0-smoothstep(cutRadius-0.02,cutRadius,r);
}
