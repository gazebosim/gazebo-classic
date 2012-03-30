uniform float retro;

uniform float near;
uniform float far;

varying vec4 point;

void main()
{
  //vec3 p = vec3(point.x, point.y, point.z - near);
  float l = length(point.xyz);

  if (l>far)
    l = far;
  
  float nl = l / far;
 
  gl_FragColor = vec4(nl, nl, nl, 0.0);
}
