uniform float retro;

uniform float near;
uniform float far;
uniform sampler2D RT;
varying vec4 point;

void main()
{
  //vec3 p = vec3(point.x, point.y, point.z - near);
  float l = length(point.xyz);

  if (l>far)
    l = far;
    
//  l = gl_FragCoord.z / gl_FragCoord.w;
    
//    float t = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
//    gl_FragColor = vec4(l, t, far, 1.0);    
    gl_FragColor = vec4(point.x, point.y, point.z, 1.0);
//  gl_FragColor = vec4(l, retro, 0, 1.0);
//  gl_FragColor = vec4(l, far, 3, 1.0);
}
