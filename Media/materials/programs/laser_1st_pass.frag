uniform float near;
uniform float far;

varying vec4 point;

void main()
{
  float l = length(point.xyz);

  if (l>far)
    l = far;
 
  float nl = (l - near)/(far - near);

  gl_FragColor = vec4(nl, nl, nl, 1.0);
}
