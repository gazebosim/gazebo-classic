varying float depth;

void main()
{
  float f = depth;

  //f = 0.0;
  gl_FragColor = vec4(f, f, f, f);
}
