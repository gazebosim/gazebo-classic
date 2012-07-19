uniform float cFarDistance;

void main()
{
  float depth = length(gl_TexCoord[0].xyz) / cFarDistance;
  gl_FragColor = vec4(depth, depth, depth, 1.0);
}
