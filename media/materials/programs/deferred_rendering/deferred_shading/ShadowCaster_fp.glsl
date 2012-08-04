uniform float farDistance;

void main()
{
  float depth = length(gl_TexCoord[0].xyz) / farDistance;
  gl_FragColor = vec4(depth, depth, depth, 0.0);
}
