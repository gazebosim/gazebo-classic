uniform sampler2D projMap;

void main (void)
{
  gl_FragColor = (gl_TexCoord[0].q > 0.0 ) ? 0 :  texture2DProj(projMap, gl_TexCoord[0]);
}