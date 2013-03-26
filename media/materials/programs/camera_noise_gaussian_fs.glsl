//uniform float mean;
//uniform float sigma;
uniform sampler2D RT;
void main()
{
  //gl_FragColor = vec4(1.0, 0.0, 1.0, 1.0);
  // Just pass through
  //gl_FragColor = gl_Color;
  gl_FragColor = texture2D(RT, gl_TexCoord[0].xy);
}
