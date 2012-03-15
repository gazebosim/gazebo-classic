uniform sampler2D tex;

uniform vec4 texSize;

vec4 mix2( in vec4 a, in vec4 b, in float f)
{
  if (a.r == 0)
  {
    if (f < 0.5)
      return a;
    else
      return b;
  }

  if (b.r == 0)
  {
    if (f < 0.5)
      return a;
    else
      return b;
  }

  return mix(a,b,f);
}

vec4 texture2D_bilinear( in vec2 uv )
{
  vec2 f;
    
  f.x = fract( uv.x );
  f.y = fract( uv.y );
  vec4 tA, tB; 
    
  vec4 t00 = texture2D( tex, uv);
  if (uv.x < texSize.x)
  {
    vec4 t10 = texture2D( tex, uv + vec2( 1.0, 0.0 )); 
    tA = mix2( t00, t10, f.x);
  }
  else
    tA = t00;
 
  if (uv.y < (texSize.y - 1)) 
  {
    vec4 t01 = texture2D( tex, uv + vec2( 0.0, 1.0 ) );
    vec4 t11;
    if (uv.x < texSize.x)
    {   
      t11 = texture2D( tex, uv + vec2( 1.0, 1.0 ) );
      tB = mix2( t01, t11, f.x );
    }   
    else
      tB = t01;
  }
  else
    tB = tA;

  return mix2( tA, tB, f.y );
}

void main()
{
  gl_FragColor = texture2D( tex, gl_TexCoord[0].st);
  //gl_FragColor = texture2D_bilinear( gl_TexCoord[0].st);
  //gl_FragColor = vec4(gl_FragCoord.x /160, gl_FragCoord.y/120,0,1);
}
