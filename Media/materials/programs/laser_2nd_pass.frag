uniform sampler2D tex1;
uniform sampler2D tex2;
uniform sampler2D tex3;

uniform vec4 texSize;
varying float tex;

//vec4 mix2( in vec4 a, in vec4 b, in float f)
//{
//  if (a.r == 0)
//  {
//    if (f < 0.5)
//      return a;
//    else
//      return b;
//  }
//
//  if (b.r == 0)
//  {
//    if (f < 0.5)
//      return a;
//    else
//      return b;
//  }
//
//  return mix(a,b,f);
//}
//
//vec4 texture2D_bilinear( in vec2 uv )
//{
//  vec2 f;
//    
//  f.x = fract( uv.x );
//  f.y = fract( uv.y );
//  vec4 tA, tB; 
//    
//  vec4 t00 = texture2D( tex, uv);
//  if (uv.x < texSize.x)
//  {
//    vec4 t10 = texture2D( tex, uv + vec2( 1.0, 0.0 )); 
//    tA = mix2( t00, t10, f.x);
//  }
//  else
//    tA = t00;
// 
//  if (uv.y < (texSize.y - 1)) 
//  {
//    vec4 t01 = texture2D( tex, uv + vec2( 0.0, 1.0 ) );
//    vec4 t11;
//    if (uv.x < texSize.x)
//    {   
//      t11 = texture2D( tex, uv + vec2( 1.0, 1.0 ) );
//      tB = mix2( t01, t11, f.x );
//    }   
//    else
//      tB = t01;
//  }
//  else
//    tB = tA;
//
//  return mix2( tA, tB, f.y );
//}

void main()
{
  if ((gl_TexCoord[0].s < 0) || (gl_TexCoord[0].s > 1) || 
      (gl_TexCoord[0].t < 0) || (gl_TexCoord[0].t > 1))
    gl_FragColor = vec4(1,1,1,1);
  else
  {
    int int_tex = int(tex*1000);
    if (int_tex == 0)
      //gl_FragColor=vec4(1,0,0,1);
      gl_FragColor = texture2D( tex1, gl_TexCoord[0].st);
    else 
      if (int_tex == 1)
        //gl_FragColor=vec4(0,1,0,1);
        gl_FragColor = texture2D( tex2, gl_TexCoord[0].st);
      else
        //gl_FragColor=vec4(0,0,1,1);
        gl_FragColor = texture2D( tex3, gl_TexCoord[0].st);
   }
    
    //gl_FragColor = texture2D_bilinear( gl_TexCoord[0].st);
}
