uniform sampler2D tex1;
uniform sampler2D tex2;
uniform sampler2D tex3;

uniform vec4 texSize;
varying float tex;

void main()
{
  if ((gl_TexCoord[0].s < 0.0) || (gl_TexCoord[0].s > 1.0) || 
      (gl_TexCoord[0].t < 0.0) || (gl_TexCoord[0].t > 1.0))
    gl_FragColor = vec4(1,1,1,1);
  else
  {
    int int_tex = int(tex * 1000.0);
    if (int_tex == 0)
      //gl_FragColor=vec4(1,0,0,1);
      gl_FragColor = texture2D( tex1, gl_TexCoord[0].st);
    else 
      if (int_tex == 1)
        //gl_FragColor=vec4(2,1,0,1);
        gl_FragColor = texture2D( tex2, gl_TexCoord[0].st);
      else
        //gl_FragColor=vec4(3,2,1,1);
        gl_FragColor = texture2D( tex3, gl_TexCoord[0].st);
   }
}
