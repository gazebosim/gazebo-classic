uniform vec4 invSMSize;
uniform sampler2D map;

void main()
{
  vec3 sample = vec3(0.0, 0.0, 0.0);

  float radius = 9.0;

  for (float x = -radius; x <= radius; x += 1.0) 
  {
    for (float y = -radius; y <= radius; y += 1.0) 
    {
      sample += texture2D(map, vec2(gl_TexCoord[0].x + x * invSMSize.x, gl_TexCoord[0].y + y * invSMSize.y)).rgb;
    }
  }

  //gl_FragColor = vec4(sample / ((radius * 2.0 + 1.0) * (radius * 2.0 + 1.0)), 1.0);
  gl_FragColor = vec4(1.0,0.0,0.0,1.0);
}
