varying vec3 worldPos;
varying float depth;

void main()
{
  // Inverse size of each grid cell
  float gsize = 1.0;

  // Width of the lines
  float gwidth = 0.15;
  float gwidthM = 2.0;

  vec3 f = abs(fract(worldPos * gsize));
  vec3 df = fwidth(worldPos * gsize);

  float mi = max(0.0, gwidth - 1.0);
  float ma = max(1.0, gwidth);

  float miM = max(0.0, gwidthM - 1.0);
  float maM = max(1.0, gwidthM);

  vec3 g = clamp((f-df*mi) / (df*(ma-mi)), max(0.0, 1.0-gwidth), 1.0);
  vec3 gM = clamp((f-df*miM) / (df*(maM-miM)), max(0.0, 1.0-gwidthM), 1.0);
  float c = 1.0 - (g.x * g.y);
  float cM = 1.0 - (gM.x * gM.y);

  // The alpha value fades out the grid as it proceeds into the distance
  float alpha = depth/50.0;

  float bound = clamp(alpha*0.1, 0.005, 0.1);
  
  if (abs(worldPos.y) < bound && g.x == 1.0) 
    gl_FragColor = vec4(cM, 0, 0, 1);
  else if (abs(worldPos.x) < bound && g.y == 1.0) 
    gl_FragColor = vec4(0, cM, 0, 1);
  else
    gl_FragColor = vec4(c, c, c, 1.0 - alpha);
}
