varying vec3 worldPos;
varying float depth;

void main()
{
  // Inverse size of each grid cell
  float gsize = 1.0;

  // Width of the lines
  float gwidth = 0.15;

  vec3 f = abs(fract(worldPos * gsize));
  vec3 df = fwidth(worldPos * gsize);

  float mi = max(0.0, gwidth - 1.0);
  float ma = max(1.0, gwidth);

  vec3 g = clamp((f-df*mi) / (df*(ma-mi)), max(0.0, 1.0-gwidth), 1.0);
  float c = 1.0 - (g.x * g.y);

  // The alpha value fades out the grid as it proceeds into the distance
  float alpha = depth/50.0;

  float bound = clamp(alpha*0.1, 0.005, 0.1);
  
  gl_FragColor = vec4(c, c, c, 1.0 - alpha);
}
