uniform vec3 farCorner;
uniform mat4 wvp;

void main()
{
  gl_Position = wvp * gl_Vertex;

  // clean up inaccuracies for the UV coords
  vec2 uv = sign(gl_Vertex.xy);

  // convert to image space
  uv = (vec2(uv.x, -uv.y) + 1.0) * 0.5;
  gl_TexCoord[0].xy = uv;

  // calculate the correct ray
  // (modify XY parameters based on screen-space quad XY)
  gl_TexCoord[1].xyz = farCorner * vec3(sign(gl_Vertex.xy), 1.0);
}
