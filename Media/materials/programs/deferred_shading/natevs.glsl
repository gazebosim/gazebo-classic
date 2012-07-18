uniform vec3 farCorner;
uniform float flip;

void main()
{
  // Clean up inaccuracies
  gl_Position.xy = sign(gl_Vertex.xy);

  gl_Position = vec4(gl_Position.xy, 0, 1);
  gl_Position.y *= flip;

  // Image-space
  gl_TexCoord[0].x = 0.5 * (1.0 + gl_Position.x);
  gl_TexCoord[0].y = 0.5 * (1.0 - gl_Position.y);

  // This ray will be interpolated and will be the ray from the camera
  // to the far clip plane, per pixel
  gl_TexCoord[1].xyz = farCorner * vec3(gl_Position.xy, 1.0);
}
