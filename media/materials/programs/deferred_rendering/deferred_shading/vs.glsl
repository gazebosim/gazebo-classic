// Post shader: Generic fullscreen quad
uniform vec3 farCorner;
uniform float flip;

void main()
{
  vec2 pos;
	// Clean up inaccuracies
  pos = sign(gl_Vertex.xy);

	gl_Position = vec4(pos, 0.0, 1.0);
	gl_Position.y *= flip;

	// Image-space
	gl_TexCoord[0].x = 0.5 * (1.0 + pos.x);
	gl_TexCoord[0].y = 0.5 * (1.0 - pos.y);

	// This ray will be interpolated and will be the ray from the camera
	// to the far clip plane, per pixel
  gl_TexCoord[1].xyz = farCorner * vec3(pos, 1.0);
}
