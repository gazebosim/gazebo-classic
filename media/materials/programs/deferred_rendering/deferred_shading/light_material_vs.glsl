uniform mat4 worldViewProj;

void main()
{
  gl_Position = worldViewProj * gl_Vertex;
  gl_TexCoord[0] = gl_Position;
}
