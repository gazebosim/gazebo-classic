varying float tex;

void main()
{
  gl_Position = ftransform();
  tex = gl_Vertex.x;
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
