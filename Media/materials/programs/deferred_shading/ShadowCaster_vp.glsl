uniform mat4 cWorldViewProj;
uniform mat4 cWorldView;

void main()
{
  gl_Position = cWorldViewProj * gl_Vertex;
  gl_TexCoord[0].xyz = (cWorldView * gl_Vertex).xyz;
}
