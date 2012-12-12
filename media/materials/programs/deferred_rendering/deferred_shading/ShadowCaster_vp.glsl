uniform mat4 worldViewProj;
uniform mat4 worldView;

void main()
{
  gl_Position = worldViewProj * gl_Vertex;
  gl_TexCoord[0].xyz = (worldView * gl_Vertex).xyz;
}
