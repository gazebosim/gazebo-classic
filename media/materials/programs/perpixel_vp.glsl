uniform mat4 world_view_mat;
uniform mat4 world_view_proj_mat;

varying vec3 vertex_world_view_pos;
varying vec3 vertex_world_norm;

void main()
{
  gl_Position = world_view_proj_mat * gl_Vertex;

  // Vertex in world space
  vertex_world_view_pos = (world_view_mat * gl_Vertex).xyz;

  // Vertex normal in world space
  vertex_world_norm = normalize( gl_NormalMatrix * gl_Normal );
}
