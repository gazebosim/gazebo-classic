uniform mat4 world_view_mat;
uniform mat4 world_view_proj_mat;
uniform mat4 inv_trans_world_view_mat;

varying vec3 vertex_world_norm;
varying vec3 vertex_world_view_pos;

void main()
{
  gl_Position = world_view_proj_mat * gl_Vertex;

  // Vertex in world space
  vertex_world_view_pos = (world_view_mat * gl_Vertex).xyz;

  // Vertex normal in world space
  vertex_world_norm = (inv_trans_world_view_mat * vec4(gl_Normal, 1.0)).xyz;

  // Pass through the diffuse component
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
