uniform mat4 world_view_mat;
uniform mat4 world_view_proj_mat;
uniform mat4 inv_trans_world_view_mat;

uniform mat4 tex_world_view_proj_mat_0;
uniform mat4 tex_world_view_proj_mat_1;
uniform mat4 tex_world_view_proj_mat_2;

varying vec3 vertex_world_norm;
varying vec3 vertex_world_view_pos;
varying vec4 vertex_light_pos_0;
varying vec4 vertex_light_pos_1;
varying vec4 vertex_light_pos_2;
varying float shadow_distance;

void main()
{
  gl_Position = world_view_proj_mat * gl_Vertex;

  // Vertex in world space
  vertex_world_view_pos = (world_view_mat * gl_Vertex).xyz;

  // Vertex normal in world space
  vertex_world_norm = (inv_trans_world_view_mat * vec4(gl_Normal, 1.0)).xyz;

  shadow_distance = gl_Position.z;

  // Position of the vertex in light space (shadow map texture coords)
  vertex_light_pos_0 = tex_world_view_proj_mat_0 * gl_Vertex;
  vertex_light_pos_1 = tex_world_view_proj_mat_1 * gl_Vertex;
  vertex_light_pos_2 = tex_world_view_proj_mat_2 * gl_Vertex;

  // Pass through the diffuse component
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
