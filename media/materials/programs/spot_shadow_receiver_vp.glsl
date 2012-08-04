uniform mat4 world_view_mat;
uniform mat4 world_mat;
uniform mat4 world_view_proj_mat;
uniform mat4 inv_trans_world_view_mat;
uniform mat4 tex_world_view_proj_mat;

varying vec3 vertex_world_view_pos;
varying vec3 vertex_world_pos;
varying vec3 vertex_world_norm;
varying vec4 vertex_light_pos;

void main()
{
  gl_Position = world_view_proj_mat * gl_Vertex;

  // Vertex in world view space
  vertex_world_view_pos = (world_view_mat * gl_Vertex).xyz;

  // Vertex in world space
  vertex_world_pos = (world_mat * gl_Vertex).xyz;

  // Vertex normal in world space
  //vertex_world_norm = (inv_trans_world_view_mat * vec4(gl_Normal, 1.0)).xyz;
  vertex_world_norm = normalize( gl_NormalMatrix * gl_Normal );

  // Position of the vertex in light space (shadow map texture coords)
  vertex_light_pos = tex_world_view_proj_mat * gl_Vertex;

  // Pass through the diffuse component
  //gl_TexCoord[0] = gl_MultiTexCoord0;

  //gl_FrontColor = gl_Color;
}
