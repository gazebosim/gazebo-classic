uniform mat4 world_view_proj_mat;
uniform vec4 texel_offsets;

varying vec4 vertex_depth;

void main()
{
  vertex_depth = world_view_proj_mat * gl_Vertex;
  gl_Position = vertex_depth;
  gl_Position.xy += texel_offsets.zw * gl_Position.w;
}
