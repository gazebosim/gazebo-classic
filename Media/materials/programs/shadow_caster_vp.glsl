uniform mat4 world_view_proj_mat;
uniform vec4 texel_offsets;

varying vec4 vertex_depth;

void main()
{
  gl_Position = world_view_proj_mat * gl_Vertex;
  vertex_depth = gl_Position;
  gl_Position.xy += texel_offsets.zw * gl_Position.w;
}
