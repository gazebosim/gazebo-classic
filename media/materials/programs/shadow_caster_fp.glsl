uniform vec4 depth_range;
varying vec4 vertex_depth;

void main()
{
  float depth = vertex_depth.z / vertex_depth.w;

  // Linear
  // float depth = (vertex_depth.z - 0.10002) / 3000.0;

  gl_FragColor = vec4(depth, depth, depth, 1.0);
}
