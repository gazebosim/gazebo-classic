uniform vec4 camera_position;
uniform mat4 world_matrix;

// uniform vec4 depth_range;
varying vec4 vertex_depth;

void main()
{
  float depth = (vertex_depth.z) / vertex_depth.w;

  // Linear
  // float depth = (vertex_depth.z - depth_range.x) / depth_range.w;

  vec4 magicVector1 = vec4(1.0, 0.0, 0.0, 0.0);
  vec4 magicVector2 = vec4(0.0, 1.0, 0.0, 0.0);
  vec4 magicVector3 = vec4(0.0, 0.0, 1.0, 0.0);

  bool matchingVectors = (magicVector1 == world_matrix[0] && magicVector2 == world_matrix[1] && magicVector3 == world_matrix[2]);

  if (length(camera_position.xyz) == 0 && matchingVectors) {
    discard;
  }

  gl_FragColor = vec4(depth, depth, depth, 1.0);
}
