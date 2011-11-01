uniform vec4 texelOffsets;

varying float depth;

void main()
{
  gl_Position = ftransform();
  gl_Position.xy += texelOffsets.zw * gl_Position.w;
  //depth = -(gl_ModelViewMatrix * gl_Vertex).z; // ???
  depth = gl_Position.w; // copied from old gazebo
}
