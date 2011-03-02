uniform vec4 texelOffsets;
uniform float pNear;
uniform float pFar;

varying float depth;

void main()
{
  gl_Position = ftransform();
  gl_Position.xy += texelOffsets.zw * gl_Position.w;

  // depth = (gl_Position.z / (pFar - pNear) + pNear) * (pFar - pNear);
  // depth = (gl_Position.z + pNear) * (pFar - gl_Position.z);
  // depth = (gl_Position.z + pNear) * (gl_Position.z);
  depth = gl_Position.w;
}
