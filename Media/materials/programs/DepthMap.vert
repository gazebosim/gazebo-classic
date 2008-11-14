uniform vec4 texelOffsets;
uniform float pNear;
uniform float pFar;

varying float depth;

void main()
{
	gl_Position = ftransform();
  gl_Position.xy += texelOffsets.zw * gl_Position.w;

  //depth = gl_Position.z / (pFar - pNear);
  depth = (gl_Position.z - pNear) / (pFar - pNear);
}
