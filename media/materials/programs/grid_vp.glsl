varying vec3 worldPos;
varying float depth;

void main()
{
  worldPos = gl_Vertex.xyz;
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  depth = gl_Position.z;
}
