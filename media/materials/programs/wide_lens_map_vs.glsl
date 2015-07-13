
varying vec2 frag_pos;

void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  frag_pos = gl_Position.xy;
}