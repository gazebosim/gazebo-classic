
uniform float ratio;

varying vec2 frag_pos;

void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  frag_pos = gl_Position.xy/gl_Position.w*vec2(-1.0,-1.0/ratio);
}