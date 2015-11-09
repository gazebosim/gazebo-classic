#version 130

// aspect ratio
uniform float ratio;

varying vec2 frag_pos;

void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  // get normalized fragment coordinate (3D to 2D window space transformation)
  frag_pos = gl_Position.xy/gl_Position.w*vec2(-1.0,-1.0/ratio);
}
