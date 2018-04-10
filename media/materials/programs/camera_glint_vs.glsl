// Simple vertex shader; just setting things up for the real work to be done in 
// camera_glint_fs.glsl.

varying vec4 point;
varying vec3 normal;


void main() 
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  gl_TexCoord[0] = gl_MultiTexCoord0;

  point = gl_ModelViewMatrix * gl_Vertex;
  normal = gl_NormalMatrix * gl_Normal;
}
