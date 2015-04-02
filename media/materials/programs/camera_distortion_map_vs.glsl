// Simple vertex shader; just setting things up for the real work to be done in
// camera_distortion_fs.glsl.
void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
