varying vec4 point;

void main()
{
  gl_Position = ftransform();

  // Vertex in world space
   point = gl_ModelViewMatrix * gl_Vertex;     
  gl_TexCoord[0] = gl_MultiTexCoord0;  
}
