// Model Level Inputs
uniform mat4 worldViewProj;

// Vertex Inputs
//in vec4 position;
//in vec2 texCoord0;
 
// Outputs
// out vec4 oPosition;
// out vec2 uv0;

void main()
{
  // Calculate output position
  //oPosition = worldViewProj * position;
  gl_Position = worldViewProj * gl_Vertex;
 
  // Simply copy the input vertex UV to the output
  //uv0 = texCoord0;
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
