#version 120

varying vec2 Texcoord;

void main( void )
{
	gl_Position = ftransform();
	Texcoord    = gl_MultiTexCoord0.xy;
}
