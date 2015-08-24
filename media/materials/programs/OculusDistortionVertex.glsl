#version 120

// Values automatically defined by Ogre/OpenGL:
attribute vec4 vertex;
attribute vec2 uv0;			// Red channel
attribute vec2 uv1;			// Green channel
attribute vec2 uv2;			// Blue channel
attribute vec4 colour;			// Vertex Colour

// Load in values defined in the material:
uniform mat4 worldViewProj;
uniform vec2 eyeToSourceUVScale;
uniform vec2 eyeToSourceUVOffset;
uniform mat4 eyeRotationStart;
uniform mat4 eyeRotationEnd;

varying vec4 gl_FrontColor;

vec2 timewarpTexCoord( vec2 texCoord, mat4 rotMat )
{
	vec3 transformed =  (rotMat * vec4( texCoord.xy, 1, 1) ).xyz;
	
	vec2 flattened = transformed.xy / transformed.z;
	
	return eyeToSourceUVScale * flattened + eyeToSourceUVOffset;
}

void main(void)
{
	/*float timewarpLerpFactor = 0.0;
	mat4 lerpedEyeRot = eyeRotationStart * (1 - timewarpLerpFactor) + eyeRotationEnd * timewarpLerpFactor;

	gl_TexCoord[0] = vec4( timewarpTexCoord( uv0, lerpedEyeRot ), 0.0, 0.0 );
	gl_TexCoord[1] = vec4( timewarpTexCoord( uv1, lerpedEyeRot ), 0.0, 0.0 );
	gl_TexCoord[2] = vec4( timewarpTexCoord( uv2, lerpedEyeRot ), 0.0, 0.0 );*/
	
	gl_TexCoord[0] = vec4( eyeToSourceUVScale * uv0 + eyeToSourceUVOffset, 0.0, 0.0 );
	gl_TexCoord[1] = vec4( eyeToSourceUVScale * uv1 + eyeToSourceUVOffset,  0.0, 0.0 );
	gl_TexCoord[2] = vec4( eyeToSourceUVScale * uv2 + eyeToSourceUVOffset,  0.0, 0.0 );

	gl_Position = worldViewProj * vertex;

	gl_FrontColor = colour;
}

