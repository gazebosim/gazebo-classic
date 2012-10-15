uniform mat4 texProjMatrix;
uniform mat4 worldMatrix;

varying vec3 normal, lightDir, eyeVec;

void main()
{	
	normal = gl_NormalMatrix * gl_Normal;
	
	vec4 posEye =  gl_ModelViewMatrix * gl_Vertex;
	vec4 posWorld = worldMatrix * gl_Vertex;
	gl_TexCoord[0] = texProjMatrix * posWorld;

	lightDir = vec3(gl_LightSource[0].position.xyz - posEye.xyz);
	eyeVec = -posEye.xyz;

	gl_Position = ftransform();		
}
