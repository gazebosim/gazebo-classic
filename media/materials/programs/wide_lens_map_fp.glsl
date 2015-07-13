
uniform sampler2D RT;
uniform samplerCube envMap;

varying vec2 frag_pos;

float pi = 3.141592653;

vec3 mercator()
{
	float alpha = frag_pos.y*pi/2.0;
	float phi	= (-frag_pos.x)*pi + pi/2.0;

	vec3 tc = vec3(
		cos(phi)*cos(alpha),
		sin(alpha),
		sin(phi)*cos(alpha)
	);

	return tc;
}

void main()
{
	// float r = length(frag_pos);
	// float f = 1;
	// float theta = 6*atan(2*r,sqrt(r*r+1));
	// vec3 tc = vec3(-sin(theta)*frag_pos.x/r,sin(theta)*frag_pos.y/r,cos(theta));

	vec3 tc = mercator();

	gl_FragColor = vec4(textureCube(envMap,tc).rgb,1);
}