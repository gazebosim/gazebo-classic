
uniform samplerCube envMap;

uniform float cutOffAngle;

uniform float f;
uniform float c1;
uniform float c2;
uniform float c3;
uniform vec3 fun;

varying vec2 frag_pos;

float pi = 3.141592653;
float r = length(frag_pos);

vec3 map(float th)
{
	return vec3(-sin(th)*frag_pos.x/r,sin(th)*frag_pos.y/r,cos(th));
}

// vec3 mercator()
// {
// 	float alpha = frag_pos.y*pi/2.0;
// 	float phi	= (-frag_pos.x)*pi + pi/2.0;

// 	vec3 tc = vec3(
// 		cos(phi)*cos(alpha),
// 		sin(alpha),
// 		sin(phi)*cos(alpha)
// 	);

// 	return tc;
// }

void main()
{
	float param = r/(c1*f);
	float theta = fun.x*asin(param)+fun.y*atan(param)+fun.z*param;

	theta = (theta-c3)*c2;

	// gl_FragColor = vec4(vec3(c1,c2,c3),1.0);
	// return;

	if(true || theta < cutOffAngle)
	{
		vec3 tc ;
		tc = map(theta);
		gl_FragColor = vec4(textureCube(envMap,tc).rgb,1);
	}
	else
		gl_FragColor = vec4(0,0,0,1.0);
}