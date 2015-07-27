
uniform samplerCube envMap;

uniform float HFOV;

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

vec3 custom()
{
	float param = r/(c1*f);

	float phi_1 = fun.x*asin(param)+fun.y*atan(param)+fun.z*param;

	return map((phi_1-c3)/c2);
}

void main()
{
	vec3 tc;
	tc = custom();

	gl_FragColor = vec4(textureCube(envMap,tc).rgb,1);
}