
uniform sampler2D diffuseMap;

void main(void)
{
	float red = texture2D(diffuseMap, gl_TexCoord[0].xy).r;
	float green = texture2D(diffuseMap, gl_TexCoord[1].xy).g;
	float blue = texture2D(diffuseMap, gl_TexCoord[2].xy).b;
	
	gl_FragColor = vec4( red, green, blue, 1.0 )*gl_Color;

	//gl_FragColor = texture2D(diffuseMap, gl_TexCoord[0].xy);//vec4( red, green, blue, 1.0 );
	//gl_FragColor = vec4( 1.0, 1.0, 1.0, 1.0 );
}
