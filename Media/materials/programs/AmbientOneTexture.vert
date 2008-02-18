varying float fogFactor;

void main()
{
  vec4 ecPosition = gl_ModelViewMatrix * gl_Vertex;

	gl_FrontColor = gl_LightModel.ambient * gl_FrontMaterial.ambient;
	
	// if you want to use more tex-coords, don't forget to assign them there
	gl_TexCoord[ 0 ] = gl_MultiTexCoord0;
	//gl_TexCoord[ 1 ] = gl_MultiTexCoord1;
	//gl_TexCoord[ 2 ] = gl_MultiTexCoord2;
	//gl_TexCoord[ 3 ] = gl_MultiTexCoord3;

  // Compute the amount of fog to apply
  if ( gl_Fog.end != 0.0)
  {
    fogFactor = clamp((gl_Fog.end - fabs(ecPosition.z)) * gl_Fog.scale,0,1);
  }
  else
  {
    fogFactor = 1.0;
  }


  
	gl_Position = ftransform();
}
