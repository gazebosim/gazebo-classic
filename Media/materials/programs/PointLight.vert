uniform vec4 lightDiffuse;
uniform vec4 lightSpecular;

varying vec4 diffuse, specular;
varying vec3 normal;
varying vec4 position;

void main()
{      
  position = gl_ModelViewMatrix * gl_Vertex;

//  normal = normalize (gl_NormalMatrix * gl_Normal);
  normal = gl_NormalMatrix * gl_Normal;

  /* Compute the diffuse, ambient and globalAmbient terms */
  diffuse = gl_FrontMaterial.diffuse * lightDiffuse;   
  specular = gl_FrontMaterial.specular * lightSpecular;      

  gl_Position = ftransform();
} 
