varying vec4 diffuse,ambientGlobal, ambient;
varying vec3 normal,lightDir,halfVector;
varying float dist;

void main(void)
{
  /*vec4 color = ambientGlobal;

  vec3 n,halfV,viewV,ldir;
  float NdotL,NdotHV;
  float att;

  n = normalize(normal);

  NdotL = max(dot(n,normalize(lightDir)),0.0);

  if (NdotL > 0.0) {

    att = 1.0 / (gl_LightSource[0].constantAttenuation +
        gl_LightSource[0].linearAttenuation * dist +
        gl_LightSource[0].quadraticAttenuation * dist * dist);
    color += att * (diffuse * NdotL + ambient);


    halfV = normalize(halfVector);
    NdotHV = max(dot(n,halfV),0.0);
    color += att * gl_FrontMaterial.specular * gl_LightSource[0].specular * 
      pow(NdotHV,gl_FrontMaterial.shininess);
  }

  */
  gl_FragColor = gl_Color;
}

