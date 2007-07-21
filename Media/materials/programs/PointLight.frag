uniform vec4 lightAttenuation;
uniform vec4 lightPositionOS;
uniform vec4 cameraPositionOS;

varying vec4 diffuse, specular;
varying vec3 normal;
varying vec4 position;

void main()
{
  vec4 color;
  vec3 n,halfV,viewV,ldir;
  float NdotL,NdotHV;

  color = vec4(0.0, 0.0, 0.0, 0.0);

  if (gl_LightSource[1].spotCosCutoff < 0.0)
  {
    float att;

    vec3 aux = vec3 (lightPositionOS - position);
    vec3 lightDir = normalize(aux);
    vec3 halfVector = vec3(lightDir + normalize(vec3(cameraPositionOS - position)));

    /* a fragment shader can't write a verying variable, hence we need
     *    a new variable to store the normalized interpolated normal */
    n = normalize(normal);

    /* compute the dot product between normal and ldir */
    NdotL = max(dot(n,normalize(lightDir)), 0.0);

    if (NdotL > 0.0) 
    {
      float toLightDist = length(aux);

      att = 1.0 / (lightAttenuation.y +
          lightAttenuation.z * toLightDist +
          lightAttenuation.w * toLightDist * toLightDist);
      color += att * (diffuse * NdotL);

      halfV = normalize(halfVector);
      NdotHV = max(dot(n,halfV),0.0);
      color += att * specular * pow(NdotHV,gl_FrontMaterial.shininess);
    }
  }

  gl_FragColor = color;
} 
