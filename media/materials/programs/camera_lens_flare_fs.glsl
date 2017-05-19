// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

uniform float time;
uniform vec2 size;
uniform vec2 lightPos;

// uniform sampler2D noise;
/*
float noise(float t)
{
  return texture(iChannel0,vec2(t, 0.0) / iChannelResolution[0].xy).x;
}
float noise(vec2 t)
{
  return texture(iChannel0,(t + vec2(iGlobalTime)) / iChannelResolution[0].xy).x;
}
*/
vec3 lensflare(vec2 uv,vec2 pos)
{
  float globalTime = 1.0;

  vec2 main = uv-pos;
  vec2 uvd = uv*(length(uv));

  float ang = atan(main.y, main.x);
  float dist = length(main); dist = pow(dist,.1);
// float n = noise(vec2((ang-globalTime/9.0)*16.0,dist*32.0));


  float n = vec2((ang-globalTime/9.0)*16.0,dist*32.0);
	
  float f0 = 1.0/(length(uv-pos)*16.0+1.0);
	
  // f0 = f0+f0*(sin((ang+globalTime/18.0 + noise(abs(ang)+n/2.0)*2.0)*12.0)*.1+dist*.1+.8);
  f0 = f0+f0*(sin((ang+globalTime/18.0 + (abs(ang)+n/2.0)*2.0)*12.0)*.1+dist*.1+.8);

  float f2 = max(1.0/(1.0+32.0*pow(length(uvd+0.8*pos),2.0)),.0)*00.25;
  float f22 = max(1.0/(1.0+32.0*pow(length(uvd+0.85*pos),2.0)),.0)*00.23;
  float f23 = max(1.0/(1.0+32.0*pow(length(uvd+0.9*pos),2.0)),.0)*00.21;
	
  vec2 uvx = mix(uv,uvd,-0.5);
	
  float f4 = max(0.01-pow(length(uvx+0.4*pos),2.4),.0)*6.0;
  float f42 = max(0.01-pow(length(uvx+0.45*pos),2.4),.0)*5.0;
  float f43 = max(0.01-pow(length(uvx+0.5*pos),2.4),.0)*3.0;

  uvx = mix(uv,uvd,-.4);

  float f5 = max(0.01-pow(length(uvx+0.2*pos),5.5),.0)*2.0;
  float f52 = max(0.01-pow(length(uvx+0.4*pos),5.5),.0)*2.0;
  float f53 = max(0.01-pow(length(uvx+0.6*pos),5.5),.0)*2.0;

  uvx = mix(uv,uvd,-0.5);

  float f6 = max(0.01-pow(length(uvx-0.3*pos),1.6),.0)*6.0;
  float f62 = max(0.01-pow(length(uvx-0.325*pos),1.6),.0)*3.0;
  float f63 = max(0.01-pow(length(uvx-0.35*pos),1.6),.0)*5.0;

  vec3 c = vec3(.0);

  c.r+=f2+f4+f5+f6; c.g+=f22+f42+f52+f62; c.b+=f23+f43+f53+f63;
  c+=vec3(f0);

  return c;
}

vec3 cc(vec3 color, float factor, float factor2) // color modifier
{
  float w = color.x+color.y+color.z;
  return mix(color,vec3(w)*factor,w*factor2);
}

void main()
{
  // vec2 uv = gl_TexCoord[0].xy / iResolution.xy - 0.5;
  // uv.x *= iResolution.x/iResolution.y; //fix aspect ratio
  vec2 uv = gl_TexCoord[0].xy / size.xy - 0.5;
  //fix aspect ratio
  uv.x *= size.x/size.y; 
  vec2 pos = vec2(0, 0);
  vec3 color = vec3(1.4,1.2,1.0)*lensflare(uv,pos);
  color = cc(color,.5,.1);
  // gl_FragColor = vec4(color,1.0);
  gl_FragColor = texture2D(RT, gl_TexCoord[0].xy) + vec4(color, 1.0);
}

