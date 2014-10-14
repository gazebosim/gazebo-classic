// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Mapping of undistorted to distorted uv coordinates.
uniform sampler2D distortionMap;

// Scale the input texture if necessary to crop black border
uniform vec3 scale;

void main()
{
  vec2 scaleCenter = vec2(0.5, 0.5);
  vec2 inputUV = (gl_TexCoord[0].xy - scaleCenter) * scale.xy + scaleCenter;
  vec4 mapUV = texture2D(distortionMap, inputUV);

  if (mapUV.x < 0.0 || mapUV.y < 0.0)
    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    gl_FragColor = texture2D(RT, mapUV.xy);
}
