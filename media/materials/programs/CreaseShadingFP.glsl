// This code in this file is adapted from OGRE Samples. The OGRE's license and
// copyright header is copied below.

/*
-----------------------------------------------------------------------------
OGRE (www.ogre3d.org) is made available under the MIT License.

Copyright (c) 2000-2013 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#version 120
varying vec2 uv;

uniform sampler2D sNormal;
uniform sampler2D sPosition;
uniform sampler2D sRandom;

// the three(four) artistic parameters
uniform float cRange;
uniform float cBias;
uniform float cAverager;
uniform float cMinimumCrease;
// Bias for the kernel size, Hack for the fixed size 11x11 stipple kernel
uniform float cKernelSize;
uniform vec4 cViewportSize;

void main()
{
  // get the view space position and normal of the fragment
  vec3 fragmentPosition = texture2D(sPosition, uv).xyz;
  vec3 fragmentNormal = texture2D(sNormal, uv).xyz;

  float totalGI = 0.0f;

  const int stippleSize = 11; // must be odd
  for (int i = 0; i < (stippleSize + 1) / 2; i++)
  {
    vec2 diagonalStart = vec2(-(stippleSize - 1.0) / 2.0, 0) + i;
    for(int j = 0; j < (stippleSize + 1) / 2; j++)
    {
      vec2 sampleOffset = diagonalStart + vec2(j, -j);

      vec2 sampleUV = uv + (sampleOffset * cViewportSize.zw * cKernelSize);
      vec3 samplePos = texture2D(sPosition, sampleUV).xyz;

      vec3 toCenter = samplePos - fragmentPosition;
      float distance = length(toCenter);

      toCenter = normalize(toCenter);
      float centerContrib = clamp((dot(toCenter, fragmentNormal) - cMinimumCrease) * cBias, 0.0, 1.0);
      float rangeAttenuation = 1.0f - clamp(distance / cRange, 0.0, 1.0);

      totalGI += centerContrib * rangeAttenuation;
    }
  }

  totalGI /= cAverager;
  gl_FragColor = 1.0 - vec4(totalGI, totalGI, totalGI, 1.0);
}
