/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org

Copyright (c) 2000-2012 Torus Knot Software Ltd
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
//-----------------------------------------------------------------------------
// Program Name: SGXLib_IntegratedPSSM
// Program Desc: Integrated PSSM functions.
// Program Type: Vertex/Pixel shader
// Language: GLSL
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void SGX_CopyDepth(in vec4 clipSpacePos,
			 out float oDepth)
{
	oDepth = clipSpacePos.z;
}

//-----------------------------------------------------------------------------
void SGX_ModulateScalar(in float vIn0, in vec4 vIn1, out vec4 vOut)
{
	vOut = vIn0 * vIn1;
}

//-----------------------------------------------------------------------------
void SGX_ApplyShadowFactor_Diffuse(in vec4 ambient,
					  in vec4 lightSum,
					  in float fShadowFactor,
					  out vec4 oLight)
{
	oLight.rgb = ambient.rgb + (lightSum.rgb - ambient.rgb) * (fShadowFactor);
	oLight.a   = lightSum.a;
}

//-----------------------------------------------------------------------------
float texture2DCompare(sampler2D depths, vec2 uv, float compare)
{
  float depth = texture(depths, uv).r;
  return (step(compare, depth) >= 1.0) ? 1.0 : 0.4;
}

//-----------------------------------------------------------------------------
float _SGX_ShadowPCF4(sampler2D shadowMap, vec4 shadowMapPos, vec2 offset)
{
  // Interpolated 3x3 PCF
  // Adapted from http://www.ogre3d.org/forums/viewtopic.php?f=1&t=78834

  if (offset.x <= 0.0 || offset.y <= 0.0 || shadowMapPos.w <= 0.0)
    return 1.0;

  shadowMapPos = shadowMapPos / shadowMapPos.w;
  vec2 uv = shadowMapPos.xy;

  vec2 texelSize = offset;
  vec2 size = 1.0 / offset;
  vec2 centroidUV = floor(uv * size + 0.5) / size;
  vec2 f = fract(uv * size + 0.5);

  // 3 x 3 kernel
  const int   X = 3;

  vec2 topLeft = centroidUV - texelSize * 1.5;

  // load all pixels needed for the computation
  // this way a pixel won't be loaded twice
  float kernel[9];
  for (int i = 0; i < X; ++i)
  {
    for (int j = 0; j < X; ++j)
    {
       kernel[i * X + j] = texture2DCompare(shadowMap,
          topLeft + vec2(i, j) * texelSize, shadowMapPos.z);
    }
  }

  float kernel_interpolated[4];

  kernel_interpolated[0] = kernel[0] + kernel[1] + kernel[3] + kernel[4];
  kernel_interpolated[0] /= 4.0;
  kernel_interpolated[1] = kernel[1] + kernel[2] + kernel[4] + kernel[5];
  kernel_interpolated[1] /= 4.0;
  kernel_interpolated[2] = kernel[3] + kernel[4] + kernel[6] + kernel[7];
  kernel_interpolated[2] /= 4.0;
  kernel_interpolated[3] = kernel[4] + kernel[5] + kernel[7] + kernel[8];
  kernel_interpolated[3] /= 4.0;

  float a = mix(kernel_interpolated[0], kernel_interpolated[1], f.y);
  float b = mix(kernel_interpolated[2], kernel_interpolated[3], f.y);
  float c = mix(a, b, f.x);
  return c;
}

//-----------------------------------------------------------------------------
void SGX_ComputeShadowFactor_PSSM3(in float fDepth,
							in vec4 vSplitPoints,
							in vec4 lightPosition0,
							in vec4 lightPosition1,
							in vec4 lightPosition2,
							in sampler2D shadowMap0,
							in sampler2D shadowMap1,
							in sampler2D shadowMap2,
							in vec4 invShadowMapSize0,
							in vec4 invShadowMapSize1,
							in vec4 invShadowMapSize2,
							out float oShadowFactor)
{
  if (fDepth  <= vSplitPoints.x)
  {
    oShadowFactor =
      _SGX_ShadowPCF4(shadowMap0, lightPosition0, invShadowMapSize0.xy);
  }
  else if (fDepth <= vSplitPoints.y)
  {
    oShadowFactor =
      _SGX_ShadowPCF4(shadowMap1, lightPosition1, invShadowMapSize1.xy);
  }
  else
  {
    oShadowFactor =
      _SGX_ShadowPCF4(shadowMap2, lightPosition2, invShadowMapSize2.xy);
  }
}
