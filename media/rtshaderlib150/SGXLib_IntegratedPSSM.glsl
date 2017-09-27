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
float _SGX_ShadowPCF4(sampler2DShadow shadowMap, vec4 shadowMapPos, vec2 invShadowMapSize)
{
  float shadow = 0.0;

  // 9-sample poisson disk blur
  vec2 poissonDisk[9] = vec2[](
    vec2( -0.676, 0.517 ),
    vec2( 0.943, -0.221 ),
    vec2( -0.223, -0.957 ),
    vec2( 0.184, 0.797 ),
    vec2( -0.228, -0.228 ),
    vec2( 0.424, -0.815 ),
    vec2( -0.854, -0.445 ),
    vec2( 0.825, 0.482 ),
    vec2( 0.179, 0.164 )
  );
  for (int i = 0; i < 9; i++)
  {
    vec4 newUV = shadowMapPos;
    newUV.xy += poissonDisk[i] * invShadowMapSize;
    // Divide by w necessary for LiSPMS, which is no longer in use
    //newUV = newUV / newUV.w;
    // This texture() does the depth compare and provides Hardware PCF as a driver hack.
    shadow += texture(shadowMap, newUV.xyz);
  }
  shadow /= 9.0;

  // smoothstep makes shadow edges appear more crisp and hides Mach bands
  return smoothstep(0.0, 1.0, shadow);
}

//-----------------------------------------------------------------------------
void SGX_ComputeShadowFactor_PSSM3(in float fDepth,
                                   in vec4 vSplitPoints,
                                   in vec4 lightPosition0,
                                   in vec4 lightPosition1,
                                   in vec4 lightPosition2,
                                   in sampler2DShadow shadowMap0,
                                   in sampler2DShadow shadowMap1,
                                   in sampler2DShadow shadowMap2,
                                   in vec4 invShadowMapSize0,
                                   in vec4 invShadowMapSize1,
                                   in vec4 invShadowMapSize2,
                                   out float oShadowFactor)
{
  oShadowFactor = 1.0;
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
  // If we don't do this comparison, everything outside the farthest shadow map
  // will appear shadowed. It would be better to skip this comparison so the
  // entire far shadow map is used. But then we would need to find another way
  // to keep everything outside that map in light instead of shadow, which
  // appears to be a difficult problem.
  else if (fDepth <= vSplitPoints.z)
  {
    oShadowFactor =
      _SGX_ShadowPCF4(shadowMap2, lightPosition2, invShadowMapSize2.xy);
  }
}
