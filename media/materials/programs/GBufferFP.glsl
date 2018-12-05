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
#extension GL_ARB_draw_buffers : enable

varying vec3 oViewPos;
varying vec3 oNormal;

uniform float cNearClipDistance;
// !!! might be 0 for infinite view projection.
uniform float cFarClipDistance;

void main()
{
  float clipDistance = cFarClipDistance - cNearClipDistance;
  gl_FragData[0] = vec4(normalize(oNormal).xyz, (length(oViewPos) - cNearClipDistance) / clipDistance); // normal + linear depth [0, 1]
  gl_FragData[1] = vec4(oViewPos, 0.0); // view space position
}
