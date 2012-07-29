/*
--------------------------------------------------------------------------------
This source file is part of SkyX.
Visit http://www.paradise-studios.net/products/skyx/

Copyright (C) 2009-2012 Xavier Verguín González <xavyiy@gmail.com>

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
--------------------------------------------------------------------------------
*/

// ------------------------- SkyX lightning -----------------------------

void main_vp(
    // IN
	float4 iPosition	        : POSITION,
	float3 iColor               : COLOR,
	float2 iUV                  : TEXCOORD0,
	// OUT
	out float4 oPosition		: POSITION,
	out float3 oUV              : TEXCOORD0,
	out float4 oData            : TEXCOORD1,
	// UNIFORM
	uniform float4x4 uWorldViewProj,
	uniform float3   uData)
{
    oPosition = mul(uWorldViewProj, iPosition);
    oUV.xy = iUV;
   
    // Alpha
    oUV.z = uData.x; 
   
    if (iColor.x > 0.5)
    {
		// Reverse y coord and mark
		oUV.y = 2+(1-oUV.y);
    }
   
    oData.xy = iColor.yz;
    oData.zw = uData.yz;
}

void main_fp(
    // IN
    float3 iUV        : TEXCOORD0,
	float4 iData      : TEXCOORD1,
	// OUT 
	out float4 oColor : COLOR,
	// UNIFORM
	uniform float3 uColor)
{    
    float intensity = 0;
	float mult = 1;
	float smoothAvance = 16;

	if (iData.x+iUV.y*(iData.y-iData.x) > iData.z)
	{
		iUV.z *= 1-saturate(length(iData.x+iUV.y*(iData.y-iData.x) - iData.z)*smoothAvance);
	}
	
	if (iUV.y > 2)
	{
		iUV.y-=2; // Get back y coord
		intensity = saturate((1-2*length(float2(0.5,0.0)-iUV.xy)))*mult;
	}
	else
	{
		intensity = (1-2*length(0.5-iUV.x))*mult;
	}
	
	intensity = pow(intensity,1/(0.1f+iUV.z));
	
	// Falling effect
	smoothAvance = 6;
	iUV.z *= 1-iData.w*saturate(length(iData.x+iUV.y*(iData.y-iData.x) - iData.z)*smoothAvance);
	
	// Final color
	oColor = float4(uColor*iUV.z*intensity,1);
}