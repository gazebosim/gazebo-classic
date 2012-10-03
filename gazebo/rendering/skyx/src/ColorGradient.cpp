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

#include "ColorGradient.h"

namespace SkyX
{
	ColorGradient::ColorGradient()
		: mMalFormed(true)
	{
	}

	ColorGradient::~ColorGradient()
	{
	}

	const Ogre::Vector3 ColorGradient::getColor(const Ogre::Real& p) const
	{
		if (mMalFormed)
		{
			SkyXLOG("Mal-formed ColorGradient");
			return Ogre::Vector3(0,0,0);
		}

		if (CFrameVector.size() == 0)
		{
			return Ogre::Vector3(0,0,0);
		}
		else if (CFrameVector.size() == 1)
		{
			return CFrameVector.at(0).first;
		}

		std::pair<int, Ogre::Real> minBound, maxBound;

		// Min value
		minBound.first = 0;
		minBound.second = -1;
		for (unsigned int k = 0; k < CFrameVector.size(); k++)
		{
			if (CFrameVector.at(k).second < p && CFrameVector.at(k).second > minBound.second)
			{
				minBound.first = k;
				minBound.second = CFrameVector.at(k).second;
			}
		}

		// Max value
		maxBound.first = 0;
		maxBound.second = 2;
		for (unsigned int k = 0; k < CFrameVector.size(); k++)
		{
			if (CFrameVector.at(k).second > p && CFrameVector.at(k).second < maxBound.second)
			{
				maxBound.first = k;
				maxBound.second = CFrameVector.at(k).second;
			}
		}

		float range = maxBound.second - minBound.second,
		      rangepoint = (p - minBound.second) / range;

		return CFrameVector.at(minBound.first).first*(1-rangepoint) + CFrameVector.at(maxBound.first).first*rangepoint;
	}

	const bool ColorGradient::_checkBounds() const
	{
		std::pair<bool, bool> existbounds;
		existbounds.first = false; existbounds.second = false;

		for (unsigned int k = 0; k < CFrameVector.size(); k++)
		{
			if (CFrameVector.at(k).second == 0)
			{
				// More than one min bound
				if (existbounds.first)
				{
					return false;
				}

				existbounds.first = true;
			}

			if (CFrameVector.at(k).second < 0 || CFrameVector.at(k).second > 1)
			{
				return false;
			}
		}

		for (unsigned int k = 0; k < CFrameVector.size(); k++)
		{
			if (CFrameVector.at(k).second == 1)
			{
				// More than one min bound
				if (existbounds.second)
				{
					return false;
				}

				existbounds.second = true;
			}
		}

		if (!existbounds.first || !existbounds.second)
		{
			return false;
		}

		return true;
	}
}