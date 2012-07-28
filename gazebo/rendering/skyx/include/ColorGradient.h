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

#ifndef _SkyX_ColorGradient_H_
#define _SkyX_ColorGradient_H_

#include "Prerequisites.h"

namespace SkyX
{
    class DllExport ColorGradient 
	{
	public:
		/** Color frame type definition
		    ColorFrame.first: Colour value
			ColorFrame.second: Position in the gradient [0,1] range
		 */
		typedef std::pair<Ogre::Vector3, Ogre::Real> ColorFrame;

	    /** Constructor
		 */
		ColorGradient();

		/** Destructor 
		 */
		~ColorGradient();

		/** Add color frame
		    @param CFrame Color frame
		 */
		inline void addCFrame(const ColorFrame& CFrame)
		{
			CFrameVector.push_back(CFrame);

			mMalFormed = !_checkBounds();
		}

		/** Clear color gradient
		 */
		inline void clear()
		{
			CFrameVector.clear();
		}

		/** Get color value
		    @param p The gradient point in [0,1] range
			@return Color at the given gradient position
		 */
		const Ogre::Vector3 getColor(const Ogre::Real& p) const;

	private:
		/** Check bounds
		    @return false if the Color gradient is mal-formed
		 */
		const bool _checkBounds() const;

		/// Mal formed color gradient?
		bool mMalFormed;

		/// Color frame vector
		std::vector<ColorFrame> CFrameVector;
	};
}

#endif