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

#ifndef _SkyX_Controller_H_
#define _SkyX_Controller_H_

#include "Prerequisites.h"

namespace SkyX
{
	/** Controller base class
	 */
    class Controller 
	{
	public:
	    /** Constructor
		    @param deleteBySkyX true to automatically destroy the controller by SkyX, false otherwise
		 */
		inline Controller(const bool& deleteBySkyX)
			: mDeleteBySkyX(deleteBySkyX)
		{
		}

		/** Destructor
		 */
		inline virtual ~Controller(){}

		/** Update controller
		    @param simDeltaTime Simulation delta time (It's not the time since last frame, it's the delta simulation time, one
								time the time since last frame has been multiplied by the time multiplier)
		 */
		inline virtual void update(const Ogre::Real& simDeltaTime){}

		/** Get sun direction
		    @return Sun direction, the Earth-to-Sun direction
		 */
		virtual Ogre::Vector3 getSunDirection() = 0;

		/** Get moon direction
		    @return Moon direction, Earth-to-Moon direction
		 */
		virtual Ogre::Vector3 getMoonDirection() = 0;

		/** Get moon phase
		    @return Moon phase in [-1,1] range, where -1 means fully covered Moon, 0 clear Moon and 1 fully covered Moon
		 */
		virtual Ogre::Real getMoonPhase() = 0;

		/** Must the controller be destroyed by SkyX?
		    @return true if yes, false if not
		 */
		inline const bool& getDeleteBySkyX() const
		{
			return mDeleteBySkyX;
		}

	private:
		/// Must the controller be destroyed by SkyX?
		bool mDeleteBySkyX;
	};
}

#endif