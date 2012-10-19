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

#ifndef _SkyX_BasicController_H_
#define _SkyX_BasicController_H_

#include "Prerequisites.h"

#include "Controller.h"

namespace SkyX
{
  /** Basic controller class
   */
  class DllExport BasicController : public Controller
  {
  public:
      /** Constructor
          @param deleteBySkyX true to automatically destroy the controller by
          SkyX, false otherwise
       */
      BasicController(const bool& deleteBySkyX = true);

      /** Update controller
          @param simDeltaTime Simulation delta time (It's not the time since
          last frame, it's the delta simulation time, one
          time the time since last frame has been multiplied by the time
          multiplier)
       */
      void update(const Ogre::Real& simDeltaTime);

      /** Set time
          @param t Time, where x = time in [0, 24]h range,
          y = sunrise hour in [0, 24]h range, z = sunset hour in [0, 24] range
       */
      inline void setTime(const Ogre::Vector3& t)
      {
          mTime = t;
          update(0);
      }

      /** Get time
          @return Current time, where x = time in [0, 24]h range,
          y = sunrise hour in [0, 24]h range, z = sunset hour in [0, 24] range
       */
      inline const Ogre::Vector3& getTime() const
      {
          return mTime;
      }

      /** Set eastn direction
          @param ed East direction, in X,Y world coords (must be normalized)
       */
      inline void setEastDirection(const Ogre::Vector2& ed)
      {
          mEastDirection = ed;
          update(0);
      }

      /** Get east direction
          @return Current east direction, in X,Y world coords
       */
      inline const Ogre::Vector2& getEastDirection() const
      {
          return mEastDirection;
      }

      /** Get sun direction
          @return Sun direction, the Earth-to-Sun direction
       */
      inline Ogre::Vector3 getSunDirection()
      {
          return mSunDirection;
      }

      /** Get moon direction
          @return Moon direction, Earth-to-Moon direction
       */
      inline Ogre::Vector3 getMoonDirection()
      {
          return mMoonDirection;
      }

      /** Set moon phase
          @param mp Moon phase in [-1,1] range, where -1 means fully
          covered Moon, 0 clear Moon and 1 fully covered Moon
       */
      inline void setMoonPhase(const Ogre::Real& mp)
      {
          mMoonPhase = mp;
      }

      /** Get moon phase
          @return Moon phase in [-1,1] range, where -1 means
          fully covered Moon, 0 clear Moon and 1 fully covered Moon
       */
      inline Ogre::Real getMoonPhase()
      {
          return mMoonPhase;
      }

  private:
      /// Time information: x = time in [0, 24]h range, y = sunrise
      /// hour in [0, 24]h range, z = sunset hour in [0, 24] range
      Ogre::Vector3 mTime;

      /// Sun direction
      Ogre::Vector3 mSunDirection;

      /// Moon direction
      Ogre::Vector3 mMoonDirection;

      /// East direction (in X,Z world coords)
      Ogre::Vector2 mEastDirection;

      /// Moon phase
      Ogre::Real mMoonPhase;
  };
}

#endif
