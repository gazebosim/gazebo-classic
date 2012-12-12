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

#ifndef _SkyX_VCloudsManager_H_
#define _SkyX_VCloudsManager_H_

#include "Prerequisites.h"

#include "VClouds/VClouds.h"
#include "ColorGradient.h"

namespace SkyX
{
  class SkyX;

    class DllExport VCloudsManager
  {
  public:
      /** Constructor
        @param s Parent SkyX pointer
     */
    VCloudsManager(SkyX *s);

    /** Destructor
     */
    ~VCloudsManager();

    /** Create all resources
        @param radius Volumetric cloud field radius, -1 to use current VClouds::mGeometrySettings::Radius
     */
    void create(const Ogre::Real& radius = -1);

    /** Update
        @param timeSinceLastFrame Time since last frame
     */
    void update(const Ogre::Real& timeSinceLastFrame);

    /** Notify camera render, to be invoked per-camera and per-frame
      @param c Rendering camera
      @remarks The VClouds system needs the CAMERA time since last frame, so here we assume that all render targets
               are being updated one time per frame(in other words, all render targets are being updated at the same rate)
         */
        void notifyCameraRender(Ogre::Camera* c);

    /** Remove all resources
     */
    void remove();

    /** Set ambient gradient
        @param AmbientGradient Ambient color gradient
     */
    inline void setAmbientGradient(const ColorGradient& AmbientGradient)
    {
      mAmbientGradient = AmbientGradient;
    }

    /** Get ambient color gradient
        @return Ambient color gradient
     */
    inline const ColorGradient& getAmbientGradient() const
    {
      return mAmbientGradient;
    }

    /** Set sun gradient
        @param SunGradient Sun color gradient
     */
    inline void setSunGradient(const ColorGradient& SunGradient)
    {
      mSunGradient = SunGradient;
    }

    /** Get sun color gradient
        @return Sun color gradient
     */
    inline const ColorGradient& getSunGradient() const
    {
      return mSunGradient;
    }

    /** Set height parameters
        @param Height x = Cloud field y-coord start, y: Field height (both in world coordinates)
      @remarks Call it before create(), for now...
               For autocalculated height based on the radius length set (-1,-1) as height params
     */
    inline void setHeight(const Ogre::Vector2& Height)
    {
      mHeight = Height;
    }

    /** Get height parameters
        @return Height: x = Cloud field y-coord start, y: Field height (both in world coordinates)
     */
    inline const Ogre::Vector2& getHeight() const
    {
      return mHeight;
    }

    /** Set autoupdate
        @Autoupdate true if you want to update volumetric clouds wind depending of the SkyX time multiplier
      @remarks You can use VClouds->setWindSpeed(..) for different winds speeds
     */
    inline void setAutoupdate(const bool& Autoupdate)
    {
      mAutoupdate = Autoupdate;
      _updateWindSpeedConfig();
    }

    /** Get autoupdate
        @return true if you want to update volumetric clouds wind depending of the SkyX time multiplier
     */
    inline const bool& getAutoupdate() const
    {
      return mAutoupdate;
    }

    /** Set wind speed
        @param WindSpeed Wind speed
     */
    inline void setWindSpeed(const Ogre::Real& WindSpeed)
    {
      mWindSpeed = WindSpeed;
      _updateWindSpeedConfig();
    }

    /** Get wind speed
        @return Wind speed
     */
    inline const Ogre::Real& getWindSpeed() const
    {
      return mWindSpeed;
    }

    /** Get VClouds
     */
    inline VClouds::VClouds* getVClouds()
    {
      return mVClouds;
    }

    /** Is moon manager created?
        @return true if yes, false if not
     */
    inline const bool& isCreated() const
    {
      return mCreated;
    }

    /** Update wind speed config
        @remarks Only for internal use
     */
    void _updateWindSpeedConfig();

  private:
    /** Set light parameters
     */
    void _setLightParameters();

    /// SkyX parent pointer
    SkyX *mSkyX;

    /// VClouds pointer
    VClouds::VClouds* mVClouds;

    /// Height parameters, x = Cloud field y-coord start,
    /// y: Field height (both in world coordinates)
    Ogre::Vector2 mHeight;

    /// Wind speed
    Ogre::Real mWindSpeed;

    /// Autoupdate wind speed depending of skyx time multiplier?
    bool mAutoupdate;

    /// Is vclouds manager created?
    bool mCreated;

    /// Current time since last frame
    Ogre::Real mCurrentTimeSinceLastFrame;

    /// Ambient and Sun color gradients
    ColorGradient mAmbientGradient;
    ColorGradient mSunGradient;
  };
}

#endif
