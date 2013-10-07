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

#ifndef _SkyX_SkyX_H_
#define _SkyX_SkyX_H_

#include "Prerequisites.h"

#include "MeshManager.h"
#include "AtmosphereManager.h"
#include "GPUManager.h"
#include "MoonManager.h"
#include "CloudsManager.h"
#include "ColorGradient.h"
#include "Controller.h"
#include "BasicController.h"
#include "VCloudsManager.h"
#include "VClouds/VClouds.h"
#include "VClouds/GeometryManager.h"
#include "VClouds/GeometryBlock.h"
#include "VClouds/FastFakeRandom.h"
#include "VClouds/Ellipsoid.h"
#include "VClouds/DataManager.h"

namespace SkyX
{
  /** SkyX class
      Create simple and beautiful skies!
    @remarks How to update SkyX:
         Updating SkyX is a very easy task that can be done manually or automatically by using listeners.
         There're two update steps in SkyX: per-frame update and per-camera update, and the order is very
         important.
         The per-frame update must be performed at first: SkyX::update(...), after that and before each
         camera render operation SkyX::notifyCameraRender(...) must be invoked.
         Both updates can be automatically performed by using listeners: the Ogre::FrameListener is used
         for the per-frame udpate, and the Ogre::RenderTargetListener is used for the per-camera update.
         For more information have a look to sample projects.

     */
  class DllExport SkyX : public Ogre::FrameListener,
                         public Ogre::RenderTargetListener
  {
  public:
    /** Render queue groups
     */
    struct RenderQueueGroups
    {
      /** Constructor
          @param s Skydome render queue group (Note: Moon =
          skydome_render_queue+1)
        @param vc VClouds render queue group
        @param vcl VClouds lightnings render queue group
       */
      inline RenderQueueGroups(const Ogre::uint8& s, const Ogre::uint8& vc,
                              const Ogre::uint8& vcl)
        : skydome(s), vclouds(vc), vcloudsLightnings(vcl)
      {
      }

      /// Skydome render queue group (Note: Moon = skydome_render_queue+1)
      Ogre::uint8 skydome;
      /// VClouds render queue group
      Ogre::uint8 vclouds;
      /// VClouds lightnings render queue group
      Ogre::uint8 vcloudsLightnings;
    };

    /** Lighting mode enumeration
        SkyX is designed for true HDR rendering, but there is a big
        number of applications which don't use HDR rendering, due to this
        fact a little exponential tone-mapping algoritm is applied to SkyX
        materials if LM_LDR is selected.
        (See: AtmosphereManager::Options::Exposure)
        Select LM_HDR if your app is designed for true HDR rendering.
        In HDR mode, we assume you're ussing a full linear rendering pipeline,
        so all textures are gamma corrected if needed.
     */
    enum LightingMode
    {
      /// Low dynamic range
      LM_LDR = 0,
      /// High dynamic range
      LM_HDR = 1
    };

    /** Contructor
        @param sm Ogre Scene manager
      @param c SkyX controller
     */
    SkyX(Ogre::SceneManager* sm, Controller* c);

    /** Destructor
     */
    ~SkyX();

    /** Create SkyX
         */
        void create();

    /** Remove SkyX (free resources)
     */
    void remove();

        /** Update (to be invoked per frame)
        @param timeSinceLastFrame Time elapsed since last frame
        @remarks Invoke this function only one time per frame.
        Per-frame update must be performed before per-camera updates through
        SkyX::notifyCameraRender(...) Also it's possible to use listeners,
        making all this process transparent, just register SkyX in ogre root
        through Ogre::Root::addFrameListener(...)
         */
        void update(const Ogre::Real &timeSinceLastFrame);

    /** Notify camera render (to be invoked per camera and per frame)
        @param c Camera
        @remarks Invoke this method manually before the camera render operation
        Per-camera updates must be performed after the per-frame update
        through SkyX::update(...) Also it's possible to use listeners,
        making all this process transparent, just register SkyX in your render
        target through Ogre::RenderTarget::addListener(...)
     */
    void notifyCameraRender(Ogre::Camera* c);

    /** Is SkyX created?
        @return true if yes, false if not
     */
    inline bool isCreated() const
    {
      return mCreated;
    }

    /** Set visible
        @param visible true to set SkyX visible, false to hide it
     */
    void setVisible(const bool& visible);

    /** Is SkyX visible?
        @return true if SkyX is visible, false otherwise
     */
    inline const bool& isVisible() const
    {
      return mVisible;
    }

    /** Set time multiplier
      @param TimeMultiplier Time multiplier
      @remarks The time multiplier can be a negative number, 0 will disable
      auto-updating For setting a custom time of day, check:
      AtmosphereManager::Options::Time
     */
    inline void setTimeMultiplier(const Ogre::Real& TimeMultiplier)
    {
      mTimeMultiplier = TimeMultiplier;
      mVCloudsManager->_updateWindSpeedConfig();
    }

    /** Get time multiplier
        @return Time multiplier
     */
    inline const Ogre::Real& getTimeMultiplier() const
    {
      return mTimeMultiplier;
    }

    /** Get mesh manager
        @return Mesh manager pointer
     */
    inline MeshManager* getMeshManager()
    {
      return mMeshManager;
    }

    /** Get atmosphere manager
        @return Atmosphere manager pointer
     */
    inline AtmosphereManager* getAtmosphereManager()
    {
      return mAtmosphereManager;
    }

    /** Get GPU manager
        @return Atmosphere manager pointer
     */
    inline GPUManager* getGPUManager()
    {
      return mGPUManager;
    }

    /** Get moon manager
        @return Moon manager
     */
    inline MoonManager* getMoonManager()
    {
      return mMoonManager;
    }

    /** Get clouds manager
        @return Clouds manager
     */
    inline CloudsManager* getCloudsManager()
    {
      return mCloudsManager;
    }

    /** Get volumetric clouds manager
        @return Volumetric clouds manager
     */
    inline VCloudsManager* getVCloudsManager()
    {
      return mVCloudsManager;
    }

    /** Set controller
        @param c Controller
     */
    inline void setController(Controller* c)
    {
      if (mController->getDeleteBySkyX())
      {
        delete mController;
      }

      mController = c;
    }

    /** Get current controller
        @return Current controller
     */
    inline Controller* getController() const
    {
      return mController;
    }

    /** Set render queue groups
        @param rqg Render queue groups
     */
    void setRenderQueueGroups(const RenderQueueGroups& rqg);

    /** Get render queue groups
        @return Current render queue groups
     */
    inline const RenderQueueGroups& getRenderQueueGroups() const
    {
      return mRenderQueueGroups;
    }

    /** Set lighting mode
      @param lm Lighting mode
      @remarks SkyX is designed for true HDR rendering, but there're a lot of
      applications that doesn't use HDR rendering, due to this a little
      exponential tone-mapping algoritm is applied to SkyX materials if
      LM_LDR is selected. (See: AtmosphereManager::Options::Exposure)
      Select LM_HDR if your app is designed for true HDR rendering.
      In HDR mode, we assume you're ussing a full linear rendering pipeline,
      so all textures are gamma corrected if needed.
     */
    void setLightingMode(const LightingMode& lm);

    /** Get lighting mode
        @return Lighting mode
     */
    inline const LightingMode& getLightingMode() const
    {
      return mLightingMode;
    }

    /** Set the starfield enabled/disabled
        @param Enabled true for starfield, false for not
     */
    void setStarfieldEnabled(const bool& Enabled);

    /** Is the starfield enable?
        @return true if the starfield is enabled, false if it isn't
     */
    inline const bool& isStarfieldEnabled() const
    {
      return mStarfield;
    }

    /** Set infinite camera far clip distance
      @param d Infinite camera far clip distance
      @remarks SkyX needs a finite camera far clip distance in order to builds
      its geometry. Since Ogre allows infinite far clip camera distances
      (camearFarClipDistance = 0) you'll need to manually provide a far clip
      distance if you're using an infinite camera far clip distance.
     */
    inline void setInfiniteCameraFarClipDistance(const Ogre::Real& d)
    {
      mInfiniteCameraFarClipDistance = d;
    }

    /** Get infinite cmaera far clip distance
        @return Infinite camera far clip distance
     */
    inline const Ogre::Real& getInfiniteCameraFarClipDistance() const
    {
      return mInfiniteCameraFarClipDistance;
    }

    /** Get scene manager
        @return Ogre scene manager
     */
    inline Ogre::SceneManager* getSceneManager()
    {
      return mSceneManager;
    }

    /** Get current rendering camera
        @return Current rendering camera
     */
    inline Ogre::Camera* getCamera()
    {
            return mCamera;
    }

    /** Frame started
        @param evt Frame event
     */
    bool frameStarted(const Ogre::FrameEvent& evt);

    /** Fired before update a render target viewport
        @param evt Render target viewport event
     */
    void preViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);

    /** Get time offset
        @return Time offset
      @remarks Only for internal use
     */
    inline const Ogre::Real& _getTimeOffset() const
    {
      return mTimeOffset;
    }

    void setEnabled(bool _enabled)
    {
      this->mEnabled = _enabled;
      this->setMoonEnabled(_enabled);
      this->setCloudsEnabled(_enabled);
    }

    inline bool getEnabled()
    {
      return this->mEnabled;
    }

    void setMoonEnabled(bool _enabled)
    {
      this->mMoonEnabled = _enabled;
      this->mMoonManager->setEnabled(_enabled);
    }

    inline bool getMoonEnabled()
    {
      return this->mMoonEnabled;
    }

    void setCloudsEnabled(bool _enabled)
    {
      this->mCloudsEnabled = _enabled;
      this->mVCloudsManager->getVClouds()->setEnabled(_enabled);
    }

    inline bool getCloudsEnabled()
    {
      return this->mCloudsEnabled;
    }

  private:

    /// Enable starfield?
    bool mStarfield;

    /// Lighting mode
    LightingMode mLightingMode;

    /// Scene manager
    Ogre::SceneManager *mSceneManager;

    /// Controller
    Controller* mController;

    /// Current rendering camera
    Ogre::Camera* mCamera;

    /// Mesh manager
    MeshManager* mMeshManager;

    /// Atmosphere manager
    AtmosphereManager* mAtmosphereManager;
    /// GPU manager
    GPUManager* mGPUManager;
    /// Moon manager
    MoonManager* mMoonManager;
    /// Clouds manager
    CloudsManager* mCloudsManager;

    /// Render queue groups
    RenderQueueGroups mRenderQueueGroups;

    /// Is SkyX created?
    bool mCreated;

    /// Last camera position
    Ogre::Vector3 mLastCameraPosition;
    /// Last camera far clip distance
    Ogre::Real mLastCameraFarClipDistance;
    /// Infinite camera far clip distance
    Ogre::Real mInfiniteCameraFarClipDistance;

    /// Is SkyX visible?
    bool mVisible;

    /// Time multiplier
    Ogre::Real mTimeMultiplier;
    /// Time offset
    Ogre::Real mTimeOffset;

    /// Volumetric clouds manager
    VCloudsManager* mVCloudsManager;

    /// True if moon is enabled
    bool mMoonEnabled;

    /// True if clouds are enabled
    bool mCloudsEnabled;

    /// True if skyx is enabled
    bool mEnabled;
  };
}

#endif
