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

#ifndef _SkyX_VClouds_VClouds_H_
#define _SkyX_VClouds_VClouds_H_

#include <vector>

#include "Prerequisites.h"

#include "VClouds/DataManager.h"
#include "VClouds/GeometryManager.h"
#include "VClouds/LightningManager.h"

namespace SkyX { namespace VClouds {

  class DllExport VClouds
  {
  public:
    /** Render queue groups
     */
    struct RenderQueueGroups
    {
      /** Constructor
        @param vc VClouds render queue group
        @param vcl VClouds lightnings render queue group
       */
      inline RenderQueueGroups(const Ogre::uint8& vc, const Ogre::uint8& vcl)
        : vclouds(vc), vcloudsLightnings(vcl)
      {
      }

      /// VClouds render queue group
      Ogre::uint8 vclouds;
      /// VClouds lightnings render queue group
      Ogre::uint8 vcloudsLightnings;
    };

    /** Geometry settings
     */
    struct GeometrySettings
    {
      /// Height: x = Altitude over the camera,
      /// y: Field height (both in world coordinates)
      Ogre::Vector2 Height;
      /// Angles
      Ogre::Radian Alpha, Beta;
      /// Radius
      float Radius;
      /// Number of blocks
      int NumberOfBlocks;
      /// Number of slices per geometry zone
      int Na, Nb, Nc;

      /** Default constructor
       */
      GeometrySettings()
        : Height(Ogre::Vector2(10, 50))
        , Alpha(Ogre::Degree(12)), Beta(Ogre::Degree(40))
        , Radius(100)
        , NumberOfBlocks(12)
        , Na(10), Nb(8), Nc(6)
      {
      }

      /** Constructor
          @param _Height x = Cloud field y-coord start,
                         y: Field height (both in world coordinates)
        @param _Radius Radius
        @param _Alpha Alpha angle
        @param _Beta Beta angle
        @param _NumberOfBlocks Number of geometry blocks
        @param _Na Number of slices in A zone
        @param _Nb Number of slices in B zone
        @param _Nc Number of slices in C zone
       */
      GeometrySettings(const Ogre::Vector2& _Height, const float& _Radius,
          const Ogre::Radian& _Alpha = Ogre::Degree(12),
          const Ogre::Radian& _Beta = Ogre::Degree(40),
          const int& _NumberOfBlocks = 12, const int& _Na = 10,
          const int& _Nb = 8, const int& _Nc = 6)
        : Height(_Height)
        , Alpha(_Alpha), Beta(_Beta)
        , Radius(_Radius)
        , NumberOfBlocks(_NumberOfBlocks)
        , Na(_Na), Nb(_Nb), Nc(_Nc)
      {
      }
    };

    /** Camera data struct
     */
    struct CameraData
    {
    public:
      /** Default constructor
       */
      inline CameraData()
        : camera(0)
        , lastPosition(Ogre::Vector3(0, 0, 0))
        , cameraOffset(Ogre::Vector2(0, 0))
        , geometryDisplacement(Ogre::Vector3(0, 0, 0))
      {
      }

      /** Constructor
          @param c Camera
       */
      explicit inline CameraData(Ogre::Camera* c)
        : camera(c)
        , lastPosition(c->getDerivedPosition())
        , cameraOffset(Ogre::Vector2(0, 0))
        , geometryDisplacement(Ogre::Vector3(0, 0, 0))
      {
      }

      /// Camera
      Ogre::Camera* camera;
      /// Last camera position
      Ogre::Vector3 lastPosition;
      /// Camera offset
      Ogre::Vector2 cameraOffset;
      /// Geometry displacement
      Ogre::Vector3 geometryDisplacement;
    };

    /** Simple constructor
      @param sm Scene manager
     */
    VClouds(Ogre::SceneManager *sm);

    /** Destructor
     */
    ~VClouds();

    /** Create
     */
    void create();

    /** Create
      @param gs Geometry settings
     */
    void create(const GeometrySettings& gs);

    /** Create
      @param Height x = Cloud field y-coord start,
       y: Field height (both in world coordinates)
      @param Radius Radius
     */
    void create(const Ogre::Vector2& Height, const float& Radius);

    /** Remove
     */
    void remove();

    /** Update, to be invoked per frame
        @param timeSinceLastFrame Time since last frame
         */
        void update(const Ogre::Real& timeSinceLastFrame);

    /** Notify camera render, to be invoked per-camera and per-frame
      @param c Rendering camera
      @param timeSinceLastCameraFrame Time since last CAMERA frame
         */
        void notifyCameraRender(Ogre::Camera* c,
                                const Ogre::Real& timeSinceLastCameraFrame);

    /** Register camera
        @param c Camera
      @remarks If a rendering camera is used(in notifyCameraRender(...))
      without having registered it before,
      all will work as expected but a warning will be logged since the
      user should manually unregister the camera one time it'll be remove
     */
    void registerCamera(Ogre::Camera* c);

    /** Unregister camera
        @param c Camera
      @remarks After having used a camera (i.e. before removing the camera),
      the user should manually unregister it
     */
    void unregisterCamera(Ogre::Camera* c);

    /** Has been create() already called?
        @return true if created() have been already called, false if not
     */
    inline const bool& isCreated() const
    {
      return mCreated;
    }

    /** Set geometry settings
        @param GeometrySettings Geometry settings
      @remarks Set geometry settings before call create(...)
     */
    inline void setGeometrySettings(const GeometrySettings& gs)
    {
      mGeometrySettings = gs;
    }

    /** Get geometry settings
        @return Geometry settings
     */
    inline const GeometrySettings& getGeometrySettings() const
    {
      return mGeometrySettings;
    }

    /** Set distance falling params
        @param DistanceFallingParams
        DistanceFallingParams.x = Distance falling factor (How much the cloud field geometry falls with the distance)
        Remember that the geometry falling is relative to the distance(height) between the camera
        and the cloud field. Typical range is [0, ~2] 0 = no falling
        DistanceFallingParams.y = Max falling (in world coords), useful when , i.e., you've water and you want to go in.
        That param will allow you to avoid the cloud field geometry falls into the ocean.
        -1 means not max falling. (default)
      @remarks See GoemetryBlock::_setVertexData(...) for more info
    */
    inline void setDistanceFallingParams(
        const Ogre::Vector2& DistanceFallingParams)
    {
      mDistanceFallingParams = DistanceFallingParams;
    }

    /** Get distance falling params
      @return DistanceFallingParams.x = Distance falling factor
      (How much the cloud field geometry falls with the distance)
      Remember that the geometry falling is relative to the distance(height)
      between the camera
      and the cloud field.
      Typical range is [0, ~2] 0 = no falling
      DistanceFallingParams.y = Max falling (in world coords),
      useful when , i.e., you've water and you want to go in.
      That param will allow you to avoid the cloud field geometry falls
      into the ocean.
      -1 means not max falling. (default)
      @remarks See GoemetryBlock::_setVertexData(...) for more info
     */
    inline const Ogre::Vector2& getDistanceFallingParams() const
    {
      return mDistanceFallingParams;
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

    /** Set wind direction
        @param WindDirection Wind direction
     */
    inline void setWindDirection(const Ogre::Radian& WindDirection)
    {
      mWindDirection = WindDirection;
    }

    /** Get wind direction
        @return Wind direction
     */
    inline const Ogre::Radian& getWindDirection() const
    {
      return mWindDirection;
    }

    /** Get wind direction as a Vector2
        @return Wind direction
     */
    inline const Ogre::Vector2 getWindDirectionV2() const
    {
      return Ogre::Vector2(Ogre::Math::Cos(mWindDirection),
                           Ogre::Math::Sin(mWindDirection));
    }

    /** Set wind speed
        @param WindSpeed Wind speed
     */
    inline void setWindSpeed(const float& WindSpeed)
    {
      mWindSpeed = WindSpeed;
    }

    /** Get wind speed
        @return Wind speed
     */
    inline const float& getWindSpeed() const
    {
      return mWindSpeed;
    }

    /** Set sun direction
        @param SunDirection Sun direction
     */
    inline void setSunDirection(const Ogre::Vector3& SunDirection)
    {
      mSunDirection = SunDirection;
    }

    /** Get sun direction
        @return Sun direction
     */
    inline const Ogre::Vector3& getSunDirection() const
    {
      return mSunDirection;
    }

    /** Set sun color
        @param SunColor Sun color
     */
    void setSunColor(const Ogre::Vector3& SunColor);

    /** Get sun color
        @return Sun color
     */
    inline const Ogre::Vector3& getSunColor() const
    {
      return mSunColor;
    }

    /** Set ambient color
        @param AmbientColor Ambient color
     */
    void setAmbientColor(const Ogre::Vector3& AmbientColor);

    /** Get Ambient color
        @return Ambient color
     */
    inline const Ogre::Vector3& getAmbientColor() const
    {
      return mAmbientColor;
    }

    /** Set light response
        @param LightResponse
           x - Sun light power
           y - Sun beta multiplier
           z - Ambient color multiplier
           w - Distance attenuation
       */
    void setLightResponse(const Ogre::Vector4& LightResponse);

    /** Get light response
        @return Light response
     */
    inline const Ogre::Vector4& getLightResponse() const
    {
      return mLightResponse;
    }

    /** Set ambient factors
        @param AmbientFactors x - constant, y - linear, z - cuadratic, w - cubic
       */
    void setAmbientFactors(const Ogre::Vector4& AmbientFactors);

    /** Get ambient factors
        @return Ambient factors
     */
    inline const Ogre::Vector4& getAmbientFactors() const
    {
      return mAmbientFactors;
    }

    /** Set global opacity
        @param GlobalOpacity Global opacity: [0,1] range 0->Transparent cloud field
     */
    inline void setGlobalOpacity(const Ogre::Real& GlobalOpacity)
    {
      mGlobalOpacity = GlobalOpacity;
    }

    /** Get global opacity
        @return Global opacity
     */
    inline const Ogre::Real& getGlobalOpacity() const
    {
      return mGlobalOpacity;
    }

    /** Set cloud field scale
        @param CloudFieldScale Cloud field scale
     */
    inline void setCloudFieldScale(const Ogre::Real& CloudFieldScale)
    {
      mCloudFieldScale = CloudFieldScale;
    }

    /** Get cloud field scale
        @return Cloud field scale
     */
    inline const Ogre::Real& getCloudFieldScale() const
    {
      return mCloudFieldScale;
    }

    /** Set noise scale
        @param NoiseScale Noise scale
     */
    inline void setNoiseScale(const Ogre::Real& NoiseScale)
    {
      mNoiseScale = NoiseScale;
    }

    /** Get noise scale
        @return Noise scale
     */
    inline const Ogre::Real& getNoiseScale() const
    {
      return mNoiseScale;
    }

    /** Set wheater parameters
        Use this funtion to update the cloud field parameters, you'll get a
        smart and smooth transition from your old setting to your new ones.
        @param Humidity Humidity, in other words: the percentage of clouds
        in [0,1] range.
      @param AverageCloudsSize Average clouds size, for example: if
      previous wheater clouds size parameter was very different from
      new one(i.e: more little)
      only the old biggest clouds are going to be keept and the little
      ones are going to be replaced
      @param DelayedResponse false to change wheather conditions over
      several updates, true to change it at the moment
     */
    void setWheater(const float& Humidity, const float& AverageCloudsSize,
        const bool& DelayedResponse);

    /** Get wheater
        @return Wheater parameters: x = Humidity,
        y = Average clouds size, both un [0,1] range
     */
    inline const Ogre::Vector2& getWheater() const
    {
      return mWheater;
    }

    /** Set visible
        @param visible true to set VClouds visible, false to hide it
     */
    void setVisible(const bool& visible);

    /** Set enable
        @param _enabled true to set VClouds to be enabled, false to hide it
     */
    void setEnabled(bool _enabled);

    /** Is VClouds visible?
        @return true if VClouds is visible, false otherwise
     */
    inline const bool& isVisible() const
    {
      return mVisible;
    }

    /** Get scene manager
        @return Ogre::SceneManager pointer
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

    /** Get data manager
        @return Data manager
     */
    inline DataManager* getDataManager()
    {
      return mDataManager;
    }

    /** Get geometry manager
        @return Geometry manager
     */
    inline GeometryManager* getGeometryManager()
    {
      return mGeometryManager;
    }

    /** Get lightning manager
        @return Lightning manager
     */
    inline LightningManager* getLightningManager()
    {
      return mLightningManager;
    }

    /** Get cameras data
        @return Cameras data
      @remarks Only for internal use
     */
    inline std::vector<CameraData>& _getCamerasData()
    {
      return mCamerasData;
    }

  private:
    /// Ogre::SceneManager pointer
    Ogre::SceneManager *mSceneManager;

    /// Current rendering camera
    Ogre::Camera* mCamera;

    /// Has been create(...) already called?
    bool mCreated;

    /// Geometry settings
    GeometrySettings mGeometrySettings;

    /// Geometry distance falling params
    Ogre::Vector2 mDistanceFallingParams;

    /// Render queue groups
    RenderQueueGroups mRenderQueueGroups;

    /// Wind direction
    Ogre::Radian mWindDirection;
    /// Wind speed
    float mWindSpeed;

    /// Wheater parameters: x = Humidity, y = Average clouds size,
    /// both un [0,1] range
    Ogre::Vector2 mWheater;
    /// Delayed response (This param is stored to allow the user call
    /// setWheater(...) before create() )
    bool mDelayedResponse;

    /// Sun direction
    Ogre::Vector3 mSunDirection;

    /// Sun color
    Ogre::Vector3 mSunColor;
    /// Ambient color
    Ogre::Vector3 mAmbientColor;

    /** Light response:
        x - Sun light power
      y - Sun beta multiplier
        z - Ambient color multiplier
        w - Distance attenuation
       */
    Ogre::Vector4 mLightResponse;
    /** Ambient factors
        x - constant, y - linear, z - cuadratic, w - cubic
     */
    Ogre::Vector4 mAmbientFactors;

    /// Global opacity
    float mGlobalOpacity;

    /// Cloud field scale
    float mCloudFieldScale;
    /// Noise scale
    float mNoiseScale;

    /// Is VClouds visible?
    bool mVisible;

    /// Data manager
    DataManager *mDataManager;
    /// Geometry manager
    GeometryManager *mGeometryManager;
    /// Lightning manager
    LightningManager *mLightningManager;

    /// Cameras data
    std::vector<CameraData> mCamerasData;

    /// Vol. clouds material
    Ogre::MaterialPtr mVolCloudsMaterial;

    /// Vol. clouds + lightning material
    Ogre::MaterialPtr mVolCloudsLightningMaterial;
  };
}}

#endif
