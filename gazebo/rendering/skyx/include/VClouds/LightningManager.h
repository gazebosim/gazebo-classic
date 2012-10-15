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

#ifndef _SkyX_VClouds_LightningManager_H_
#define _SkyX_VClouds_LightningManager_H_

#include <vector>

#include "Prerequisites.h"

#include "Lightning.h"

namespace SkyX { namespace VClouds {

  class VClouds;

  class DllExport LightningManager
  {
  public:
    /** Listener class
     */
    class Listener
    {
    public:
      /** Lightning added
        @param l Lightning that has been added
        @remarks Useful when, for example, the app needs to known when a
        Lightning has been created (by manually invoking
        LightningManager::addLightning(...) or automatically based on the
        lightning creation probabilities) in order to play a sound, etc.
        The lightning position is accessible through
        Lightning::getSceneNode()->getPosition().
       */
      inline virtual void lightningAdded(Lightning* l) {}
      virtual ~Listener() {}
    };

    /** Constructor
      @param vc VClouds pointer
     */
    explicit LightningManager(VClouds* vc);

    /** Destructor
     */
    ~LightningManager();

    /** Create
     */
    void create();

    /** Remove
     */
    void remove();

    /** Update, to be invoked per frame
        @param timeSinceLastFrame Time since last frame
         */
        void update(const Ogre::Real& timeSinceLastFrame);

    /** Add lightning
      @param p Lightning position
      @param d Lightning direction
      @param l Lightning length
      @return The lightning or null in error case (the max number of
      simultaneous lightnings is 3)
      @remarks The lightning will be automatically destroyed one time
      it'll be finished, so the returned ptr will not
      be available one time the lightning will have disappeared
     */
    Lightning* addLightning(const Ogre::Vector3& p, const Ogre::Vector3& d,
        const Ogre::Real l,
        const Ogre::uint32& div = static_cast<Ogre::uint32>(
          Ogre::Math::RangeRandom(12, 30)));

    /** Update material
        @remarks To be invoked before each camera rendering process
     */
    void updateMaterial();

    /** Add listener
        @param listener Listener
     */
    inline void addListener(Listener* listener)
    {
      mListeners.push_back(listener);
    }

    /** Remove listener
        @param listener Listener to be removed
     */
    void removeListener(Listener* listener);

    /** Remove listeners
     */
    inline void removeListeners()
    {
      mListeners.clear();
    }

    /** Get listeners
        @return Listeners
     */
    inline const std::vector<Listener*>& getListeners() const
    {
      return mListeners;
    }

    /** Enable or disable the lightning system
        @param enable true to enable the lightning system, false otherwise
     */
    void setEnabled(const bool& enable);

    /** Get whether the lightning system is enabled or not
        @return true if the lightning system is enabled, false otherwise
      @remarks Even if the lightning system is disabled, you'll be able to manually add lightnings trhough LightningManager::addLightning(...)
     */
    inline const bool& isEnabled() const
    {
      return mEnabled;
    }

    /** Set lightning color
        @param c Lightning color
     */
    void setLightningColor(const Ogre::Vector3& c);

    /** Get lightning color
        @return Lightning color
     */
    inline const Ogre::Vector3& getLightningColor() const
    {
      return mLightningColor;
    }

    /** Set lightning time multiplier
        @param c Lightning time multiplier
      @remarks Changes applies to new lightnings, not to existing ones
     */
    void setLightningTimeMultiplier(const Ogre::Real& m)
    {
      mLightningTimeMultiplier = m;
    }

    /** Set lightning time multiplier
        @return Lightning time multiplier
     */
    inline const Ogre::Real& getLightningTimeMultiplier() const
    {
      return mLightningTimeMultiplier;
    }

    /** Set average lightning apparition time
        @param alat Average lightning apparition time
     */
    inline void setAverageLightningApparitionTime(const Ogre::Real& alat)
    {
       mAverageLightningApparitionTime = alat;
       mRemainingTime = alat;
    }

    /** Get average lightning apparition time
        @return Average lightning apparition time
     */
    inline const Ogre::Real& getAverageLightningApparitionTime() const
    {
      return mAverageLightningApparitionTime;
    }

    /** Has been create() already called?
        @return true if created() have been already called, false if not
     */
    inline const bool& isCreated() const
    {
      return mCreated;
    }

    /** Update render queue group
        @param rqg Render queue group
        @remarks Only for internal use.
        Use VClouds::setRenderQueueGroups(...) instead.
     */
    void _updateRenderQueueGroup(const Ogre::uint8& rqg);

    /** Set visible
        @param v Visible?
      @remarks Only for internal use. Use VClouds::setVisible(...) instead.
     */
    void _setVisible(const bool& v);

  private:
    /// VClouds pointer
    VClouds *mVClouds;

    /// Lightnings
    std::vector<Lightning*> mLightnings;
    /// Scene nodes
    std::vector<Ogre::SceneNode*> mSceneNodes;

    /// Is the lightning system enabled?
    bool mEnabled;

    /// Lightning color
    Ogre::Vector3 mLightningColor;
    /// Lightning time multiplier
    Ogre::Real mLightningTimeMultiplier;

    /// Average lightning apparition time (in seconds)
    Ogre::Real mAverageLightningApparitionTime;
    /// Remaining time for next lightning
    Ogre::Real mRemainingTime;

    /// Vol. clouds + lightning material
    Ogre::MaterialPtr mVolCloudsLightningMaterial;
    /// Lightning material
    Ogre::MaterialPtr mLightningMaterial;

    /// Listeners
    std::vector<Listener*> mListeners;

    /// Has been create() already called?
    bool mCreated;
  };
}}
#endif
