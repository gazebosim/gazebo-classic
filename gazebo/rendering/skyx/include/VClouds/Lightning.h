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

#ifndef _SkyX_VClouds_Lightning_H_
#define _SkyX_VClouds_Lightning_H_

#include "Prerequisites.h"

namespace SkyX { namespace VClouds{

	class DllExport Lightning
	{
	public:
		/** Segment struct
		 */
		struct Segment
		{
		public:
			/** Default constructor
			 */
			Segment()
				: a(Ogre::Vector3())
				, b(Ogre::Vector3())
			{
			}

			/** Constructor
			    @param a_ First segment point (Start)
				@param b_ Second segment point (End)
			 */
			Segment(const Ogre::Vector3& a_, const Ogre::Vector3& b_)
				: a(a_)
				, b(b_)
			{
			}

			/// Segment start
			Ogre::Vector3 a;
			/// Segment end
			Ogre::Vector3 b;
		};

		/** Constructor
		    @param sm Scene manager
			@param sn Scene node
			@param orig Lighting origin
			@param dir Lighting direction
			@param l Lighting lenth
			@param d Divisions
			@param rec Recursivity level
			@param tm Time multiplier
			@param wm Width multiplier
			@param b Bounds
		 */
		Lightning(Ogre::SceneManager* sm, Ogre::SceneNode* sn, const Ogre::Vector3& orig, const Ogre::Vector3& dir, const Ogre::Real& l, 
			const Ogre::uint32& d, const Ogre::uint32& rec, const Ogre::Real& tm,  const Ogre::Real& wm, const Ogre::Vector2& b = Ogre::Vector2(0,1));

		/** Destructor
		 */
		~Lightning();

		/** Create
		 */
		void create();

		/** Remove
		 */
		void remove();

		/** Update
		    @param timeSinceLastFrame Time since last frame
         */
        void update(Ogre::Real timeSinceLastFrame);

		/** Get ray direction
		    @return Ray direction
		 */
		inline const Ogre::Vector3& getDirection() const
		{
			return mDirection;
		}

		/** Get ray length
		    @return Ray length
		 */
		inline const Ogre::Real& getLength() const
		{
			return mLength;
		}

		/** Get lightning intensity
		    @return Lightning intensity
		 */
		inline const Ogre::Real& getIntensity() const
		{
			return mIntensity;
		}

		/** Get billboard set
		    @return Billboard set
		 */
		inline Ogre::BillboardSet* getBillboardSet() const
		{
			return mBillboardSet;
		}

		/** Get scene node
		    @return Scene node
		 */
		inline Ogre::SceneNode* getSceneNode() const
		{
			return mSceneNode;
		}

		/** Has the ray finished?
		    @return true if the ray has finished, false otherwise
		 */
		inline const bool& isFinished() const
		{
			return mFinished;
		}

		/** Update render queue group
		    @param rqg Render queue group
		    @remarks Only for internal use. Use VClouds::setRenderQueueGroups(...) instead.
		 */
		void _updateRenderQueueGroup(const Ogre::uint8& rqg);

	private:
		/** Update data
		    @param alpha Alpha
			@param currentPos Current position
			@param parentTime Parent time
		 */
		void _updateData(const Ogre::Real& alpha, const Ogre::Real& currentPos, const Ogre::Real& parentTime);

		/// Ray origin
		Ogre::Vector3 mOrigin;
		/// Ray direction
		Ogre::Vector3 mDirection;
		/// Ray length
		Ogre::Real mLength;

		/// Real ray length (total segments length amount)
		Ogre::Real mRealLength;
		/// Number of divisions
		Ogre::uint32 mDivisions;
		/// Recursivity level
		Ogre::uint32 mRecursivity;
		/// Width multiplier
		Ogre::Real mWidthMultiplier;
		/// Ray bounds (for internal visual calculations)
		Ogre::Vector2 mBounds;
		/// Angle range (Little values -> Less derivations, bigger values -> More derivations)
		Ogre::Vector2 mAngleRange;

		/// Current elapsed time
		Ogre::Real mTime;
		/// Global time multiplier
		Ogre::Real mTimeMultiplier;
		/// Per step time multipliers
		Ogre::Vector3 mTimeMultipliers;

		/// Lightning intensity
		Ogre::Real mIntensity;

		/// Segments
		std::vector<Segment> mSegments;
		/// Children lightnings
		std::vector<Lightning*> mChildren;

		/// Billboard set
		Ogre::BillboardSet* mBillboardSet;
		/// Scene manager
		Ogre::SceneManager* mSceneManager;
		/// Scene node
		Ogre::SceneNode* mSceneNode;

		/// Has been create() already called?
		bool mCreated;
		/// Has the ray finished?
		bool mFinished;
	};

}}

#endif