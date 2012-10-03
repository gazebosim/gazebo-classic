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

#include "VClouds/Lightning.h"

namespace SkyX { namespace VClouds
{
	Lightning::Lightning(Ogre::SceneManager* sm, Ogre::SceneNode* sn, const Ogre::Vector3& orig, const Ogre::Vector3& dir, 
		const Ogre::Real& l, const Ogre::uint32& d, const Ogre::uint32& rec, const Ogre::Real& tm, const Ogre::Real& wm, const Ogre::Vector2& b)
		: mOrigin(orig)
		, mDirection(dir)
		, mLength(l)
		, mRealLength(0)
		, mDivisions(d)
		, mRecursivity(rec)
		, mTime(0)
		, mTimeMultiplier(tm)
		, mIntensity(0)
		, mWidthMultiplier(wm)
		, mBounds(b)
		, mAngleRange(Ogre::Vector2(Ogre::Math::RangeRandom(0.3,0.5), Ogre::Math::RangeRandom(0.6,0.8)))
		, mTimeMultipliers(Ogre::Vector3(Ogre::Math::RangeRandom(1.75,4.25), Ogre::Math::RangeRandom(0.4,1.25f), Ogre::Math::RangeRandom(0.2,1.0f)))
		, mSegments(std::vector<Lightning::Segment>())
		, mChildren(std::vector<Lightning*>())
		, mBillboardSet(0)
		, mSceneManager(sm)
		, mSceneNode(sn)
		, mCreated(false)
		, mFinished(false)
	{
	}

	Lightning::~Lightning()
	{
		remove();
	}

	void Lightning::create()
	{
		remove();

		Ogre::Vector3 end = mOrigin + mDirection*mLength;
		Ogre::Vector3 current, last = mOrigin;

		// Create ray segments
		for(Ogre::uint32 k = 1; k < mDivisions+1; k++)
		{
			Ogre::Vector3 current = mOrigin + mDirection*mLength*(static_cast<Ogre::Real>(k)/mDivisions);

			current += (mLength/(mDivisions*3))*Ogre::Vector3(
				Ogre::Math::RangeRandom(-1, 1), Ogre::Math::RangeRandom(-1, 1), Ogre::Math::RangeRandom(-1, 1));

			mSegments.push_back(Segment(last, current));

			mRealLength += (current-last).length();

			last = current;
		}

		// Create the associated billboard set
		mBillboardSet = mSceneManager->createBillboardSet();
		mBillboardSet->setMaterialName("SkyX_Lightning");
		mBillboardSet->setBillboardType(Ogre::BBT_ORIENTED_SELF);

		Ogre::Real width = mWidthMultiplier*3*(static_cast<Ogre::Real>(mRecursivity)/4+1)*Ogre::Math::RangeRandom(0.5f, 2.5f-mRecursivity/3);

		// Create the associated billboard for each segment
		Ogre::Real delta;
		Ogre::Vector2 bounds;
		Ogre::Billboard* bb;
		for(Ogre::uint32 k = 0; k < mSegments.size(); k++)
		{
			delta = 1.0f / mSegments.size();
			bounds = Ogre::Vector2(k*delta,(k+1)*delta);

			bounds = Ogre::Vector2(mBounds.x, mBounds.x) + bounds*(mBounds.y-mBounds.x);

			bb = mBillboardSet->createBillboard((mSegments.at(k).a+mSegments.at(k).b)/2);
			bb->setDimensions(width, (mSegments.at(k).a-mSegments.at(k).b).length());
			bb->setColour(Ogre::ColourValue(0,bounds.x,bounds.y));
			bb->mDirection = (mSegments.at(k).a-mSegments.at(k).b).normalisedCopy();

			bb = mBillboardSet->createBillboard(mSegments.at(k).a + (mSegments.at(k).a-mSegments.at(k).b).normalisedCopy()*width/2);
			bb->setDimensions(width, width);
			bb->setColour(Ogre::ColourValue(1,bounds.x,bounds.x));
			bb->mDirection = (mSegments.at(k).a-mSegments.at(k).b).normalisedCopy();
			
			bb = mBillboardSet->createBillboard(mSegments.at(k).b - (mSegments.at(k).a-mSegments.at(k).b).normalisedCopy()*width/2);
			bb->setDimensions(width, width);
			bb->setColour(Ogre::ColourValue(1,bounds.y,bounds.y));
			bb->mDirection = -(mSegments.at(k).a-mSegments.at(k).b).normalisedCopy();
		
			width *= 1-(1.0f/(mRecursivity*mRecursivity+1.0f))*(1.0f/mSegments.size());
		}

		mBillboardSet->_updateBounds();

		mSceneNode->attachObject(mBillboardSet);

		mBillboardSet->setCustomParameter(0, Ogre::Vector4(1,0,0,0));

		// Ramifications
		if (mRecursivity > 0)
		{
			Ogre::Real angle;
			Ogre::Vector3 dir;
			Ogre::Real lengthMult;
			for (Ogre::uint32 k = 0; k < mDivisions-1; k++)
			{
				angle = (mSegments.at(k).b-mSegments.at(k).a).normalisedCopy().dotProduct(
					((mSegments.at(k+1).b-mSegments.at(k+1).a).normalisedCopy()));

				if (angle < Ogre::Math::RangeRandom(mAngleRange.x, mAngleRange.y))
				{
					dir = (mSegments.at(k).b-mSegments.at(k).a).normalisedCopy();
					dir.x *= Ogre::Math::RangeRandom(0.8f, 1.2f);
					dir.y *= Ogre::Math::RangeRandom(0.8f, 1.2f);
					dir.z *= Ogre::Math::RangeRandom(0.8f, 1.2f);
					dir.normalise();

					delta = 1.0f / mSegments.size();
					bounds = Ogre::Vector2(mBounds.x+(mBounds.y-mBounds.x)*(k+1)*delta,1);

					lengthMult = Ogre::Math::RangeRandom(0.1f, 0.7f);

					Lightning* lightning = new Lightning(mSceneManager, mSceneNode, mSegments.at(k).b, dir, lengthMult*mLength, 2+mDivisions*lengthMult, mRecursivity-1, mTimeMultiplier, mWidthMultiplier, bounds);
					lightning->create();
					
					mChildren.push_back(lightning);
				}
			}
		}

		mCreated = true;
	}

	void Lightning::remove()
	{
		if (!mCreated)
		{
			return;
		}

		mSceneNode->detachObject(mBillboardSet);
		mSceneManager->destroyBillboardSet(mBillboardSet);

		mSegments.clear();

		for(Ogre::uint32 k = 0; k < mChildren.size(); k++)
		{
			delete mChildren.at(k);
		}

		mChildren.clear();

		mFinished = false;

		mCreated = false;
	}

	void Lightning::update(Ogre::Real timeSinceLastFrame)
	{
		if (!mCreated)
		{
			return;
		}

		timeSinceLastFrame *= mTimeMultiplier;

		// mTime timeline: (Note: Time multipliers are random)
		// 0.0 -> 1.2 : Ray creation(fordward effect) + Big flash
		// 1.2 -> 2.0 : Sinus flashing pattern
		// 2.0 -> 3.0 Ray fading

		Ogre::Real alpha = 0.5f;
		Ogre::Real maxAlpha = 1.5f;

		if (mTime < 1)
		{
			mTime += timeSinceLastFrame*mTimeMultipliers.x;

			if (mTime > 2) mTime = 1.5f; // Prevent big changes

			if (mTime > 0.8f) // Big flash start
			{
				alpha += (mTime-0.8f)*(maxAlpha/0.2f);
			}
		}
		else if (mTime > 1 && mTime < 2)
		{
			mTime += timeSinceLastFrame*mTimeMultipliers.y;

			if (mTime > 3) mTime = 2.5f; // Prevent big changes

			if (mTime < 1.2f) // Big flash end
			{
				alpha += (0.2f-(mTime-1.0f))*(maxAlpha/0.2f);
			}
			else // Sinus flashing pattern
			{
				alpha += Ogre::Math::Abs(Ogre::Math::Sin((mTime-1.2f)*1.5f*mTimeMultipliers.x));
			}
		}
		else if (mTime > 2) // Ray fading
		{
			mTime += timeSinceLastFrame*mTimeMultipliers.z;

			 if (mTime > 3) 
			 {
				 mTime = 3; // Prevent big changes
				 mFinished = true;
			 }

			alpha += Ogre::Math::Abs(Ogre::Math::Sin((2-1.2f)*1.5f*mTimeMultipliers.x));
			alpha *= 3.0f-mTime;
		}

		mIntensity = alpha;

		_updateData(alpha, mTime > 1 ? 1 : mTime, mTime);
	}

	void Lightning::_updateRenderQueueGroup(const Ogre::uint8& rqg)
	{
		mBillboardSet->setRenderQueueGroup(rqg);

		for(Ogre::uint32 k = 0; k < mChildren.size(); k++)
		{	
			mChildren.at(k)->_updateRenderQueueGroup(rqg);
		}
	}

	void Lightning::_updateData(const Ogre::Real& alpha, const Ogre::Real& currentPos, const Ogre::Real& parentTime)
	{
		Ogre::Vector4 params = Ogre::Vector4(alpha,currentPos,(3-mRecursivity)*0.075f+(mBounds.x*1.5f+0.2f)*0.85f,0);

		if (parentTime > 1 && parentTime < 2.2f)
		{
			params.z *= Ogre::Math::Clamp<Ogre::Real>(1.5-parentTime,0.2f,1);
		}
		else if (parentTime > 2.2f)
		{
			 params.z *= Ogre::Math::Clamp<Ogre::Real>((-2.2f+parentTime)*1.25f,0.2f,1);
		}

		mBillboardSet->setCustomParameter(0, params);

		for(Ogre::uint32 k = 0; k < mChildren.size(); k++)
		{
			mChildren.at(k)->_updateData(alpha*0.75f, currentPos, parentTime);
		}
	}
}}