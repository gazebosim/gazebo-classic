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

#include "VClouds/DataManager.h"

#include "VClouds/VClouds.h"
#include "VClouds/Ellipsoid.h"

namespace SkyX { namespace VClouds
{
	DataManager::DataManager(VClouds *vc)
		: mVClouds(vc)
		, mCellsCurrent(0)
		, mCellsTmp(0)
		, mFFRandom(0)
		, mNx(0), mNy(0), mNz(0)
		, mCurrentTransition(0)
		, mUpdateTime(10.0f)
		, mStep(0), mXStart(0), mXEnd(0)
		, mMaxNumberOfClouds(250)
		, mVolTexToUpdate(true)
		, mCreated(false)
	{
		for (int k = 0; k < 2; k++)
		{
			mVolTextures[k].setNull();
		}
	}

	DataManager::~DataManager()
	{
		remove();
	}

	void DataManager::remove()
	{
		if (!mCreated)
		{
			return;
		}

		for (int k = 0; k < 2; k++)
		{
			Ogre::TextureManager::getSingleton().remove(mVolTextures[k]->getName());
			mVolTextures[k].setNull();
		}

		_delete3DCellArray(mCellsCurrent, mNx, mNy);
		_delete3DCellArray(mCellsTmp, mNx, mNy);

		delete mFFRandom;

		mNx = mNy = mNz = 0;

		mCreated = false;
	}

	void DataManager::update(const Ogre::Real &timeSinceLastFrame)
	{
		if (mVolTexToUpdate)
		{
			mCurrentTransition += timeSinceLastFrame;

			mXEnd = static_cast<int>((mCurrentTransition / mUpdateTime)*4*mNx);
			if (mXEnd > 4*mNx)
			{
				mXEnd = 4*mNx;
			}

			if (mXEnd/mNx != mXStart/mNx)
			{
				for (int k = mStep; k <= mXEnd/mNx; k++)
				{
					_performCalculations(mNx, mNy, mNz, k, mXStart%mNx, (mXEnd/mNx != k) ? mNx : mXEnd%mNx);
					mXStart = 0;
				}
			}
			else
			{
				if (mXStart != mXEnd)
				{
					_performCalculations(mNx, mNy, mNz, mXEnd/mNx, mXStart%mNx, mXEnd%mNx);
				}
			}

			mStep = mXEnd/mNx;
			mXStart = mXEnd;
			
			if (mCurrentTransition >= mUpdateTime)
			{
				_updateVolTextureData(mCellsCurrent, VOL_TEX0, mNx, mNy, mNz);

				mCurrentTransition = mUpdateTime;
				mVolTexToUpdate = !mVolTexToUpdate;
				mStep = mXStart = mXEnd = 0;
			}
		}
		else
		{
			mCurrentTransition -= timeSinceLastFrame;

			mXEnd = static_cast<int>(((mUpdateTime-mCurrentTransition) / mUpdateTime)*4*mNx);
			if (mXEnd < 0)
			{
				mXEnd = 0;
			}

			if (mXEnd/mNx != mXStart/mNx)
			{
				for (int k = mStep; k <= mXEnd/mNx; k++)
				{
					_performCalculations(mNx, mNy, mNz, k, mXStart%mNx, (mXEnd/mNx != k) ? mNx : mXEnd%mNx);
					mXStart = 0;
				}
			}
			else
			{
				if (mXStart != mXEnd)
				{
					_performCalculations(mNx, mNy, mNz, mXEnd/mNx, mXStart%mNx, mXEnd%mNx);
				}
			}

			mStep = mXEnd/mNx;
			mXStart = mXEnd;

			if (mCurrentTransition <= 0)
			{
				_updateVolTextureData(mCellsCurrent, VOL_TEX1, mNx, mNy, mNz);

				mCurrentTransition = 0;
				mVolTexToUpdate = !mVolTexToUpdate;
				mStep = mXStart = mXEnd = 0;
			}
		}
	}

	void DataManager::create(const int &nx, const int &ny, const int &nz)
	{
		remove();

		mNx = nx; mNy = ny; mNz = nz;

		mFFRandom = new FastFakeRandom(1024, 0, 1);

		_initData(nx, ny, nz);

		for (int k = 0; k < 2; k++)
		{
			_createVolTexture(static_cast<VolTextureId>(k), nx, ny, nz);
		}

		_updateVolTextureData(mCellsCurrent, VOL_TEX0, mNx, mNy, mNz);
		_updateVolTextureData(mCellsCurrent, VOL_TEX1, mNx, mNy, mNz);

		mCreated = true;
	}

	void DataManager::forceToUpdateData()
	{
		// Finish current update process
		_performCalculations(mNx, mNy, mNz, mStep, mXStart%mNx, mNx);
		for (int k = mStep+1; k < 4; k++)
		{
			_performCalculations(mNx, mNy, mNz, k, 0, mNx);
		}
		mStep = mXStart = mXEnd = 0;

		if (mVolTexToUpdate)
		{
			_updateVolTextureData(mCellsCurrent, VOL_TEX0, mNx, mNy, mNz);
			mCurrentTransition = mUpdateTime;
		}
		else
		{
			_updateVolTextureData(mCellsCurrent, VOL_TEX1, mNx, mNy, mNz);
			mCurrentTransition = 0;
		}

		mVolTexToUpdate = !mVolTexToUpdate;
	}

	void DataManager::_initData(const int& nx, const int& ny, const int& nz)
	{
		mCellsCurrent = _create3DCellArray(nx, ny, nz);
		mCellsTmp     = _create3DCellArray(nx, ny, nz, false);
	}

	DataManager::Cell *** DataManager::_create3DCellArray(const int& nx, const int& ny, const int& nz, const bool& init)
	{
		Cell ***c = new Cell** [nx];

		int u, v, w;

		for (u = 0; u < nx; u++)
		{
			c[u] = new Cell* [ny];

			for (v = 0; v < ny; v++)
			{
				 c[u][v] = new Cell[nz];
			}
		}

		if (!init)
		{
			return c;
		}

		for (u = 0; u < nx; u++)
		{
			for (v = 0; v < ny; v++)
			{
				for (w = 1; w < nz; w++)
				{
					c[u][v][w].act = false;
					c[u][v][w].cld = false;
					c[u][v][w].hum = false;

					c[u][v][w].pact = 0;
					c[u][v][w].pext = 1;
					c[u][v][w].phum = 0;

					c[u][v][w].dens = 0.0f;
					c[u][v][w].light = 1.0f;
				}
			}
		}

		return c;
	}

	void DataManager::_delete3DCellArray(Cell ***c, const int& nx, const int& ny)
	{
		int u, v;

		for (u = 0; u < nx; u++)
		{
			for (v = 0; v < ny; v++)
			{
				delete [] c[u][v];
			}

			delete [] c[u];
		}

		delete [] c;
	}

	void DataManager::_copy3DCellArraysData(Cell ***src, Cell ***dest, const int& nx, const int& ny, const int& nz)
	{
		int u, v, w;

		for (u = 0; u < nx; u++)
		{
			for (v = 0; v < ny; v++)
			{
				for (w = 0; w < nz; w++)
				{
					dest[u][v][w].act = src[u][v][w].act;
					dest[u][v][w].cld = src[u][v][w].cld;
					dest[u][v][w].hum = src[u][v][w].hum;

					dest[u][v][w].pact = src[u][v][w].pact;
					dest[u][v][w].pext = src[u][v][w].pext;
					dest[u][v][w].phum = src[u][v][w].phum;

					dest[u][v][w].dens = src[u][v][w].dens;
					dest[u][v][w].light = src[u][v][w].light;
				}
			}
		}
	}

	void DataManager::setWheater(const float& Humidity, const float& AverageCloudsSize, const bool& delayedResponse)
	{
		int numberofclouds = static_cast<int>(Humidity * mMaxNumberOfClouds);
		Ogre::Vector3 maxcloudsize = AverageCloudsSize*Ogre::Vector3(mNx/14, mNy/14, static_cast<int>(static_cast<float>(mNz)/2.75));

		// Update old clouds with new parameters
		Ogre::Vector3 currentdimensions, currentPosition;
		std::vector<Ellipsoid*>::const_iterator mEllipsoidsIt;

		for(mEllipsoidsIt = mEllipsoids.begin(); mEllipsoidsIt != mEllipsoids.end(); mEllipsoidsIt++)
		{
			// Update size
			currentdimensions = (*mEllipsoidsIt)->getDimensions();

			if (currentdimensions.x / maxcloudsize.x < 0.5 || currentdimensions.x / maxcloudsize.x > 2)
			{
				currentdimensions.x = maxcloudsize.x + Ogre::Math::RangeRandom(-0.2,0.2)*maxcloudsize.x;
			}
			if (currentdimensions.y / maxcloudsize.y < 0.5 || currentdimensions.y / maxcloudsize.y > 2)
			{
				currentdimensions.y = maxcloudsize.y + Ogre::Math::RangeRandom(-0.2,0.2)*maxcloudsize.y;
			}
			if (currentdimensions.z / maxcloudsize.z < 0.5 || currentdimensions.z / maxcloudsize.z > 2)
			{
				currentdimensions.z = maxcloudsize.z + Ogre::Math::RangeRandom(-0.2,0.15)*maxcloudsize.z;
			}

			(*mEllipsoidsIt)->setDimensions(currentdimensions);

			// Update position
			currentPosition = (*mEllipsoidsIt)->getPosition();
			(*mEllipsoidsIt)->setPosition(Ogre::Vector3(currentPosition.x,currentPosition.y,static_cast<int>(Ogre::Math::RangeRandom(currentdimensions.z+2,mNz-currentdimensions.z-2))));
		}

		// Remove some clouds if needed
		while (static_cast<unsigned int>(numberofclouds) < mEllipsoids.size())
		{
			mEllipsoids.pop_back();
		}

		// Add new clouds if needed
		Ogre::Vector3 newclouddimensions;
		while (static_cast<unsigned int>(numberofclouds) > mEllipsoids.size())
		{
			newclouddimensions = maxcloudsize*Ogre::Vector3(Ogre::Math::RangeRandom(0.5, 2), Ogre::Math::RangeRandom(0.5, 2), Ogre::Math::RangeRandom(0.75, 1));
			addEllipsoid(new Ellipsoid(newclouddimensions.x,  newclouddimensions.y,  newclouddimensions.z, mNx, mNy, mNz, (int)Ogre::Math::RangeRandom(0, mNx), (int)Ogre::Math::RangeRandom(0, mNy), static_cast<int>(Ogre::Math::RangeRandom(newclouddimensions.z+2,mNz-newclouddimensions.z-2)), Ogre::Math::RangeRandom(1,5.0f)), false);
		}

		_updateProbabilities(mCellsCurrent, mNx, mNy, mNz, delayedResponse);

		if (!delayedResponse)
		{
			for (int k = 0; k < 4; k++)
			{
				_performCalculations(mNx, mNy, mNz, k, 0, mNx);
			}
			mStep = mXStart = mXEnd = 0;

			_updateVolTextureData(mCellsCurrent, VOL_TEX0, mNx, mNy, mNz);
			_updateVolTextureData(mCellsCurrent, VOL_TEX1, mNx, mNy, mNz);
		}
	}

	void DataManager::addEllipsoid(Ellipsoid *e, const bool& UpdateProbabilities)
	{
		mEllipsoids.push_back(e);

		if (UpdateProbabilities)
		{
			e->updateProbabilities(mCellsCurrent,mNx,mNy,mNz);
		}
	}

	void DataManager::_clearProbabilities(Cell*** c, const int& nx, const int& ny, const int& nz, const bool& clearData)
	{
		int u, v, w;

		for (u = 0; u < nx; u++)
		{
			for (v = 0; v < ny; v++)
			{
				for (w = 0; w < nz; w++)
				{
					c[u][v][w].pact = 0;
					c[u][v][w].pext = 1;
					c[u][v][w].phum = 0;

					if (clearData)
					{
						c[u][v][w].act = false;
						c[u][v][w].cld = false;
						c[u][v][w].hum = false;

						c[u][v][w].dens = 0;
						c[u][v][w].light = 0;
					}
				}
			}
		}
	}

	void DataManager::_updateProbabilities(Cell*** c, const int& nx, const int& ny, const int& nz, const bool& delayedResponse)
	{
		_clearProbabilities(c,nx,ny,nz,!delayedResponse);

		std::vector<Ellipsoid*>::const_iterator mEllipsoidsIt;

		for(mEllipsoidsIt = mEllipsoids.begin(); mEllipsoidsIt != mEllipsoids.end(); mEllipsoidsIt++)
		{
			(*mEllipsoidsIt)->updateProbabilities(c,nx,ny,nz,delayedResponse);
		}
	}

	const Ogre::Real DataManager::_getLightAbsorcionAt(Cell*** c, const int& nx, const int& ny, const int& nz, const int& x, const int& y, const int& z, const Ogre::Vector3& d, const float& att) const
	{
		Ogre::Real step = 1, factor = 1;
		Ogre::Vector3 pos = Ogre::Vector3(x, y, z);
		bool outOfBounds = false;
		int u, v, uu, vv,
		    current_iteration = 0, max_iterations = 8;

		while(!outOfBounds)
		{
			if ( (int)pos.z >= nz || (int)pos.z < 0 || factor <= 0 || current_iteration >= max_iterations)
			{
				outOfBounds = true;
			}
			else
			{
				u = (int)pos.x; v = (int)pos.y;

				uu = (u<0) ? (u + nx) : u; if (u>=nx) { uu-= nx; }
				vv = (v<0) ? (v + ny) : v; if (v>=ny) { vv-= ny; }

				factor -= c[uu][vv][(int)pos.z].dens*att*(1-static_cast<float>(current_iteration)/max_iterations);
				pos += step*(-d);

				current_iteration++;
			}
		}

		return Ogre::Math::Clamp<Ogre::Real>(factor,0,1);
	}

	void DataManager::_performCalculations(const int& nx, const int& ny, const int& nz, const int& step, const int& xStart, const int& xEnd)
	{
		int u, v, w;

		switch (step)
		{
			case 0:
			{
				for (u = xStart; u < xEnd; u++)
				{
					for (v = 0; v < ny; v++)
					{
						for (w = 0; w < nz; w++)
						{
							// ti+1                       ti
							mCellsCurrent[u][v][w].hum = mCellsCurrent[u][v][w].hum || (mFFRandom->get() < mCellsCurrent[u][v][w].phum);
							mCellsCurrent[u][v][w].cld = mCellsCurrent[u][v][w].cld && (mFFRandom->get() > mCellsCurrent[u][v][w].pext);
							mCellsCurrent[u][v][w].act = mCellsCurrent[u][v][w].act || (mFFRandom->get() < mCellsCurrent[u][v][w].pact);

							// Copy act in the temporal buffer, for _fact(...)
							mCellsTmp[u][v][w].act = mCellsCurrent[u][v][w].act;
						}
					}
				}
			}
			break;
			case 1:
			{
				for (u = xStart; u < xEnd; u++)
				{
					for (v = 0; v < ny; v++)
					{
						for (w = 0; w < nz; w++)
						{
							// ti+1                       ti
							mCellsCurrent[u][v][w].hum =  mCellsCurrent[u][v][w].hum && !mCellsCurrent[u][v][w].act;
							mCellsCurrent[u][v][w].cld =  mCellsCurrent[u][v][w].cld ||  mCellsCurrent[u][v][w].act;
							mCellsCurrent[u][v][w].act = !mCellsCurrent[u][v][w].act &&  mCellsCurrent[u][v][w].hum && _fact(mCellsTmp, nx, ny, nz, u,v,w);
						}
					}
				}
			}
			break;
			case 2:
			{
				// Continous density
				for (u = xStart; u < xEnd; u++)
				{
					for (v = 0; v < ny; v++)
					{
						for (w = 0; w < nz; w++)
						{
						   mCellsCurrent[u][v][w].dens = _getDensityAt(mCellsCurrent, nx, ny, nz, u,v,w, 1/*TODOOOO!!!*/, 1.15f);
						  // mCellsCurrent[u][v][w].dens = _getDensityAt(mCellsCurrent,u,v,w);
						}
					}
				}
			}
			break;
			case 3:
			{
				// Light scattering
				Ogre::Vector3 SunDir = Ogre::Vector3(mVClouds->getSunDirection().x, mVClouds->getSunDirection().z, mVClouds->getSunDirection().y);

				for (u = xStart; u < xEnd; u++)
				{
					for (v = 0; v < ny; v++)
					{
						for (w = 0; w < nz; w++)
						{
							mCellsCurrent[u][v][w].light = _getLightAbsorcionAt(mCellsCurrent, nx, ny, nz, u,v,w, SunDir, 0.15f/*TODO!!!!*/);
						}
					}
				}
			}
			break;
		}
	}

	const bool DataManager::_fact(Cell ***c, const int& nx, const int& ny, const int& nz, const int& x, const int& y, const int& z) const
	{
		bool i1m, j1m, k1m, 
			 i1r, j1r, k1r, 
			 i2r, i2m, j2r, j2m, k2r;

		i1m = ((x+1)>=nx) ? c[0][y][z].act : c[x+1][y][z].act;
		j1m = ((y+1)>=ny) ? c[x][0][z].act : c[x][y+1][z].act;
		k1m = ((z+1)>=nz) ? false : c[x][y][z+1].act;

		i1r = ((x-1)<0) ? c[nx-1][y][z].act : c[x-1][y][z].act;
		j1r = ((y-1)<0) ? c[x][ny-1][z].act : c[x][y-1][z].act;
		k1r = ((z-1)<0) ? false : c[x][y][z-1].act;

		i2r = ((x-2)<0) ? c[nx-2][y][z].act : c[x-2][y][z].act;
		j2r = ((y-2)<0) ? c[x][ny-2][z].act : c[x][y-2][z].act;
		k2r = ((z-2)<0) ? false : c[x][y][z-2].act;

		i2m = ((x+2)>=nx) ? c[1][y][z].act : c[x+2][y][z].act;
		j2m = ((y+2)>=ny) ? c[x][1][z].act : c[x][y+2][z].act;

		return i1m || j1m || k1m  || i1r || j1r || k1r || i2r || i2m || j2r || j2m || k2r;
	}

	const float DataManager::_getDensityAt(Cell ***c, const int& nx, const int& ny, const int& nz, const int& x, const int& y, const int& z, const int& r, const float& strength) const
	{		
		int zr = ((z-r)<0) ? 0 : z-r,
			zm = ((z+r)>=nz) ? nz : z+r,
            u, uu, v, vv, w, 
			clouds = 0, div = 0;

		for (u = x-r; u <= x+r; u++)
		{
			for (v = y-r; v <= y+r; v++)
			{
				for (w = zr; w < zm; w++)
				{
					// x/y Seamless!
					uu = (u<0) ? (u + nx) : u; if (u>=nx) { uu-= nx; }
					vv = (v<0) ? (v + ny) : v; if (v>=ny) { vv-= ny; }

					clouds += c[uu][vv][w].cld ? 1 : 0;
					div++;
				}
			}
		}

		return Ogre::Math::Clamp<float>(strength*((float)clouds)/div, 0, 1);
	}

	const float DataManager::_getDensityAt(Cell ***c, const int& x, const int& y, const int& z) const
	{
		return c[x][y][z].cld ? 1.0f : 0.0f;
	}

	void DataManager::_createVolTexture(const VolTextureId& TexId, const int& nx, const int& ny, const int& nz)
	{
		mVolTextures[static_cast<int>(TexId)] 
		    = Ogre::TextureManager::getSingleton().
				createManual("_SkyX_VolCloudsData"+Ogre::StringConverter::toString(TexId),
				SKYX_RESOURCE_GROUP, 
				Ogre::TEX_TYPE_3D,
				nx, ny, nz, 0,
				Ogre::PF_BYTE_RGB);

		mVolTextures[static_cast<int>(TexId)]->load();

		static_cast<Ogre::MaterialPtr>(
			Ogre::MaterialManager::getSingleton().getByName("SkyX_VolClouds"))
			->getTechnique(0)->getPass(0)->getTextureUnitState(static_cast<int>(TexId))
				->setTextureName("_SkyX_VolCloudsData"+Ogre::StringConverter::toString(TexId), Ogre::TEX_TYPE_3D);
		static_cast<Ogre::MaterialPtr>(
			Ogre::MaterialManager::getSingleton().getByName("SkyX_VolClouds_Lightning"))
			->getTechnique(0)->getPass(0)->getTextureUnitState(static_cast<int>(TexId))
				->setTextureName("_SkyX_VolCloudsData"+Ogre::StringConverter::toString(TexId), Ogre::TEX_TYPE_3D);
	}

	void DataManager::_updateVolTextureData(Cell ***c, const VolTextureId& TexId, const int& nx, const int& ny, const int& nz)
	{
		Ogre::HardwarePixelBufferSharedPtr buffer = mVolTextures[TexId]->getBuffer(0,0);
		
		buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
		const Ogre::PixelBox &pb = buffer->getCurrentLock();

		Ogre::uint32 *pbptr = static_cast<Ogre::uint32*>(pb.data);
		size_t x, y, z;

		for (z=pb.front; z<pb.back; z++) 
        {
            for (y=pb.top; y<pb.bottom; y++)
            {
                for (x=pb.left; x<pb.right; x++)
				{
					Ogre::PixelUtil::packColour(c[x][y][z].dens/* TODO!!!! */, c[x][y][z].light, 0, 0, pb.format, &pbptr[x]);
                } 
                pbptr += pb.rowPitch;
            }
            pbptr += pb.getSliceSkip();
        }

		buffer->unlock();
	}
}}