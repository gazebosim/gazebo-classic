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

#ifndef _SkyX_VClouds_DataManager_H_
#define _SkyX_VClouds_DataManager_H_

#include "Prerequisites.h"

#include "VClouds/FastFakeRandom.h"

namespace SkyX { namespace VClouds{

	class VClouds;
	class Ellipsoid;

	class DllExport DataManager 
	{
	public:
		/** Cell struct
		 */
		struct Cell
		{
			/// Humidity, phase and cloud
			bool hum, act, cld;

			/// Probabilities
			float phum, pext, pact;

			/// Continous density
			float dens;

			/// Light absorcion
			float light;
		};

		/** Volumetric textures enumeration
		 */
		enum VolTextureId
		{
			VOL_TEX0 = 0,
			VOL_TEX1 = 1
		};

		/** Constructor
		    @param vc VClouds parent pointer
		 */
		DataManager(VClouds *vc);

		/** Destructor
		 */
		~DataManager();

		/** Create
		    @param nx X complexity
			@param ny Y complexity
			@param nz Z complexity
		 */
		void create(const int& nx, const int& ny, const int& nz);

		/** Update
			@param timeSinceLastFrame Time elapsed since last frame
		 */
		void update(const Ogre::Real &timeSinceLastFrame);

		/** Remove
		 */
		void remove();

		/** Has been create(...) already called?
		    @return true if created() have been already called
		 */
		inline const bool& isCreated() const
		{
			return mCreated;
		}

		/** Set update time
		    @param UpdateTime Time elapsed between data calculations, a little freeze could be experimented during these calculations on old CPU's
		 */
		inline void setUpdateTime(const float& UpdateTime)
		{
			mUpdateTime = UpdateTime;
		}

		/** Get update time
		    @return Update time
		 */
		inline const Ogre::Real& getUpdateTime() const
		{
			return mUpdateTime;
		}

		/** Get current interpolation factor
		    @return Interpolation factor
			@remarks Only for internal use
		 */
		inline const Ogre::Real _getInterpolation() const
		{
			return mCurrentTransition/mUpdateTime;
		}

		/** Set wheater parameters
		    Use this funtion to update the cloud field parameters, you'll get a smart and smooth transition from your old 
			setting to your new ones.
			@param Humidity Humidity, in other words: the percentage of clouds in [0,1] range.
			@param AverageCloudsSize Average clouds size, for example: if previous wheater clouds size parameter was very different from new one(i.e: more little)
			       only the old biggest clouds are going to be keept and the little ones are going to be replaced
		    @param delayedResponse false to change wheather conditions over several updates, true to change it at the moment
		 */
		void setWheater(const float& Humidity, const float& AverageCloudsSize, const bool& delayedResponse = true);

		/** Add ellipsoid: clouds are modelled as ellipsoids in our simulation approach, so.. different kind of clouds 
		    can be modelled with ellipsoids compositions.
			@param e Ellipsoid
			@param UpdateProbabilities Update probabilities?
		 */
		void addEllipsoid(Ellipsoid *e, const bool& UpdateProbabilities = true);

		/** Forces the data manager to calculate the next step right now
		 */
		void forceToUpdateData();

	private:

		/** Initialize data
		    @param nx X complexity
			@param ny Y complexity
			@param nz Z complexity
		 */
		void _initData(const int& nx, const int& ny, const int& nz);

		/** Create tridimensional cell array
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param init Initialize values
			@return Cell 3d pointer
		 */
		Cell *** _create3DCellArray(const int& nx, const int& ny, const int& nz, const bool& init = true);

		/** Delete tridimensional cell array
			@param c Cell pointer to be deleted
			@param nx X size
			@param ny Y size
		 */
		void _delete3DCellArray(Cell ***c, const int& nx, const int& ny);

		/** Copy 3d cells arrays data
			@param src Source
			@param dest Dest
			@param nx X size
			@param ny Y size
			@param nz Z size
		*/
		void _copy3DCellArraysData(Cell ***src, Cell ***dest, const int& nx, const int& ny, const int& nz);

		/** Perform celullar automata simulation
		    @param nx X size
			@param ny Y size
			@param nz Z size
			@param step Calculation step. Valid steps are 0,1,2,3.
			@param xStart x start cell (included)
			@param xEnd x end cell (not included, until xEnd-1)
		 */
		void _performCalculations(const int& nx, const int& ny, const int& nz, const int& step, const int& xStart, const int& xEnd);

		/** Update volumetric texture data
		    @param c Cells data
			@param TexId Texture Id
		    @param nx X size
			@param ny Y size
			@param nz Z size
		 */
		void _updateVolTextureData(Cell ***c, const VolTextureId& TexId, const int& nx, const int& ny, const int& nz);

		/** Get continous density at a point
		    @param c Cells data
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param x x Coord
			@param y y Coord
			@param z z Coord 
			@param r Radius
			@param sgtrength Strength
		 */	
		const float _getDensityAt(Cell ***c, const int& nx, const int& ny, const int& nz, const int& x, const int& y, const int& z, const int& r, const float& strength) const;

		/** Get discrete density at a point
		    @param c Cells data
			@param x x Coord
			@param y y Coord
			@param z z Coord 
		 */	
		const float _getDensityAt(Cell ***c, const int& x, const int& y, const int& z) const;

		/** Fact funtion
		    @param c Cells data
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param x x Coord
			@param y y Coord
			@param z z Coord 
		 */
		const bool _fact(Cell ***c, const int& nx, const int& ny, const int& nz, const int& x, const int& y, const int& z) const;

		/** Clear probabilities
		    @param c Cells data
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param clearData Clear data?
		 */
		void _clearProbabilities(Cell*** c, const int& nx, const int& ny, const int& nz, const bool& clearData);

		/** Update probabilities based from the Ellipsoid vector
		    @param c Cells data
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param delayedResponse false to change wheather conditions over several updates, true to change it at the moment
		 */
		void _updateProbabilities(Cell*** c, const int& nx, const int& ny, const int& nz, const bool& delayedResponse);

		/** Get light absorcion factor at a point
			@param c Cells data
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param x x Coord
			@param y y Coord
			@param z z Coord 
			@param d Light direction
			@param att Attenuation factor
		 */
		const Ogre::Real _getLightAbsorcionAt(Cell*** c, const int& nx, const int& ny, const int& nz, const int& x, const int& y, const int& z, const Ogre::Vector3& d, const float& att) const;

		/** Create volumetric texture
			@param TexId Texture Id
			@param nx X size
			@param ny Y size
			@param nz Z size
		 */
		void _createVolTexture(const VolTextureId& TexId, const int& nx, const int& ny, const int& nz);

		/// Simulation data
		Cell ***mCellsCurrent,
			 ***mCellsTmp;

		/// Current transition
		float mCurrentTransition;
		/// Update time
		float mUpdateTime;
		/// Current calculation state
		int mStep, mXStart, mXEnd;

		/// Complexities
		int mNx, mNy, mNz;

		/// Volumetric textures array
		Ogre::TexturePtr mVolTextures[2];
		/// Current texture
		bool mVolTexToUpdate;

		/// Has been create(...) already called?
		bool mCreated;

		/// Fast fake random
		FastFakeRandom *mFFRandom;

		/// Max number of clouds(Ellipsoids)
		int mMaxNumberOfClouds;
		/// Ellipsoids
		std::vector<Ellipsoid*> mEllipsoids;

		/// SkyX parent pointer
		VClouds *mVClouds;
	};

}}

#endif