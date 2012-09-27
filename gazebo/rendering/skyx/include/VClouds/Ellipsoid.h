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

#ifndef _SkyX_VClouds_Ellipsoid_H_
#define _SkyX_VClouds_Ellipsoid_H_

#include "Prerequisites.h"

#include "VClouds/DataManager.h"

namespace SkyX { namespace VClouds{

	/** Ellipsoid class
	    x^2   y^2   z^2
		/   + /   + /    = 1
		a^2   b^2   c^2
	 */
	class DllExport Ellipsoid
	{
	public:
		/** Constructor
		    @param a A constant
			@param b B constant
			@param c C constant
			@param nx X size
			@param ny Y size
			@param nz Z size
			@param x x Coord (position)
			@param y y Coord (position)
			@param z z Coord (position)
			@param Density Cloud density
		 */
		Ellipsoid(const int& a, const int& b, const int& c, 
			      const int& nx, const int& ny, const int& nz,
				  const int& x, const int& y, const int& z, 
				  const Ogre::Real& DynLibManager = 1.0f);

		/** Destructor 
		 */
		~Ellipsoid();

		/** Move the ellipsoid
		    @param Ax x increment
			@param Ay y increment
			@param Az z increment
		 */
		void move(const int& Ax, const int& Ay, const int& Az);

		/** Get probabilities at a point
			@param x x Coord
			@param y y Coord
			@param z z Coord 
			@return Probabilities (Hum, Ext, Act)
		 */
		const Ogre::Vector3 getProbabilities(const int& x, const int& y, const int& z) const;

		/** Update probabilities
			@param c Cells
			@param nx X complexity
			@param ny Y complexity
			@param nz Z complexity
			@param delayedResponse true to get a delayed response, updating only probabilities, false to also set clouds
		 */
		void updateProbabilities(DataManager::Cell ***c, const int &nx, const int &ny, const int &nz, const bool& delayedResponse = true);

		/** Determines if the ellipsoid is out of the cells domain and needs to be removed
		 */
		const bool isOutOfCells() const;

		/** Get dimensions
		    @return Ellipsoid dimensions
		 */
		inline const Ogre::Vector3 getDimensions() const
		{
			return Ogre::Vector3(mA, mB, mC);
		}

		/** Get position
		    @return Position
		 */
		inline const Ogre::Vector3 getPosition() const
		{
			return Ogre::Vector3(mX, mY, mZ);
		}

		/** Set dimensions
		    @param Dimensions New dimensions
		 */
		void setDimensions(const Ogre::Vector3& Dimensions);

		/** Set position
		    @param Position New position
		 */
		inline void setPosition(const Ogre::Vector3& Position)
		{
			mX = Position.x;
			mY = Position.y;
			mZ = Position.z;
		}

	private:
		/** Get length
		    @param x x Coord
			@param y y Coord
			@param z z Coord
			@return [0,1] range where 0 is the center of the ellipsoid and 1 the superfice
		 */
		const float _getLength(const int& x, const int& y, const int& z) const;

		/// Ellipsoid parameters
		int mA, mB, mC, mA2, mB2, mC2;

		/// Position
		int mX, mY, mZ;

		/// Cells size
		int mNx, mNy, mNz;

		/// Cloud density
		Ogre::Real mDensity;
	};

}}

#endif