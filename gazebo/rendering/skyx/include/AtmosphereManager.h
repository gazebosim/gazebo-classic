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

#ifndef _SkyX_AtmosphereManager_H_
#define _SkyX_AtmosphereManager_H_

#include "Prerequisites.h"

namespace SkyX
{
	class SkyX;

    class DllExport AtmosphereManager 
	{
	public:

		/** Atmosphere options 
		 */
		struct Options 
		{
			/// Inner atmosphere radius
			Ogre::Real InnerRadius;
			/// Outer atmosphere radius
			Ogre::Real OuterRadius;
			/// Height position, in [0, 1] range, 0=InnerRadius, 1=OuterRadius
			Ogre::Real HeightPosition;

			/// Rayleigh multiplier
			Ogre::Real RayleighMultiplier;
			/// Mie multiplier
			Ogre::Real MieMultiplier;
			/// Sun intensity
			Ogre::Real SunIntensity;

			/// WaveLength for RGB channels
			Ogre::Vector3 WaveLength;

			/// Phase function
			Ogre::Real G;

			/// Exposure coeficient
			Ogre::Real Exposure;

			/// Number of samples
			int NumberOfSamples;

			/** Default constructor
			 */
			Options()
				: InnerRadius(9.77501f)
				, OuterRadius(10.2963f)
				, HeightPosition(0.01f)
				, RayleighMultiplier(0.0022f)
				, MieMultiplier(0.000675f)
				, SunIntensity(30)
				, WaveLength(Ogre::Vector3(0.57f, 0.54f, 0.44f))
				, G(-0.991f)
				, Exposure(2.0f)
				, NumberOfSamples(4)
			{
			}

			/** Constructor
                @param _InnerRadius Inner atmosphere radius
				@param _OuterRadius Outer atmosphere radius
				@param _HeightPosition Height position, in [0, 1] range, 0=InnerRadius, 1=OuterRadius
				@param _RayleighMultiplier Rayleigh multiplier
				@param _MieMultiplier Mie multiplier
				@param _SunIntensity Sun intensity
				@param _WaveLength Wave length for RGB channels
				@param _G Phase function
				@param _Exposure Exposure
				@param _NumerOfSamples Number of samples
			 */
			Options(const Ogre::Real&	 _InnerRadius,
				    const Ogre::Real&	 _OuterRadius,
					const Ogre::Real&	 _HeightPosition,
					const Ogre::Real&	 _RayleighMultiplier,
					const Ogre::Real&	 _MieMultiplier,
					const Ogre::Real&    _SunIntensity,
					const Ogre::Vector3& _WaveLength,
					const Ogre::Real&    _G,
					const Ogre::Real&    _Exposure,
					const int&			 _NumerOfSamples)
				: InnerRadius(_InnerRadius)
				, OuterRadius(_OuterRadius)
				, HeightPosition(_HeightPosition)
				, RayleighMultiplier(_RayleighMultiplier)
				, MieMultiplier(_MieMultiplier)
				, SunIntensity(_SunIntensity)
				, WaveLength(_WaveLength)
				, G(_G)
				, Exposure(_Exposure)
				, NumberOfSamples(_NumerOfSamples)
			{
			}
		};

	    /** Constructor
		    @param s Parent SkyX pointer
		 */
		AtmosphereManager(SkyX *s);

		/** Destructor 
		 */
		~AtmosphereManager();

		/** Set options
		    @param _Options New options
		 */
		inline void setOptions(const Options& _Options)
		{
			_update(_Options);
		}

		/** Get current options 
		    @return Current options
		 */
		inline const Options& getOptions() const
		{
			return mOptions;
		}

		/** Get current atmosphere color at the given direction
		    @param Direction *Normalised* direction
			@return Atmosphere color at the especified direction
		 */
		const Ogre::Vector3 getColorAt(const Ogre::Vector3& Direction) const;

		/** Update atmoshpere
		    @param NewOptions Update only the differences between actual parameters and new ones.
			@param ForceToUpdateAll Forces to upload all current parameters to skyx material.
			@remarks Current options parameters are updated if needed.
		 */
		void _update(const Options& NewOptions, const bool& ForceToUpdateAll = false);
	
	private:
		/** Shader scale funtion 
		    @param cos Cos
			@param uScaleDepth Scale Depth
			@return Scale
		 */
		const float _scale(const float& cos, const float& uScaleDepth) const;

		/// Our options
		Options mOptions;

		/// SkyX parent pointer
		SkyX *mSkyX;
	};
}

#endif
