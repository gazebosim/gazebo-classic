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

#ifndef _SkyX_CloudsManager_H_
#define _SkyX_CloudsManager_H_

#include "Prerequisites.h"

#include "ColorGradient.h"

namespace SkyX
{
	class SkyX;

	class DllExport CloudLayer 
	{
	public:
		/** Cloud layer options 
		 */
		struct Options 
		{
			/// Cloud layer height
			Ogre::Real Height;
			/// Cloud layer scale
			Ogre::Real Scale;
			/// Wind direction
			Ogre::Vector2 WindDirection;
			/// Time multiplier
			Ogre::Real TimeMultiplier;

			/// Distance attenuation
			Ogre::Real DistanceAttenuation;
			/// Detail attenuation
			Ogre::Real DetailAttenuation;
			/// Cloud layer height volume(For volumetric effects on the gpu)
			Ogre::Real HeightVolume;
			/// Volumetric displacement(For volumetric effects on the gpu)
			Ogre::Real VolumetricDisplacement;


			/** Default constructor
			 */
			Options()
				: Height(100)
				, Scale(0.001f)
				, WindDirection(Ogre::Vector2(1,1))
				, TimeMultiplier(0.125f)
				, DistanceAttenuation(0.05f)
				, DetailAttenuation(1)
				, HeightVolume(0.25f)
				, VolumetricDisplacement(0.01f)
			{
			}

			/** Constructor
			    @param _Height Cloud layer height
				@param _Scale Clouds scale
				@param _WindDirection Clouds movement direction
				@param _TimeMultiplier Time multiplier factor
			 */
			Options(const Ogre::Real& _Height, 
				    const Ogre::Real& _Scale, 
					const Ogre::Vector2& _WindDirection, 
					const Ogre::Real& _TimeMultiplier)
				: Height(_Height)
				, Scale(_Scale)
				, WindDirection(_WindDirection)
				, TimeMultiplier(_TimeMultiplier)
				, DistanceAttenuation(0.05f)
				, DetailAttenuation(1)
				, HeightVolume(0.25f)
				, VolumetricDisplacement(0.01f)
			{
			}

			/** Constructor
			    @param _Height Cloud layer height
				@param _Scale Clouds scale
				@param _WindDirection Clouds movement direction
				@param _TimeMultiplier Time multiplier factor
				@param _DistanceAttenuation Distance attenuation
				@param _DetailAttenuation Detail attenuation
				@param _HeightVolume Height volume(For volumetric effects on the gpu)
				@param _VolumetricDisplacement Volumetric displacement(For volumetric effects on the gpu)
				
			 */
			Options(const Ogre::Real& _Height, 
				    const Ogre::Real& _Scale, 
					const Ogre::Vector2& _WindDirection, 
					const Ogre::Real& _TimeMultiplier,
					const Ogre::Real& _DistanceAttenuation,
					const Ogre::Real& _DetailAttenuation,
					const Ogre::Real& _HeightVolume,
					const Ogre::Real& _VolumetricDisplacement)
				: Height(_Height)
				, Scale(_Scale)
				, WindDirection(_WindDirection)
				, TimeMultiplier(_TimeMultiplier)
				, DistanceAttenuation(_DistanceAttenuation)
				, DetailAttenuation(_DetailAttenuation)
				, HeightVolume(_HeightVolume)
				, VolumetricDisplacement(_VolumetricDisplacement)
			{
			}
		};


		/** Default onstructor
		    @param s SkyX parent pointer
		 */
		CloudLayer(SkyX *s);

		/** Constructor
		    @param s SkyX parent pointer
		    @param o Cloud layer options
		 */
		CloudLayer(SkyX *s, const Options& o);

		/** Destructor
		 */
        ~CloudLayer();

		/** Set options
		    @param o New options
		 */
		inline void setOptions(const Options& o)
		{
			mOptions = o;
			_updatePassParameters();
		}

		/** Get options
		    @return Cloud layer options
		 */
		inline const Options& getOptions() const
		{
			return mOptions;
		}

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

		/** Register layer
		    @param CloudLayerPass Pass where register the cloud layer
		 */
		void _registerCloudLayer(Ogre::Pass* CloudLayerPass);

		/** Unregister cloud pass
		 */
		void _unregister();

		/** Update internal cloud pass parameters
		 */
		void _updateInternalPassParameters();

	private:
		/** Update cloud pass parameters
		 */
		void _updatePassParameters();

		/// Cloud layer options
		Options mOptions;

		/// Ambient and Sun color gradients
		ColorGradient mAmbientGradient;
		ColorGradient mSunGradient;

		/// Cloud layer pass
		Ogre::Pass *mCloudLayerPass;

		/// SkyX parent pointer
		SkyX *mSkyX;
	};

    class DllExport CloudsManager 
	{
	public:
		/** Constructor
		    @param s SkyX parent pointer
		 */
		CloudsManager(SkyX *h);

		/** Destructor
		 */
        ~CloudsManager();

		/** Update cloud layers
		 */
		void update();

		/** Add a cloud layer
		    @param o Cloud layer options
			@return Cloud layer
		 */
		CloudLayer* add(const CloudLayer::Options& o);

		/** Remove the specified cloud layer
		 */
		void remove(CloudLayer *cl);

		/** Remove all cloud layers
		 */
		void removeAll();

		/** Register all
		 */
		void registerAll();

		/** Unregister cloud layer
		    @param cl Cloud layer to be unregistered
		 */
		void unregister(CloudLayer* cl);

		/** Unregister all cloud layers
		 */
		void unregisterAll();

		/** Get cloud layers
		    @return Cloud layers
		 */
		inline const std::vector<CloudLayer*>& getCloudLayers() const
		{
			return mCloudLayers;
		}

	private:
		/// Cloud layers std::vector
		std::vector<CloudLayer*> mCloudLayers;
		/// Cloud layers iterator
		std::vector<CloudLayer*>::iterator CloudLayersIt;

		/// SkyX parent pointer
		SkyX *mSkyX;
	};
}

#endif