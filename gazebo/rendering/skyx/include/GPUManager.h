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

#ifndef _SkyX_GPUManager_H_
#define _SkyX_GPUManager_H_

#include "Prerequisites.h"

namespace SkyX
{
	class SkyX;

    class DllExport GPUManager 
	{
	public:
		/** Gpu program enum
		 */
		enum GpuProgram
		{
			// Vertex program
			GPUP_VERTEX   = 0,
			// Fragment program
			GPUP_FRAGMENT = 1
		};

	    /** Constructor
		    @param s Parent SkyX pointer
		 */
		GPUManager(SkyX *s);

		/** Destructor 
		 */
		~GPUManager();

		/** Add ground pass (Use for atmospheric scattering effect on the terrain)
		    @param GroundPass Ground pass
			@param AtmosphereRaidus Atmosphere radius (typically far carmera clip plane)
			@param SBT Scene blend type
		 */
		void addGroundPass(Ogre::Pass* GroundPass, const Ogre::Real& AtmosphereRadius, const Ogre::SceneBlendType& SBT = Ogre::SBT_ADD);

		/** Set gpu program int parameter
		    @param GpuP Gpu program type (Vertex/Fragment)
			@param Name param name
			@param Value value
			@param UpdateGroundPasses true to update ground passes
		 */
		void setGpuProgramParameter(const GpuProgram &GpuP, const Ogre::String &Name, const int &Value, const bool& UpdateGroundPasses = true);

		/** Set gpu program Ogre::Real parameter
		    @param GpuP Gpu program type (Vertex/Fragment)
			@param Name param name
			@param Value value
			@param UpdateGroundPasses true to update ground passes
		 */
		void setGpuProgramParameter(const GpuProgram &GpuP, const Ogre::String &Name, const Ogre::Real &Value, const bool& UpdateGroundPasses = true);

		/** Set gpu program Ogre::Vector2 parameter
		    @param GpuP Gpu program type (Vertex/Fragment)
			@param Name param name
			@param Value value
			@param UpdateGroundPasses true to update ground passes
		 */
		void setGpuProgramParameter(const GpuProgram &GpuP, const Ogre::String &Name, const Ogre::Vector2 &Value, const bool& UpdateGroundPasses = true); 

		/** Set gpu program Ogre::Vector3 parameter
		    @param GpuP Gpu program type (Vertex/Fragment)
			@param Name param name
			@param Value value
			@param UpdateGroundPasses true to update ground passes
		 */
		void setGpuProgramParameter(const GpuProgram &GpuP, const Ogre::String &Name, const Ogre::Vector3 &Value, const bool& UpdateGroundPasses = true); 

		/** Get skydome material name
		    @return Skydome material name
		 */
		const Ogre::String getSkydomeMaterialName() const;

		/** Get moon material name
		    @return Moon material name
		 */
		inline const Ogre::String getMoonMaterialName() const
		{
			return "SkyX_Moon";
		}

		/** Update fragment program materials
		    @remarks Only for internal use
		 */
		void _updateFP();

		/** Notify skydome material changed
		    @remarks Only for internal use
		 */
		inline void _notifySkydomeMaterialChanged()
		{
			mSkydomeMaterial = static_cast<Ogre::MaterialPtr>(Ogre::MaterialManager::getSingleton().getByName(getSkydomeMaterialName()));

			if (mSkydomeMaterial.isNull())
			{
				SkyXLOG("Error in SkyX::GPUManager: '" + getSkydomeMaterialName() + "' material not found");
				return;
			}
		}
		
	private:
		/** Set texture HW gamma correction
		    @param n Texture name
			@param g True to enable gamma correction, false to disable it
		 */
		void _setTextureHWGammaCorrection(const Ogre::String& n, const bool& g);

		/// Skydome material
		Ogre::MaterialPtr mSkydomeMaterial;
		/// Ground pass vector
		std::vector<Ogre::Pass*> mGroundPasses;

		/// SkyX parent pointer
		SkyX *mSkyX;
	};
}

#endif