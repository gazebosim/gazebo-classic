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

#include "GPUManager.h"

#include "SkyX.h"

namespace SkyX
{
	GPUManager::GPUManager(SkyX *s)
		: mSkyX(s)
		, mGroundPasses(std::vector<Ogre::Pass*>())
	{
		_notifySkydomeMaterialChanged();
	}

	GPUManager::~GPUManager()
	{
	}

	void GPUManager::addGroundPass(Ogre::Pass* GroundPass, const Ogre::Real& AtmosphereRadius, const Ogre::SceneBlendType& SBT)
	{
		GroundPass->setVertexProgram("SkyX_Ground_VP");
		if (mSkyX->getLightingMode() == SkyX::LM_LDR)
		{
			GroundPass->setFragmentProgram("SkyX_Ground_LDR_FP");
		}
		else
		{
			GroundPass->setFragmentProgram("SkyX_Ground_HDR_FP");
		}

		GroundPass->getVertexProgramParameters()->setNamedConstant("uSkydomeRadius", AtmosphereRadius*10);

		GroundPass->setLightingEnabled(false);
		
		GroundPass->setDepthCheckEnabled(true);
		GroundPass->setDepthWriteEnabled(false);

		GroundPass->setSceneBlending(SBT);

		///TODO
        mGroundPasses.push_back(GroundPass);

		mSkyX->getAtmosphereManager()->_update(mSkyX->getAtmosphereManager()->getOptions(), true);
	}

	void GPUManager::_updateFP()
	{
		Ogre::String fp_name = "SkyX_Ground_HDR_FP";

		if (mSkyX->getLightingMode() == SkyX::LM_LDR)
		{
			fp_name = "SkyX_Ground_LDR_FP";
		}

		for(unsigned int k = 0; k < mGroundPasses.size(); k++)
		{
			mGroundPasses.at(k)->setFragmentProgram(fp_name);
		}
	}

	void GPUManager::setGpuProgramParameter(const GpuProgram &GpuP,  const Ogre::String &Name, const int &Value, const bool& UpdateGroundPasses)
	{
		if (!mSkyX->getMeshManager()->isCreated())
		{
			return;
		}

		Ogre::GpuProgramParametersSharedPtr Parameters;

		switch (GpuP)
		{
		    case GPUP_VERTEX:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getVertexProgramParameters();
			}
			break;

			case GPUP_FRAGMENT:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
			}
			break;
		}

		Parameters->setNamedConstant(Name, Value);

		if (!UpdateGroundPasses)
		{
			return;
		}

		std::vector<Ogre::Pass*>::iterator PassIt;

		for(PassIt = mGroundPasses.begin(); PassIt != mGroundPasses.end(); PassIt++)
		{
			if (!(*PassIt))
			{
				mGroundPasses.erase(PassIt);
				continue;
			}

			switch (GpuP)
			{
			    case GPUP_VERTEX:
				{
					Parameters = (*PassIt)->getVertexProgramParameters();
				}
				break;

			    case GPUP_FRAGMENT:
				{
					Parameters = (*PassIt)->getFragmentProgramParameters();
				}
				break;
			}

			Parameters->setNamedConstant(Name, Value);
		}
	}

	void GPUManager::setGpuProgramParameter(const GpuProgram &GpuP,  const Ogre::String &Name, const Ogre::Real &Value, const bool& UpdateGroundPasses)
	{
		if (!mSkyX->getMeshManager()->isCreated())
		{
			return;
		}

		Ogre::GpuProgramParametersSharedPtr Parameters;

		switch (GpuP)
		{
		    case GPUP_VERTEX:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getVertexProgramParameters();
			}
			break;

			case GPUP_FRAGMENT:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
			}
			break;
		}

		Parameters->setNamedConstant(Name, Value);

		if (!UpdateGroundPasses)
		{
			return;
		}

		std::vector<Ogre::Pass*>::iterator PassIt;

		for(PassIt = mGroundPasses.begin(); PassIt != mGroundPasses.end(); PassIt++)
		{
			if (!(*PassIt))
			{
				mGroundPasses.erase(PassIt);
				continue;
			}

			switch (GpuP)
			{
			    case GPUP_VERTEX:
				{
					Parameters = (*PassIt)->getVertexProgramParameters();
				}
				break;

			    case GPUP_FRAGMENT:
				{
					Parameters = (*PassIt)->getFragmentProgramParameters();
				}
				break;
			}

			Parameters->setNamedConstant(Name, Value);
		}
	}

	void GPUManager::setGpuProgramParameter(const GpuProgram &GpuP,  const Ogre::String &Name, const Ogre::Vector2 &Value, const bool& UpdateGroundPasses)
	{
		if (!mSkyX->getMeshManager()->isCreated())
		{
			return;
		}

		Ogre::GpuProgramParametersSharedPtr Parameters;

		switch (GpuP)
		{
		    case GPUP_VERTEX:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getVertexProgramParameters();
			}
			break;

			case GPUP_FRAGMENT:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
			}
			break;
		}

		float Value_[2] = {Value.x, Value.y};

		Parameters->setNamedConstant(Name, Value_, 1, 2);

		if (!UpdateGroundPasses)
		{
			return;
		}

		std::vector<Ogre::Pass*>::iterator PassIt;

		for(PassIt = mGroundPasses.begin(); PassIt != mGroundPasses.end(); PassIt++)
		{
			if (!(*PassIt))
			{
				mGroundPasses.erase(PassIt);
				continue;
			}

			switch (GpuP)
			{
			    case GPUP_VERTEX:
				{
					Parameters = (*PassIt)->getVertexProgramParameters();
				}
				break;

			    case GPUP_FRAGMENT:
				{
					Parameters = (*PassIt)->getFragmentProgramParameters();
				}
				break;
			}

			Parameters->setNamedConstant(Name, Value_, 1, 2);
		}
	}

	void GPUManager::setGpuProgramParameter(const GpuProgram &GpuP,  const Ogre::String &Name, const Ogre::Vector3 &Value, const bool& UpdateGroundPasses)
	{
		if (!mSkyX->getMeshManager()->isCreated())
		{
			return;
		}

		Ogre::GpuProgramParametersSharedPtr Parameters;

		switch (GpuP)
		{
		    case GPUP_VERTEX:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getVertexProgramParameters();
			}
			break;

			case GPUP_FRAGMENT:
			{
				Parameters = mSkydomeMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
			}
			break;
		}

		Parameters->setNamedConstant(Name, Value);

		if (!UpdateGroundPasses)
		{
			return;
		}

		std::vector<Ogre::Pass*>::iterator PassIt;

		for(PassIt = mGroundPasses.begin(); PassIt != mGroundPasses.end(); PassIt++)
		{
			if (!(*PassIt))
			{
				mGroundPasses.erase(PassIt);
				continue;
			}

			switch (GpuP)
			{
			    case GPUP_VERTEX:
				{
					Parameters = (*PassIt)->getVertexProgramParameters();
				}
				break;

			    case GPUP_FRAGMENT:
				{
					Parameters = (*PassIt)->getFragmentProgramParameters();
				}
				break;
			}

			Parameters->setNamedConstant(Name, Value);
		}
	}

	const Ogre::String GPUManager::getSkydomeMaterialName() const
	{
		Ogre::String starfield = (mSkyX->isStarfieldEnabled()) ? "STARFIELD_" : "";

		return (mSkyX->getLightingMode() == SkyX::LM_LDR) ? "SkyX_Skydome_" + starfield + "LDR" : "SkyX_Skydome_" + starfield + "HDR";
	}
}