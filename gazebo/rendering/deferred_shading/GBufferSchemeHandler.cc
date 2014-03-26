/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <OgreTechnique.h>

#include "gazebo/common/Exception.hh"
#include "gazebo/rendering/deferred_shading/GBufferSchemeHandler.hh"

using namespace gazebo;
using namespace rendering;

const std::string GBufferSchemeHandler::normal_map_pattern = "normal";

/////////////////////////////////////////////////
Ogre::Technique *GBufferSchemeHandler::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/,
    const Ogre::String &_schemeName,
    Ogre::Material *_originalMaterial,
    uint16_t _lodIndex,
    const Ogre::Renderable *_rend)
{
  Ogre::MaterialManager &matMgr = Ogre::MaterialManager::getSingleton();

  Ogre::String curSchemeName = matMgr.getActiveScheme();

  matMgr.setActiveScheme(Ogre::MaterialManager::DEFAULT_SCHEME_NAME);
  Ogre::Technique *originalTechnique =
    _originalMaterial->getBestTechnique(_lodIndex, _rend);
  matMgr.setActiveScheme(curSchemeName);

  Ogre::Technique *gBufferTech = _originalMaterial->createTechnique();
  gBufferTech->removeAllPasses();
  gBufferTech->setSchemeName(_schemeName);

  Ogre::Technique *noGBufferTech = _originalMaterial->createTechnique();
  noGBufferTech->removeAllPasses();
  noGBufferTech->setSchemeName("NoGBuffer");

  for (int i = 0; i < originalTechnique->getNumPasses(); ++i)
  {
    Ogre::Pass *originalPass = originalTechnique->getPass(i);
    PassProperties props = this->InspectPass(originalPass,
                                             _lodIndex, _rend);

    if (!props.isDeferred)
    {
      // Just copy the technique so it gets rendered regularly
      Ogre::Pass *clonePass = noGBufferTech->createPass();
      *clonePass = *originalPass;
      continue;
    }

    Ogre::Pass *newPass = gBufferTech->createPass();
    MaterialGenerator::Perm perm = this->GetPermutation(props);

    const Ogre::MaterialPtr &templateMat =
      this->materialGenerator.GetMaterial(perm);

    // We assume that the GBuffer technique contains only one pass.
    *newPass = *(templateMat->getTechnique(0)->getPass(0));
    this->FillPass(newPass, originalPass, props);
  }

  return gBufferTech;
}

/////////////////////////////////////////////////
bool GBufferSchemeHandler::CheckNormalMap(
    Ogre::TextureUnitState *_tus, GBufferSchemeHandler::PassProperties &_props)
{
  bool isNormal = false;
  Ogre::String lowerCaseAlias = _tus->getTextureNameAlias();
  Ogre::StringUtil::toLowerCase(lowerCaseAlias);
  if (lowerCaseAlias.find(normal_map_pattern) != Ogre::String::npos)
  {
    isNormal = true;
  }
  else
  {
    Ogre::String lowerCaseName = _tus->getTextureName();
    Ogre::StringUtil::toLowerCase(lowerCaseName);
    if (lowerCaseName.find(normal_map_pattern) != Ogre::String::npos)
    {
      isNormal = true;
    }
  }

  if (isNormal)
  {
    if (_props.normalMap == 0)
    {
      _props.normalMap = _tus;
    }
    else
    {
      // TODO: Replace with gazebo exception
      OGRE_EXCEPT(Ogre::Exception::ERR_DUPLICATE_ITEM,
          "Multiple normal map patterns matches",
          "GBufferSchemeHandler::InspectPass");
    }
  }

  return isNormal;
}

/////////////////////////////////////////////////
GBufferSchemeHandler::PassProperties GBufferSchemeHandler::InspectPass(
  Ogre::Pass *_pass, uint16_t /*_lodIndex*/,
  const Ogre::Renderable * /*_rend*/)
{
  PassProperties props;

  // TODO : Use renderable to indicate wether this has skinning.
  // Probably use same const cast that renderSingleObject uses.
  if (_pass->hasVertexProgram())
    props.isSkinned = _pass->getVertexProgram()->isSkeletalAnimationIncluded();
  else
    props.isSkinned = false;

  for (int i = 0; i < _pass->getNumTextureUnitStates(); ++i)
  {
    Ogre::TextureUnitState *tus = _pass->getTextureUnitState(i);
    if (!this->CheckNormalMap(tus, props))
    {
      props.regularTextures.push_back(tus);
    }

    if (tus->getEffects().size() > 0)
      props.isDeferred = false;
  }

  if (_pass->getDiffuse() != Ogre::ColourValue::White)
    props.hasDiffuseColor = true;

  // Check transparency
  if (_pass->getDestBlendFactor() != Ogre::SBF_ZERO)
  {
    // TODO : Better ways to do this
    props.isDeferred = false;
  }

  return props;
}

/////////////////////////////////////////////////
MaterialGenerator::Perm GBufferSchemeHandler::GetPermutation(
    const PassProperties &_props)
{
  MaterialGenerator::Perm perm = 0;
  switch (_props.regularTextures.size())
  {
    case 0:
      {
        perm |= GBufferMaterialGenerator::GBP_NO_TEXTURES;

        if (_props.normalMap != 0)
          perm |= GBufferMaterialGenerator::GBP_ONE_TEXCOORD;
        else
          perm |= GBufferMaterialGenerator::GBP_NO_TEXCOORDS;
        break;
      }
    case 1:
      {
        perm |= GBufferMaterialGenerator::GBP_ONE_TEXTURE;
        perm |= GBufferMaterialGenerator::GBP_ONE_TEXCOORD;
        break;
      }
    case 2:
      {
        perm |= GBufferMaterialGenerator::GBP_TWO_TEXTURES;
        // TODO : When do we use two texcoords?
        perm |= GBufferMaterialGenerator::GBP_ONE_TEXCOORD;
        break;
      }
    case 3:
      {
        perm |= GBufferMaterialGenerator::GBP_THREE_TEXTURES;
        perm |= GBufferMaterialGenerator::GBP_ONE_TEXCOORD;
        break;
      }
    default:
      {
        gzthrow("Can not generate G-Buffer materials for '>3 regular-texture'\
            objects");
      }
  }

  if (_props.isSkinned)
    perm |= GBufferMaterialGenerator::GBP_SKINNED;

  if (_props.normalMap != 0)
    perm |= GBufferMaterialGenerator::GBP_NORMAL_MAP;

  if (_props.hasDiffuseColor)
    perm |= GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR;

  return perm;
}

/////////////////////////////////////////////////
void GBufferSchemeHandler::FillPass(
    Ogre::Pass *_gBufferPass, Ogre::Pass *_originalPass,
    const PassProperties &_props)
{
  // Reference the correct textures. Normal map first!
  int texUnitIndex = 0;

  if (_props.normalMap != 0)
  {
    *(_gBufferPass->getTextureUnitState(texUnitIndex)) = *(_props.normalMap);
    ++texUnitIndex;
  }

  for (size_t i = 0; i < _props.regularTextures.size(); ++i)
  {
    *(_gBufferPass->getTextureUnitState(texUnitIndex++)) =
      *(_props.regularTextures[i]);
  }

  _gBufferPass->setAmbient(_originalPass->getAmbient());
  _gBufferPass->setDiffuse(_originalPass->getDiffuse());
  _gBufferPass->setSpecular(_originalPass->getSpecular());
  _gBufferPass->setShininess(_originalPass->getShininess());
  _gBufferPass->setCullingMode(_originalPass->getCullingMode());
  _gBufferPass->setLightingEnabled(false);
}
