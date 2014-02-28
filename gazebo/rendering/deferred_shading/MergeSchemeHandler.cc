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


#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "gazebo/rendering/deferred_shading/MergeSchemeHandler.hh"

using namespace gazebo;
using namespace rendering;

// The merge pass doesn't necesarily need normals, since they can be
// retreived from the GBuffer. However, getting them from the input stream is
// faster, and allows us to ignore the GBuffer completely in this step(less
// texture sampling is the whole purpose of this technique).

const std::string MergeSchemeHandler::normal_map_pattern = "normal";

/////////////////////////////////////////////////
Ogre::Technique *MergeSchemeHandler::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String &_schemeName,
    Ogre::Material *_originalMaterial, uint16_t _lodIndex,
    const Ogre::Renderable *_rend)
{
  Ogre::MaterialManager &matMgr = Ogre::MaterialManager::getSingleton();
  Ogre::String curSchemeName = matMgr.getActiveScheme();

  matMgr.setActiveScheme(Ogre::MaterialManager::DEFAULT_SCHEME_NAME);
  Ogre::Technique *originalTechnique =
    _originalMaterial->getBestTechnique(_lodIndex, _rend);
  matMgr.setActiveScheme(curSchemeName);

  Ogre::Technique *mergeTech = _originalMaterial->createTechnique();
  mergeTech->removeAllPasses();
  mergeTech->setSchemeName(_schemeName);

  Ogre::Technique *noMergeTech = _originalMaterial->createTechnique();
  noMergeTech->removeAllPasses();
  // objects that don't render in the GBuffer, won't need to be merged
  noMergeTech->setSchemeName("NoGBuffer");

  for (uint16_t i = 0; i < originalTechnique->getNumPasses(); ++i)
  {
    Ogre::Pass *originalPass = originalTechnique->getPass(i);
    PassProperties props = InspectPass(originalPass, _lodIndex, _rend);

    if (!props.isDeferred)
    {
      // Just copy the technique so it gets rendered regularly
      Ogre::Pass *clonePass = noMergeTech->createPass();
      *clonePass = *originalPass;
      continue;
    }

    Ogre::Pass *newPass = mergeTech->createPass();
    MaterialGenerator::Perm perm = this->GetPermutation(props);

    const Ogre::MaterialPtr &templateMat =
      this->materialGenerator->GetMaterial(perm);

    // We assume that the Merge technique contains only one pass. But its true.
    *newPass = *(templateMat->getTechnique(0)->getPass(0));
    this->FillPass(newPass, originalPass, props);
  }

  return mergeTech;
}

/////////////////////////////////////////////////
bool MergeSchemeHandler::CheckNormalMap(Ogre::TextureUnitState *_tus,
    MergeSchemeHandler::PassProperties &_props)
{
  bool isNormal = false;
  Ogre::String lowerCaseAlias = _tus->getTextureNameAlias();
  Ogre::StringUtil::toLowerCase(lowerCaseAlias);

  if (lowerCaseAlias.find(normal_map_pattern) != Ogre::String::npos)
    isNormal = true;
  else
  {
    Ogre::String lowerCaseName = _tus->getTextureName();
    Ogre::StringUtil::toLowerCase(lowerCaseName);
    if (lowerCaseName.find(normal_map_pattern) != Ogre::String::npos)
      isNormal = true;
  }

  if (isNormal)
  {
    if (_props.normalMap == 0)
      _props.normalMap = _tus;
    else
    {
      OGRE_EXCEPT(Ogre::Exception::ERR_DUPLICATE_ITEM,
          "Multiple normal map patterns matches",
          "MergeSchemeHandler::inspectPass");
    }
  }
  return isNormal;
}

/////////////////////////////////////////////////
MergeSchemeHandler::PassProperties MergeSchemeHandler::InspectPass(
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

  for (uint16_t i = 0; i < _pass->getNumTextureUnitStates(); ++i)
  {
    Ogre::TextureUnitState *tus = _pass->getTextureUnitState(i);
    if (!this->CheckNormalMap(tus, props))
      props.regularTextures.push_back(tus);
    if (tus->getEffects().size() > 0)
      props.isDeferred = false;
  }

  if (_pass->getDiffuse() != Ogre::ColourValue::White)
  {
    props.hasDiffuseColor = true;
  }

  // Check transparency
  if (_pass->getDestBlendFactor() != Ogre::SBF_ZERO)
  {
    // TODO : Better ways to do this
    props.isDeferred = false;
  }

  return props;
}

/////////////////////////////////////////////////
MaterialGenerator::Perm MergeSchemeHandler::GetPermutation(
    const PassProperties &_props)
{
  MaterialGenerator::Perm perm = 0;
  switch (_props.regularTextures.size())
  {
    case 0:
      {
        perm |= MergeMaterialGenerator::MP_NO_TEXTURES;
        if (_props.normalMap != 0)
          perm |= MergeMaterialGenerator::MP_ONE_TEXCOORD;
        else
          perm |= MergeMaterialGenerator::MP_NO_TEXCOORDS;
        break;
      }
    case 1:
      {
        perm |= MergeMaterialGenerator::MP_ONE_TEXTURE;
        perm |= MergeMaterialGenerator::MP_ONE_TEXCOORD;
        break;
      }
    case 2:
      {
        perm |= MergeMaterialGenerator::MP_TWO_TEXTURES;
        // TODO : When do we use two texcoords?
        perm |= MergeMaterialGenerator::MP_ONE_TEXCOORD;
        break;
      }
    case 3:
      {
        perm |= MergeMaterialGenerator::MP_THREE_TEXTURES;
        perm |= MergeMaterialGenerator::MP_ONE_TEXCOORD;
        break;
      }
    default:
      {
        OGRE_EXCEPT(Ogre::Exception::ERR_NOT_IMPLEMENTED,
            "Can not generate Merge materials for '>3 regular-texture' objects",
            "MergeSchemeHandler::inspectPass");
      }
  }

  if (_props.isSkinned)
    perm |= MergeMaterialGenerator::MP_SKINNED;

  if (_props.normalMap != 0)
    perm |= MergeMaterialGenerator::MP_NORMAL_MAP;

  if (_props.hasDiffuseColor)
    perm |= MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR;

  return perm;
}

/////////////////////////////////////////////////
void MergeSchemeHandler::FillPass(Ogre::Pass *_mergePass,
    Ogre::Pass *_originalPass, const PassProperties &_props)
{
  // Reference the correct textures. Normal map first!
  int texUnitIndex = 0;

  if (_props.normalMap != 0)
  {
    *(_mergePass->getTextureUnitState(texUnitIndex)) = *(_props.normalMap);
    texUnitIndex++;
  }

  for (size_t i = 0; i < _props.regularTextures.size(); ++i)
  {
    *(_mergePass->getTextureUnitState(texUnitIndex)) =
      *(_props.regularTextures[i]);
    ++texUnitIndex;
  }

  // Use the LBuffer as an input
  _mergePass->getTextureUnitState(texUnitIndex)->setContentType(
      Ogre::TextureUnitState::CONTENT_COMPOSITOR);

  _mergePass->getTextureUnitState(texUnitIndex)->setCompositorReference(
      this->techName + "/ShowLit", "LBuffer");

  _mergePass->getTextureUnitState(texUnitIndex)->setDesiredFormat(
      Ogre::PF_FLOAT16_RGBA);

  _mergePass->getTextureUnitState(texUnitIndex)->setTextureFiltering(
      Ogre::TFO_NONE);

  ++texUnitIndex;

  // If we're using inferred lighting, handle the extra GBuffer input
  if (this->useDSF)
  {
    _mergePass->getTextureUnitState(texUnitIndex)->setContentType(
        Ogre::TextureUnitState::CONTENT_COMPOSITOR);
    _mergePass->getTextureUnitState(texUnitIndex)->setCompositorReference(
        this->techName + "/GBuffer", "mrt_output", 1);
    _mergePass->getTextureUnitState(texUnitIndex)->setTextureFiltering(
        Ogre::TFO_NONE);
    _mergePass->getTextureUnitState(texUnitIndex)->setDesiredFormat(
        Ogre::PF_FLOAT16_RGBA);
  }

  _mergePass->setAmbient(_originalPass->getAmbient());
  _mergePass->setDiffuse(_originalPass->getDiffuse());
  _mergePass->setSpecular(_originalPass->getSpecular());
  _mergePass->setShininess(_originalPass->getShininess());
  _mergePass->setCullingMode(_originalPass->getCullingMode());
  _mergePass->setLightingEnabled(false);
}
