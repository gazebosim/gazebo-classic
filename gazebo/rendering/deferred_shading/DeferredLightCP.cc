/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <Ogre.h>

#include "gazebo/common/Exception.hh"
#include "gazebo/rendering/deferred_shading/DeferredLightCP.hh"
#include "gazebo/rendering/deferred_shading/LightMaterialGenerator.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
void InjectTechnique(Ogre::SceneManager *_sm, Ogre::Technique *_tech,
                     Ogre::Renderable *_rend, const Ogre::LightList *_lightList)
{
  for (uint16_t i = 0; i < _tech->getNumPasses(); ++i)
  {
    Ogre::Pass *pass = _tech->getPass(i);

    if (_lightList != 0)
      _sm->_injectRenderWithPass(pass, _rend, false, false, _lightList);
    else
      _sm->_injectRenderWithPass(pass, _rend, false);
  }
}

/////////////////////////////////////////////////
DeferredLightRenderOperation::DeferredLightRenderOperation(
  Ogre::CompositorInstance *_instance, const Ogre::CompositionPass *_pass)
{
  this->viewport = _instance->getChain()->getViewport();

  /// Get the names of the GBuffer textures
  /// Texture 0 is the color buffer
  /// Texture 1 is the normal buffer
  const Ogre::CompositionPass::InputTex &input0 = _pass->getInput(0);
  this->texName0 = _instance->getTextureInstanceName(input0.name,
      input0.mrtIndex);

  const Ogre::CompositionPass::InputTex &input1 = _pass->getInput(1);
  this->texName1 = _instance->getTextureInstanceName(input1.name,
      input1.mrtIndex);

  // Create lights material generator
  this->lightMaterialGenerator = new LightMaterialGenerator();

  // Create the ambient light
  this->ambientLight = new AmbientLight();
  const Ogre::MaterialPtr &mat = this->ambientLight->getMaterial();
  mat->load();
}

/////////////////////////////////////////////////
DeferredLight *DeferredLightRenderOperation::CreateDeferredLight(
    Ogre::Light *_light)
{
  DeferredLight *rv = new DeferredLight(this->lightMaterialGenerator, _light);
  this->lights[_light] = rv;
  return rv;
}

/////////////////////////////////////////////////
void DeferredLightRenderOperation::execute(Ogre::SceneManager *_sm,
                                           Ogre::RenderSystem * /*_rs*/)
{
  Ogre::Camera *cam = this->viewport->getCamera();
  Ogre::Technique *tech;

  // Update the ambient light based on the camera
  this->ambientLight->UpdateFromCamera(cam);

  tech = this->ambientLight->getMaterial()->getBestTechnique();
  InjectTechnique(_sm, tech, this->ambientLight, 0);

  const Ogre::LightList &lightList = _sm->_getLightsAffectingFrustum();
  for (Ogre::LightList::const_iterator it = lightList.begin();
       it != lightList.end(); ++it)
  {
    Ogre::Light *light = *it;
    Ogre::LightList ll;
    ll.push_back(light);

    // if (++i != 2) continue;
    // if (light->getType() != Light::LT_DIRECTIONAL) continue;
    // if (light->getDiffuseColour() != ColourValue::Red) continue;

    LightsMap::iterator dLightIt = this->lights.find(light);
    DeferredLight* dLight = 0;
    if (dLightIt == this->lights.end())
      dLight = this->CreateDeferredLight(light);
    else
    {
      dLight = dLightIt->second;
      dLight->UpdateFromParent();
    }

    dLight->UpdateFromCamera(cam);
    tech = dLight->getMaterial()->getBestTechnique();

    // Update shadow texture
    if (dLight->getCastShadows())
    {
      Ogre::SceneManager::RenderContext *context = _sm->_pauseRendering();

      _sm->prepareShadowTextures(cam, this->viewport, &ll);
      _sm->_resumeRendering(context);

      Ogre::Pass *pass = tech->getPass(0);
      Ogre::TextureUnitState *tus = pass->getTextureUnitState("ShadowMap");
      if (!tus)
        gzthrow("Invalid texture unit state");

      const Ogre::TexturePtr &shadowTex = _sm->getShadowTexture(0);
      if (tus->_getTexturePtr() != shadowTex)
        tus->_setTexturePtr(shadowTex);
    }

    InjectTechnique(_sm, tech, dLight, &ll);
  }
}

/////////////////////////////////////////////////
DeferredLightRenderOperation::~DeferredLightRenderOperation()
{
  for (LightsMap::iterator it = this->lights.begin();
       it != this->lights.end(); ++it)
  {
    delete it->second;
  }
  this->lights.clear();

  delete this->ambientLight;
  delete this->lightMaterialGenerator;
}
