/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/model/ModelEditorTypes.hh"
#include "gazebo/gui/model/EditorMaterialSwitcher.hh"

using namespace gazebo;
using namespace gui;


/////////////////////////////////////////////////
EditorMaterialSwitcher::EditorMaterialSwitcher(
    const rendering::CameraPtr &_camera)
{
  this->camera = _camera;
  this->materialScheme = "";

  if (!this->camera)
  {
    gzerr << "Cannot create a material switcher for the model editor. "
          << "Camera is NULL" << std::endl;
    return;
  }

  this->materialListener.reset(new EditorMaterialListener(this->camera));
  this->renderTargetListener.reset(new EditorRenderTargetListener(
      this->materialListener));
}

/////////////////////////////////////////////////
void EditorMaterialSwitcher::SetMaterialScheme(const std::string &_scheme)
{
  if (!this->camera || !this->camera->GetViewport())
    return;

  this->materialScheme = _scheme;
  if (_scheme.empty())
  {
    this->camera->GetViewport()->setMaterialScheme(
        this->originalMaterialScheme);
    this->camera->GetViewport()->getTarget()->removeListener(
        this->renderTargetListener.get());
  }
  else
  {
    this->originalMaterialScheme =
        this->camera->GetViewport()->getMaterialScheme();

    this->camera->GetViewport()->setMaterialScheme(_scheme);
    this->camera->GetViewport()->getTarget()->addListener(
        this->renderTargetListener.get());
  }
}

/////////////////////////////////////////////////
std::string EditorMaterialSwitcher::MaterialScheme() const
{
  return this->materialScheme;
}

/////////////////////////////////////////////////
EditorRenderTargetListener::EditorRenderTargetListener(
    const EditorMaterialListenerPtr &_switcher)
  : materialListener(_switcher)
{
}

/////////////////////////////////////////////////
void EditorRenderTargetListener::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().addListener(
      this->materialListener.get());
}

/////////////////////////////////////////////////
void EditorRenderTargetListener::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(
      this->materialListener.get());
}

/////////////////////////////////////////////////
EditorMaterialListener::EditorMaterialListener(
    const rendering::CameraPtr &_camera)
  : camera(_camera)
{
}

/////////////////////////////////////////////////
Ogre::Technique *EditorMaterialListener::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String & /*_schemeName*/,
    Ogre::Material *_originalMaterial, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  if (_rend && typeid(*_rend) == typeid(Ogre::SubEntity))
  {
    std::string material = "";

    const Ogre::SubEntity *subEntity =
      static_cast<const Ogre::SubEntity *>(_rend);

    // use the original material for gui visuals
    if (!(subEntity->getParent()->getVisibilityFlags() &
        (GZ_VISIBILITY_ALL &  ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE))))
    {
      Ogre::Technique *originalTechnique = _originalMaterial->getTechnique(0);
      if (originalTechnique)
        return originalTechnique;
    }
    else
    {
      Ogre::Entity *entity = subEntity->getParent();
      if (!entity)
        return NULL;

      std::string userAny = "";
      try
      {
        userAny = Ogre::any_cast<std::string>(
            entity->getUserObjectBindings().getUserAny());
      }
      catch(Ogre::Exception &e)
      {
        return NULL;
      }

      rendering::VisualPtr result =
          this->camera->GetScene()->GetVisual(userAny);

      if (!result)
        return NULL;

      if (result->IsPlane())
      {
        // use grey color for planes
        material = "Gazebo/EditorPlane";
      }
      else
      {
        // set the rest of the visuals to use the model editor
        // background material
        material = "Gazebo/Editor";
      }
    }

    if (material.empty())
      return NULL;

    // set the material for the models
    Ogre::ResourcePtr res =
        Ogre::MaterialManager::getSingleton().getByName(material);
    if (res.isNull())
    {
        Ogre::MaterialManager::getSingleton().load(material,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }

    // OGRE 1.9 changes the shared pointer definition
    // But the 1.9 RC, which we're using on Windows, doesn't have the
    // staticCast change.  It will be in the final release.
    #if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0)) || defined(_WIN32)
    // Make sure we keep the same depth properties so that
    // certain overlay objects can be picked by the mouse.
    Ogre::Technique *newTechnique =
        static_cast<Ogre::MaterialPtr>(res)->getTechnique(0);
    #else
    Ogre::Technique *newTechnique =
        res.staticCast<Ogre::Material>()->getTechnique(0);
    #endif

    return newTechnique;
  }
  return NULL;
}
