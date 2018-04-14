/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "plugins/IRSensorPlugin.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  /// \brief Material switcher for the model editor used to toggle the
  /// material of the model.
  class IRMaterialHandler
      : public Ogre::RenderTargetListener, Ogre::MaterialManager::Listener
  {
    /// \brief Constructor
    /// \param[in] _camera Pointer to the camera whose viewport will be
    /// updated to see the effect of the material switch.
    public: IRMaterialHandler(const rendering::CameraPtr &_camera);

    /// \brief Destructor
    public: ~IRMaterialHandler() = default;

    /// \brief Set the material scheme that will be applied to the models
    /// in the editor
    /// \param[in] _scheme Name of material scheme
    public: void SetMaterialScheme(const std::string &_scheme);

    public: void SetTargets(const std::set<std::string> &_targets);

    /// \brief Get the material scheme applied to the models in the editor
    /// \return Name of material scheme
    public: std::string MaterialScheme() const;

    /// \brief Ogre's pre-render update callback
    /// \param[in] _evt Ogre render target event containing information about
    /// the source render target.
    public: virtual void preRenderTargetUpdate(
                const Ogre::RenderTargetEvent &_evt);

    /// \brief Ogre's post-render update callback
    /// \param[in] _evt Ogre render target event containing information about
    /// the source render target.
    public: virtual void postRenderTargetUpdate(
                const Ogre::RenderTargetEvent &_evt);

    /// \brief Ogre callback that is used to specify the material to use when
    /// the requested scheme is not found
    /// \param[in] _schemeIndex Index of scheme requested
    /// \param[in] _schemeName Name of scheme requested
    /// \param[in] _originalMaterial Orignal material that does not contain
    /// the requested scheme
    /// \param[in] _lodIndex The material level-of-detail
    /// \param[in] _rend Pointer to the Ogre::Renderable object requesting
    /// the use of the techinique
    /// \return The Ogre material technique to use when scheme is not found.
    public: virtual Ogre::Technique *handleSchemeNotFound(
                uint16_t _schemeIndex, const Ogre::String &_schemeName,
                Ogre::Material *_originalMaterial, uint16_t _lodIndex,
                const Ogre::Renderable *_rend);

    /// \brief Pointer to the camera
    private: rendering::CameraPtr camera;

    /// \brief Name of the original material scheme
    private: std::string originalMaterialScheme;

    /// \brief Name of the material scheme being used.
    private: std::string materialScheme;

    private: std::set<std::string> targets;
  };
}

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(IRSensorPlugin)

/////////////////////////////////////////////////
IRMaterialHandler::IRMaterialHandler(
    const rendering::CameraPtr &_camera)
{
  this->camera = _camera;

  this->materialScheme = "";

  if (!this->camera)
  {
    gzerr << "Cannot create a material handler. "
          << "Camera is NULL" << std::endl;
    return;
  }

  std::string bgMatStr = "Gazebo/IRBGBlack";
  if (!Ogre::MaterialManager::getSingleton().resourceExists(bgMatStr))
  {
    Ogre::MaterialPtr bgMat = Ogre::MaterialManager::getSingleton().create(
      bgMatStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Technique *tech = bgMat->getTechnique(0);
    Ogre::Pass *pass = tech->getPass(0);
    pass->setAmbient(0, 0, 0);
    pass->setDiffuse(0, 0, 0, 1);
    pass->setSpecular(0, 0, 0, 1);
    bgMat->load();
  }

  std::string targetMatStr = "Gazebo/IRTargetWhite";
  if (!Ogre::MaterialManager::getSingleton().resourceExists(targetMatStr))
  {
    Ogre::MaterialPtr targetMat = Ogre::MaterialManager::getSingleton().create(
      targetMatStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Technique *tech = targetMat->getTechnique(0);
    tech->setLightingEnabled(false);
    Ogre::Pass *pass = tech->getPass(0);
    pass->setAmbient(1, 1, 1);
    pass->setDiffuse(1, 1, 1, 1);
    pass->setSpecular(1, 1, 1, 1);
    targetMat->load();
  }
}

/////////////////////////////////////////////////
void IRMaterialHandler::SetMaterialScheme(const std::string &_scheme)
{
  if (!this->camera || !this->camera->OgreViewport())
    return;

  this->materialScheme = _scheme;

  if (_scheme.empty())
  {
    this->camera->OgreViewport()->setMaterialScheme(
        this->originalMaterialScheme);
    this->camera->OgreViewport()->getTarget()->removeListener(
        this);
  }
  else
  {
    this->originalMaterialScheme =
        this->camera->OgreViewport()->getMaterialScheme();

    this->camera->OgreViewport()->setMaterialScheme(_scheme);
    this->camera->OgreViewport()->getTarget()->addListener(
        this);
  }
}

/////////////////////////////////////////////////
void IRMaterialHandler::SetTargets(const std::set<std::string> &_targets)
{
  this->targets = _targets;

}

/////////////////////////////////////////////////
std::string IRMaterialHandler::MaterialScheme() const
{
  return this->materialScheme;
}

/////////////////////////////////////////////////
void IRMaterialHandler::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  this->camera->OgreViewport()->setBackgroundColour(
      Ogre::ColourValue(0, 0, 0, 1));

  this->camera->OgreViewport()->setShadowsEnabled(false);

  rendering::ScenePtr scene = this->camera->GetScene();
  for (auto it = this->targets.begin(); it != this->targets.end();)
  {
    rendering::VisualPtr vis = scene->GetVisual(*it);
    if (!vis)
    {
      gzerr << "Fiducial target not found: " << *it << std::endl;
      ++it;
      continue;
    }

    ignition::math::Vector3d geomSize = vis->GetGeometrySize();

    rendering::VisualPtr visGlow(new rendering::Visual(*it+"_glow", vis));
    visGlow->Load();
    visGlow->AttachMesh("unit_sphere");
    eisGlow->SetScale(2*geomSize);
    visGlow->SetTransparency(1.0);

    Ogre::SceneNode *node = visGlow->GetSceneNode();
    Ogre::MovableObject *obj = node->getAttachedObject(0);
    obj->getUserObjectBindings().setUserAny(Ogre::Any(std::string("ir_glow")));
    this->targets.erase(++it);
  }


  Ogre::MaterialManager::getSingleton().addListener(this);
}

/////////////////////////////////////////////////
void IRMaterialHandler::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

/////////////////////////////////////////////////
Ogre::Technique *IRMaterialHandler::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String & /*_schemeName*/,
    Ogre::Material *_originalMaterial, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  if (_rend && typeid(*_rend) == typeid(Ogre::SubEntity))
  {
    std::string material = "";

    const Ogre::SubEntity *subEntity =
      static_cast<const Ogre::SubEntity *>(_rend);

    if (!subEntity)
    {
      gzerr << "Unable to get an Ogre sub-entity" << std::endl;
      return nullptr;
    }

    Ogre::Entity *entity = subEntity->getParent();
    if (!entity)
    {
      gzerr << "Unable to get an Ogre entity" << std::endl;
      return nullptr;
    }

    if (entity->getUserObjectBindings().getUserAny().isEmpty())
      return nullptr;

    std::string userAny = "";
    try
    {
      userAny = Ogre::any_cast<std::string>(
          entity->getUserObjectBindings().getUserAny());
    }
    catch(Ogre::Exception &e)
    {
      gzerr << "Unable to cast Ogre user data" << std::endl;
      return nullptr;
    }

//    if (this->targets.find(userAny+"_glow") != this->targets.end())
    if (userAny == "ir_glow")
    {
      material = "Gazebo/IRTargetWhite";
    }
    else
    {
      material = "Gazebo/IRBGBlack";
    }

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
  return nullptr;
}


/////////////////////////////////////////////////
IRSensorPlugin::IRSensorPlugin()
: SensorPlugin()
{
}

/////////////////////////////////////////////////
IRSensorPlugin::~IRSensorPlugin()
{
  this->newFrameConnection.reset();
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void IRSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer." << std::endl;

  this->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "IRSensorPlugin requires a CameraSensor." << std::endl;
    return;
  }

  this->camera = this->parentSensor->Camera();

  if (!this->parentSensor)
  {
    gzerr << "IRSensorPlugin not attached to a camera sensor" << std::endl;
    return;
  }

  // load the fiducial targets
  if (_sdf->HasElement("target"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("target");
    while (elem)
    {
      this->targets.insert(elem->Get<std::string>());
      elem = elem->GetNextElement("target");
    }
  }
  else
  {
    gzwarn << "No IR targets specified!" << std::endl;
  }

  this->materialHandler.reset(new IRMaterialHandler(camera));
  this->materialHandler->SetMaterialScheme("IR");
  this->materialHandler->SetTargets(targets);

/*
  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&IRSensorPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));
*/
}

/////////////////////////////////////////////////
void IRSensorPlugin::OnNewFrame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/camera/me.jpg");
    */
}
