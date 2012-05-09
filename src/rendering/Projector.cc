/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Projector
 * Author: Jared Duke, (some maintainence by John Hsu)
 */

#include "rendering/Projector.hh"

using namespace gazebo;
using namespace rendering;

typedef std::map<std::string, Ogre::Pass*> OgrePassMap;
typedef OgrePassMap::iterator OgrePassMapIterator;

/////////////////////////////////////////////////
Projector::Projector(rendering::VisualPtr _parent)
{
  this->fov = M_PI * 0.25;
  this->nearClip = 0.25;
  this->farClip = 15.0;

  this->parentVisual = _parent;
  this->scene = _parent->GetScene();
}

/////////////////////////////////////////////////
Projector::~Projector()
{
  // Ogre cleanup
  Ogre::Root::getSingletonPtr()->removeFrameListener(&this->projector);
}

/////////////////////////////////////////////////
void Projector::Load(sdf::ElementPtr _sdf)
{
  /*
  if (!_sdf->HasElement("xyz"))
  {
    gzmsg << "projector plugin missing <xyz>, defaults to 0s\n";
    this->xyz = math::Vector3(0, 0, 0);
  }
  else
    this->xyz = _sdf->GetElement("xyz")->GetValueVector3();

  if (!_sdf->HasElement("rpy"))
  {
    gzmsg << "projector plugin missing <rpy>, defaults to 0s\n";
    this->rpy = math::Vector3(0, 0, 0);
  }
  else
    this->rpy = _sdf->GetElement("rpy")->GetValueVector3();

  this->textureName = "";
  if (_sdf->HasElement("textureName"))
    this->textureName = _sdf->GetElement("textureName")->GetValueString();

  this->filterTextureName = "";
  if (_sdf->HasElement("filterTextureName"))
    this->filterTextureName =
      _sdf->GetElement("filterTextureName")->GetValueString();

  this->fov = Ogre::Math::PI*0.25;
  if (_sdf->HasElement("fov"))
    this->fov = _sdf->GetElement("fov")->GetValueDouble();

  this->nearClip = 0.1;
  if (_sdf->HasElement("nearClip"))
    this->nearClip = _sdf->GetElement("nearClip")->GetValueDouble();

  this->farClip = 15.0;
  if (_sdf->HasElement("farClip"))
    this->farClip = _sdf->GetElement("farClip")->GetValueDouble();

  // UpdateShaders when a new model is added to the world (e.g. via gui)
  this->add_model_event_ = gazebo::event::Events::ConnectAddEntity(
    boost::bind(&Projector::UpdateShaders, this));

  // Start projector
  this->add_model_event_ = gazebo::event::Events::ConnectWorldUpdateStart(
    boost::bind(&Projector::ToggleProjector, this, true));
    */
}

/////////////////////////////////////////////////
void Projector::LoadImage(const std::string &_textureName)
{
  this->projector.setTextureName(_textureName);
}

/////////////////////////////////////////////////
void Projector::ToggleProjector(bool _projectorOn)
{
  if (!this->parentVisual)
  {
    gzerr << "Projector does not have a valid parent visual\n";
    return;
  }

  int retryCount = 0;

  // Initialize the projector
  while (!this->projector.isInit && retryCount < 10)
  {
    // init
    this->projector.Init(
        this->parentVisual->GetSceneNode(),
        this->scene->GetManager(),
        this->textureName, this->filterTextureName,
        this->nearClip, this->farClip, this->fov,
        this->parentVisual->GetName() + "_Projector"
        this->parentVisual->GetName() + "_ProjectorFilter");

    // set the projector pose relative to body
    this->projector.SetPose(this->xyz, this->rpy);

    // Add the projector as an Ogre frame listener
    Ogre::Root::getSingletonPtr()->addFrameListener(&this->this->projector);

    if (!this->projector.isInit)
    {
      gzwarn << "starting projector failed, retrying in 1 sec.\n";
      sleep(1);
      retryCount++;
    }
  }

  // if not headless
  if (this->projector.isInit)
  {
    // gzmsg << "projector initialized, toggling state to ["
    //       << _projectorOn << "]\n";
    this->lock.lock();
    this->this->projector.setEnabled(_projectorOn);
    this->lock.unlock();
    this->UpdateShaders();
  }
  else
    gzwarn << "could not start projector, toggle failed\n";
}

/////////////////////////////////////////////////
void Projector::UpdateShaders()
{
  rendering::RTShaderSystem::Instance()->UpdateShaders();
}


/////////////////////////////////////////////////
Projector::Projector::Projector()
{
  this->isEnabled = false;
  this->isInit = false;
  this->isUsingShaders = false;

  this->projectorNode = NULL;
  this->projectorFilterNode = NULL;
  this->projectorQuery = NULL;
  this->projectorFrustum = NULL;
  this->projectorFilterFrustum = NULL;

  this->projectorNodeName = "Projector";
  this->projectorFilterNodeName = "projectorFilter";
}

/////////////////////////////////////////////////
Projector::Projector::~Projector()
{
  removeProjectorPassFromMaterials();

  if (this->projectorNode)
  {
    this->projectorNode->detachObject(this->projectorFrustum);
    this->parentSceneNode->removeAndDestroyChild(this->projectorNodeName);
    this->projectorNode = NULL;
  }
  if (this->projectorFilterNode)
  {
    this->projectorFilterNode->detachObject(this->projectorFilterFrustum);
    this->parentSceneNode->removeAndDestroyChild(this->projectorFilterNodeName);
    this->projectorFilterNode = NULL;
  }

  delete this->projectorFrustum;
  delete this->projectorFilterFrustum;

  if (this->projectorQuery)
    this->sceneMgr->destroyQuery(this->projectorQuery);
}

/////////////////////////////////////////////////
void Projector::Projector::init(Ogre::SceneNode *sceneNodePtr,
                                      Ogre::SceneManager *sceneMgrPtr,
                                      Ogre::String textureName,
                                      Ogre::String filterTextureName,
                                      double nearDist,
                                      double farDist,
                                      double fov,
                                      std::string projectorNodeName,
                                      std::string projectorFilterNodeName)
{
  if (this->isInit)
    return;

  this->projectorNodeName = projectorNodeName;
  this->projectorFilterNodeName = projectorFilterNodeName;

  this->projectorFrustum = new Ogre::Frustum();
  this->projectorFilterFrustum = new Ogre::Frustum();
  this->projectorFilterFrustum->setProjectionType(Ogre::PT_ORTHOGRAPHIC);

  this->sceneMgr = sceneMgrPtr;
  this->projectorQuery = this->sceneMgr->createPlaneBoundedVolumeQuery(
      Ogre::PlaneBoundedVolumeList());

  this->parentSceneNode = sceneNodePtr;
  this->setSceneNode();
  this->setTextureName(textureName);
  this->setFilterTextureName(filterTextureName);
  this->setFrustumClipDistance(nearDist, farDist);
  this->setFrustumFOV(fov);

  this->isInit = true;
}

/////////////////////////////////////////////////
bool Projector::Projector::frameStarted(const Ogre::FrameEvent &evt)
{
  if (!isInit)
    init(this->projectorNode);

  if (isEnabled && !projectedTextureName.empty())
  {
    addProjectorPassToVisibleMaterials();
    // addProjectorPassToAllMaterials();
  }
  else
  {
    removeProjectorPassFromMaterials();
  }

  return true;
}

/////////////////////////////////////////////////
bool Projector::Projector::frameEnded(const Ogre::FrameEvent &evt)
{
  return true;
}

/////////////////////////////////////////////////
bool Projector::Projector::frameRenderingQueued(
  const Ogre::FrameEvent &evt)
{
  return true;
}

/////////////////////////////////////////////////
void Projector::Projector::setEnabled(bool enabled)
{
  this->isEnabled = enabled;
}

/////////////////////////////////////////////////
void Projector::Projector::setUsingShaders(bool usingShaders)
{
  this->isUsingShaders = usingShaders;
}

/////////////////////////////////////////////////
void Projector::Projector::setSceneNode()
{
  if (this->projectorNode)
  {
    this->projectorNode->detachObject(this->projectorFrustum);
    this->parentSceneNode->removeAndDestroyChild(this->projectorNodeName);
    this->projectorNode = NULL;
  }
  if (this->projectorFilterNode)
  {
    this->projectorFilterNode->detachObject(this->projectorFilterFrustum);
    this->parentSceneNode->removeAndDestroyChild(this->projectorFilterNodeName);
    this->projectorFilterNode = NULL;
  }

  this->projectorNode       = this->parentSceneNode->createChildSceneNode(
                              this->projectorNodeName);
  this->projectorFilterNode = this->parentSceneNode->createChildSceneNode(
                              this->projectorFilterNodeName);

  if (this->projectorNode)
  {
    this->projectorNode->attachObject(this->projectorFrustum);
  }
  if (this->projectorFilterNode)
  {
    this->projectorFilterNode->attachObject(this->projectorFilterFrustum);
    this->projectorFilterNode->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y));
  }
}

/////////////////////////////////////////////////
void Projector::Projector::SetPose(const math::Pose &_pose)
{
  Ogre::Quaternion ogreQuaternion = Conversions::Convert(_pose.rot);
  Ogre::Quaternion offsetQuaternion;

  this->projectorNode->setPosition(Conversions::Convert(_pose.pos));
  this->projectorNode->setOrientation(ogreQuaternion);
  this->projectorFilterNode->setPosition(Conversions::Convert(_pos.rot));

  offsetQuaternion = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);
  this->projectorFilterNode->setOrientation(offsetQuaternion + ogreQuaternion);
}

/////////////////////////////////////////////////
void Projector::Projector::SetTextureName(const std::string &_textureName)
{
  this->projectedTextureName = _textureName;
}

/////////////////////////////////////////////////
void Projector::Projector::SetFilterTextureName(const std::string &_textureName)
{
  this->projectedFilterTextureName = _textureName;
}

/////////////////////////////////////////////////
void Projector::Projector::SetFrustumClipDistance(double _near, double _far)
{
  this->projectorFrustum->setNearClipDistance(_near);
  this->projectorFilterFrustum->setNearClipDistance(_near);
  this->projectorFrustum->setFarClipDistance(_far);
  this->projectorFilterFrustum->setFarClipDistance(_far);
}

/////////////////////////////////////////////////
void Projector::Projector::SetFrustumFOV(double _fov)
{
  this->projectorFrustum->setFOVy(Ogre::Radian(_fov));
  this->projectorFilterFrustum->setFOVy(Ogre::Radian(_fov));
}

/////////////////////////////////////////////////
void Projector::Projector::AddPassToAllMaterials()
{
  std::list<std::string> allMaterials;

  Ogre::SceneManager::MovableObjectIterator it =
    this->sceneMgr->getMovableObjectIterator("Entity");

  while (it.hasMoreElements())
  {
    Ogre::Entity* entity = dynamic_cast<Ogre::Entity*>(it.getNext());
    if (entity && entity->getName().find("visual") != std::string::npos)

    for (unsigned int i = 0; i < entity->getNumSubEntities(); i++)
    {
      allMaterials.push_back(entity->getSubEntity(i)->getMaterialName());
    }
  }

  addProjectorPassToMaterials(allMaterials);
}

/////////////////////////////////////////////////
void Projector::Projector::AddPassToVisibleMaterials()
{
  Ogre::PlaneBoundedVolumeList volumeList;
  volumeList.push_back(projectorFrustum->getPlaneBoundedVolume());

  this->projectorQuery->setVolumes(volumeList);
  SceneQueryResult result = this->projectorQuery->execute();

  std::list<std::string> newVisibleMaterials;

  // Find all visible materials
  Ogre::SceneQueryResultMovableList::iterator it;
  for (it = result.movables.begin(); it != result.movables.end(); ++it)
  {
    Ogre::Entity* entity = dynamic_cast<Ogre::Entity*>(*it);
    if (entity && entity->getName().find("visual") != std::string::npos)
    {
      for (unsigned int i = 0; i < entity->getNumSubEntities(); i++)
      {
        // addProjectorPassToMaterial(
        //   entity->getSubEntity(i)->getMaterialName());
        newVisibleMaterials.push_back(
          entity->getSubEntity(i)->getMaterialName());
      }
    }
  }

  addProjectorPassToMaterials(newVisibleMaterials);
}

/////////////////////////////////////////////////
void Projector::Projector::AddPassToMaterials(std::list<std::string> &_matList)
{
  _matList.remove("");
  _matList.unique();

  // Loop through all existing passes, removing those for materials
  //   not in the newlist and skipping pass creation for those in the
  //   newlist that have already been created
  OgrePassMapIterator used = projectorTargets.begin();
  while (used != projectorTargets.end())
  {
    std::list<std::string>::iterator visibleMaterial =
      std::find(_matList.begin(), _matList.end(), used->first);

    // Remove the pass if it applies to a material not in the new list
    if (visibleMaterial == _matList.end())
    {
      std::string invisibleMaterial = used->first;
      ++used;
      removeProjectorPassFromMaterial(invisibleMaterial);
    }
    // Otherwise remove it from the list of passes to be added
    else
    {
      _matList.remove(used->first);
      ++used;
    }
  }

  // Add pass for new materials
  while (!_matList.empty())
  {
    addProjectorPassToMaterial(_matList.front());
    _matList.erase(_matList.begin());
  }
}

/////////////////////////////////////////////////
void Projector::Projector::AddPassToMaterial(const std::string &_matName)
{
  if (projectorTargets.find(_matName) != projectorTargets.end())
  {
    gzerr << "Adding a Material [" << _matName << "] that already exists.";
    return;
  }

  Ogre::MaterialPtr mat =
    (Ogre::MaterialPtr)MaterialManager::getSingleton().getByName(matName);
  Ogre::Pass *pass = mat->getTechnique(0)->createPass();

  if (isUsingShaders)
  {
    Ogre::Matrix4 viewProj = projectorFrustum->getProjectionMatrix()
      * projectorFrustum->getViewMatrix();
    pass->setVertexProgram("GazeboWorlds/TexProjectionVP");

    // pass->setFragmentProgram("GazeboWorlds/TexProjectionFP");
    Ogre::GpuProgramParametersSharedPtr vsParams =
      pass->getVertexProgramParameters();
    Ogre::GpuProgramParametersSharedPtr psParams =
      pass->getFragmentProgramParameters();
    // vsParams->setNamedAutoConstant(
    //   "worldViewProjMatrix",
    //   GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
    // vsParams->setNamedAutoConstant(
    //   "worldMatrix",GpuProgramParameters::ACT_WORLD_MATRIX);
    // vsParams->setNamedConstant("texViewProjMatrix", viewProj);
    vsParams->setNamedAutoConstant(
      "worldMatrix", GpuProgramParameters::ACT_WORLD_MATRIX);
    vsParams->setNamedConstant("texProjMatrix", viewProj);
    // psParams->setNamedConstant("projMap", viewProj);
    pass->setVertexProgramParameters(vsParams);
    // pass->setFragmentProgramParameters(psParams);
  }

  Ogre::TextureUnitState *texState =
    pass->createTextureUnitState(projectedTextureName);
  texState->setProjectiveTexturing(true, projectorFrustum);
  texState->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);
  texState->setTextureFiltering(Ogre::TFO_ANISOTROPIC);
  texState->setTextureBorderColour(Ogre::ColourValue(0.0, 0.0, 0.0, 0.0));
  texState->setColourOperation(Ogre::LBO_ALPHA_BLEND);

  texState = pass->createTextureUnitState(projectedFilterTextureName);
  texState->setProjectiveTexturing(true, projectorFilterFrustum);
  texState->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  texState->setTextureFiltering(Ogre::TFO_NONE);

  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthBias(1);
  pass->setLightingEnabled(false);

  this->projectorTargets[ matName ] = pass;
}

/////////////////////////////////////////////////
void Projector::Projector::RemovePassFromMaterials()
{
  for (OgrePassMap::const_iterator it = this->projectorTargets.begin();
      it != this->projectorTargets.end(); ++it)
  {
    it->second->getParent()->removePass(it->second->getIndex());
  }
  this->projectorTargets.clear();
}

/////////////////////////////////////////////////
void Projector::Projector::RemovePassFromMaterial(const std::string &_matName)
{
  this->projectorTargets[_matName]->getParent()->removePass(
    this->projectorTargets[_matName]->getIndex());
  this->projectorTargets.erase(this->projectorTargets.find(_matName));
}
