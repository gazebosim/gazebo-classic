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
 * Desc: Projector plugin
 * Author: Jared Duke, (some maintainence by John Hsu)
 */

#include <assert.h>

#include <Ogre.h>
#include <OgreMath.h>
#include <OgreSceneNode.h>
#include <OgreFrustum.h>
#include <OgreSceneQuery.h>

#include <algorithm>
#include <utility>
#include <sstream>

#include "physics/physics.h"
#include "ProjectorPlugin.hh"

#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"
#include "rendering/Visual.hh"
#include "rendering/RTShaderSystem.hh"

#include "common/Console.hh"

using namespace gazebo;


typedef std::map<std::string, Ogre::Pass*> OgrePassMap;
typedef OgrePassMap::iterator OgrePassMapIterator;


////////////////////////////////////////////////////////////////////////////////
// Constructor
ProjectorPlugin::ProjectorPlugin()
{
  this->fov = Ogre::Math::PI*.25;
  this->nearClipDist = .25;
  this->farClipDist = 15;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
ProjectorPlugin::~ProjectorPlugin()
{
  // Ogre cleanup
  getRootP()->removeFrameListener(&this->projector_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void ProjectorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->world = _parent->GetWorld();
  this->myParent = _parent;

  // projector node and filter node names
  std::ostringstream pn_stream;
  std::ostringstream pfn_stream;
  if (this->myParent)
  {
    pn_stream << this->myParent->GetName() << "_ProjectorNode";
    pfn_stream << this->myParent->GetName() << "_ProjectorFilterNode";
  }
  else
  {
    pn_stream << this->myBody->GetModel()->GetName() << "_ProjectorNode";
    pfn_stream << this->myBody->GetModel()->GetName() << "_ProjectorFilterNode";
  }
  this->projectorNodeName = pn_stream.str();
  this->projectorFilterNodeName = pfn_stream.str();


  // load parameters
  this->bodyName = "";
  if (_sdf->HasElement("bodyName"))
    this->bodyName = _sdf->GetElement("bodyName")->GetValueString();
  this->myBody = _parent->GetLink(this->bodyName);
  if (!this->myBody)
  {
    gzerr << "projector plugin error: bodyName: [" << this->bodyName
          << "] does not exist\n";
    return;
  }

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

  this->nearClipDist = 0.1;
  if (_sdf->HasElement("nearClipDist"))
    this->nearClipDist = _sdf->GetElement("nearClipDist")->GetValueDouble();

  this->farClipDist = 15.0;
  if (_sdf->HasElement("farClipDist"))
    this->farClipDist = _sdf->GetElement("farClipDist")->GetValueDouble();


  // UpdateShaders when a new model is added to the world (e.g. via gui)
  this->add_model_event_ = gazebo::event::Events::ConnectAddEntity(
    boost::bind(&ProjectorPlugin::UpdateShaders, this));
}


////////////////////////////////////////////////////////////////////////////////
// Load a texture into the projector
void ProjectorPlugin::LoadImage(const std::string &_texture_name)
{
  // if not headless
  {
    this->lock.lock();
    this->projector_.setTextureName(_texture_name);
    this->lock.unlock();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Toggle the activation of the projector
void ProjectorPlugin::ToggleProjector(bool _projectorOn)
{
  // Initialize the projector
  while (!projector_.isInit)
  {
    // get scene visual node
    this->scene = rendering::get_scene(this->world->GetName());
    this->myVisual = this->scene->GetVisual(this->myBody->GetScopedName());
    if (!this->myVisual)
    {
      gzmsg << "visual for bodyName: [" << this->myBody->GetScopedName()
            << "] not found\n";
    }
    else
    {
      this->mySceneNode = this->myVisual->GetSceneNode();

      // init
      projector_.init(this->mySceneNode,
                      this->scene->GetManager(),
                      this->textureName, this->filterTextureName,
                      this->nearClipDist, this->farClipDist, this->fov,
                      this->myBody->GetName()+"/projectorNodeName",
                      this->myBody->GetName()+"/projectorFilterNodeName");

      // set the projector pose relative to body
      projector_.setPose(this->xyz, this->rpy);

      // Add the projector as an Ogre frame listener
      getRootP()->addFrameListener(&this->projector_);
    }

    if (!projector_.isInit)
    {
      gzmsg << "starting projector failed, retrying in 1 sec.";
      sleep(1);
    }
  }

  // if not headless
  {
    this->lock.lock();
    this->projector_.setEnabled(_projectorOn);
    this->lock.unlock();
  }

  this->UpdateShaders();
}


////////////////////////////////////////////////////////////////////////////////
// update shaders so texture pass is active
void ProjectorPlugin::UpdateShaders()
{
  rendering::RTShaderSystem::Instance()->UpdateShaders();
}

Ogre::Root* ProjectorPlugin::getRootP()
{
  return Ogre::Root::getSingletonPtr();
}

////////////////////////////////////////////////////////////////////////////////
ProjectorPlugin::Projector::Projector()
{
  this->isEnabled = false;
  this->isInit = false;
  this->isUsingShaders = false;

  this->projectorNode = NULL;
  this->projectorFilterNode = NULL;
  this->projectorQuery = NULL;
  this->projectorFrustum = NULL;
  this->projectorFilterFrustum = NULL;

  this->projectorNodeName = "ProjectorNode";
  this->projectorFilterNodeName = "projectorFilterNode";
}

////////////////////////////////////////////////////////////////////////////////
ProjectorPlugin::Projector::~Projector()
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

void ProjectorPlugin::Projector::init(Ogre::SceneNode *sceneNodePtr,
                                      Ogre::SceneManager *sceneMgrPtr,
                                      Ogre::String textureName,
                                      Ogre::String filterTextureName,
                                      double nearDist,
                                      double farDist,
                                      double fov,
                                      std::string projectorNodeName,
                                      std::string projectorFilterNodeName)
{
  if        (this->isInit)
    return;

  this->projectorNodeName       = projectorNodeName;
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


////////////////////////////////////////////////////////////////////////////////
bool ProjectorPlugin::Projector::frameStarted(const Ogre::FrameEvent &evt)
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

////////////////////////////////////////////////////////////////////////////////
bool ProjectorPlugin::Projector::frameEnded(const Ogre::FrameEvent &evt)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool ProjectorPlugin::Projector::frameRenderingQueued(
  const Ogre::FrameEvent &evt)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setEnabled(bool enabled)
{
  this->isEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setUsingShaders(bool usingShaders)
{
  this->isUsingShaders = usingShaders;
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setSceneNode()
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

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setPose(math::Vector3 xyz,
  math::Quaternion rpy)
{
  gzdbg << "projector pose[" << xyz << "] [" << rpy << "]\n";

  this->projectorNode->setPosition(xyz.x, xyz.y, xyz.z);
  this->projectorNode->setOrientation(rpy.w, rpy.x, rpy.y, rpy.z);
  this->projectorFilterNode->setPosition(xyz.x, xyz.y, xyz.z);
  Ogre::Quaternion offset_q =
    Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);
  this->projectorFilterNode->setOrientation(
    offset_q+Ogre::Quaternion(rpy.w, rpy.x, rpy.y, rpy.z));
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setTextureName(
  const Ogre::String& textureName)
{
  this->projectedTextureName = textureName;
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setFilterTextureName(
  const Ogre::String& textureName)
{
  this->projectedFilterTextureName = textureName;
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setFrustumClipDistance(double nearDist,
  double farDist)
{
  this->projectorFrustum->setNearClipDistance(nearDist);
  this->projectorFilterFrustum->setNearClipDistance(nearDist);
  this->projectorFrustum->setFarClipDistance(farDist);
  this->projectorFilterFrustum->setFarClipDistance(farDist);
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::setFrustumFOV(double fovInRadians)
{
  this->projectorFrustum->setFOVy(Ogre::Radian(fovInRadians));
  this->projectorFilterFrustum->setFOVy(Ogre::Radian(fovInRadians));
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::addProjectorPassToAllMaterials()
{
  using namespace Ogre;

  std::list<std::string> allMaterials;

  SceneManager::MovableObjectIterator it =
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

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::addProjectorPassToVisibleMaterials()
{
  using namespace Ogre;

  PlaneBoundedVolumeList volumeList;
  volumeList.push_back(projectorFrustum->getPlaneBoundedVolume());

  this->projectorQuery->setVolumes(volumeList);
  SceneQueryResult result = this->projectorQuery->execute();

  std::list<std::string> newVisibleMaterials;

  // Find all visible materials
  SceneQueryResultMovableList::iterator it;
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

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::addProjectorPassToMaterials(
  std::list<std::string>& matList)
{
  matList.remove("");
  matList.unique();

  // Loop through all existing passes, removing those for materials
  //   not in the newlist and skipping pass creation for those in the
  //   newlist that have already been created
  OgrePassMapIterator used = projectorTargets.begin();
  while (used != projectorTargets.end())
  {
    std::list<std::string>::iterator visibleMaterial =
      std::find(matList.begin(), matList.end(), used->first);

    // Remove the pass if it applies to a material not in the new list
    if (visibleMaterial == matList.end())
    {
      std::string invisibleMaterial = used->first;
      ++used;
      removeProjectorPassFromMaterial(invisibleMaterial);
    }
    // Otherwise remove it from the list of passes to be added
    else
    {
      matList.remove(used->first);
      ++used;
    }
  }

  // Add pass for new materials
  while (!matList.empty())
  {
    addProjectorPassToMaterial(matList.front());
    matList.erase(matList.begin());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::addProjectorPassToMaterial(std::string matName)
{
  if (projectorTargets.find(matName) != projectorTargets.end())
  {
    gzdbg << "Adding a Material [" << matName << "] that already exists.";
    return;
  }

  using namespace Ogre;

  MaterialPtr mat =
    (MaterialPtr)MaterialManager::getSingleton().getByName(matName);
  Pass *pass = mat->getTechnique(0)->createPass();

  if (isUsingShaders)
  {
    Matrix4 viewProj = projectorFrustum->getProjectionMatrix()
      * projectorFrustum->getViewMatrix();
    pass->setVertexProgram("GazeboWorlds/TexProjectionVP");
    // pass->setFragmentProgram("GazeboWorlds/TexProjectionFP");
    GpuProgramParametersSharedPtr vsParams =
      pass->getVertexProgramParameters();
    GpuProgramParametersSharedPtr psParams =
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

  TextureUnitState *texState =
    pass->createTextureUnitState(projectedTextureName);
  texState->setProjectiveTexturing(true, projectorFrustum);
  texState->setTextureAddressingMode(TextureUnitState::TAM_BORDER);
  texState->setTextureFiltering(TFO_ANISOTROPIC);
  texState->setTextureBorderColour(ColourValue(0.0, 0.0, 0.0, 0.0));
  texState->setColourOperation(LBO_ALPHA_BLEND);

  texState = pass->createTextureUnitState(projectedFilterTextureName);
  texState->setProjectiveTexturing(true, projectorFilterFrustum);
  texState->setTextureAddressingMode(TextureUnitState::TAM_CLAMP);
  texState->setTextureFiltering(TFO_NONE);

  pass->setSceneBlending(SBT_TRANSPARENT_ALPHA);
  pass->setDepthBias(1);
  pass->setLightingEnabled(false);

  this->projectorTargets[ matName ] = pass;
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::removeProjectorPassFromMaterials()
{
  for (OgrePassMap::const_iterator it = this->projectorTargets.begin();
      it != this->projectorTargets.end(); ++it)
  {
    it->second->getParent()->removePass(it->second->getIndex());
  }
  this->projectorTargets.clear();
}

////////////////////////////////////////////////////////////////////////////////
void ProjectorPlugin::Projector::removeProjectorPassFromMaterial(
  std::string matName)
{
  this->projectorTargets[matName]->getParent()->removePass(
    this->projectorTargets[matName]->getIndex());
  this->projectorTargets.erase(this->projectorTargets.find(matName));
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ProjectorPlugin);

