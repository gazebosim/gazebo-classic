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
#include <boost/lexical_cast.hpp>

#include "rendering/ogre.h"
#include "rendering/Visual.hh"
#include "rendering/Conversions.hh"
#include "rendering/Scene.hh"
#include "common/MeshManager.hh"
#include "rendering/SelectionObj.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
SelectionObj::SelectionObj(Scene *scene_)
  : scene(scene_)
{
  this->active = false;
  this->CreateMaterials();
}

//////////////////////////////////////////////////
SelectionObj::~SelectionObj()
{
  this->node.reset();
}

//////////////////////////////////////////////////
void SelectionObj::Init()
{
  this->node.reset(new Visual("selection_obj_visual",
                              this->scene->GetWorldVisual()));

  Visual::InsertMesh(common::MeshManager::Instance()->GetMesh("unit_box"));

  Ogre::SceneNode *transNode =
    this->node->GetSceneNode()->createChildSceneNode("trans_node");

  transNode->setInheritOrientation(false);

  Ogre::SceneNode *xBox[2], *yBox[2], *zBox[2];

  for (int i = 0; i < 2; i++)
  {
    xBox[i] = transNode->createChildSceneNode(
        "selection_transX" + boost::lexical_cast<std::string>(i));
    xBox[i]->setInheritScale(true);

    yBox[i] = transNode->createChildSceneNode(
        "selection_transY" + boost::lexical_cast<std::string>(i));
    yBox[i]->setInheritScale(true);

    zBox[i] = transNode->createChildSceneNode(
        "selection_transZ" + boost::lexical_cast<std::string>(i));
    zBox[i]->setInheritScale(true);
  }

  Ogre::MovableObject *boxObjX1 =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          "__SELECTION_OBJX1__", "unit_box"));
  boxObjX1->setCastShadows(false);
  boxObjX1->setUserAny(Ogre::Any(std::string("transx")));
  ((Ogre::Entity*)boxObjX1)->setMaterialName(
    "__GAZEBO_TRANSX_SELECTION_MATERIAL__");
  boxObjX1->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::MovableObject *boxObjX2 =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          "__SELECTION_OBJX2__", "unit_box"));
  boxObjX2->setCastShadows(false);
  boxObjX2->setUserAny(Ogre::Any(std::string("transx")));
  ((Ogre::Entity*)boxObjX2)->setMaterialName(
    "__GAZEBO_TRANSX_SELECTION_MATERIAL__");
  boxObjX2->setVisibilityFlags(GZ_VISIBILITY_GUI);


  Ogre::MovableObject *boxObjY1 =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          "__SELECTION_OBJY1__", "unit_box"));
  boxObjY1->setCastShadows(false);
  boxObjY1->setUserAny(Ogre::Any(std::string("transy")));
  ((Ogre::Entity*)boxObjY1)->setMaterialName(
    "__GAZEBO_TRANSY_SELECTION_MATERIAL__");
  boxObjY1->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::MovableObject *boxObjY2 =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          "__SELECTION_OBJY2__", "unit_box"));
  boxObjY2->setCastShadows(false);
  boxObjY2->setUserAny(Ogre::Any(std::string("transy")));
  ((Ogre::Entity*)boxObjY2)->setMaterialName(
    "__GAZEBO_TRANSY_SELECTION_MATERIAL__");
  boxObjY2->setVisibilityFlags(GZ_VISIBILITY_GUI);


  Ogre::MovableObject *boxObjZ1 =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          "__SELECTION_OBJZ1__", "unit_box"));
  boxObjZ1->setCastShadows(false);
  boxObjZ1->setUserAny(Ogre::Any(std::string("transz")));
  ((Ogre::Entity*)boxObjZ1)->setMaterialName(
    "__GAZEBO_TRANSZ_SELECTION_MATERIAL__");
  boxObjZ1->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::MovableObject *boxObjZ2 =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          "__SELECTION_OBJZ2__", "unit_box"));
  boxObjZ2->setCastShadows(false);
  boxObjZ2->setUserAny(Ogre::Any(std::string("transz")));
  ((Ogre::Entity*)boxObjZ2)->setMaterialName(
    "__GAZEBO_TRANSZ_SELECTION_MATERIAL__");
  boxObjZ2->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->boxSize = 0.2;

  xBox[0]->attachObject(boxObjX1);
  xBox[0]->setInheritOrientation(false);
  xBox[0]->setScale(this->boxSize, this->boxSize, this->boxSize);
  xBox[0]->setPosition(1.5, 0, 0);

  xBox[1]->attachObject(boxObjX2);
  xBox[1]->setInheritOrientation(false);
  xBox[1]->setScale(this->boxSize, this->boxSize, this->boxSize);
  xBox[1]->setPosition(-1.5, 0, 0);

  yBox[0]->attachObject(boxObjY1);
  yBox[0]->setInheritOrientation(false);
  yBox[0]->setScale(this->boxSize, this->boxSize, this->boxSize);
  yBox[0]->setPosition(0, 1.5, 0);

  yBox[1]->attachObject(boxObjY2);
  yBox[1]->setInheritOrientation(false);
  yBox[1]->setScale(this->boxSize, this->boxSize, this->boxSize);
  yBox[1]->setPosition(0, -1.5, 0);


  zBox[0]->attachObject(boxObjZ1);
  zBox[0]->setInheritOrientation(false);
  zBox[0]->setScale(this->boxSize, this->boxSize, this->boxSize);
  zBox[0]->setPosition(0, 0, 1.5);

  zBox[1]->attachObject(boxObjZ2);
  zBox[1]->setInheritOrientation(false);
  zBox[1]->setScale(this->boxSize, this->boxSize, this->boxSize);
  zBox[1]->setPosition(0, 0, -1.5);

  this->node->SetVisible(false);
}

//////////////////////////////////////////////////
void SelectionObj::Attach(VisualPtr _visual)
{
  this->Clear();
  if (_visual)
  {
    Ogre::Node *transNode;

    math::Box box = _visual->GetBoundingBox();
    _visual->AttachVisual(this->node);

    std::cout << _visual->GetPose() << "\n";
    box = box - _visual->GetPose().pos;
    std::cout << "Box[" << box << "]\n";

    transNode = this->node->GetSceneNode()->getChild("trans_node");
    transNode->getChild("selection_transX0")->setPosition(
        -box.GetXLength()*0.5 - this->boxSize, 0, 
        box.GetZLength()*0.5 + box.min.z);
    transNode->getChild("selection_transX1")->setPosition(
        box.GetXLength()*0.5 + this->boxSize, 0, 
        box.GetZLength()*0.5 + box.min.z);

    transNode->getChild("selection_transY0")->setPosition(0,
        -box.GetYLength()*0.5 - this->boxSize, 
        box.GetZLength()*0.5 + box.min.z);
    transNode->getChild("selection_transY1")->setPosition(0,
        box.GetYLength()*0.5 + this->boxSize, 
        box.GetZLength()*0.5 + box.min.z);

    transNode->getChild("selection_transZ0")->setPosition(0, 0,
        box.min.z - this->boxSize);
    transNode->getChild("selection_transZ1")->setPosition(0, 0,
        box.max.z + this->boxSize);

    this->node->SetVisible(true);
    this->visualName = _visual->GetName();
  }
}

//////////////////////////////////////////////////
void SelectionObj::Clear()
{
  if (this->node->GetSceneNode()->getParentSceneNode())
  {
    this->node->GetSceneNode()->getParentSceneNode()->removeChild(
        this->node->GetSceneNode());
  }

  this->node->SetVisible(false);
}

std::string SelectionObj::GetVisualName() const
{
  return this->visualName;
}

bool SelectionObj::IsActive() const
{
  return this->active;
}

void SelectionObj::SetActive(bool _active)
{
  this->active = _active;
}

void SelectionObj::SetHighlight(const std::string &_mod)
{
  Ogre::ColourValue color;

  std::map<std::string, std::string> matNames;
  matNames["transx"] = "__GAZEBO_TRANSX_SELECTION_MATERIAL__";
  matNames["transy"] = "__GAZEBO_TRANSY_SELECTION_MATERIAL__";
  matNames["transz"] = "__GAZEBO_TRANSZ_SELECTION_MATERIAL__";

  std::map<std::string, std::string >::iterator iter;
  for (iter = matNames.begin(); iter != matNames.end(); ++iter)
  {
    Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(iter->second);

    if (!mat.isNull())
    {
      Ogre::Technique *technique = mat->getTechnique(0);
      Ogre::Pass *pass = technique->getPass(0);
      Ogre::TextureUnitState *texState = pass->getTextureUnitState(1);

      if (iter->first != _mod)
        texState->setColourOperationEx(Ogre::LBX_ADD, Ogre::LBS_MANUAL,
            Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 0));
      else
      {
        texState->setColourOperationEx(Ogre::LBX_ADD, Ogre::LBS_MANUAL,
            Ogre::LBS_CURRENT, Ogre::ColourValue(.2, .2, .2));
      }
    }
  }
}

void SelectionObj::CreateMaterials()
{
  Ogre::MaterialPtr mat;
  Ogre::Technique *tech;
  Ogre::Pass *pass;
  Ogre::TextureUnitState *texState;

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANSX_SELECTION_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(1.0, 0.0, 0.0);
  pass->setDiffuse(1.0, 0.0, 0.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(1, 0, 0));
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_ADD, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 0));


  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANSY_SELECTION_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(0.0, 1.0, 0.0);
  pass->setDiffuse(0.0, 1.0, 0.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 1, 0));
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_ADD, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 0));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANSZ_SELECTION_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(0.0, 0.0, 1.0);
  pass->setDiffuse(0.0, 0.0, 1.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 1));
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_ADD, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 0));
}
