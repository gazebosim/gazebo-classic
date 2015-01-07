/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Inertial.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/gui/model/PartVisualConfig.hh"
#include "gazebo/gui/model/PartGeneralConfig.hh"
#include "gazebo/gui/model/PartCollisionConfig.hh"

#include "gazebo/gui/model/ModelData.hh"

using namespace gazebo;
using namespace gui;


/////////////////////////////////////////////////
std::string ModelData::GetTemplateSDFString()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='template_model'>"
    << "<pose>0 0 0.0 0 0 0</pose>"
    << "<link name ='link'>"
    <<   "<visual name ='visual'>"
    <<     "<pose>0 0 0.0 0 0 0</pose>"
    <<     "<geometry>"
    <<       "<box>"
    <<         "<size>1.0 1.0 1.0</size>"
    <<       "</box>"
    <<     "</geometry>"
    <<     "<material>"
    <<       "<lighting>true</lighting>"
    <<       "<ambient>1 1 1 1</ambient>"
    <<       "<diffuse>1 1 1 1</diffuse>"
    <<       "<specular>1 1 1 1</specular>"
    <<       "<script>"
    <<         "<uri>file://media/materials/scripts/gazebo.material</uri>"
    <<         "<name>Gazebo/Grey</name>"
    <<       "</script>"
    <<     "</material>"
    <<   "</visual>"
    << "</link>"
    << "<static>true</static>"
    << "</model>"
    << "</sdf>";

  return newModelStr.str();
}

/////////////////////////////////////////////////
CollisionData::CollisionData()
{
  this->collisionSDF.reset(new sdf::Element);
  sdf::initFile("collision.sdf", this->collisionSDF);
}

/////////////////////////////////////////////////
PartData::PartData()
{
  this->partSDF.reset(new sdf::Element);
  sdf::initFile("link.sdf", this->partSDF);

  this->inspector = new PartInspector;
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));
  connect(this->inspector->GetVisualConfig(),
      SIGNAL(VisualAdded(const std::string &)),
      this, SLOT(OnAddVisual(const std::string &)));

  connect(this->inspector->GetCollisionConfig(),
      SIGNAL(CollisionAdded(const std::string &)),
      this, SLOT(OnAddCollision(const std::string &)));

  connect(this->inspector->GetVisualConfig(),
      SIGNAL(VisualRemoved(const std::string &)), this,
      SLOT(OnRemoveVisual(const std::string &)));

  connect(this->inspector->GetCollisionConfig(),
      SIGNAL(CollisionRemoved(const std::string &)),
      this, SLOT(OnRemoveCollision(const std::string &)));

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&PartData::Update, this)));
  this->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
PartData::~PartData()
{
  event::Events::DisconnectPreRender(this->connections[0]);
}

/////////////////////////////////////////////////
std::string PartData::GetName() const
{
  return this->partSDF->Get<std::string>("name");
}

/////////////////////////////////////////////////
void PartData::SetName(const std::string &_name)
{
  this->partSDF->GetAttribute("name")->Set(_name);
  this->inspector->SetName(_name);
}

/////////////////////////////////////////////////
math::Pose PartData::GetPose() const
{
  return this->partSDF->Get<math::Pose>("pose");
}

/////////////////////////////////////////////////
void PartData::SetPose(const math::Pose &_pose)
{
  this->partSDF->GetElement("pose")->Set(_pose);

  PartGeneralConfig *generalConfig = this->inspector->GetGeneralConfig();
  generalConfig->SetPose(_pose);
}

/////////////////////////////////////////////////
void PartData::AddVisual(rendering::VisualPtr _visual)
{
  PartVisualConfig *visualConfig = this->inspector->GetVisualConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_visual->GetSDF());

  // override transparency value
  visualMsg.set_transparency(1.0);

  // some of the default values do not transfer to the visualMsg
  // so set them here and find a better way to fix this in the future.
  //visualMsg.mutable_material()->set_lighting(true);
/*  msgs::Set(visualMsg.mutable_material()->mutable_ambient(),
      math::Color(0, 0, 0, 0);
  msgs::Set(visualMsg.mutable_material()->mutable_diffuse(),
      math::Color(0, 0, 0, 0);
  msgs::Set(visualMsg.mutable_material()->mutable_specular(),
      math::Color(0, 0, 0, 0);
  msgs::Set(visualMsg.mutable_material()->mutable_ambient(),
      math::Color(0, 0, 0, 0);*/

  this->visuals[_visual] = visualMsg;

/*  std::cerr << "_visual->GetSDF() "  <<
      _visual->GetSDF()->ToString("") << std::endl;;

  std::cerr << "===================================" << std::endl;
  std::cerr << "visualMsg "  <<
      visualMsg.DebugString() << std::endl;*/

  std::string partName = this->partVisual->GetName();
  std::string visName = _visual->GetName();
  std::string leafName =
      visName.substr(visName.find(partName)+partName.size()+1);

  visualConfig->AddVisual(leafName, &visualMsg);
}

/////////////////////////////////////////////////
void PartData::AddCollision(rendering::VisualPtr _collisionVis)
{
  PartCollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_collisionVis->GetSDF());

  sdf::ElementPtr collisionSDF(new sdf::Element);
  sdf::initFile("collision.sdf", collisionSDF);

  msgs::Collision collisionMsg;
  collisionMsg.set_name(_collisionVis->GetName());
  msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
  geomMsg->CopyFrom(visualMsg.geometry());

  this->collisions[_collisionVis] = collisionMsg;

  std::string partName = this->partVisual->GetName();
  std::string visName = _collisionVis->GetName();
  std::string leafName =
      visName.substr(visName.find(partName)+partName.size()+1);

  collisionConfig->AddCollision(leafName, &collisionMsg);
}

/////////////////////////////////////////////////
void PartData::OnApply()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  PartGeneralConfig *generalConfig = this->inspector->GetGeneralConfig();

  this->partSDF = msgs::LinkToSDF(*generalConfig->GetData(), this->partSDF);

  // update visuals
  if (!this->visuals.empty())
  {
    this->partVisual->SetWorldPose(this->GetPose());

    PartVisualConfig *visualConfig = this->inspector->GetVisualConfig();
    std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
    for (it = this->visuals.begin(); it != this->visuals.end(); ++it)
    {
      std::string name = it->first->GetName();
      std::string partName = this->partVisual->GetName();
      std::string leafName =
          name.substr(name.find(partName)+partName.size()+1);

      msgs::Visual *updateMsg = visualConfig->GetData(leafName);
      if (updateMsg)
      {
//         std::cerr << " updateMsg " << updateMsg->DebugString() << std::endl;

        msgs::Visual visualMsg = it->second;

        // update the visualMsg that will be used to generate the sdf.
        updateMsg->clear_scale();
        visualMsg.CopyFrom(*updateMsg);
        it->second = visualMsg;

        // std::cerr << " updateMsg after " << updateMsg->DebugString() << std::endl;

//        msgs::Visual msg;
//        msg.CopyFrom(*updateMsg);


        this->visualUpdateMsgs.push_back(updateMsg);

        //it->first->UpdateFromMsg(ConstVisualPtr(updateMsg));

      }
/*      if (this->visuals[i]->GetMeshName() != visual->GetGeometry(i))
      {
        this->visuals[i]->DetachObjects();
        this->visuals[i]->AttachMesh(visual->GetGeometry(i));
      }
      if (this->visuals[i]->GetMaterialName() != visual->GetMaterial(i))
      {
        this->visuals[i]->SetMaterial(visual->GetMaterial(i), false);
      }

      this->visuals[i]->SetPose(visual->GetPose(i));
      this->visuals[i]->SetTransparency(visual->GetTransparency(i));
      this->visuals[i]->SetScale(visual->GetGeometryScale(i));*/
    }
  }

  // update collisions
  if (!this->collisions.empty())
  {
    PartCollisionConfig *collisionConfig =
        this->inspector->GetCollisionConfig();
    std::map<rendering::VisualPtr, msgs::Collision>::iterator it;
    for (it = this->collisions.begin(); it != this->collisions.end(); ++it)
    {
      std::string name = it->first->GetName();
      std::string partName = this->partVisual->GetName();
      std::string leafName =
          name.substr(name.find(partName)+partName.size()+1);

      msgs::Collision *updateMsg = collisionConfig->GetData(leafName);
      if (updateMsg)
      {
        msgs::Collision collisionMsg = it->second;
        collisionMsg.CopyFrom(*updateMsg);
        it->second = collisionMsg;

        this->collisionUpdateMsgs.push_back(updateMsg);
      }
    }
  }
}

/////////////////////////////////////////////////
void PartData::OnAddVisual(const std::string &_name)
{
  // add a visual when the user adds a visual via the inspector's visual tab
  PartVisualConfig *visualConfig = this->inspector->GetVisualConfig();

  std::ostringstream visualName;
  visualName << this->partVisual->GetName() << "_" << _name;

  //visualConfig->SetName(visualConfig->GetVisualCount()-1, visualName.str());

  rendering::VisualPtr visVisual;
  rendering::VisualPtr refVisual;
  if (!this->visuals.empty())
  {
    // add new visual by cloning last instance
    refVisual = this->visuals.rbegin()->first;
    visVisual = refVisual->Clone(visualName.str(), this->partVisual);
    visVisual->SetScale(math::Vector3::One);
  }
  else
  {
    // create new visual based on sdf template (box)
    sdf::SDFPtr modelTemplateSDF(new sdf::SDF);
    modelTemplateSDF->SetFromString(
        ModelData::GetTemplateSDFString());

    visVisual.reset(new rendering::Visual(visualName.str(),
        this->partVisual));
    visVisual->SetScale(math::Vector3::One);
    sdf::ElementPtr visualElem =  modelTemplateSDF->root
        ->GetElement("model")->GetElement("link")->GetElement("visual");
    visVisual->Load(visualElem);
    visVisual->SetScale(math::Vector3::One);
    this->partVisual->GetScene()->AddVisual(visVisual);
  }

  msgs::Visual visualMsg = msgs::VisualFromSDF(visVisual->GetSDF());
  // store the correct transparency setting
  if (refVisual)
    visualMsg.set_transparency(this->visuals[refVisual].transparency());
  visualConfig->UpdateVisual(_name, &visualMsg);
  this->visuals[visVisual] = visualMsg;
  visVisual->SetTransparency(0.4);
}

/////////////////////////////////////////////////
void PartData::OnAddCollision(const std::string &_name)
{
  // add a collision when the user adds a collision via the inspector's
  // collision tab
  PartCollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();

  std::ostringstream collisionName;
  collisionName << this->partVisual->GetName() << "_" << _name;

  rendering::VisualPtr collisionVis;
  if (!this->collisions.empty())
  {
    // add new collision by cloning last instance
    collisionVis = this->collisions.rbegin()->first->Clone(collisionName.str(),
        this->partVisual);
  }
  else
  {
    // create new collision based on sdf template (box)
    sdf::SDFPtr modelTemplateSDF(new sdf::SDF);
    modelTemplateSDF->SetFromString(
        ModelData::GetTemplateSDFString());

    collisionVis.reset(new rendering::Visual(collisionName.str(),
        this->partVisual));
    sdf::ElementPtr collisionElem =  modelTemplateSDF->root
        ->GetElement("model")->GetElement("link")->GetElement("visual");
    collisionVis->Load(collisionElem);
    this->partVisual->GetScene()->AddVisual(collisionVis);
  }

  msgs::Visual visualMsg = msgs::VisualFromSDF(collisionVis->GetSDF());
  msgs::Collision collisionMsg;
  collisionMsg.set_name(collisionVis->GetName());
  msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
  geomMsg->CopyFrom(visualMsg.geometry());

  collisionConfig->UpdateCollision(_name, &collisionMsg);
  this->collisions[collisionVis] = collisionMsg;

  collisionVis->SetMaterial("Gazebo/Orange");
  collisionVis->SetTransparency(0.8);

  // fix for transparency alpha compositing
  Ogre::MovableObject *colObj = collisionVis->GetSceneNode()->
      getAttachedObject(0);
  colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
}

/////////////////////////////////////////////////
void PartData::OnRemoveVisual(const std::string &_name)
{
  // find and remove visual when the user removes it in the
  // inspector's visual tab
  std::ostringstream name;
  name << this->partVisual->GetName() << "_" << _name;
  std::string visualName = name.str();

  std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
  for (it = this->visuals.begin(); it != this->visuals.end(); ++it)
  {
    if (visualName == it->first->GetName())
    {
      this->partVisual->DetachVisual(it->first);
      this->partVisual->GetScene()->RemoveVisual(it->first);
      this->visuals.erase(it);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PartData::OnRemoveCollision(const std::string &_name)
{
  // find and remove collision visual when the user removes it in the
  // inspector's collision tab
  std::ostringstream name;
  name << this->partVisual->GetName() << "_" << _name;
  std::string collisoinName = name.str();

  std::map<rendering::VisualPtr, msgs::Collision>::iterator it;
  for (it = this->collisions.begin(); it != this->collisions.end(); ++it)
  {
    if (collisoinName == it->first->GetName())
    {
      this->partVisual->DetachVisual(it->first);
      this->partVisual->GetScene()->RemoveVisual(it->first);
      this->collisions.erase(it);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PartData::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);

  while (!this->visualUpdateMsgs.empty())
  {
    boost::shared_ptr<gazebo::msgs::Visual> updateMsgPtr;
    updateMsgPtr.reset(new msgs::Visual);
    updateMsgPtr->CopyFrom(*this->visualUpdateMsgs.front());

    this->visualUpdateMsgs.erase(this->visualUpdateMsgs.begin());
    std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
    for (it = this->visuals.begin(); it != this->visuals.end(); ++it)
    {
      if (it->second.name() == updateMsgPtr->name())
      {
        //std::string origMatName = it->first->GetMaterialName();

        // make visual semi-transparent here
        // but generated sdf will use the correct transparency value
        float transparency = 0.4;
//        msgs::Material *materialMsg = updateMsgPtr->mutable_material();
//        materialMsg->mutable_ambient()->set_a(transparency);
//        materialMsg->mutable_diffuse()->set_a(transparency);
//        materialMsg->mutable_specular()->set_a(transparency);
//        materialMsg->mutable_emissive()->set_a(transparency);

          //std::cerr << " updateMsg " << updateMsgPtr->DebugString() << std::endl;
        it->first->UpdateFromMsg(updateMsgPtr);
        it->first->SetTransparency(transparency);

        // if (it->first->GetMaterialName() != origMatName)
        break;
      }
    }
  }

  while (!this->collisionUpdateMsgs.empty())
  {
    msgs::Collision collisoinMsg = *this->collisionUpdateMsgs.front();
    this->collisionUpdateMsgs.erase(this->collisionUpdateMsgs.begin());
    std::map<rendering::VisualPtr, msgs::Collision>::iterator it;
    for (it = this->collisions.begin(); it != this->collisions.end(); ++it)
    {
      if (it->second.name() == collisoinMsg.name())
      {
        msgs::Visual collisionVisMsg;
        msgs::Geometry *geomMsg = collisionVisMsg.mutable_geometry();
        geomMsg->CopyFrom(collisoinMsg.geometry());
        msgs::Pose *poseMsg = collisionVisMsg.mutable_pose();
        poseMsg->CopyFrom(collisoinMsg.pose());

        boost::shared_ptr<gazebo::msgs::Visual> updateMsgPtr;
        updateMsgPtr.reset(new msgs::Visual);
        updateMsgPtr->CopyFrom(collisionVisMsg);

        std::string origGeomType = it->first->GetGeometryType();

        it->first->UpdateFromMsg(updateMsgPtr);

        // fix for transparency alpha compositing
        if (it->first->GetGeometryType() != origGeomType)
        {
          Ogre::MovableObject *colObj = it->first->GetSceneNode()->
              getAttachedObject(0);
          colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
        }
        break;
      }
    }
  }
}
