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

#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/gui/model/LinkInspector.hh"
#include "gazebo/gui/model/VisualConfig.hh"
#include "gazebo/gui/model/LinkConfig.hh"
#include "gazebo/gui/model/CollisionConfig.hh"

#include "gazebo/gui/GuiIface.hh"
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
double ModelData::GetEditTransparency()
{
  return 0.4;
}

/////////////////////////////////////////////////
PartData::PartData()
{
  this->partSDF.reset(new sdf::Element);
  sdf::initFile("link.sdf", this->partSDF);

  this->scale = math::Vector3::One;

  this->inspector = new LinkInspector;
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

  // note the destructor removes this connection with the assumption that it is
  // the first one in the vector
  this->connections.push_back(event::Events::ConnectPreRender(
      boost::bind(&PartData::Update, this)));
  this->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
PartData::~PartData()
{
  event::Events::DisconnectPreRender(this->connections[0]);
  delete this->inspector;
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

  LinkConfig *linkConfig = this->inspector->GetLinkConfig();
  linkConfig->SetPose(_pose);
}

/////////////////////////////////////////////////
void PartData::SetScale(const math::Vector3 &_scale)
{
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();
  for (auto it = this->visuals.begin(); it != this->visuals.end(); ++it)
  {
    std::string name = it->first->GetName();
    std::string partName = this->partVisual->GetName();
    std::string leafName =
        name.substr(name.find(partName)+partName.size()+2);
    visualConfig->SetGeometry(leafName, it->first->GetScale());
  }

  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  for (auto it = this->collisions.begin(); it != this->collisions.end(); ++it)
  {
    std::string name = it->first->GetName();
    std::string partName = this->partVisual->GetName();
    std::string leafName =
        name.substr(name.find(partName)+partName.size()+2);
    collisionConfig->SetGeometry(leafName,  it->first->GetScale());
  }

  this->scale = _scale;
}

/////////////////////////////////////////////////
math::Vector3 PartData::GetScale() const
{
  return this->scale;
}

/////////////////////////////////////////////////
void PartData::Load(sdf::ElementPtr _sdf)
{
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();

  this->SetName(_sdf->Get<std::string>("name"));
  this->SetPose(_sdf->Get<math::Pose>("pose"));

  if (_sdf->HasElement("inertial"))
  {
    sdf::ElementPtr inertialElem = _sdf->GetElement("inertial");
    this->partSDF->GetElement("inertial")->Copy(inertialElem);

    msgs::Link linkMsg;
    msgs::Inertial *inertialMsg = linkMsg.mutable_inertial();

    if (inertialElem->HasElement("mass"))
    {
      double mass = inertialElem->Get<double>("mass");
      inertialMsg->set_mass(mass);
    }

    if (inertialElem->HasElement("pose"))
    {
      math::Pose inertialPose = inertialElem->Get<math::Pose>("pose");
      msgs::Set(inertialMsg->mutable_pose(), inertialPose);
    }

    if (inertialElem->HasElement("inertia"))
    {
      sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
      inertialMsg->set_ixx(inertiaElem->Get<double>("ixx"));
      inertialMsg->set_ixy(inertiaElem->Get<double>("ixy"));
      inertialMsg->set_ixz(inertiaElem->Get<double>("ixz"));
      inertialMsg->set_iyy(inertiaElem->Get<double>("iyy"));
      inertialMsg->set_iyz(inertiaElem->Get<double>("iyz"));
      inertialMsg->set_izz(inertiaElem->Get<double>("izz"));
    }
    linkConfig->Update(ConstLinkPtr(&linkMsg));
  }

  if (_sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = _sdf->GetElement("sensor");
    while (sensorElem)
    {
      this->partSDF->InsertElement(sensorElem->Clone());
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }
}

/////////////////////////////////////////////////
void PartData::UpdateConfig()
{
  // set new geom size if scale has changed.
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();
  for (auto &it : this->visuals)
  {
    std::string name = it.first->GetName();
    std::string leafName = name;
    size_t idx = name.find_last_of("::");
    if (idx != std::string::npos)
      leafName = name.substr(idx+1);
    visualConfig->SetGeometry(leafName, it.first->GetScale(),
        it.first->GetMeshName());

    msgs::Visual *updateMsg = visualConfig->GetData(leafName);
    msgs::Visual visualMsg = it.second;
    updateMsg->clear_scale();
    msgs::Material *matMsg = updateMsg->mutable_material();
    // clear empty colors so they are not used by visual updates
    common::Color emptyColor;
    if (msgs::Convert(matMsg->ambient()) == emptyColor)
      matMsg->clear_ambient();
    if (msgs::Convert(matMsg->diffuse()) == emptyColor)
      matMsg->clear_diffuse();
    if (msgs::Convert(matMsg->specular()) == emptyColor)
      matMsg->clear_specular();
    if (msgs::Convert(matMsg->emissive()) == emptyColor)
      matMsg->clear_emissive();

    if (matMsg->has_diffuse())
      matMsg->mutable_diffuse()->set_a(1.0-updateMsg->transparency());

    visualMsg.CopyFrom(*updateMsg);
    it.second = visualMsg;
  }
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  for (auto &colIt : this->collisions)
  {
    std::string name = colIt.first->GetName();
    std::string leafName = name;
    size_t idx = name.find_last_of("::");
    if (idx != std::string::npos)
      leafName = name.substr(idx+1);
    collisionConfig->SetGeometry(leafName, colIt.first->GetScale(),
        colIt.first->GetMeshName());

    msgs::Collision *updateMsg = collisionConfig->GetData(leafName);
    msgs::Collision collisionMsg = colIt.second;
    collisionMsg.CopyFrom(*updateMsg);
    colIt.second = collisionMsg;
  }
}

/////////////////////////////////////////////////
void PartData::AddVisual(rendering::VisualPtr _visual)
{
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_visual->GetSDF());

  // override transparency value
  visualMsg.set_transparency(0.0);
  this->visuals[_visual] = visualMsg;

  std::string visName = _visual->GetName();
  std::string leafName = visName;
  size_t idx = visName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = visName.substr(idx+1);

  visualConfig->AddVisual(leafName, &visualMsg);
}

/////////////////////////////////////////////////
void PartData::AddCollision(rendering::VisualPtr _collisionVis)
{
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_collisionVis->GetSDF());

  sdf::ElementPtr collisionSDF(new sdf::Element);
  sdf::initFile("collision.sdf", collisionSDF);

  std::string visName = _collisionVis->GetName();
  std::string leafName = visName;
  size_t idx = visName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = visName.substr(idx+1);

  msgs::Collision collisionMsg;
  collisionMsg.set_name(leafName);
  msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
  geomMsg->CopyFrom(visualMsg.geometry());
  msgs::Pose *poseMsg = collisionMsg.mutable_pose();
  poseMsg->CopyFrom(visualMsg.pose());

  this->collisions[_collisionVis] = collisionMsg;
  collisionConfig->AddCollision(leafName, &collisionMsg);
}

/////////////////////////////////////////////////
PartData* PartData::Clone(const std::string &_newName)
{
  PartData *clonePart = new PartData();

  clonePart->Load(this->partSDF);
  clonePart->SetName(_newName);

  std::string partVisualName = this->partVisual->GetName();
  std::string cloneVisName = _newName;
  size_t partIdx = partVisualName.find("::");
  if (partIdx != std::string::npos)
    cloneVisName = partVisualName.substr(0, partIdx+2) + _newName;

  // clone partVisual;
  rendering::VisualPtr linkVisual(new rendering::Visual(cloneVisName,
      this->partVisual->GetParent()));
  linkVisual->Load();

  clonePart->partVisual = linkVisual;

  std::map<rendering::VisualPtr, msgs::Visual>::iterator visIt;
  for (visIt = this->visuals.begin(); visIt != this->visuals.end(); ++visIt)
  {
    std::string newVisName = visIt->first->GetName();
    size_t idx = newVisName.find_last_of("::");
    if (idx != std::string::npos)
      newVisName = cloneVisName + newVisName.substr(idx-1);
    else
      newVisName = cloneVisName + "::" + newVisName;
    clonePart->AddVisual(visIt->first->Clone(newVisName,
        clonePart->partVisual));
  }

  linkVisual->SetTransparency(ModelData::GetEditTransparency());
  std::map<rendering::VisualPtr, msgs::Collision>::iterator colIt;
  for (colIt = this->collisions.begin(); colIt != this->collisions.end();
      ++colIt)
  {
    std::string newColName = colIt->first->GetName();
    size_t idx = newColName.find_last_of("::");
    if (idx != std::string::npos)
      newColName = cloneVisName + newColName.substr(idx-1);
    else
      newColName = cloneVisName + "::" + newColName;
    rendering::VisualPtr collisionVis = colIt->first->Clone(newColName,
        clonePart->partVisual);
    collisionVis->SetTransparency(
       math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
    // fix for transparency alpha compositing
    Ogre::MovableObject *colObj = collisionVis->GetSceneNode()->
        getAttachedObject(0);
    colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
    clonePart->AddCollision(collisionVis);
  }
  return clonePart;
}

/////////////////////////////////////////////////
void PartData::OnApply()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();

  this->partSDF = msgs::LinkToSDF(*linkConfig->GetData(), this->partSDF);
  this->partVisual->SetWorldPose(this->GetPose());

  // update visuals
  if (!this->visuals.empty())
  {
    VisualConfig *visualConfig = this->inspector->GetVisualConfig();
    for (auto &it : this->visuals)
    {
      std::string name = it.first->GetName();
      std::string leafName = name;
      size_t idx = name.find_last_of("::");
      if (idx != std::string::npos)
        leafName = name.substr(idx+1);
      msgs::Visual *updateMsg = visualConfig->GetData(leafName);
      if (updateMsg)
      {
        msgs::Visual visualMsg = it.second;

        // update the visualMsg that will be used to generate the sdf.
        updateMsg->clear_scale();
        msgs::Material *matMsg = updateMsg->mutable_material();
        msgs::Material::Script *scriptMsg = matMsg->mutable_script();

        common::Color emptyColor;
        bool matScriptChanged = false;
        bool colorChanged = false;
        common::Color ambient;
        common::Color diffuse;
        common::Color specular;
        common::Color emissive;

        std::string matName = it.first->GetMaterialName();
        std::string uniqueMatName = name + "_MATERIAL_";
        size_t visMatIdx = matName.find(uniqueMatName);
        if (visMatIdx != std::string::npos)
          matName = matName.substr(visMatIdx + uniqueMatName.size());

        if (matName != scriptMsg->name() && !scriptMsg->name().empty())
        {
          rendering::Material::GetMaterialAsColor(scriptMsg->name(), ambient,
              diffuse, specular, emissive);
          visualConfig->SetMaterial(leafName, scriptMsg->name(), ambient,
              diffuse, specular, emissive);

          matScriptChanged = true;
        }
        else
        {
          ambient = msgs::Convert(matMsg->ambient());
          diffuse = msgs::Convert(matMsg->diffuse());
          specular = msgs::Convert(matMsg->specular());
          emissive = msgs::Convert(matMsg->emissive());
          if (ambient != it.first->GetAmbient()
              || diffuse != it.first->GetDiffuse()
              || specular != it.first->GetSpecular()
              || emissive != it.first->GetEmissive())
          {
            colorChanged = true;
          }

          if (colorChanged)
            scriptMsg->clear_name();
        }

        // update material or color, but not both
        // clear empty colors so they are not used by visual updates
        if (matScriptChanged || !colorChanged ||
            msgs::Convert(matMsg->ambient()) == emptyColor)
          matMsg->clear_ambient();
        if (matScriptChanged || !colorChanged ||
            msgs::Convert(matMsg->diffuse()) == emptyColor)
          matMsg->clear_diffuse();
        if (matScriptChanged || !colorChanged ||
            msgs::Convert(matMsg->specular()) == emptyColor)
          matMsg->clear_specular();
        if (matScriptChanged || !colorChanged ||
            msgs::Convert(matMsg->emissive()) == emptyColor)
          matMsg->clear_emissive();

        if (matMsg->has_diffuse())
          matMsg->mutable_diffuse()->set_a(1.0-updateMsg->transparency());

        visualMsg.CopyFrom(*updateMsg);
        it.second = visualMsg;

        this->visualUpdateMsgs.push_back(updateMsg);
      }
    }
  }

  // update collisions
  if (!this->collisions.empty())
  {
    CollisionConfig *collisionConfig =
        this->inspector->GetCollisionConfig();
    for (auto &it : this->collisions)
    {
      std::string name = it.first->GetName();
      std::string leafName = name;
      size_t idx = name.find_last_of("::");
      if (idx != std::string::npos)
        leafName = name.substr(idx+1);
      msgs::Collision *updateMsg = collisionConfig->GetData(leafName);
      if (updateMsg)
      {
        msgs::Collision collisionMsg = it.second;
        collisionMsg.CopyFrom(*updateMsg);
        it.second = collisionMsg;

        this->collisionUpdateMsgs.push_back(updateMsg);
      }
    }
  }
}

/////////////////////////////////////////////////
void PartData::OnAddVisual(const std::string &_name)
{
  // add a visual when the user adds a visual via the inspector's visual tab
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();

  std::ostringstream visualName;
  visualName << this->partVisual->GetName() << "::" << _name;

  rendering::VisualPtr visVisual;
  rendering::VisualPtr refVisual;
  if (!this->visuals.empty())
  {
    // add new visual by cloning last instance
    refVisual = this->visuals.rbegin()->first;
    visVisual = refVisual->Clone(visualName.str(), this->partVisual);
  }
  else
  {
    // create new visual based on sdf template (box)
    sdf::SDFPtr modelTemplateSDF(new sdf::SDF);
    modelTemplateSDF->SetFromString(
        ModelData::GetTemplateSDFString());

    visVisual.reset(new rendering::Visual(visualName.str(),
        this->partVisual));
    sdf::ElementPtr visualElem =  modelTemplateSDF->root
        ->GetElement("model")->GetElement("link")->GetElement("visual");
    visVisual->Load(visualElem);
  }

  msgs::Visual visualMsg = msgs::VisualFromSDF(visVisual->GetSDF());

  // store the correct transparency setting
  if (refVisual)
    visualMsg.set_transparency(this->visuals[refVisual].transparency());
  visualConfig->UpdateVisual(_name, ConstVisualPtr(&visualMsg));
  this->visuals[visVisual] = visualMsg;
  visVisual->SetTransparency(visualMsg.transparency() *
      (1-ModelData::GetEditTransparency()-0.1)
      + ModelData::GetEditTransparency());
}

/////////////////////////////////////////////////
void PartData::OnAddCollision(const std::string &_name)
{
  // add a collision when the user adds a collision via the inspector's
  // collision tab
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();

  std::stringstream collisionName;
  collisionName << this->partVisual->GetName() << "::" << _name;

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
    // orange
    common::Color ambient;
    common::Color diffuse;
    common::Color specular;
    common::Color emissive;
    rendering::Material::GetMaterialAsColor("Gazebo/Orange", ambient, diffuse,
        specular, emissive);
    collisionVis->SetAmbient(ambient);
    collisionVis->SetDiffuse(diffuse);
    collisionVis->SetSpecular(specular);
    collisionVis->SetEmissive(emissive);
    this->partVisual->GetScene()->AddVisual(collisionVis);
  }

  msgs::Visual visualMsg = msgs::VisualFromSDF(collisionVis->GetSDF());
  msgs::Collision collisionMsg;
  collisionMsg.set_name(_name);
  msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
  geomMsg->CopyFrom(visualMsg.geometry());

  collisionConfig->UpdateCollision(_name, ConstCollisionPtr(&collisionMsg));
  this->collisions[collisionVis] = collisionMsg;

  collisionVis->SetTransparency(
      math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));

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
  name << this->partVisual->GetName() << "::" << _name;
  std::string visualName = name.str();

  for (auto it = this->visuals.begin(); it != this->visuals.end(); ++it)
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
  name << this->partVisual->GetName() << "::" << _name;
  std::string collisionName = name.str();

  for (auto it = this->collisions.begin(); it != this->collisions.end(); ++it)
  {
    if (collisionName == it->first->GetName())
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
    for (auto &it : this->visuals)
    {
      if (it.second.name() == updateMsgPtr->name())
      {
        // make visual semi-transparent here
        // but generated sdf will use the correct transparency value
        it.first->UpdateFromMsg(updateMsgPtr);
        it.first->SetTransparency(updateMsgPtr->transparency() *
            (1-ModelData::GetEditTransparency()-0.1)
            + ModelData::GetEditTransparency());
        break;
      }
    }
  }

  while (!this->collisionUpdateMsgs.empty())
  {
    msgs::Collision collisionMsg = *this->collisionUpdateMsgs.front();
    this->collisionUpdateMsgs.erase(this->collisionUpdateMsgs.begin());
    for (auto &it : this->collisions)
    {
      if (it.second.name() == collisionMsg.name())
      {
        msgs::Visual collisionVisMsg;
        msgs::Geometry *geomMsg = collisionVisMsg.mutable_geometry();
        geomMsg->CopyFrom(collisionMsg.geometry());
        msgs::Pose *poseMsg = collisionVisMsg.mutable_pose();
        poseMsg->CopyFrom(collisionMsg.pose());

        boost::shared_ptr<gazebo::msgs::Visual> updateMsgPtr;
        updateMsgPtr.reset(new msgs::Visual);
        updateMsgPtr->CopyFrom(collisionVisMsg);
        std::string origGeomType = it.first->GetGeometryType();
        it.first->UpdateFromMsg(updateMsgPtr);

        // fix for transparency alpha compositing
        if (it.first->GetGeometryType() != origGeomType)
        {
          Ogre::MovableObject *colObj = it.first->GetSceneNode()->
              getAttachedObject(0);
          colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
        }
        break;
      }
    }
  }
}
