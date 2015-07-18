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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

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
LinkData::LinkData()
{
  this->linkSDF.reset(new sdf::Element);
  sdf::initFile("link.sdf", this->linkSDF);

  this->scale = math::Vector3::One;

  this->inspector = new LinkInspector();
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));
  connect(this->inspector, SIGNAL(Accepted()), this, SLOT(OnAccept()));
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
      boost::bind(&LinkData::Update, this)));
  this->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
LinkData::~LinkData()
{
  event::Events::DisconnectPreRender(this->connections[0]);
  delete this->inspector;
}

/////////////////////////////////////////////////
std::string LinkData::GetName() const
{
  return this->linkSDF->Get<std::string>("name");
}

/////////////////////////////////////////////////
void LinkData::SetName(const std::string &_name)
{
  this->linkSDF->GetAttribute("name")->Set(_name);
  this->inspector->SetName(_name);
}

/////////////////////////////////////////////////
math::Pose LinkData::GetPose() const
{
  return this->linkSDF->Get<math::Pose>("pose");
}

/////////////////////////////////////////////////
void LinkData::SetPose(const math::Pose &_pose)
{
  this->linkSDF->GetElement("pose")->Set(_pose);

  LinkConfig *linkConfig = this->inspector->GetLinkConfig();
  linkConfig->SetPose(_pose);
}

/////////////////////////////////////////////////
void LinkData::SetScale(const math::Vector3 &_scale)
{
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();
  for (auto it = this->visuals.begin(); it != this->visuals.end(); ++it)
  {
    std::string name = it->first->GetName();
    std::string linkName = this->linkVisual->GetName();
    std::string leafName =
        name.substr(name.find(linkName)+linkName.size()+2);
    visualConfig->SetGeometry(leafName, it->first->GetScale());
  }

  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  for (auto it = this->collisions.begin(); it != this->collisions.end(); ++it)
  {
    std::string name = it->first->GetName();
    std::string linkName = this->linkVisual->GetName();
    std::string leafName =
        name.substr(name.find(linkName)+linkName.size()+2);
    collisionConfig->SetGeometry(leafName,  it->first->GetScale());
  }

  if (this->scale == _scale)
    return;

  // update link inertial values
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();
  sdf::ElementPtr inertialElem = this->linkSDF->GetElement("inertial");
  double volScale = _scale.GetLength() / this->scale.GetLength();

  sdf::ElementPtr massElem = inertialElem->GetElement("mass");
  double newMass = massElem->Get<double>() * volScale;
  massElem->Set(newMass);
  linkConfig->SetMass(newMass);

  // sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");

  sdf::ElementPtr inertialPoseElem = inertialElem->GetElement("pose");
  math::Pose newPose = inertialPoseElem->Get<math::Pose>();
  newPose.pos *= volScale;

  inertialPoseElem->Set(newPose);
  linkConfig->SetInertialPose(newPose);

  this->scale = _scale;
}

/////////////////////////////////////////////////
math::Vector3 LinkData::GetScale() const
{
  return this->scale;
}

/////////////////////////////////////////////////
void LinkData::Load(sdf::ElementPtr _sdf)
{
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();

  this->SetName(_sdf->Get<std::string>("name"));
  this->SetPose(_sdf->Get<math::Pose>("pose"));

  msgs::LinkPtr linkMsgPtr(new msgs::Link);
  if (_sdf->HasElement("inertial"))
  {
    sdf::ElementPtr inertialElem = _sdf->GetElement("inertial");
    this->linkSDF->GetElement("inertial")->Copy(inertialElem);

    msgs::Inertial *inertialMsg = linkMsgPtr->mutable_inertial();

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
  }
  if (_sdf->HasElement("self_collide"))
  {
    sdf::ElementPtr selfCollideSDF = _sdf->GetElement("self_collide");
    linkMsgPtr->set_self_collide(selfCollideSDF->Get<bool>(""));
    this->linkSDF->InsertElement(selfCollideSDF->Clone());
  }
  if (_sdf->HasElement("kinematic"))
  {
    sdf::ElementPtr kinematicSDF = _sdf->GetElement("kinematic");
    linkMsgPtr->set_kinematic(kinematicSDF->Get<bool>());
    this->linkSDF->InsertElement(kinematicSDF->Clone());
  }
  if (_sdf->HasElement("must_be_base_link"))
  {
    sdf::ElementPtr baseLinkSDF = _sdf->GetElement("must_be_base_link");
    // TODO link.proto is missing the must_be_base_link field.
    // linkMsgPtr->set_must_be_base_link(baseLinkSDF->Get<bool>());
    this->linkSDF->InsertElement(baseLinkSDF->Clone());
  }
  if (_sdf->HasElement("velocity_decay"))
  {
    sdf::ElementPtr velocityDecaySDF = _sdf->GetElement("velocity_decay");
    // TODO link.proto is missing the velocity_decay field.
    // linkMsgPtr->set_velocity_decay(velocityDecaySDF->Get<double>());
    this->linkSDF->InsertElement(velocityDecaySDF->Clone());
  }
  linkConfig->Update(linkMsgPtr);

  if (_sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = _sdf->GetElement("sensor");
    while (sensorElem)
    {
      this->linkSDF->InsertElement(sensorElem->Clone());
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }
}

/////////////////////////////////////////////////
void LinkData::UpdateConfig()
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
void LinkData::AddVisual(rendering::VisualPtr _visual)
{
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_visual->GetSDF());

  this->visuals[_visual] = visualMsg;

  std::string visName = _visual->GetName();
  std::string leafName = visName;
  size_t idx = visName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = visName.substr(idx+1);

  visualConfig->AddVisual(leafName, &visualMsg);
}

/////////////////////////////////////////////////
void LinkData::AddCollision(rendering::VisualPtr _collisionVis)
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
LinkData* LinkData::Clone(const std::string &_newName)
{
  LinkData *cloneLink = new LinkData();

  cloneLink->Load(this->linkSDF);
  cloneLink->SetName(_newName);

  std::string linkVisualName = this->linkVisual->GetName();
  std::string cloneVisName = _newName;
  size_t linkIdx = linkVisualName.find("::");
  if (linkIdx != std::string::npos)
    cloneVisName = linkVisualName.substr(0, linkIdx+2) + _newName;

  // clone linkVisual;
  rendering::VisualPtr linkVis(new rendering::Visual(cloneVisName,
      this->linkVisual->GetParent()));
  linkVis->Load();

  cloneLink->linkVisual = linkVis;

  for (auto &visIt : this->visuals)
  {
    std::string newVisName = visIt.first->GetName();
    size_t idx = newVisName.find_last_of("::");
    if (idx != std::string::npos)
      newVisName = cloneVisName + newVisName.substr(idx-1);
    else
      newVisName = cloneVisName + "::" + newVisName;

    rendering::VisualPtr cloneVis =
        visIt.first->Clone(newVisName, cloneLink->linkVisual);

    // override transparency
    cloneVis->SetTransparency(visIt.second.transparency());
    cloneLink->AddVisual(cloneVis);
    cloneVis->SetTransparency(visIt.second.transparency() *
        (1-ModelData::GetEditTransparency()-0.1)
        + ModelData::GetEditTransparency());
  }

  for (auto &colIt : this->collisions)
  {
    std::string newColName = colIt.first->GetName();
    size_t idx = newColName.find_last_of("::");
    if (idx != std::string::npos)
      newColName = cloneVisName + newColName.substr(idx-1);
    else
      newColName = cloneVisName + "::" + newColName;
    rendering::VisualPtr collisionVis = colIt.first->Clone(newColName,
        cloneLink->linkVisual);
    collisionVis->SetTransparency(
       math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
    // fix for transparency alpha compositing
    Ogre::MovableObject *colObj = collisionVis->GetSceneNode()->
        getAttachedObject(0);
    colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
    cloneLink->AddCollision(collisionVis);
  }
  return cloneLink;
}

/////////////////////////////////////////////////
void LinkData::OnAccept()
{
  if (this->Apply())
    this->inspector->accept();
}

/////////////////////////////////////////////////
void LinkData::OnApply()
{
  this->Apply();
}

/////////////////////////////////////////////////
bool LinkData::Apply()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();

  this->linkSDF = msgs::LinkToSDF(*linkConfig->GetData(), this->linkSDF);
  this->linkVisual->SetPose(this->GetPose());

  std::vector<msgs::Visual *> visualUpdateMsgsTemp;
  std::vector<msgs::Collision *> collisionUpdateMsgsTemp;

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

        // check if the geometry is valid
        msgs::Geometry *geomMsg = updateMsg->mutable_geometry();

        // warnings when changing from/to polyline
        for (auto &vis : this->visuals)
        {
          if (vis.second.name() != updateMsg->name())
            continue;

          if (vis.second.mutable_geometry()->type() != geomMsg->type())
          {
            // Changing from polyline, give option to cancel
            if (vis.second.mutable_geometry()->type() ==
                msgs::Geometry::POLYLINE)
            {
              std::string msg =
                  "Once you change the geometry, you can't go "
                  "back to polyline.\n\n"
                  "Do you wish to continue?\n";

              QMessageBox msgBox(QMessageBox::Warning,
                  QString("Changing polyline geometry"), QString(msg.c_str()));

              QPushButton *cancelButton =
                  msgBox.addButton("Cancel", QMessageBox::RejectRole);
              QPushButton *saveButton = msgBox.addButton("Ok",
                  QMessageBox::AcceptRole);
              msgBox.setDefaultButton(saveButton);
              msgBox.setEscapeButton(cancelButton);
              msgBox.exec();
              if (msgBox.clickedButton() != saveButton)
                return false;
            }
            // Changing to polyline: not allowed
            else if (geomMsg->type() == msgs::Geometry::POLYLINE)
            {
              std::string msg =
                  "It's not possible to change into polyline.\n"
                  "Please select another geometry type for ["
                  + leafName + "].";

              QMessageBox::warning(linkConfig, QString(
                  "Invalid geometry conversion"), QString(msg.c_str()),
                  QMessageBox::Ok, QMessageBox::Ok);
              return false;
            }
          }
        }

        if (geomMsg->type() == msgs::Geometry::MESH)
        {
          msgs::MeshGeom *meshGeom = geomMsg->mutable_mesh();
          QFileInfo info(QString::fromStdString(meshGeom->filename()));
          if (!info.isFile() || (info.completeSuffix().toLower() != "dae" &&
              info.completeSuffix().toLower() != "stl"))
          {
            std::string msg = "\"" + meshGeom->filename() +
                "\" is not a valid mesh file.\nPlease select another file for ["
                + leafName + "].";

            QMessageBox::warning(linkConfig, QString("Invalid Mesh File"),
                QString(msg.c_str()), QMessageBox::Ok, QMessageBox::Ok);
            return false;
          }
        }

        // update the visualMsg that will be used to generate the sdf.
        updateMsg->clear_scale();
        msgs::Material *matMsg = updateMsg->mutable_material();
        msgs::Material::Script *scriptMsg = matMsg->mutable_script();

        common::Color emptyColor;
        common::Color matAmbient;
        common::Color matDiffuse;
        common::Color matSpecular;
        common::Color matEmissive;
        rendering::Material::GetMaterialAsColor(scriptMsg->name(), matAmbient,
            matDiffuse, matSpecular, matEmissive);

        common::Color ambient = msgs::Convert(matMsg->ambient());
        common::Color diffuse = msgs::Convert(matMsg->diffuse());
        common::Color specular = msgs::Convert(matMsg->specular());
        common::Color emissive = msgs::Convert(matMsg->emissive());

        if (ambient == emptyColor)
        {
          matMsg->clear_ambient();
          ambient = matAmbient;
        }
        if (diffuse == emptyColor)
        {
          matMsg->clear_diffuse();
          diffuse = matDiffuse;
        }
        if (specular == emptyColor)
        {
          matMsg->clear_specular();
          specular = matSpecular;
        }
        if (emissive == emptyColor)
        {
          matMsg->clear_emissive();
          emissive = matEmissive;
        }

        visualConfig->SetMaterial(leafName, scriptMsg->name(), ambient,
            diffuse, specular, emissive);

        visualMsg.CopyFrom(*updateMsg);
        it.second = visualMsg;

        visualUpdateMsgsTemp.push_back(updateMsg);
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

        // check if the geometry is valid
        msgs::Geometry *geomMsg = updateMsg->mutable_geometry();

        // warnings when changing from/to polyline
        for (auto &col : this->collisions)
        {
          if (col.second.name() != updateMsg->name())
            continue;

          if (col.second.mutable_geometry()->type() != geomMsg->type())
          {
            // Changing from polyline, give option to cancel
            if (col.second.mutable_geometry()->type() ==
                msgs::Geometry::POLYLINE)
            {
              std::string msg =
                  "Once you change the geometry, you can't go "
                  "back to polyline.\n\n"
                  "Do you wish to continue?\n";

              QMessageBox msgBox(QMessageBox::Warning,
                  QString("Changing polyline geometry"), QString(msg.c_str()));

              QPushButton *cancelButton =
                  msgBox.addButton("Cancel", QMessageBox::RejectRole);
              QPushButton *saveButton = msgBox.addButton("Ok",
                  QMessageBox::AcceptRole);
              msgBox.setDefaultButton(saveButton);
              msgBox.setEscapeButton(cancelButton);
              msgBox.exec();
              if (msgBox.clickedButton() != saveButton)
                return false;
            }
            // Changing to polyline: not allowed
            else if (geomMsg->type() == msgs::Geometry::POLYLINE)
            {
              std::string msg =
                  "It's not possible to change into polyline.\n"
                  "Please select another geometry type for ["
                  + leafName + "].";

              QMessageBox::warning(linkConfig, QString(
                  "Invalid geometry conversion"), QString(msg.c_str()),
                  QMessageBox::Ok, QMessageBox::Ok);
              return false;
            }
          }
        }

        if (geomMsg->type() == msgs::Geometry::MESH)
        {
          msgs::MeshGeom *meshGeom = geomMsg->mutable_mesh();
          QFileInfo info(QString::fromStdString(meshGeom->filename()));
          if (!info.isFile() || (info.completeSuffix().toLower() != "dae" &&
              info.completeSuffix().toLower() != "stl"))
          {
            std::string msg = "\"" + meshGeom->filename() +
                "\" is not a valid mesh file.\nPlease select another file for ["
                + leafName + "].";

            QMessageBox::warning(linkConfig, QString("Invalid Mesh File"),
                QString(msg.c_str()), QMessageBox::Ok, QMessageBox::Ok);
            return false;
          }
        }

        collisionMsg.CopyFrom(*updateMsg);
        it.second = collisionMsg;

        collisionUpdateMsgsTemp.push_back(updateMsg);
      }
    }
  }

  // Only send update messages if all visuals and collisions are valid
  this->visualUpdateMsgs.insert(this->visualUpdateMsgs.end(),
      visualUpdateMsgsTemp.begin(), visualUpdateMsgsTemp.end());
  this->collisionUpdateMsgs.insert(this->collisionUpdateMsgs.end(),
      collisionUpdateMsgsTemp.begin(), collisionUpdateMsgsTemp.end());
  return true;
}

/////////////////////////////////////////////////
void LinkData::OnAddVisual(const std::string &_name)
{
  // add a visual when the user adds a visual via the inspector's visual tab
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();

  std::ostringstream visualName;
  visualName << this->linkVisual->GetName() << "::" << _name;

  rendering::VisualPtr visVisual;
  rendering::VisualPtr refVisual;
  if (!this->visuals.empty())
  {
    // add new visual by cloning last instance
    refVisual = this->visuals.rbegin()->first;
    visVisual = refVisual->Clone(visualName.str(), this->linkVisual);
  }
  else
  {
    // create new visual based on sdf template (box)
    sdf::SDFPtr modelTemplateSDF(new sdf::SDF);
    modelTemplateSDF->SetFromString(
        ModelData::GetTemplateSDFString());

    visVisual.reset(new rendering::Visual(visualName.str(),
        this->linkVisual));
    sdf::ElementPtr visualElem =  modelTemplateSDF->Root()
        ->GetElement("model")->GetElement("link")->GetElement("visual");
    visVisual->Load(visualElem);
  }

  msgs::Visual visualMsg = msgs::VisualFromSDF(visVisual->GetSDF());
  // store the correct transparency setting
  if (refVisual)
    visualMsg.set_transparency(this->visuals[refVisual].transparency());

  msgs::VisualPtr visualMsgPtr(new msgs::Visual);
  visualMsgPtr->CopyFrom(visualMsg);
  visualConfig->UpdateVisual(_name, visualMsgPtr);
  this->visuals[visVisual] = visualMsg;
  visVisual->SetTransparency(visualMsg.transparency() *
      (1-ModelData::GetEditTransparency()-0.1)
      + ModelData::GetEditTransparency());
}

/////////////////////////////////////////////////
void LinkData::OnAddCollision(const std::string &_name)
{
  // add a collision when the user adds a collision via the inspector's
  // collision tab
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();

  std::stringstream collisionName;
  collisionName << this->linkVisual->GetName() << "::" << _name;

  rendering::VisualPtr collisionVis;
  if (!this->collisions.empty())
  {
    // add new collision by cloning last instance
    collisionVis = this->collisions.rbegin()->first->Clone(collisionName.str(),
        this->linkVisual);
  }
  else
  {
    // create new collision based on sdf template (box)
    sdf::SDFPtr modelTemplateSDF(new sdf::SDF);
    modelTemplateSDF->SetFromString(
        ModelData::GetTemplateSDFString());

    collisionVis.reset(new rendering::Visual(collisionName.str(),
        this->linkVisual));
    sdf::ElementPtr collisionElem =  modelTemplateSDF->Root()
        ->GetElement("model")->GetElement("link")->GetElement("visual");
    collisionVis->Load(collisionElem);
    collisionVis->SetMaterial("Gazebo/Orange");
  }

  msgs::Visual visualMsg = msgs::VisualFromSDF(collisionVis->GetSDF());
  msgs::Collision collisionMsg;
  collisionMsg.set_name(_name);
  msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
  geomMsg->CopyFrom(visualMsg.geometry());

  msgs::CollisionPtr collisionMsgPtr(new msgs::Collision);
  collisionMsgPtr->CopyFrom(collisionMsg);
  collisionConfig->UpdateCollision(_name, collisionMsgPtr);
  this->collisions[collisionVis] = collisionMsg;

  collisionVis->SetTransparency(
      math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));

  // fix for transparency alpha compositing
  Ogre::MovableObject *colObj = collisionVis->GetSceneNode()->
      getAttachedObject(0);
  colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
}

/////////////////////////////////////////////////
void LinkData::OnRemoveVisual(const std::string &_name)
{
  // find and remove visual when the user removes it in the
  // inspector's visual tab
  std::ostringstream name;
  name << this->linkVisual->GetName() << "::" << _name;
  std::string visualName = name.str();

  for (auto it = this->visuals.begin(); it != this->visuals.end(); ++it)
  {
    if (visualName == it->first->GetName())
    {
      this->linkVisual->DetachVisual(it->first);
      this->linkVisual->GetScene()->RemoveVisual(it->first);
      this->visuals.erase(it);
      break;
    }
  }
}

/////////////////////////////////////////////////
void LinkData::OnRemoveCollision(const std::string &_name)
{
  // find and remove collision visual when the user removes it in the
  // inspector's collision tab
  std::ostringstream name;
  name << this->linkVisual->GetName() << "::" << _name;
  std::string collisionName = name.str();

  for (auto it = this->collisions.begin(); it != this->collisions.end(); ++it)
  {
    if (collisionName == it->first->GetName())
    {
      this->linkVisual->DetachVisual(it->first);
      this->linkVisual->GetScene()->RemoveVisual(it->first);
      this->collisions.erase(it);
      break;
    }
  }
}

/////////////////////////////////////////////////
void LinkData::Update()
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
