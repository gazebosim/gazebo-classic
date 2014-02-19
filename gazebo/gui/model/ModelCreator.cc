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

#include <sstream>
#include <boost/filesystem.hpp>

#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/math/Quaternion.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/physics/Inertial.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelManipulator.hh"

#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelCreatorPrivate.hh"
#include "gazebo/gui/model/ModelCreator.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelCreator::ModelCreator()
  : dataPtr(new ModelCreatorPrivate)
{
  this->dataPtr->modelName = "";

  this->dataPtr->modelTemplateSDF.reset(new sdf::SDF);
  this->dataPtr->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());

  this->dataPtr->boxCounter = 0;
  this->dataPtr->cylinderCounter = 0;
  this->dataPtr->sphereCounter = 0;
  this->dataPtr->modelCounter = 0;
  this->dataPtr->customCounter = 0;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->makerPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");
  this->dataPtr->requestPub =
      this->dataPtr->node->Advertise<msgs::Request>("~/request");

  this->dataPtr->jointMaker = new JointMaker();

  connect(g_deleteAct, SIGNAL(DeleteSignal(const std::string &)), this,
          SLOT(OnDelete(const std::string &)));

  this->Reset();
}

/////////////////////////////////////////////////
ModelCreator::~ModelCreator()
{
  this->Reset();
  this->dataPtr->node->Fini();
  this->dataPtr->node.reset();
  this->dataPtr->modelTemplateSDF.reset();
  this->dataPtr->requestPub.reset();
  this->dataPtr->makerPub.reset();

  delete this->dataPtr->jointMaker;
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
std::string ModelCreator::CreateModel()
{
  this->Reset();
  return this->dataPtr->modelName;
}

/////////////////////////////////////////////////
void ModelCreator::AddJoint(JointMaker::JointType _type)
{
  this->Stop();
  if (this->dataPtr->jointMaker)
    this->dataPtr->jointMaker->CreateJoint(_type);
}

/////////////////////////////////////////////////
std::string ModelCreator::AddBox(const math::Vector3 &_size,
    const math::Pose &_pose)
{
  if (!this->dataPtr->modelVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "unit_box_" << this->dataPtr->boxCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(linkName,
      this->dataPtr->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
      linkVisual));
  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/GreyTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  ((geomElem->AddElement("box"))->AddElement("size"))->Set(_size);

  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _size.z/2));
  }

  this->CreatePart(visVisual);

  this->dataPtr->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddSphere(double _radius,
    const math::Pose &_pose)
{
  if (!this->dataPtr->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "unit_sphere_" << this->dataPtr->sphereCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      linkName, this->dataPtr->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/GreyTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  ((geomElem->AddElement("sphere"))->GetElement("radius"))->Set(_radius);

  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _radius));
  }

  this->CreatePart(visVisual);
  this->dataPtr->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddCylinder(double _radius, double _length,
    const math::Pose &_pose)
{
  if (!this->dataPtr->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "unit_cylinder_" << this->dataPtr->cylinderCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      linkName, this->dataPtr->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/GreyTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  sdf::ElementPtr cylinderElem = geomElem->AddElement("cylinder");
  (cylinderElem->GetElement("radius"))->Set(_radius);
  (cylinderElem->GetElement("length"))->Set(_length);

  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _length/2));
  }

  this->CreatePart(visVisual);
  this->dataPtr->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddCustom(const std::string &_path,
    const math::Vector3 &_scale, const math::Pose &_pose)
{
  if (!this->dataPtr->modelVisual)
    this->Reset();

  std::string path = _path;

  std::ostringstream linkNameStream;
  linkNameStream << "custom_" << this->dataPtr->customCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      this->dataPtr->modelName + "::" +
      linkName, this->dataPtr->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/GreyTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  sdf::ElementPtr meshElem = geomElem->AddElement("mesh");
  meshElem->GetElement("scale")->Set(_scale);
  meshElem->GetElement("uri")->Set(path);
  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _scale.z/2));
  }

  this->CreatePart(visVisual);
  this->dataPtr->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
void ModelCreator::CreatePart(const rendering::VisualPtr &_visual)
{
  PartData *part = new PartData;
  part->name = _visual->GetName();
  part->visuals.push_back(_visual);

  part->gravity = true;
  part->selfCollide = false;
  part->kinematic = false;

  part->inertial.reset(new physics::Inertial);

  this->dataPtr->allParts[part->name] = part;
}

/////////////////////////////////////////////////
void ModelCreator::RemovePart(const std::string &_partName)
{
  if (!this->dataPtr->modelVisual)
  {
    this->Reset();
    return;
  }

  if (this->dataPtr->allParts.find(_partName) == this->dataPtr->allParts.end())
    return;

  PartData *part = this->dataPtr->allParts[_partName];
  if (!part)
    return;

  for (unsigned int i = 0; i < part->visuals.size(); ++i)
  {
    rendering::VisualPtr vis = part->visuals[i];
    rendering::VisualPtr visParent = vis->GetParent();
    rendering::ScenePtr scene = vis->GetScene();
    scene->RemoveVisual(vis);
    if (visParent)
      scene->RemoveVisual(visParent);
  }

  part->inertial.reset();

  this->dataPtr->allParts.erase(_partName);
}

/////////////////////////////////////////////////
void ModelCreator::Reset()
{
  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
    return;

  KeyEventHandler::Instance()->AddPressFilter("model_part",
      boost::bind(&ModelCreator::OnKeyPressPart, this, _1));

  MouseEventHandler::Instance()->AddReleaseFilter("model_part",
      boost::bind(&ModelCreator::OnMouseReleasePart, this, _1));

  MouseEventHandler::Instance()->AddMoveFilter("model_part",
      boost::bind(&ModelCreator::OnMouseMovePart, this, _1));

  MouseEventHandler::Instance()->AddDoubleClickFilter("model_part",
      boost::bind(&ModelCreator::OnMouseDoubleClickPart, this, _1));

  this->dataPtr->jointMaker->Reset();
  this->dataPtr->selectedVis.reset();

  std::stringstream ss;
  ss << "defaultModel_" << this->dataPtr->modelCounter++;
  this->dataPtr->modelName = ss.str();

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  this->dataPtr->isStatic = false;
  this->dataPtr->autoDisable = true;

  while (this->dataPtr->allParts.size() > 0)
    this->RemovePart(this->dataPtr->allParts.begin()->first);
  this->dataPtr->allParts.clear();

  if (this->dataPtr->modelVisual)
    scene->RemoveVisual(this->dataPtr->modelVisual);

  this->dataPtr->modelVisual.reset(new rendering::Visual(
      this->dataPtr->modelName, scene->GetWorldVisual()));

  this->dataPtr->modelVisual->Load();
  this->dataPtr->modelPose = math::Pose::Zero;
  this->dataPtr->modelVisual->SetPose(this->dataPtr->modelPose);
  scene->AddVisual(this->dataPtr->modelVisual);
}

/////////////////////////////////////////////////
void ModelCreator::SetModelName(const std::string &_modelName)
{
  this->dataPtr->modelName = _modelName;
}

/////////////////////////////////////////////////
std::string ModelCreator::GetModelName() const
{
  return this->dataPtr->modelName;
}

/////////////////////////////////////////////////
void ModelCreator::SetStatic(bool _static)
{
  this->dataPtr->isStatic = _static;
}

/////////////////////////////////////////////////
void ModelCreator::SetAutoDisable(bool _auto)
{
  this->dataPtr->autoDisable = _auto;
}

/////////////////////////////////////////////////
void ModelCreator::SaveToSDF(const std::string &_savePath)
{
  std::ofstream savefile;
  boost::filesystem::path path;
  path = boost::filesystem::operator/(_savePath,
      this->dataPtr->modelName + ".sdf");
  savefile.open(path.string().c_str());
  if (savefile.is_open())
  {
    savefile << this->dataPtr->modelSDF->ToString();
    savefile.close();
  }
  else
  {
    gzerr << "Unable to open file for writing: '" << path.string().c_str()
        << "'. Possibly a permission issue." << std::endl;
  }
}

/////////////////////////////////////////////////
void ModelCreator::FinishModel()
{
  event::Events::setSelectedEntity("", "normal");
  this->Reset();
  this->CreateTheEntity();
}

/////////////////////////////////////////////////
void ModelCreator::CreateTheEntity()
{
  msgs::Factory msg;
  msg.set_sdf(this->dataPtr->modelSDF->ToString());
  this->dataPtr->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
std::string ModelCreator::GetTemplateSDFString()
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
void ModelCreator::AddPart(PartType _type)
{
  this->Stop();

  this->dataPtr->addPartType = _type;
  if (_type != PART_NONE)
  {
    switch (_type)
    {
      case PART_BOX:
      {
        this->AddBox();
        break;
      }
      case PART_SPHERE:
      {
        this->AddSphere();
        break;
      }
      case PART_CYLINDER:
      {
        this->AddCylinder();
        break;
      }
      default:
      {
        gzwarn << "Unknown part type '" << _type << "'. " <<
            "Part not added" << std::endl;
        break;
      }
    }
  }
}

/////////////////////////////////////////////////
void ModelCreator::Stop()
{
  if (this->dataPtr->addPartType != PART_NONE && this->dataPtr->mouseVisual)
  {
    for (unsigned int i = 0; i < this->dataPtr->mouseVisual->GetChildCount(); ++i)
        this->RemovePart(this->dataPtr->mouseVisual->GetChild(0)->GetName());
    this->dataPtr->mouseVisual.reset();
  }
  if (this->dataPtr->jointMaker)
    this->dataPtr->jointMaker->Stop();
}

/////////////////////////////////////////////////
void ModelCreator::OnDelete(const std::string &_part)
{
  if (_part == this->dataPtr->modelName)
  {
    this->Reset();
    return;
  }

  if (this->dataPtr->jointMaker)
    this->dataPtr->jointMaker->RemoveJointsByPart(_part);

  this->RemovePart(_part);
}

/////////////////////////////////////////////////
bool ModelCreator::OnKeyPressPart(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    this->Stop();
  }
  else if (_event.key == Qt::Key_Delete)
  {
    if (this->dataPtr->selectedVis)
    {
      this->OnDelete(this->dataPtr->selectedVis->GetName());
      this->dataPtr->selectedVis.reset();
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseReleasePart(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  if (this->dataPtr->mouseVisual)
  {
    emit PartAdded();
    this->dataPtr->mouseVisual.reset();
    this->AddPart(PART_NONE);
    return true;
  }

  // In mouse normal mode, let users select a part if the parent model
  // is currently selected.
  rendering::VisualPtr vis = gui::get_active_camera()->GetVisual(_event.pos);
  if (vis)
  {
    if (this->dataPtr->allParts.find(vis->GetName()) !=
        this->dataPtr->allParts.end())
    {
      if (gui::get_active_camera()->GetScene()->GetSelectedVisual()
          == this->dataPtr->modelVisual || this->dataPtr->selectedVis)
      {
        if (this->dataPtr->selectedVis)
          this->dataPtr->selectedVis->SetHighlighted(false);
        else
          event::Events::setSelectedEntity("", "normal");

        this->dataPtr->selectedVis = vis;
        this->dataPtr->selectedVis->SetHighlighted(true);
        return true;
      }
    }
    else if (this->dataPtr->selectedVis)
    {
      this->dataPtr->selectedVis->SetHighlighted(false);
      this->dataPtr->selectedVis.reset();
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseMovePart(const common::MouseEvent &_event)
{
  if (!this->dataPtr->mouseVisual)
    return false;

  if (!gui::get_active_camera())
    return false;

  math::Pose pose = this->dataPtr->mouseVisual->GetWorldPose();
  pose.pos = ModelManipulator::GetMousePositionOnPlane(
      gui::get_active_camera(), _event);

  if (!_event.shift)
  {
    pose.pos = ModelManipulator::SnapPoint(pose.pos);
  }
  pose.pos.z = this->dataPtr->mouseVisual->GetWorldPose().pos.z;

  this->dataPtr->mouseVisual->SetWorldPose(pose);

  return true;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseDoubleClickPart(const common::MouseEvent &_event)
{
  rendering::VisualPtr vis = gui::get_active_camera()->GetVisual(_event.pos);
  if (vis)
  {
    if (this->dataPtr->allParts.find(vis->GetName()) !=
        this->dataPtr->allParts.end())
    {
      // TODO part inspector code goes here
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
JointMaker *ModelCreator::GetJointMaker() const
{
  return this->dataPtr->jointMaker;
}

/////////////////////////////////////////////////
void ModelCreator::GenerateSDF()
{
  sdf::ElementPtr modelElem;
  sdf::ElementPtr linkElem;
  sdf::ElementPtr visualElem;
  sdf::ElementPtr collisionElem;

  this->dataPtr->modelSDF.reset(new sdf::SDF);
  this->dataPtr->modelSDF->SetFromString(this->GetTemplateSDFString());

  modelElem = this->dataPtr->modelSDF->root->GetElement("model");

  linkElem = modelElem->GetElement("link");
  sdf::ElementPtr templateLinkElem = linkElem->Clone();
  sdf::ElementPtr templateVisualElem = templateLinkElem->GetElement(
      "visual")->Clone();
  sdf::ElementPtr templateCollisionElem = templateLinkElem->GetElement(
      "collision")->Clone();
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(this->dataPtr->modelName);

  boost::unordered_map<std::string, PartData *>::iterator partsIt;

  // set center of all parts to be origin
  math::Vector3 mid;
  for (partsIt = this->dataPtr->allParts.begin();
      partsIt != this->dataPtr->allParts.end(); ++partsIt)
  {
    PartData *part = partsIt->second;
    rendering::VisualPtr visual = part->visuals[0];
    mid += visual->GetParent()->GetWorldPose().pos;
  }
  mid /= this->dataPtr->allParts.size();
  this->dataPtr->origin.pos = mid;
  modelElem->GetElement("pose")->Set(this->dataPtr->origin);

  // loop through all parts and generate sdf
  for (partsIt = this->dataPtr->allParts.begin();
      partsIt != this->dataPtr->allParts.end(); ++partsIt)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    PartData *part = partsIt->second;
    rendering::VisualPtr visual = part->visuals[0];
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(visual->GetParent()->GetName());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose()
        - this->dataPtr->origin);
    newLinkElem->GetElement("gravity")->Set(part->gravity);
    newLinkElem->GetElement("self_collide")->Set(part->selfCollide);
    newLinkElem->GetElement("kinematic")->Set(part->kinematic);
    sdf::ElementPtr inertialElem = newLinkElem->GetElement("inertial");
    inertialElem->GetElement("mass")->Set(part->inertial->GetMass());
    inertialElem->GetElement("pose")->Set(part->inertial->GetPose());
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    inertiaElem->GetElement("ixx")->Set(part->inertial->GetIXX());
    inertiaElem->GetElement("iyy")->Set(part->inertial->GetIYY());
    inertiaElem->GetElement("izz")->Set(part->inertial->GetIZZ());
    inertiaElem->GetElement("ixy")->Set(part->inertial->GetIXY());
    inertiaElem->GetElement("ixz")->Set(part->inertial->GetIXZ());
    inertiaElem->GetElement("iyz")->Set(part->inertial->GetIYZ());

    modelElem->InsertElement(newLinkElem);

    visualElem->GetAttribute("name")->Set(visual->GetParent()->GetName()
        + "_visual");
    collisionElem->GetAttribute("name")->Set(visual->GetParent()->GetName()
        + "_collision");
    visualElem->GetElement("pose")->Set(visual->GetPose());
    collisionElem->GetElement("pose")->Set(visual->GetPose());

    sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
    geomElem->ClearElements();

    math::Vector3 scale = visual->GetScale();
    if (visual->GetParent()->GetName().find("unit_box") != std::string::npos)
    {
      sdf::ElementPtr boxElem = geomElem->AddElement("box");
      (boxElem->GetElement("size"))->Set(scale);
    }
    else if (visual->GetParent()->GetName().find("unit_cylinder")
       != std::string::npos)
    {
      sdf::ElementPtr cylinderElem = geomElem->AddElement("cylinder");
      (cylinderElem->GetElement("radius"))->Set(scale.x/2.0);
      (cylinderElem->GetElement("length"))->Set(scale.z);
    }
    else if (visual->GetParent()->GetName().find("unit_sphere")
        != std::string::npos)
    {
      sdf::ElementPtr sphereElem = geomElem->AddElement("sphere");
      (sphereElem->GetElement("radius"))->Set(scale.x/2.0);
    }
    else if (visual->GetParent()->GetName().find("custom")
        != std::string::npos)
    {
      sdf::ElementPtr customElem = geomElem->AddElement("mesh");
      (customElem->GetElement("scale"))->Set(scale);
      (customElem->GetElement("uri"))->Set(visual->GetMeshName());
    }
    sdf::ElementPtr geomElemClone = geomElem->Clone();
    geomElem =  collisionElem->GetElement("geometry");
    geomElem->ClearElements();
    geomElem->InsertElement(geomElemClone->GetFirstElement());
  }

  // Add joint sdf elements
  this->dataPtr->jointMaker->GenerateSDF();
  sdf::ElementPtr jointsElem = this->dataPtr->jointMaker->GetSDF();

  sdf::ElementPtr jointElem;
  if (jointsElem->HasElement("joint"))
    jointElem = jointsElem->GetElement("joint");
  while (jointElem)
  {
    modelElem->InsertElement(jointElem->Clone());
    jointElem = jointElem->GetNextElement("joint");
  }

  // Model settings
  modelElem->GetElement("static")->Set(this->dataPtr->isStatic);
  modelElem->GetElement("allow_auto_disable")->Set(this->dataPtr->autoDisable);
}
