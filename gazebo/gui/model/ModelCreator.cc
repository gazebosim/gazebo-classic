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

#include <boost/filesystem.hpp>
#include <sstream>
#include <string>

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
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/ModelAlign.hh"

#include "gazebo/gui/model/ModelData.hh"
#include "gazebo/gui/model/PartGeneralConfig.hh"
#include "gazebo/gui/model/PartVisualConfig.hh"
#include "gazebo/gui/model/PartCollisionConfig.hh"
#include "gazebo/gui/model/PartInspector.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelCreator.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelCreator::ModelCreator()
{
  this->active = false;
  this->modelName = "";

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(ModelData::GetTemplateSDFString());

  this->partCounter = 0;
  this->customCounter = 0;
  this->modelCounter = 0;

  this->editTransparency = 0.4;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->makerPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->requestPub = this->node->Advertise<msgs::Request>("~/request");

  this->jointMaker = new JointMaker();

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));
  this->inspectAct = new QAction(tr("Open Part Inspector"), this);
  connect(this->inspectAct, SIGNAL(triggered()), this,
      SLOT(OnOpenInspector()));

  connect(g_deleteAct, SIGNAL(DeleteSignal(const std::string &)), this,
          SLOT(OnDelete(const std::string &)));

  this->connections.push_back(
      gui::Events::ConnectAlignMode(
        boost::bind(&ModelCreator::OnAlignMode, this, _1, _2, _3, _4)));

  this->connections.push_back(
      gui::Events::ConnectManipMode(
        boost::bind(&ModelCreator::OnManipMode, this, _1)));

  g_copyAct->setEnabled(false);
  g_pasteAct->setEnabled(false);

  connect(g_copyAct, SIGNAL(triggered()), this, SLOT(OnCopy()));
  connect(g_pasteAct, SIGNAL(triggered()), this, SLOT(OnPaste()));

  this->Reset();
}

/////////////////////////////////////////////////
ModelCreator::~ModelCreator()
{
  this->Reset();
  this->node->Fini();
  this->node.reset();
  this->modelTemplateSDF.reset();
  this->requestPub.reset();
  this->makerPub.reset();

  delete jointMaker;
}

/////////////////////////////////////////////////
void ModelCreator::OnEdit(bool _checked)
{
  if (_checked)
  {
    this->active = true;
    KeyEventHandler::Instance()->AddPressFilter("model_creator",
        boost::bind(&ModelCreator::OnKeyPress, this, _1));

    MouseEventHandler::Instance()->AddPressFilter("model_creator",
        boost::bind(&ModelCreator::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddReleaseFilter("model_creator",
        boost::bind(&ModelCreator::OnMouseRelease, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("model_creator",
        boost::bind(&ModelCreator::OnMouseMove, this, _1));

    MouseEventHandler::Instance()->AddDoubleClickFilter("model_creator",
        boost::bind(&ModelCreator::OnMouseDoubleClick, this, _1));

    this->jointMaker->EnableEventHandlers();
  }
  else
  {
    this->active = false;
    KeyEventHandler::Instance()->RemovePressFilter("model_creator");
    MouseEventHandler::Instance()->RemovePressFilter("model_creator");
    MouseEventHandler::Instance()->RemoveReleaseFilter("model_creator");
    MouseEventHandler::Instance()->RemoveMoveFilter("model_creator");
    MouseEventHandler::Instance()->RemoveDoubleClickFilter("model_creator");
    this->jointMaker->DisableEventHandlers();
    this->jointMaker->Stop();

    this->DeselectAll();
  }
}

/////////////////////////////////////////////////
std::string ModelCreator::CreateModel()
{
  this->Reset();
  return this->modelName;
}

/////////////////////////////////////////////////
void ModelCreator::AddJoint(const std::string &_type)
{
  this->Stop();
  if (this->jointMaker)
    this->jointMaker->AddJoint(_type);
}

/////////////////////////////////////////////////
std::string ModelCreator::AddBox(const math::Vector3 &_size,
    const math::Pose &_pose)
{
  if (!this->modelVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(linkName,
      this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
      linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");


  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  ((geomElem->AddElement("box"))->AddElement("size"))->Set(_size);

  visVisual->Load(visualElem);

  linkVisual->SetTransparency(this->editTransparency);
  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _size.z/2));
  }

  this->CreatePart(visVisual);

  this->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddSphere(double _radius,
    const math::Pose &_pose)
{
  if (!this->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  ((geomElem->AddElement("sphere"))->GetElement("radius"))->Set(_radius);

  visVisual->Load(visualElem);

  linkVisual->SetTransparency(this->editTransparency);
  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _radius));
  }

  this->CreatePart(visVisual);

  this->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddCylinder(double _radius, double _length,
    const math::Pose &_pose)
{
  if (!this->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  sdf::ElementPtr cylinderElem = geomElem->AddElement("cylinder");
  (cylinderElem->GetElement("radius"))->Set(_radius);
  (cylinderElem->GetElement("length"))->Set(_length);

  visVisual->Load(visualElem);

  linkVisual->SetTransparency(this->editTransparency);
  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _length/2));
  }

  this->CreatePart(visVisual);

  this->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddCustom(const std::string &_path,
    const math::Vector3 &_scale, const math::Pose &_pose)
{
  if (!this->modelVisual)
    this->Reset();

  std::string path = _path;

  std::ostringstream linkNameStream;
  linkNameStream << "custom_" << this->customCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << linkName << "_visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  sdf::ElementPtr meshElem = geomElem->AddElement("mesh");
  meshElem->GetElement("scale")->Set(_scale);
  meshElem->GetElement("uri")->Set(path);
  visVisual->Load(visualElem);

  linkVisual->SetTransparency(this->editTransparency);
  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _scale.z/2));
  }

  this->CreatePart(visVisual);

  this->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
PartData *ModelCreator::CreatePart(const rendering::VisualPtr &_visual)
{
  PartData *part = new PartData();
  part->partVisual = _visual->GetParent();

  part->partVisual = _visual->GetParent();
  part->AddVisual(_visual);
  //part->visuals.push_back(_visual);

  std::string partName = part->partVisual->GetName();
  part->SetName(partName);
  part->SetPose(part->partVisual->GetWorldPose());
  this->allParts[partName] = part;
  return part;
}

/////////////////////////////////////////////////
void ModelCreator::RemovePart(const std::string &_partName)
{
  if (!this->modelVisual)
  {
    this->Reset();
    return;
  }

  if (this->allParts.find(_partName) == this->allParts.end())
    return;

  PartData *part = this->allParts[_partName];
  if (!part)
    return;

  rendering::ScenePtr scene = part->partVisual->GetScene();
  std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
  for (it = part->visuals.begin(); it != part->visuals.end(); ++it)
  {
    rendering::VisualPtr vis = it->first;
    scene->RemoveVisual(vis);
  }

  scene->RemoveVisual(part->partVisual);

  part->partVisual.reset();
  delete part->inspector;

  this->allParts.erase(_partName);
}

/////////////////////////////////////////////////
void ModelCreator::Reset()
{
  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
    return;

  this->jointMaker->Reset();
  this->selectedVisuals.clear();
  g_copyAct->setEnabled(false);
  g_pasteAct->setEnabled(false);

  std::stringstream ss;
  ss << "defaultModel_" << this->modelCounter++;
  this->modelName = ss.str();

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  this->isStatic = false;
  this->autoDisable = true;

  while (this->allParts.size() > 0)
    this->RemovePart(this->allParts.begin()->first);
  this->allParts.clear();

  if (this->modelVisual)
    scene->RemoveVisual(this->modelVisual);

  this->modelVisual.reset(new rendering::Visual(this->modelName,
      scene->GetWorldVisual()));

  this->modelVisual->Load();
  this->modelPose = math::Pose::Zero;
  this->modelVisual->SetPose(this->modelPose);
  scene->AddVisual(this->modelVisual);
}

/////////////////////////////////////////////////
void ModelCreator::SetModelName(const std::string &_modelName)
{
  this->modelName = _modelName;
}

/////////////////////////////////////////////////
std::string ModelCreator::GetModelName() const
{
  return this->modelName;
}

/////////////////////////////////////////////////
void ModelCreator::SetStatic(bool _static)
{
  this->isStatic = _static;
}

/////////////////////////////////////////////////
void ModelCreator::SetAutoDisable(bool _auto)
{
  this->autoDisable = _auto;
}

/////////////////////////////////////////////////
void ModelCreator::SaveToSDF(const std::string &_savePath)
{
  std::ofstream savefile;
  boost::filesystem::path path;
  path = boost::filesystem::operator/(_savePath, this->modelName + ".sdf");
  savefile.open(path.string().c_str());
  if (savefile.is_open())
  {
    savefile << this->modelSDF->ToString();
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
  msg.set_sdf(this->modelSDF->ToString());
  this->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelCreator::AddPart(PartType _type)
{
  this->Stop();

  this->addPartType = _type;
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
  if (this->addPartType != PART_NONE && this->mouseVisual)
  {
    for (unsigned int i = 0; i < this->mouseVisual->GetChildCount(); ++i)
        this->RemovePart(this->mouseVisual->GetName());
    this->mouseVisual.reset();
  }
  if (this->jointMaker)
    this->jointMaker->Stop();
}

/////////////////////////////////////////////////
void ModelCreator::OnDelete(const std::string &_entity)
{
  // check if it's our model
  if (_entity == this->modelName)
  {
    this->Reset();
    return;
  }

  // if it's a link
  if (this->allParts.find(_entity) != this->allParts.end())
  {
    if (this->jointMaker)
      this->jointMaker->RemoveJointsByPart(_entity);
    this->RemovePart(_entity);
    return;
  }

  // if it's a visual
  rendering::VisualPtr vis =
      gui::get_active_camera()->GetScene()->GetVisual(_entity);
  if (vis)
  {
    rendering::VisualPtr parentLink = vis->GetParent();
    if (this->allParts.find(parentLink->GetName()) != this->allParts.end())
    {
      // remove the parent link if it's the only child
      if (parentLink->GetChildCount() == 1)
      {
        if (this->jointMaker)
          this->jointMaker->RemoveJointsByPart(parentLink->GetName());
        this->RemovePart(parentLink->GetName());
        return;
      }
    }
  }
}

/////////////////////////////////////////////////
bool ModelCreator::OnKeyPress(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    this->Stop();
  }
  else if (_event.key == Qt::Key_Delete)
  {
    if (!this->selectedVisuals.empty())
    {
      for (std::vector<rendering::VisualPtr>::iterator it
          = this->selectedVisuals.begin(); it != this->selectedVisuals.end();)
      {
        (*it)->SetHighlighted(false);
        this->OnDelete((*it)->GetName());
        it = this->selectedVisuals.erase(it);
      }
    }
  }
  else if (_event.control)
  {
    if (_event.key == Qt::Key_C && _event.control)
    {
      g_copyAct->trigger();
      return true;
    }
    if (_event.key == Qt::Key_V && _event.control)
    {
      g_pasteAct->trigger();
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMousePress(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);
  if (vis && !vis->IsPlane())
  {
    if (this->allParts.find(vis->GetParent()->GetName()) ==
          this->allParts.end())
    {
      // Prevent interaction with other models, send event only to
      // user camera
      userCamera->HandleMouseEvent(_event);
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseRelease(const common::MouseEvent &_event)
{
  if (this->jointMaker->GetState() != JointMaker::JOINT_NONE)
    return false;

  if (this->mouseVisual)
  {
    // set the part data pose
    if (this->allParts.find(this->mouseVisual->GetName()) !=
        this->allParts.end())
    {
      PartData *part = this->allParts[this->mouseVisual->GetName()];
      part->SetPose(this->mouseVisual->GetWorldPose());
    }

    // reset and return
    emit PartAdded();
    this->mouseVisual.reset();
    this->AddPart(PART_NONE);
    return true;
  }

  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);
  if (vis)
  {
    rendering::VisualPtr partVis = vis->GetParent();
    // Is part
    if (this->allParts.find(partVis->GetName()) !=
        this->allParts.end())
    {
      // Not in multi-selection mode.
      if (!(QApplication::keyboardModifiers() & Qt::ControlModifier))
      {
        this->DeselectAll();

        // Highlight and selected clicked part
        partVis->SetHighlighted(true);
        this->selectedVisuals.push_back(partVis);
      }
      // Multi-selection mode
      else
      {
        std::vector<rendering::VisualPtr>::iterator it =
            std::find(this->selectedVisuals.begin(),
            this->selectedVisuals.end(), vis);
        // Highlight and selected clicked part if not already selected
        if (it == this->selectedVisuals.end())
        {
          partVis->SetHighlighted(true);
          this->selectedVisuals.push_back(partVis);
        }
        // Deselect if already selected
        else
        {
          partVis->SetHighlighted(false);
          this->selectedVisuals.erase(it);
        }
      }
      g_copyAct->setEnabled(!this->selectedVisuals.empty());
      g_alignAct->setEnabled(this->selectedVisuals.size() > 1);
      return true;
    }
    // Not part
    else
    {
      this->DeselectAll();

      g_alignAct->setEnabled(false);
      g_copyAct->setEnabled(!this->selectedVisuals.empty());

      // Prevent interaction with other models, send event only to
      // user camera
      userCamera->HandleMouseEvent(_event);
      return true;
    }
  }
  return false;



}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseMove(const common::MouseEvent &_event)
{
  this->lastMouseEvent = _event;
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  if (!this->mouseVisual)
  {
    rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);
    if (vis && !vis->IsPlane())
    {
      if (this->allParts.find(vis->GetParent()->GetName()) ==
        this->allParts.end())
      {
        // Prevent interaction with other models, send event only to
        // user camera
        QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
        userCamera->HandleMouseEvent(_event);
        return true;
      }
    }
    return false;
  }

  math::Pose pose = this->mouseVisual->GetWorldPose();
  pose.pos = ModelManipulator::GetMousePositionOnPlane(
      userCamera, _event);

  if (!_event.shift)
  {
    pose.pos = ModelManipulator::SnapPoint(pose.pos);
  }
  pose.pos.z = this->mouseVisual->GetWorldPose().pos.z;

  this->mouseVisual->SetWorldPose(pose);

  return true;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseDoubleClick(const common::MouseEvent &_event)
{
  // open the part inspector on double click
 rendering::VisualPtr vis = gui::get_active_camera()->GetVisual(_event.pos);
  if (!vis)
    return false;

  if (this->allParts.find(vis->GetParent()->GetName()) !=
      this->allParts.end())
  {
    this->OpenInspector(vis->GetParent()->GetName());
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
void ModelCreator::OnOpenInspector()
{
  this->OpenInspector(this->inspectVis->GetName());
  this->inspectVis.reset();
}

/////////////////////////////////////////////////
void ModelCreator::OpenInspector(const std::string &_name)
{
  PartData *part = this->allParts[_name];
  part->SetPose(part->partVisual->GetWorldPose());
/*  PartGeneralConfig *generalConfig = part->inspector->GetGeneralConfig();
  generalConfig->SetPose(part->GetPose());*/

/*  PartVisualConfig *visualConfig = part->inspector->GetVisualConfig();
  for (unsigned int i = 0; i < part->visuals.size(); ++i)
  {
    if (i >= visualConfig->GetVisualCount())
      visualConfig->AddVisual();

  }

  PartCollisionConfig *collisionConfig = part->inspector->GetCollisionConfig();
  for (unsigned int i = 0; i < part->collisions.size(); ++i)
  {
    if (i >= collisionConfig->GetCollisionCount())
      collisionConfig->AddCollision();

  }*/

  part->inspector->show();
}

/////////////////////////////////////////////////
void ModelCreator::OnCopy()
{
  if (!g_editModelAct->isChecked())
    return;

  if (!this->selectedVisuals.empty())
  {
    this->copiedPartNames.clear();
    for (unsigned int i = 0; i < this->selectedVisuals.size(); ++i)
    {
      this->copiedPartNames.push_back(this->selectedVisuals[i]->GetName());
    }
    g_pasteAct->setEnabled(true);
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnPaste()
{
  if (this->copiedPartNames.empty() || !g_editModelAct->isChecked())
  {
    return;
  }

  // For now, only copy the last selected model
  boost::unordered_map<std::string, PartData *>::iterator it =
    this->allParts.find(this->copiedPartNames.back());
  if (it != this->allParts.end())
  {
    PartData *copiedPart = it->second;
    if (!copiedPart)
      return;

    this->Stop();
    this->DeselectAll();

    std::string linkName = copiedPart->GetName() + "_clone";

    if (!this->modelVisual)
    {
      this->Reset();
    }

    rendering::VisualPtr linkVisual(new rendering::Visual(
        linkName, this->modelVisual));
    linkVisual->Load();

    std::ostringstream visualName;
    visualName << linkName << "_visual";
    rendering::VisualPtr visVisual;

    math::Pose clonePose;

    if (copiedPart->visuals.empty())
    {
      visVisual = rendering::VisualPtr(new rendering::Visual(visualName.str(),
          linkVisual));
      sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
          ->GetElement("model")->GetElement("link")->GetElement("visual");
      visVisual->Load(visualElem);
    }
    else
    {
      rendering::VisualPtr copiedVisual = copiedPart->visuals.rbegin()->first;
      visVisual = copiedVisual->Clone(visualName.str(), linkVisual);
      clonePose = copiedVisual->GetWorldPose();
    }

    rendering::UserCameraPtr userCamera = gui::get_active_camera();
    if (userCamera)
    {
      math::Vector3 mousePosition =
        ModelManipulator::GetMousePositionOnPlane(userCamera,
                                                  this->lastMouseEvent);
      clonePose.pos.x = mousePosition.x;
      clonePose.pos.y = mousePosition.y;
    }

    linkVisual->SetWorldPose(clonePose);
    linkVisual->SetTransparency(this->editTransparency);

    this->addPartType = PART_CUSTOM;
    this->CreatePart(visVisual);
    this->mouseVisual = linkVisual;
  }
}

/////////////////////////////////////////////////
JointMaker *ModelCreator::GetJointMaker() const
{
  return this->jointMaker;
}

/////////////////////////////////////////////////
void ModelCreator::GenerateSDF()
{
  sdf::ElementPtr modelElem;
  sdf::ElementPtr linkElem;

  this->modelSDF.reset(new sdf::SDF);
  this->modelSDF->SetFromString(ModelData::GetTemplateSDFString());

  modelElem = this->modelSDF->root->GetElement("model");

  linkElem = modelElem->GetElement("link");
  sdf::ElementPtr templateLinkElem = linkElem->Clone();
  /*sdf::ElementPtr templateVisualElem = templateLinkElem->GetElement(
      "visual")->Clone();
  sdf::ElementPtr templateCollisionElem = templateLinkElem->GetElement(
      "collision")->Clone();*/
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(this->modelName);

  boost::unordered_map<std::string, PartData *>::iterator partsIt;

  // set center of all parts to be origin
  math::Vector3 mid;
  for (partsIt = this->allParts.begin(); partsIt != this->allParts.end();
       ++partsIt)
  {
    PartData *part = partsIt->second;
    mid += part->GetPose().pos;
  }
  mid /= this->allParts.size();
  this->origin.pos = mid;
  modelElem->GetElement("pose")->Set(this->origin);

  // loop through all parts and generate sdf
  for (partsIt = this->allParts.begin(); partsIt != this->allParts.end();
       ++partsIt)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    PartData *part = partsIt->second;
    sdf::ElementPtr newLinkElem = part->partSDF->Clone();


    modelElem->InsertElement(newLinkElem);

    std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
    for (it = part->visuals.begin(); it != part->visuals.end(); ++it)
    {
      rendering::VisualPtr visual = it->first;
      sdf::ElementPtr visualElem = visual->GetSDF()->Clone();


      newLinkElem->InsertElement(visualElem);
      //newLinkElem->InsertElement(collisionElem);
    }
  }

  // Add joint sdf elements
  this->jointMaker->GenerateSDF();
  sdf::ElementPtr jointsElem = this->jointMaker->GetSDF();

  sdf::ElementPtr jointElem;
  if (jointsElem->HasElement("joint"))
    jointElem = jointsElem->GetElement("joint");
  while (jointElem)
  {
    modelElem->InsertElement(jointElem->Clone());
    jointElem = jointElem->GetNextElement("joint");
  }

  // Model settings
  modelElem->GetElement("static")->Set(this->isStatic);
  modelElem->GetElement("allow_auto_disable")->Set(this->autoDisable);
}

/////////////////////////////////////////////////
void ModelCreator::OnAlignMode(const std::string &_axis,
    const std::string &_config, const std::string &_target, bool _preview)
{
  // Align links, not visuals
  std::vector<rendering::VisualPtr> selectedLinks;
  for (unsigned int i = 0; i < this->selectedVisuals.size(); ++i)
  {
    selectedLinks.push_back(this->selectedVisuals[i]);
  }

  ModelAlign::Instance()->AlignVisuals(selectedLinks, _axis, _config,
      _target, !_preview);
}

/////////////////////////////////////////////////
void ModelCreator::DeselectAll()
{
  if (!this->selectedVisuals.empty())
  {
    for (unsigned int i = 0; i < this->selectedVisuals.size(); ++i)
    {
      this->selectedVisuals[i]->SetHighlighted(false);
    }
    this->selectedVisuals.clear();
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnManipMode(const std::string &_mode)
{
  if (!this->active)
    return;

  if (!this->selectedVisuals.empty())
  {
    ModelManipulator::Instance()->SetAttachedVisual(
        this->selectedVisuals.back());
  }

  ModelManipulator::Instance()->SetManipulationMode(_mode);
  ModelSnap::Instance()->Reset();

  // deselect 0 to n-1 models.
  if (this->selectedVisuals.size() > 1)
  {
    for (std::vector<rendering::VisualPtr>::iterator it
        = this->selectedVisuals.begin(); it != --this->selectedVisuals.end();)
    {
       (*it)->SetHighlighted(false);
       it = this->selectedVisuals.erase(it);
    }
  }
}
