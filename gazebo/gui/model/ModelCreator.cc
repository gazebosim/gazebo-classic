/*
 * Copyright 2015 Open Source Robotics Foundation
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
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/physics/Inertial.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/ModelAlign.hh"
#include "gazebo/gui/SaveDialog.hh"

#include "gazebo/gui/model/ModelData.hh"
#include "gazebo/gui/model/PartGeneralConfig.hh"
#include "gazebo/gui/model/PartVisualConfig.hh"
#include "gazebo/gui/model/PartCollisionConfig.hh"
#include "gazebo/gui/model/PartInspector.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"

using namespace gazebo;
using namespace gui;

const std::string ModelCreator::modelDefaultName = "Untitled";
const std::string ModelCreator::previewName = "ModelPreview";

/////////////////////////////////////////////////
ModelCreator::ModelCreator()
{
  this->active = false;

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(ModelData::GetTemplateSDFString());

  this->manipMode = "";
  this->partCounter = 0;
  this->modelCounter = 0;

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
      gui::Events::ConnectEditModel(
      boost::bind(&ModelCreator::OnEditModel, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectSaveModelEditor(
      boost::bind(&ModelCreator::OnSave, this)));

  this->connections.push_back(
      gui::model::Events::ConnectSaveAsModelEditor(
      boost::bind(&ModelCreator::OnSaveAs, this)));

  this->connections.push_back(
      gui::model::Events::ConnectNewModelEditor(
      boost::bind(&ModelCreator::OnNew, this)));

  this->connections.push_back(
      gui::model::Events::ConnectExitModelEditor(
      boost::bind(&ModelCreator::OnExit, this)));

  this->connections.push_back(
    gui::model::Events::ConnectModelNameChanged(
      boost::bind(&ModelCreator::OnNameChanged, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectModelChanged(
      boost::bind(&ModelCreator::ModelChanged, this)));

  this->connections.push_back(
      gui::Events::ConnectAlignMode(
        boost::bind(&ModelCreator::OnAlignMode, this, _1, _2, _3, _4)));

  this->connections.push_back(
      gui::Events::ConnectManipMode(
        boost::bind(&ModelCreator::OnManipMode, this, _1)));

  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&ModelCreator::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&ModelCreator::Update, this)));

  g_copyAct->setEnabled(false);
  g_pasteAct->setEnabled(false);

  connect(g_copyAct, SIGNAL(triggered()), this, SLOT(OnCopy()));
  connect(g_pasteAct, SIGNAL(triggered()), this, SLOT(OnPaste()));

  this->saveDialog = new SaveDialog(SaveDialog::MODEL);

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
  this->connections.clear();

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
void ModelCreator::OnEditModel(const std::string &_modelName)
{
  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
    return;

  if(!this->active)
  {
    gzwarn << "Model Editor must be active before loading a model. " <<
              "Not loading model " << _modelName << std::endl;
    return;
  }

  // Get SDF model element from model name
  boost::shared_ptr<msgs::Response> response =
    transport::request(get_world(), "world_sdf");

  msgs::GzString msg;
  // Make sure the response is correct
  if (response->response() != "error" && response->type() == msg.GetTypeName())
  {
    // Parse the response message
    msg.ParseFromString(response->serialized_data());

    // Parse the string into sdf
    sdf::SDF sdf_parsed;
    sdf_parsed.SetFromString(msg.data());
    // Check that sdf contains world
    if (sdf_parsed.root->HasElement("world") &&
        sdf_parsed.root->GetElement("world")->HasElement("model"))
    {
      sdf::ElementPtr world = sdf_parsed.root->GetElement("world");
      sdf::ElementPtr model = world->GetElement("model");
      while (model)
      {
        if (_modelName.compare(model->GetAttribute("name")->GetAsString()) == 0)
        {
          // Remove model from the scene to substitute with the preview visual
          rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
          rendering::VisualPtr vis = scene->GetVisual(_modelName);
          scene->RemoveVisual(vis);

          this->LoadSDF(model);
          return;
        }
        model = model->GetNextElement("model");
      }
      gzwarn << "Couldn't find SDF for " << _modelName << ". Not loading it."
          << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void ModelCreator::LoadSDF(sdf::ElementPtr _modelElem)
{
  // Reset preview visual in case there was something already loaded
  this->Reset();

  // Model general info
  // Keep previewModel with previewName to avoid conflicts
  if (_modelElem->HasElement("pose"))
    this->modelPose = _modelElem->Get<math::Pose>("pose");
  else
    this->modelPose = math::Pose::Zero;
  this->previewVisual->SetPose(this->modelPose);

  // Links
  sdf::ElementPtr linkElem = _modelElem->GetElement("link");
  while (linkElem)
  {
    this->CreatePartFromSDF(linkElem);
    linkElem = linkElem->GetNextElement("link");
  }
  // Joints

}

/////////////////////////////////////////////////
void ModelCreator::OnNew()
{
  if (this->allParts.empty())
  {
    this->Reset();
    gui::model::Events::newModel();
    return;
  }
  QString msg;
  QMessageBox msgBox(QMessageBox::Warning, QString("New"), msg);
  QPushButton *cancelButton = msgBox.addButton("Cancel", QMessageBox::YesRole);
  QPushButton *saveButton = msgBox.addButton("Save", QMessageBox::YesRole);

  switch (this->currentSaveState)
  {
    case ALL_SAVED:
    {
      msg.append("Are you sure you want to close this model and open a new "
                 "canvas?\n\n");
      msgBox.addButton("New Canvas", QMessageBox::ApplyRole);
      saveButton->hide();
      break;
    }
    case UNSAVED_CHANGES:
    case NEVER_SAVED:
    {
      msg.append("You have unsaved changes. Do you want to save this model "
                 "and open a new canvas?\n\n");
      msgBox.addButton("Don't Save", QMessageBox::ApplyRole);
      break;
    }
    default:
      return;
  }

  msg.append("Once you open a new canvas, your current model will no longer "
             "be editable.");
  msgBox.setText(msg);

  msgBox.exec();

  if (msgBox.clickedButton() != cancelButton)
  {
    if (msgBox.clickedButton() == saveButton)
    {
      if (!this->OnSave())
      {
        return;
      }
    }

    this->Reset();
    gui::model::Events::newModel();
  }
}

/////////////////////////////////////////////////
bool ModelCreator::OnSave()
{
  switch (this->currentSaveState)
  {
    case UNSAVED_CHANGES:
    {
      this->SaveModelFiles();
      gui::model::Events::saveModel(this->modelName);
      return true;
    }
    case NEVER_SAVED:
    {
      return this->OnSaveAs();
    }
    default:
      return false;
  }
}

/////////////////////////////////////////////////
bool ModelCreator::OnSaveAs()
{
  if (this->saveDialog->OnSaveAs())
  {
    // Prevent changing save location
    this->currentSaveState = ALL_SAVED;
    // Get name set by user
    this->SetModelName(this->saveDialog->GetModelName());
    // Update name on palette
    gui::model::Events::saveModel(this->modelName);
    // Generate and save files
    this->SaveModelFiles();
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void ModelCreator::OnNameChanged(const std::string &_name)
{
  if (_name.compare(this->modelName) == 0)
    return;

  this->SetModelName(_name);
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::OnExit()
{
  if (this->allParts.empty())
  {
    this->Reset();
    gui::model::Events::newModel();
    gui::model::Events::finishModel();
    return;
  }

  switch (this->currentSaveState)
  {
    case ALL_SAVED:
    {
      QString msg("Once you exit the Model Editor, "
      "your model will no longer be editable.\n\n"
      "Are you ready to exit?\n\n");
      QMessageBox msgBox(QMessageBox::NoIcon, QString("Exit"), msg);
      msgBox.addButton("Exit", QMessageBox::ApplyRole);
      QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);
      msgBox.exec();
      if (msgBox.clickedButton() == cancelButton)
      {
        return;
      }
      this->FinishModel();
      break;
    }
    case UNSAVED_CHANGES:
    case NEVER_SAVED:
    {
      QString msg("Save Changes before exiting?\n\n"
          "Note: Once you exit the Model Editor, "
          "your model will no longer be editable.\n\n");

      QMessageBox msgBox(QMessageBox::NoIcon, QString("Exit"), msg);
      QPushButton *cancelButton = msgBox.addButton("Cancel",
          QMessageBox::ApplyRole);
      QPushButton *saveButton = msgBox.addButton("Save and Exit",
          QMessageBox::ApplyRole);
      msgBox.addButton("Don't Save, Exit", QMessageBox::ApplyRole);
      msgBox.exec();
      if (msgBox.clickedButton() == cancelButton)
        return;

      if (msgBox.clickedButton() == saveButton)
      {
        if (!this->OnSave())
        {
          return;
        }
      }
      break;
    }
    default:
      return;
  }

  // Create entity on main window up to the saved point
  if (this->currentSaveState != NEVER_SAVED)
    this->FinishModel();

  this->Reset();

  gui::model::Events::newModel();
  gui::model::Events::finishModel();
}

/////////////////////////////////////////////////
void ModelCreator::SaveModelFiles()
{
  this->saveDialog->GenerateConfig();
  this->saveDialog->SaveToConfig();
  this->GenerateSDF();
  std::cout << this->modelSDF->ToString() << std::endl;
  this->saveDialog->SaveToSDF(this->modelSDF);
  this->currentSaveState = ALL_SAVED;
}

/////////////////////////////////////////////////
std::string ModelCreator::CreateModel()
{
  this->Reset();
  return this->folderName;
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
  if (!this->previewVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(linkName,
      this->previewVisual));
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

  linkVisual->SetTransparency(ModelData::GetEditTransparency());
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
  if (!this->previewVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      linkName, this->previewVisual));
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

  linkVisual->SetTransparency(ModelData::GetEditTransparency());
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
  if (!this->previewVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      linkName, this->previewVisual));
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

  linkVisual->SetTransparency(ModelData::GetEditTransparency());
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
  if (!this->previewVisual)
    this->Reset();

  std::string path = _path;

  std::ostringstream linkNameStream;
  linkNameStream << "part_" << this->partCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->previewName +
      "::" + linkName, this->previewVisual));
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

  linkVisual->SetTransparency(ModelData::GetEditTransparency());
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
  part->AddVisual(_visual);

  // create collision with identical geometry
  rendering::VisualPtr collisionVis =
      _visual->Clone(part->partVisual->GetName() + "_collision",
      part->partVisual);

  // orange
  collisionVis->SetAmbient(common::Color(1.0, 0.5, 0.05));
  collisionVis->SetDiffuse(common::Color(1.0, 0.5, 0.05));
  collisionVis->SetSpecular(common::Color(0.5, 0.5, 0.5));
  collisionVis->SetTransparency(
      math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
  // fix for transparency alpha compositing
  Ogre::MovableObject *colObj = collisionVis->GetSceneNode()->
      getAttachedObject(0);
  colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
  part->AddCollision(collisionVis);

  std::string partName = part->partVisual->GetName();
  part->SetName(partName);
  part->SetPose(part->partVisual->GetWorldPose());
  this->allParts[partName] = part;

  rendering::ScenePtr scene = part->partVisual->GetScene();
  scene->AddVisual(part->partVisual);

  this->ModelChanged();

  return part;
}

/////////////////////////////////////////////////
PartData *ModelCreator::CreatePartFromSDF(sdf::ElementPtr _linkElem)
{
  PartData *part = new PartData;

  // General link info
  part->SetName(_linkElem->Get<std::string>("name"));
//  part->scale = math::Vector3::One;
//  part->gravity = _linkElem->Get<bool>("gravity");
//  part->selfCollide = _linkElem->Get<bool>("self_collide");
//  part->kinematic = _linkElem->Get<bool>("kinematic");

  math::Pose linkPose;
  if (_linkElem->HasElement("pose"))
    linkPose = _linkElem->Get<math::Pose>("pose");
  else
    linkPose.Set(0, 0, 0, 0, 0, 0);

  part->SetPose(this->modelPose + linkPose);

  rendering::VisualPtr linkVisual(new rendering::Visual(part->GetName(),
      this->previewVisual));
  linkVisual->Load();
  linkVisual->SetPose(linkPose);
  part->partVisual = linkVisual;

  // Visuals
  int visualIndex = 0;
  sdf::ElementPtr visualElem;

  if (_linkElem->HasElement("visual"))
    visualElem = _linkElem->GetElement("visual");

  while (visualElem)
  {
    math::Pose visualPose;
    if (visualElem->HasElement("pose"))
      visualPose = visualElem->Get<math::Pose>("pose");
    else
      visualPose.Set(0, 0, 0, 0, 0, 0);

    // Hack alert
    std::string shapeSuffix;
    if (visualElem->HasElement("geometry"))
    {
      if (visualElem->GetElement("geometry")->HasElement("box"))
        shapeSuffix = "_unit_box";
      else if (visualElem->GetElement("geometry")->HasElement("sphere"))
        shapeSuffix = "_unit_sphere";
      else if (visualElem->GetElement("geometry")->HasElement("cylinder"))
        shapeSuffix = "_unit_cylinder";
      else
        shapeSuffix = "_custom";
    }

    std::ostringstream visualName;
    visualName << part->GetName() << "::Visual_" << shapeSuffix
      << visualIndex++;
    rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
          linkVisual));

    visVisual->Load(visualElem);
    visVisual->SetPose(visualPose);
    part->AddVisual(visVisual);

    visualElem = visualElem->GetNextElement("visual");
  }
  linkVisual->SetTransparency(ModelData::GetEditTransparency());

  this->allParts[part->GetName()] = part;

  rendering::ScenePtr scene = part->partVisual->GetScene();
  scene->AddVisual(part->partVisual);

  this->ModelChanged();

  return part;
}

/////////////////////////////////////////////////
void ModelCreator::RemovePart(const std::string &_partName)
{
  if (!this->previewVisual)
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
  std::map<rendering::VisualPtr, msgs::Collision>::iterator colIt;
  for (colIt = part->collisions.begin(); colIt != part->collisions.end();
      ++colIt)
  {
    rendering::VisualPtr vis = colIt->first;
    scene->RemoveVisual(vis);
  }

  scene->RemoveVisual(part->partVisual);

  part->partVisual.reset();
  delete part->inspector;

  this->allParts.erase(_partName);
  this->ModelChanged();
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

  this->currentSaveState = NEVER_SAVED;
  this->SetModelName(this->modelDefaultName);

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  this->isStatic = false;
  this->autoDisable = true;

  while (this->allParts.size() > 0)
    this->RemovePart(this->allParts.begin()->first);
  this->allParts.clear();

  if (this->previewVisual)
    scene->RemoveVisual(this->previewVisual);

  this->previewVisual.reset(new rendering::Visual(this->previewName,
      scene->GetWorldVisual()));

  this->previewVisual->Load();
  this->modelPose = math::Pose::Zero;
  this->previewVisual->SetPose(this->modelPose);
  scene->AddVisual(this->previewVisual);
}

/////////////////////////////////////////////////
void ModelCreator::SetModelName(const std::string &_modelName)
{
  this->modelName = _modelName;
  this->saveDialog->SetModelName(_modelName);

  this->folderName = this->saveDialog->
      GetFolderNameFromModelName(this->modelName);

  if (this->currentSaveState == NEVER_SAVED)
  {
    // Set new saveLocation
    boost::filesystem::path oldPath(this->saveDialog->GetSaveLocation());

    boost::filesystem::path newPath = oldPath.parent_path() / this->folderName;
    this->saveDialog->SetSaveLocation(newPath.string());
  }
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
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::SetAutoDisable(bool _auto)
{
  this->autoDisable = _auto;
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::FinishModel()
{
  event::Events::setSelectedEntity("", "normal");
  this->CreateTheEntity();
  this->Reset();
}

/////////////////////////////////////////////////
void ModelCreator::CreateTheEntity()
{
  msgs::Factory msg;
  // Create a new name if the model exists
  if (!this->modelSDF->root->HasElement("model"))
  {
    gzerr << "Generated invalid SDF! Cannot create entity." << std::endl;
    return;
  }
  msg.set_sdf(this->modelSDF->ToString());
  this->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelCreator::AddPart(PartType _type)
{
  if (!this->previewVisual)
  {
    this->Reset();
  }

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

  if (this->jointMaker->GetState() != JointMaker::JOINT_NONE)
  {
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);
  if (vis)
  {
    if (!vis->IsPlane() && gui::get_entity_id(vis->GetRootVisual()->GetName()))
    {
      // Handle snap from GLWidget
      if (g_snapAct->isChecked())
        return false;

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
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
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

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);
  if (vis)
  {
    rendering::VisualPtr partVis = vis->GetParent();
    // Is part
    if (this->allParts.find(partVis->GetName()) !=
        this->allParts.end())
    {
      // Handle snap from GLWidget
      if (g_snapAct->isChecked())
        return false;

      // trigger part inspector on right click
      if (_event.button == common::MouseEvent::RIGHT)
      {
        this->inspectVis = vis->GetParent();
        QMenu menu;
        menu.addAction(this->inspectAct);
        menu.exec(QCursor::pos());
        return true;
      }

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
            this->selectedVisuals.end(), partVis);
        // Highlight and select clicked part if not already selected
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

      if (!vis->IsPlane())
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
      // Main window models always handled here
      if (this->allParts.find(vis->GetParent()->GetName()) ==
          this->allParts.end())
      {
        // Prevent highlighting for snapping
        if (this->manipMode == "snap" || this->manipMode == "select" ||
            this->manipMode == "")
        {
          // Don't change cursor on hover
          QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
          userCamera->HandleMouseEvent(_event);
        }
        // Allow ModelManipulator to work while dragging handle over this
        else if (_event.dragging)
        {
          ModelManipulator::Instance()->OnMouseMoveEvent(_event);
        }
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
  part->UpdateConfig();
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

    if (!this->previewVisual)
    {
      this->Reset();
    }

    rendering::VisualPtr linkVisual(new rendering::Visual(
        linkName, this->previewVisual));
    linkVisual->Load();

    std::ostringstream visualName;
    visualName << linkName << "_visual";
    rendering::VisualPtr visVisual;

    math::Pose clonePose;
    math::Vector3 cloneScale;

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
      cloneScale = copiedVisual->GetParent()->GetScale();
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

    linkVisual->SetScale(cloneScale);
    linkVisual->SetWorldPose(clonePose);
    linkVisual->SetTransparency(ModelData::GetEditTransparency());

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
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(this->folderName);

  // set center of all parts to be origin
  boost::unordered_map<std::string, PartData *>::iterator partsIt;
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
    part->UpdateConfig();

    sdf::ElementPtr newLinkElem = part->partSDF->Clone();
    newLinkElem->GetElement("pose")->Set(part->partVisual->GetWorldPose()
        - this->origin);

    modelElem->InsertElement(newLinkElem);

    // visuals
    std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
    for (it = part->visuals.begin(); it != part->visuals.end(); ++it)
    {
      rendering::VisualPtr visual = it->first;
      msgs::Visual visualMsg = it->second;
      sdf::ElementPtr visualElem = visual->GetSDF()->Clone();
      visualElem->GetElement("transparency")->Set<double>(
          visualMsg.transparency());
      newLinkElem->InsertElement(visualElem);
    }

    // collisions
    std::map<rendering::VisualPtr, msgs::Collision>::iterator colIt;
    for (colIt = part->collisions.begin(); colIt != part->collisions.end();
        ++colIt)
    {
      sdf::ElementPtr collisionElem = msgs::CollisionToSDF(colIt->second);
      newLinkElem->InsertElement(collisionElem);
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
  ModelAlign::Instance()->AlignVisuals(this->selectedVisuals, _axis, _config,
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

  this->manipMode = _mode;

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

/////////////////////////////////////////////////
void ModelCreator::OnSetSelectedEntity(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  this->DeselectAll();
}

/////////////////////////////////////////////////
void ModelCreator::ModelChanged()
{
  if (this->currentSaveState != NEVER_SAVED)
    this->currentSaveState = UNSAVED_CHANGES;
}

/////////////////////////////////////////////////
void ModelCreator::Update()
{
  boost::unordered_map<std::string, PartData *>::iterator partsIt;
  for (partsIt = this->allParts.begin(); partsIt != this->allParts.end();
       ++partsIt)
  {
    PartData *part = partsIt->second;
    if (part->GetPose() != part->partVisual->GetWorldPose() ||
        part->scale != part->partVisual->GetScale())
    {
      part->SetPose(part->partVisual->GetWorldPose());
      part->scale = part->partVisual->GetScale();
      this->ModelChanged();
    }
  }
}
