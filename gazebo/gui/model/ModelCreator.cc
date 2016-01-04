/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include <boost/filesystem.hpp>
#include <sstream>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/SVGLoader.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/ModelAlign.hh"
#include "gazebo/gui/SaveDialog.hh"
#include "gazebo/gui/MainWindow.hh"

#include "gazebo/gui/model/ModelData.hh"
#include "gazebo/gui/model/LinkInspector.hh"
#include "gazebo/gui/model/ModelPluginInspector.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/ModelCreatorPrivate.hh"

using namespace gazebo;
using namespace gui;

const std::string ModelCreatorPrivate::modelDefaultName = "Untitled";
const std::string ModelCreatorPrivate::previewName = "ModelPreview";

/////////////////////////////////////////////////
ModelCreator::ModelCreator() : dataPtr(new ModelCreatorPrivate)
{
  this->dataPtr->active = false;

  this->dataPtr->modelTemplateSDF.reset(new sdf::SDF);
  this->dataPtr->modelTemplateSDF->SetFromString(
      ModelData::GetTemplateSDFString());

  this->dataPtr->updateMutex = new boost::recursive_mutex();

  this->dataPtr->manipMode = "";
  this->dataPtr->linkCounter = 0;
  this->dataPtr->modelCounter = 0;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->makerPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");
  this->dataPtr->requestPub =
      this->dataPtr->node->Advertise<msgs::Request>("~/request");

  this->dataPtr->jointMaker = new JointMaker();

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));

  this->dataPtr->inspectAct = new QAction(tr("Open Link Inspector"), this);
  connect(this->dataPtr->inspectAct, SIGNAL(triggered()), this,
      SLOT(OnOpenInspector()));

  if (g_deleteAct)
  {
    connect(g_deleteAct, SIGNAL(DeleteSignal(const std::string &)), this,
        SLOT(OnDelete(const std::string &)));
  }

  this->dataPtr->connections.push_back(
      gui::Events::ConnectEditModel(
      std::bind(&ModelCreator::OnEditModel, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectSaveModelEditor(
      std::bind(&ModelCreator::OnSave, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectSaveAsModelEditor(
      std::bind(&ModelCreator::OnSaveAs, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectNewModelEditor(
      std::bind(&ModelCreator::OnNew, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectExitModelEditor(
      std::bind(&ModelCreator::OnExit, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectModelNameChanged(
      std::bind(&ModelCreator::OnNameChanged, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectModelChanged(
      std::bind(&ModelCreator::ModelChanged, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectOpenLinkInspector(
      std::bind(&ModelCreator::OpenInspector, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectOpenModelPluginInspector(
      std::bind(&ModelCreator::OpenModelPluginInspector, this,
      std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectAlignMode(
      std::bind(&ModelCreator::OnAlignMode, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
      std::placeholders::_5)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectManipMode(
      std::bind(&ModelCreator::OnManipMode, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
      std::bind(&ModelCreator::OnSetSelectedEntity, this,
      std::placeholders::_1, std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectSetSelectedLink(
      std::bind(&ModelCreator::OnSetSelectedLink, this, std::placeholders::_1,
      std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectSetSelectedModelPlugin(
      std::bind(&ModelCreator::OnSetSelectedModelPlugin, this,
      std::placeholders::_1, std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectScaleEntity(
      std::bind(&ModelCreator::OnEntityScaleChanged, this,
      std::placeholders::_1, std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectShowLinkContextMenu(
      std::bind(&ModelCreator::ShowContextMenu, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectShowModelPluginContextMenu(
      std::bind(&ModelCreator::ShowModelPluginContextMenu, this,
      std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectRequestLinkRemoval(
      std::bind(&ModelCreator::RemoveEntity, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectRequestModelPluginRemoval(
      std::bind(&ModelCreator::RemoveModelPlugin, this,
      std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      event::Events::ConnectPreRender(std::bind(&ModelCreator::Update, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectModelPropertiesChanged(
      std::bind(&ModelCreator::OnPropertiesChanged, this, std::placeholders::_1,
      std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectRequestModelPluginInsertion(
      std::bind(&ModelCreator::OnAddModelPlugin, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));

  if (g_copyAct)
  {
    g_copyAct->setEnabled(false);
    connect(g_copyAct, SIGNAL(triggered()), this, SLOT(OnCopy()));
  }
  if (g_pasteAct)
  {
    g_pasteAct->setEnabled(false);
    connect(g_pasteAct, SIGNAL(triggered()), this, SLOT(OnPaste()));
  }

  this->dataPtr->saveDialog = new SaveDialog(SaveDialog::MODEL);

  this->Reset();
}

/////////////////////////////////////////////////
ModelCreator::~ModelCreator()
{
  while (!this->dataPtr->allLinks.empty())
    this->RemoveLinkImpl(this->dataPtr->allLinks.begin()->first);

  while (!this->dataPtr->allNestedModels.empty())
    this->RemoveNestedModelImpl(this->dataPtr->allNestedModels.begin()->first);

  this->dataPtr->allNestedModels.clear();
  this->dataPtr->allLinks.clear();
  this->dataPtr->allModelPlugins.clear();
  this->dataPtr->node->Fini();
  this->dataPtr->node.reset();
  this->dataPtr->modelTemplateSDF.reset();
  this->dataPtr->requestPub.reset();
  this->dataPtr->makerPub.reset();
  this->dataPtr->connections.clear();

  delete this->dataPtr->saveDialog;
  delete this->dataPtr->updateMutex;

  delete this->dataPtr->jointMaker;
}

/////////////////////////////////////////////////
void ModelCreator::OnEdit(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->active = true;
    this->dataPtr->modelCounter++;
    KeyEventHandler::Instance()->AddPressFilter("model_creator",
        std::bind(&ModelCreator::OnKeyPress, this, std::placeholders::_1));

    MouseEventHandler::Instance()->AddPressFilter("model_creator",
        std::bind(&ModelCreator::OnMousePress, this, std::placeholders::_1));

    MouseEventHandler::Instance()->AddReleaseFilter("model_creator",
        std::bind(&ModelCreator::OnMouseRelease, this, std::placeholders::_1));

    MouseEventHandler::Instance()->AddMoveFilter("model_creator",
        std::bind(&ModelCreator::OnMouseMove, this, std::placeholders::_1));

    MouseEventHandler::Instance()->AddDoubleClickFilter("model_creator",
        std::bind(&ModelCreator::OnMouseDoubleClick, this,
        std::placeholders::_1));

    this->dataPtr->jointMaker->EnableEventHandlers();
  }
  else
  {
    this->dataPtr->active = false;
    KeyEventHandler::Instance()->RemovePressFilter("model_creator");
    MouseEventHandler::Instance()->RemovePressFilter("model_creator");
    MouseEventHandler::Instance()->RemoveReleaseFilter("model_creator");
    MouseEventHandler::Instance()->RemoveMoveFilter("model_creator");
    MouseEventHandler::Instance()->RemoveDoubleClickFilter("model_creator");
    this->dataPtr->jointMaker->DisableEventHandlers();
    this->dataPtr->jointMaker->Stop();

    this->DeselectAll();
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnEditModel(const std::string &_modelName)
{
  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
  {
    gzerr << "Unable to edit model. GUI camera or scene is NULL"
        << std::endl;
    return;
  }

  if (!this->dataPtr->active)
  {
    gzwarn << "Model Editor must be active before loading a model. " <<
              "Not loading model " << _modelName << std::endl;
    return;
  }

  // Get SDF model element from model name
  // TODO replace with entity_info and parse gazebo.msgs.Model msgs
  // or handle model_sdf requests in world.
  boost::shared_ptr<msgs::Response> response =
    transport::request(gui::get_world(), "world_sdf");

  msgs::GzString msg;
  // Make sure the response is correct
  if (response->type() == msg.GetTypeName())
  {
    // Parse the response message
    msg.ParseFromString(response->serialized_data());

    // Parse the string into sdf
    sdf::SDF sdfParsed;
    sdfParsed.SetFromString(msg.data());

    // Check that sdf contains world
    if (sdfParsed.Root()->HasElement("world") &&
        sdfParsed.Root()->GetElement("world")->HasElement("model"))
    {
      sdf::ElementPtr world = sdfParsed.Root()->GetElement("world");
      sdf::ElementPtr model = world->GetElement("model");
      while (model)
      {
        if (model->GetAttribute("name")->GetAsString() == _modelName)
        {
          // Create the root model
          this->CreateModelFromSDF(model);

          // Hide the model from the scene to substitute with the preview visual
          this->SetModelVisible(_modelName, false);

          rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
          rendering::VisualPtr visual = scene->GetVisual(_modelName);

          ignition::math::Pose3d pose;
          if (visual)
          {
            pose = visual->GetWorldPose().Ign();
            this->dataPtr->previewVisual->SetWorldPose(pose);
          }

          this->dataPtr->serverModelName = _modelName;
          this->dataPtr->serverModelSDF = model;
          this->dataPtr->modelPose = pose;

          return;
        }
        model = model->GetNextElement("model");
      }
      gzwarn << "Couldn't find SDF for " << _modelName << ". Not loading it."
          << std::endl;
    }
  }
  else
  {
    GZ_ASSERT(response->type() == msg.GetTypeName(),
        "Received incorrect response from 'world_sdf' request.");
  }
}

/////////////////////////////////////////////////
NestedModelData *ModelCreator::CreateModelFromSDF(
    const sdf::ElementPtr &_modelElem, const rendering::VisualPtr &_parentVis,
    const bool _emit)
{
  rendering::VisualPtr modelVisual;
  std::stringstream modelNameStream;
  std::string nestedModelName;
  NestedModelData *modelData = new NestedModelData();

  // If no parent vis, this is the root model
  if (!_parentVis)
  {
    // Reset preview visual in case there was something already loaded
    this->Reset();

    // Keep previewModel with previewName to avoid conflicts
    modelVisual = this->dataPtr->previewVisual;
    modelNameStream << this->dataPtr->previewName << "_" <<
        this->dataPtr->modelCounter;

    // Model general info
    if (_modelElem->HasAttribute("name"))
      this->SetModelName(_modelElem->Get<std::string>("name"));

    if (_modelElem->HasElement("pose"))
    {
      this->dataPtr->modelPose =
          _modelElem->Get<ignition::math::Pose3d>("pose");
    }
    else
    {
      this->dataPtr->modelPose = ignition::math::Pose3d::Zero;
    }
    this->dataPtr->previewVisual->SetPose(this->dataPtr->modelPose);

    if (_modelElem->HasElement("static"))
      this->dataPtr->isStatic = _modelElem->Get<bool>("static");
    if (_modelElem->HasElement("allow_auto_disable"))
      this->dataPtr->autoDisable = _modelElem->Get<bool>("allow_auto_disable");
    gui::model::Events::modelPropertiesChanged(this->dataPtr->isStatic,
        this->dataPtr->autoDisable);
    gui::model::Events::modelNameChanged(this->ModelName());

    modelData->modelVisual = modelVisual;
  }
  // Nested models are attached to a parent visual
  else
  {
    // Internal name
    std::stringstream parentNameStream;
    parentNameStream << _parentVis->GetName();
    if (_parentVis->GetName() == this->dataPtr->previewName)
      parentNameStream << "_" << this->dataPtr->modelCounter;

    modelNameStream << parentNameStream.str() << "::" <<
        _modelElem->Get<std::string>("name");
    nestedModelName = modelNameStream.str();

    // Generate unique name
    auto itName = this->dataPtr->allNestedModels.find(nestedModelName);
    int nameCounter = 0;
    std::string uniqueName;
    while (itName != this->dataPtr->allNestedModels.end())
    {
      std::stringstream uniqueNameStr;
      uniqueNameStr << nestedModelName << "_" << nameCounter++;
      uniqueName = uniqueNameStr.str();
      itName = this->dataPtr->allNestedModels.find(uniqueName);
    }
    if (!uniqueName.empty())
      nestedModelName = uniqueName;

    // Model Visual
    modelVisual.reset(new rendering::Visual(nestedModelName, _parentVis));
    modelVisual->Load();
    modelVisual->SetTransparency(ModelData::GetEditTransparency());

    if (_modelElem->HasElement("pose"))
      modelVisual->SetPose(_modelElem->Get<ignition::math::Pose3d>("pose"));

    // Only keep SDF and preview visual
    std::string leafName = nestedModelName;
    leafName = leafName.substr(leafName.rfind("::")+2);

    modelData->modelSDF = _modelElem;
    modelData->modelVisual = modelVisual;
    modelData->SetName(leafName);
    modelData->SetPose(_modelElem->Get<ignition::math::Pose3d>("pose"));
  }

  // Notify nested model insertion
  if (_parentVis)
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->allNestedModels[nestedModelName] = modelData;

    // fire nested inserted events only when the nested model is
    //  not attached to the mouse
    if (_emit)
      gui::model::Events::nestedModelInserted(nestedModelName);
  }

  // Recursively load models nested in this model
  // This must be done after other widgets were notified about the current
  // model but before making joints
  sdf::ElementPtr nestedModelElem;
  if (_modelElem->HasElement("model"))
     nestedModelElem = _modelElem->GetElement("model");
  while (nestedModelElem)
  {
    if (this->dataPtr->canonicalModel.empty())
      this->dataPtr->canonicalModel = nestedModelName;

    NestedModelData *nestedModelData =
        this->CreateModelFromSDF(nestedModelElem, modelVisual, _emit);
    rendering::VisualPtr nestedModelVis = nestedModelData->modelVisual;
    modelData->models[nestedModelVis->GetName()] = nestedModelVis;
    nestedModelElem = nestedModelElem->GetNextElement("model");
  }

  // Links
  sdf::ElementPtr linkElem;
  if (_modelElem->HasElement("link"))
    linkElem = _modelElem->GetElement("link");
  while (linkElem)
  {
    LinkData *linkData = this->CreateLinkFromSDF(linkElem, modelVisual);

    // if its parent is not the preview visual then the link has to be nested
    if (modelVisual != this->dataPtr->previewVisual)
      linkData->nested = true;
    rendering::VisualPtr linkVis = linkData->linkVisual;

    modelData->links[linkVis->GetName()] = linkVis;
    linkElem = linkElem->GetNextElement("link");
  }

  // Don't load joints or plugins for nested models
  if (!_parentVis)
  {
    // Joints
    sdf::ElementPtr jointElem;
    if (_modelElem->HasElement("joint"))
       jointElem = _modelElem->GetElement("joint");

    while (jointElem)
    {
      this->dataPtr->jointMaker->CreateJointFromSDF(jointElem,
          modelNameStream.str());
      jointElem = jointElem->GetNextElement("joint");
    }

    // Plugins
    sdf::ElementPtr pluginElem;
    if (_modelElem->HasElement("plugin"))
      pluginElem = _modelElem->GetElement("plugin");
    while (pluginElem)
    {
      this->AddModelPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  return modelData;
}

/////////////////////////////////////////////////
void ModelCreator::OnNew()
{
  this->Stop();

  if (this->dataPtr->allLinks.empty() &&
      this->dataPtr->allNestedModels.empty() &&
      this->dataPtr->allModelPlugins.empty())
  {
    this->Reset();
    gui::model::Events::newModel();
    return;
  }
  QString msg;
  QMessageBox msgBox(QMessageBox::Warning, QString("New"), msg);
  QPushButton *cancelButton = msgBox.addButton("Cancel",
      QMessageBox::RejectRole);
  msgBox.setEscapeButton(cancelButton);
  QPushButton *saveButton = new QPushButton("Save");

  switch (this->dataPtr->currentSaveState)
  {
    case ALL_SAVED:
    {
      msg.append("Are you sure you want to close this model and open a new "
                 "canvas?\n\n");
      QPushButton *newButton =
          msgBox.addButton("New Canvas", QMessageBox::AcceptRole);
      msgBox.setDefaultButton(newButton);
      break;
    }
    case UNSAVED_CHANGES:
    case NEVER_SAVED:
    {
      msg.append("You have unsaved changes. Do you want to save this model "
                 "and open a new canvas?\n\n");
      msgBox.addButton("Don't Save", QMessageBox::DestructiveRole);
      msgBox.addButton(saveButton, QMessageBox::AcceptRole);
      msgBox.setDefaultButton(saveButton);
      break;
    }
    default:
      return;
  }

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
  this->Stop();

  switch (this->dataPtr->currentSaveState)
  {
    case UNSAVED_CHANGES:
    {
      this->SaveModelFiles();
      gui::model::Events::saveModel(this->dataPtr->modelName);
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
  this->Stop();

  if (this->dataPtr->saveDialog->OnSaveAs())
  {
    // Prevent changing save location
    this->dataPtr->currentSaveState = ALL_SAVED;
    // Get name set by user
    this->SetModelName(this->dataPtr->saveDialog->GetModelName());
    // Update name on palette
    gui::model::Events::saveModel(this->dataPtr->modelName);
    // Generate and save files
    this->SaveModelFiles();
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void ModelCreator::OnNameChanged(const std::string &_name)
{
  if (_name.compare(this->dataPtr->modelName) == 0)
    return;

  this->SetModelName(_name);
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::OnExit()
{
  this->Stop();

  if (this->dataPtr->allLinks.empty() &&
      this->dataPtr->allNestedModels.empty() &&
      this->dataPtr->allModelPlugins.empty())
  {
    if (!this->dataPtr->serverModelName.empty())
      this->SetModelVisible(this->dataPtr->serverModelName, true);
    this->Reset();
    gui::model::Events::newModel();
    gui::model::Events::finishModel();
    return;
  }

  switch (this->dataPtr->currentSaveState)
  {
    case ALL_SAVED:
    {
      QString msg("Are you ready to exit?\n\n");
      QMessageBox msgBox(QMessageBox::NoIcon, QString("Exit"), msg);

      QPushButton *cancelButton = msgBox.addButton("Cancel",
          QMessageBox::RejectRole);
      QPushButton *exitButton =
          msgBox.addButton("Exit", QMessageBox::AcceptRole);
      msgBox.setDefaultButton(exitButton);
      msgBox.setEscapeButton(cancelButton);

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
      QString msg("Save Changes before exiting?\n\n");

      QMessageBox msgBox(QMessageBox::NoIcon, QString("Exit"), msg);
      QPushButton *cancelButton = msgBox.addButton("Cancel",
          QMessageBox::RejectRole);
      msgBox.addButton("Don't Save, Exit", QMessageBox::DestructiveRole);
      QPushButton *saveButton = msgBox.addButton("Save and Exit",
          QMessageBox::AcceptRole);
      msgBox.setDefaultButton(cancelButton);
      msgBox.setDefaultButton(saveButton);

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
  if (this->dataPtr->currentSaveState != NEVER_SAVED)
    this->FinishModel();
  else
    this->SetModelVisible(this->dataPtr->serverModelName, true);

  this->Reset();

  gui::model::Events::newModel();
  gui::model::Events::finishModel();
}

/////////////////////////////////////////////////
void ModelCreator::OnPropertiesChanged(const bool _static,
    const bool _autoDisable)
{
  this->dataPtr->autoDisable = _autoDisable;
  this->dataPtr->isStatic = _static;
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::SaveModelFiles()
{
  this->dataPtr->saveDialog->GenerateConfig();
  this->dataPtr->saveDialog->SaveToConfig();
  this->GenerateSDF();
  this->dataPtr->saveDialog->SaveToSDF(this->dataPtr->modelSDF);
  this->dataPtr->currentSaveState = ALL_SAVED;
}

/////////////////////////////////////////////////
std::string ModelCreator::CreateModel()
{
  this->Reset();
  return this->dataPtr->folderName;
}

/////////////////////////////////////////////////
void ModelCreator::AddJoint(const std::string &_type)
{
  this->Stop();
  if (this->dataPtr->jointMaker)
    this->dataPtr->jointMaker->AddJoint(_type);
}

/////////////////////////////////////////////////
std::string ModelCreator::AddShape(const EntityType _type,
    const ignition::math::Vector3d &_size, const ignition::math::Pose3d &_pose,
    const std::string &_uri, const unsigned int _samples)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  std::stringstream linkNameStream;
  linkNameStream << this->dataPtr->previewName << "_" <<
      this->dataPtr->modelCounter << "::link_" << this->dataPtr->linkCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(linkName,
      this->dataPtr->previewVisual));
  linkVisual->Load();
  linkVisual->SetTransparency(ModelData::GetEditTransparency());

  std::ostringstream visualName;
  visualName << linkName << "::visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
      linkVisual));
  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->Root()
      ->GetElement("model")->GetElement("link")->GetElement("visual");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();

  if (_type == ENTITY_CYLINDER)
  {
    sdf::ElementPtr cylinderElem = geomElem->AddElement("cylinder");
    (cylinderElem->GetElement("radius"))->Set(_size.X()*0.5);
    (cylinderElem->GetElement("length"))->Set(_size.Z());
  }
  else if (_type == ENTITY_SPHERE)
  {
    ((geomElem->AddElement("sphere"))->GetElement("radius"))->
        Set(_size.X()*0.5);
  }
  else if (_type == ENTITY_MESH)
  {
    sdf::ElementPtr meshElem = geomElem->AddElement("mesh");
    meshElem->GetElement("scale")->Set(_size);
    meshElem->GetElement("uri")->Set(_uri);
  }
  else if (_type == ENTITY_POLYLINE)
  {
    QFileInfo info(QString::fromStdString(_uri));
    if (!info.isFile() || info.completeSuffix().toLower() != "svg")
    {
      gzerr << "File [" << _uri << "] not found or invalid!" << std::endl;
      return std::string();
    }

    common::SVGLoader svgLoader(_samples);
    std::vector<common::SVGPath> paths;
    svgLoader.Parse(_uri, paths);

    if (paths.empty())
    {
      gzerr << "No paths found on file [" << _uri << "]" << std::endl;
      return std::string();
    }

    // SVG paths do not map to sdf polylines, because we now allow a contour
    // to be made of multiple svg disjoint paths.
    // For this reason, we compute the closed polylines that can be extruded
    // in this step
    std::vector< std::vector<ignition::math::Vector2d> > closedPolys;
    std::vector< std::vector<ignition::math::Vector2d> > openPolys;
    svgLoader.PathsToClosedPolylines(paths, 0.05, closedPolys, openPolys);
    if (closedPolys.empty())
    {
      gzerr << "No closed polylines found on file [" << _uri << "]"
        << std::endl;
      return std::string();
    }
    if (!openPolys.empty())
    {
      gzmsg << "There are " << openPolys.size() << "open polylines. "
        << "They will be ignored." << std::endl;
    }
    // Find extreme values to center the polylines
    ignition::math::Vector2d min(paths[0].polylines[0][0]);
    ignition::math::Vector2d max(min);

    for (auto const &poly : closedPolys)
    {
      for (auto const &pt : poly)
      {
        if (pt.X() < min.X())
          min.X() = pt.X();
        if (pt.Y() < min.Y())
          min.Y() = pt.Y();
        if (pt.X() > max.X())
          max.X() = pt.X();
        if (pt.Y() > max.Y())
          max.Y() = pt.Y();
      }
    }
    for (auto const &poly : closedPolys)
    {
      sdf::ElementPtr polylineElem = geomElem->AddElement("polyline");
      polylineElem->GetElement("height")->Set(_size.Z());

      for (auto const &p : poly)
      {
        // Translate to center
        ignition::math::Vector2d pt = p - min - (max-min)*0.5;
        // Swap X and Y so Z will point up
        // (in 2D it points into the screen)
        sdf::ElementPtr pointElem = polylineElem->AddElement("point");
        pointElem->Set(
            ignition::math::Vector2d(pt.Y()*_size.Y(), pt.X()*_size.X()));
      }
    }
  }
  else
  {
    if (_type != ENTITY_BOX)
    {
      gzwarn << "Unknown link type '" << _type << "'. " <<
          "Adding a box" << std::endl;
    }

    ((geomElem->AddElement("box"))->GetElement("size"))->Set(_size);
  }

  visVisual->Load(visualElem);
  this->CreateLink(visVisual);
  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  linkVisual->SetPose(_pose);

  // insert over ground plane for now
  auto linkPos = linkVisual->GetWorldPose().Ign().Pos();
  if (_type == ENTITY_BOX || _type == ENTITY_CYLINDER || _type == ENTITY_SPHERE)
  {
    linkPos.Z() = _size.Z() * 0.5;
  }
  // override orientation as it's more natural to insert objects upright rather
  // than inserting it in the model frame.
  linkVisual->SetWorldPose(ignition::math::Pose3d(linkPos,
      ignition::math::Quaterniond()));

  this->dataPtr->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
void ModelCreator::CreateLink(const rendering::VisualPtr &_visual)
{
  LinkData *link = new LinkData();

  msgs::Model model;
  double mass = 1.0;

  // set reasonable inertial values based on geometry
  std::string geomType = _visual->GetGeometryType();
  if (geomType == "cylinder")
    msgs::AddCylinderLink(model, mass, 0.5, 1.0);
  else if (geomType == "sphere")
    msgs::AddSphereLink(model, mass, 0.5);
  else
    msgs::AddBoxLink(model, mass, ignition::math::Vector3d::One);
  link->Load(msgs::LinkToSDF(model.link(0)));

  MainWindow *mainWindow = gui::get_main_window();
  if (mainWindow)
  {
    connect(gui::get_main_window(), SIGNAL(Close()), link->inspector,
        SLOT(close()));
  }

  link->linkVisual = _visual->GetParent();
  link->AddVisual(_visual);

  link->inspector->SetLinkId(link->linkVisual->GetName());

  // override transparency
  _visual->SetTransparency(_visual->GetTransparency() *
      (1-ModelData::GetEditTransparency()-0.1)
      + ModelData::GetEditTransparency());

  // create collision with identical geometry
  rendering::VisualPtr collisionVis =
      _visual->Clone(link->linkVisual->GetName() + "::collision",
      link->linkVisual);

  // orange
  collisionVis->SetMaterial("Gazebo/Orange");
  collisionVis->SetTransparency(
      ignition::math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
  // fix for transparency alpha compositing
  Ogre::MovableObject *colObj = collisionVis->GetSceneNode()->
      getAttachedObject(0);
  colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);
  link->AddCollision(collisionVis);

  std::string linkName = link->linkVisual->GetName();

  std::string leafName = linkName;
  size_t idx = linkName.rfind("::");
  if (idx != std::string::npos)
    leafName = linkName.substr(idx+2);

  link->SetName(leafName);

  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->allLinks[linkName] = link;
    if (this->dataPtr->canonicalLink.empty())
      this->dataPtr->canonicalLink = linkName;
  }

  rendering::ScenePtr scene = link->linkVisual->GetScene();

  this->ModelChanged();
}

/////////////////////////////////////////////////
LinkData *ModelCreator::CloneLink(const std::string &_linkName)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  auto it = this->dataPtr->allLinks.find(_linkName);
  if (it == this->dataPtr->allLinks.end())
  {
    gzerr << "No link with name: " << _linkName << " found."  << std::endl;
    return NULL;
  }

  // generate unique name.
  std::string newName = _linkName + "_clone";
  auto itName = this->dataPtr->allLinks.find(newName);
  int nameCounter = 0;
  while (itName != this->dataPtr->allLinks.end())
  {
    std::stringstream newLinkName;
    newLinkName << _linkName << "_clone_" << nameCounter++;
    newName = newLinkName.str();
    itName = this->dataPtr->allLinks.find(newName);
  }

  std::string leafName = newName;
  size_t idx = newName.rfind("::");
  if (idx != std::string::npos)
    leafName = newName.substr(idx+2);
  LinkData *link = it->second->Clone(leafName);

  this->dataPtr->allLinks[newName] = link;

  this->ModelChanged();

  return link;
}

/////////////////////////////////////////////////
LinkData *ModelCreator::CreateLinkFromSDF(const sdf::ElementPtr &_linkElem,
    const rendering::VisualPtr &_parentVis)
{
  LinkData *link = new LinkData();
  MainWindow *mainWindow = gui::get_main_window();
  if (mainWindow)
  {
    connect(gui::get_main_window(), SIGNAL(Close()), link->inspector,
        SLOT(close()));
  }

  link->Load(_linkElem);

  // Link
  std::stringstream linkNameStream;
  std::string leafName = link->GetName();

  if (_parentVis->GetName() == this->dataPtr->previewName)
  {
    linkNameStream << this->dataPtr->previewName << "_" <<
        this->dataPtr->modelCounter << "::";
  }
  else
  {
    linkNameStream << _parentVis->GetName() << "::";
  }
  linkNameStream << leafName;
  std::string linkName = linkNameStream.str();

  if (this->dataPtr->canonicalLink.empty())
    this->dataPtr->canonicalLink = linkName;

  link->SetName(leafName);

  // if link name is scoped, it could mean that it's from an included model.
  // The joint maker needs to know about this in order to specify the correct
  // parent and child links in sdf generation step.
  if (leafName.find("::") != std::string::npos)
    this->dataPtr->jointMaker->AddScopedLinkName(leafName);

  rendering::VisualPtr linkVisual(new rendering::Visual(linkName, _parentVis));
  linkVisual->Load();
  linkVisual->SetPose(link->Pose());
  link->linkVisual = linkVisual;
  link->inspector->SetLinkId(link->linkVisual->GetName());

  // Visuals
  int visualIndex = 0;
  sdf::ElementPtr visualElem;

  if (_linkElem->HasElement("visual"))
    visualElem = _linkElem->GetElement("visual");

  linkVisual->SetTransparency(ModelData::GetEditTransparency());

  while (visualElem)
  {
    // Visual name
    std::string visualName;
    if (visualElem->HasAttribute("name"))
    {
      visualName = linkName + "::" + visualElem->Get<std::string>("name");
      visualIndex++;
    }
    else
    {
      std::stringstream visualNameStream;
      visualNameStream << linkName << "::visual_" << visualIndex++;
      visualName = visualNameStream.str();
      gzwarn << "SDF missing visual name attribute. Created name " << visualName
          << std::endl;
    }
    rendering::VisualPtr visVisual(new rendering::Visual(visualName,
        linkVisual));
    visVisual->Load(visualElem);

    // Visual pose
    ignition::math::Pose3d visualPose;
    if (visualElem->HasElement("pose"))
      visualPose = visualElem->Get<ignition::math::Pose3d>("pose");
    else
      visualPose.Set(0, 0, 0, 0, 0, 0);
    visVisual->SetPose(visualPose);

    // Add to link
    link->AddVisual(visVisual);

    // override transparency
    visVisual->SetTransparency(visVisual->GetTransparency() *
        (1-ModelData::GetEditTransparency()-0.1)
        + ModelData::GetEditTransparency());

    visualElem = visualElem->GetNextElement("visual");
  }

  // Collisions
  int collisionIndex = 0;
  sdf::ElementPtr collisionElem;

  if (_linkElem->HasElement("collision"))
    collisionElem = _linkElem->GetElement("collision");

  while (collisionElem)
  {
    // Collision name
    std::string collisionName;
    if (collisionElem->HasAttribute("name"))
    {
      collisionName = linkName + "::" + collisionElem->Get<std::string>("name");
      collisionIndex++;
    }
    else
    {
      std::ostringstream collisionNameStream;
      collisionNameStream << linkName << "::collision_" << collisionIndex++;
      collisionName = collisionNameStream.str();
      gzwarn << "SDF missing collision name attribute. Created name " <<
          collisionName << std::endl;
    }
    rendering::VisualPtr colVisual(new rendering::Visual(collisionName,
        linkVisual));

    // Collision pose
    ignition::math::Pose3d collisionPose;
    if (collisionElem->HasElement("pose"))
      collisionPose = collisionElem->Get<ignition::math::Pose3d>("pose");
    else
      collisionPose.Set(0, 0, 0, 0, 0, 0);

    // Make a visual element from the collision element
    sdf::ElementPtr colVisualElem =  this->dataPtr->modelTemplateSDF->Root()
        ->GetElement("model")->GetElement("link")->GetElement("visual");

    sdf::ElementPtr geomElem = colVisualElem->GetElement("geometry");
    geomElem->ClearElements();
    geomElem->Copy(collisionElem->GetElement("geometry"));

    colVisual->Load(colVisualElem);
    colVisual->SetPose(collisionPose);
    colVisual->SetMaterial("Gazebo/Orange");
    colVisual->SetTransparency(ignition::math::clamp(
        ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
    // fix for transparency alpha compositing
    Ogre::MovableObject *colObj = colVisual->GetSceneNode()->
        getAttachedObject(0);
    colObj->setRenderQueueGroup(colObj->getRenderQueueGroup()+1);

    // Add to link
    msgs::Collision colMsg = msgs::CollisionFromSDF(collisionElem);
    link->AddCollision(colVisual, &colMsg);

    collisionElem = collisionElem->GetNextElement("collision");
  }

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Top-level links only
  if (_parentVis == this->dataPtr->previewVisual)
    gui::model::Events::linkInserted(linkName);

  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->allLinks[linkName] = link;
  }

  this->ModelChanged();

  return link;
}

/////////////////////////////////////////////////
void ModelCreator::RemoveNestedModelImpl(const std::string &_nestedModelName)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
    return;
  }

  NestedModelData *modelData = NULL;
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    if (this->dataPtr->allNestedModels.find(_nestedModelName) ==
        this->dataPtr->allNestedModels.end())
    {
      return;
    }
    modelData = this->dataPtr->allNestedModels[_nestedModelName];
  }

  if (!modelData)
    return;

  // Copy before reference is deleted.
  std::string nestedModelName(_nestedModelName);

  // remove all its models
  for (auto &modelIt : modelData->models)
    this->RemoveNestedModelImpl(modelIt.first);

  // remove all its links and joints
  for (auto &linkIt : modelData->links)
  {
    // if it's a link
    if (this->dataPtr->allLinks.find(linkIt.first) !=
        this->dataPtr->allLinks.end())
    {
      if (this->dataPtr->jointMaker)
      {
        this->dataPtr->jointMaker->RemoveJointsByLink(linkIt.first);
      }
      this->RemoveLinkImpl(linkIt.first);
    }
  }

  rendering::ScenePtr scene = modelData->modelVisual->GetScene();
  if (scene)
  {
    scene->RemoveVisual(modelData->modelVisual);
  }

  modelData->modelVisual.reset();
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->allNestedModels.erase(_nestedModelName);
    delete modelData;
  }
  gui::model::Events::nestedModelRemoved(nestedModelName);

  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::RemoveLinkImpl(const std::string &_linkName)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
    return;
  }

  LinkData *link = NULL;
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    auto linkIt = this->dataPtr->allLinks.find(_linkName);
    if (linkIt == this->dataPtr->allLinks.end())
      return;
    link = linkIt->second;
  }

  if (!link)
    return;

  // Copy before reference is deleted.
  std::string linkName(_linkName);

  rendering::ScenePtr scene = link->linkVisual->GetScene();
  if (scene)
  {
    for (auto &it : link->visuals)
    {
      rendering::VisualPtr vis = it.first;
      scene->RemoveVisual(vis);
    }
    scene->RemoveVisual(link->linkVisual);
    for (auto &colIt : link->collisions)
    {
      rendering::VisualPtr vis = colIt.first;
      scene->RemoveVisual(vis);
    }

    scene->RemoveVisual(link->linkVisual);
  }

  link->linkVisual.reset();
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->allLinks.erase(linkName);
    delete link;
  }
  gui::model::Events::linkRemoved(linkName);

  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::Reset()
{
  delete this->dataPtr->saveDialog;
  this->dataPtr->saveDialog = new SaveDialog(SaveDialog::MODEL);

  this->dataPtr->jointMaker->Reset();
  this->dataPtr->selectedLinks.clear();

  if (g_copyAct)
    g_copyAct->setEnabled(false);

  if (g_pasteAct)
    g_pasteAct->setEnabled(false);

  this->dataPtr->currentSaveState = NEVER_SAVED;
  this->SetModelName(this->dataPtr->modelDefaultName);
  this->dataPtr->serverModelName = "";
  this->dataPtr->serverModelSDF.reset();
  this->dataPtr->serverModelVisible.clear();
  this->dataPtr->canonicalLink = "";

  this->dataPtr->modelTemplateSDF.reset(new sdf::SDF);
  this->dataPtr->modelTemplateSDF->SetFromString(
      ModelData::GetTemplateSDFString());

  this->dataPtr->modelSDF.reset(new sdf::SDF);

  this->dataPtr->isStatic = false;
  this->dataPtr->autoDisable = true;
  gui::model::Events::modelPropertiesChanged(this->dataPtr->isStatic,
      this->dataPtr->autoDisable);
  gui::model::Events::modelNameChanged(this->ModelName());

  while (!this->dataPtr->allLinks.empty())
    this->RemoveLinkImpl(this->dataPtr->allLinks.begin()->first);
  this->dataPtr->allLinks.clear();

  while (!this->dataPtr->allNestedModels.empty())
    this->RemoveNestedModelImpl(this->dataPtr->allNestedModels.begin()->first);
  this->dataPtr->allNestedModels.clear();

  this->dataPtr->allModelPlugins.clear();

  if (!gui::get_active_camera() ||
    !gui::get_active_camera()->GetScene())
  return;

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (this->dataPtr->previewVisual)
    scene->RemoveVisual(this->dataPtr->previewVisual);

  this->dataPtr->previewVisual.reset(new rendering::Visual(
      this->dataPtr->previewName, scene->GetWorldVisual()));

  this->dataPtr->previewVisual->Load();
  this->dataPtr->modelPose = ignition::math::Pose3d::Zero;
  this->dataPtr->previewVisual->SetPose(this->dataPtr->modelPose);
}

/////////////////////////////////////////////////
void ModelCreator::SetModelName(const std::string &_modelName)
{
  this->dataPtr->modelName = _modelName;
  this->dataPtr->saveDialog->SetModelName(_modelName);

  this->dataPtr->folderName = this->dataPtr->saveDialog->
      GetFolderNameFromModelName(this->dataPtr->modelName);

  if (this->dataPtr->currentSaveState == NEVER_SAVED)
  {
    // Set new saveLocation
    boost::filesystem::path oldPath(
        this->dataPtr->saveDialog->GetSaveLocation());

    auto newPath = oldPath.parent_path() / this->dataPtr->folderName;
    this->dataPtr->saveDialog->SetSaveLocation(newPath.string());
  }
}

/////////////////////////////////////////////////
std::string ModelCreator::ModelName() const
{
  return this->dataPtr->modelName;
}

/////////////////////////////////////////////////
void ModelCreator::SetStatic(const bool _static)
{
  this->dataPtr->isStatic = _static;
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::SetAutoDisable(const bool _auto)
{
  this->dataPtr->autoDisable = _auto;
  this->ModelChanged();
}

/////////////////////////////////////////////////
void ModelCreator::FinishModel()
{
  if (!this->dataPtr->serverModelName.empty())
  {
    // delete model on server first before spawning the updated one.
    transport::request(gui::get_world(), "entity_delete",
        this->dataPtr->serverModelName);
    int timeoutCounter = 0;
    int timeout = 100;
    while (timeoutCounter < timeout)
    {
      boost::shared_ptr<msgs::Response> response =
          transport::request(gui::get_world(), "entity_info",
          this->dataPtr->serverModelName);
      // Make sure the response is correct
      if (response->response() == "nonexistent")
        break;

      common::Time::MSleep(100);
      timeoutCounter++;
    }
  }
  event::Events::setSelectedEntity("", "normal");
  this->CreateTheEntity();
  this->Reset();
}

/////////////////////////////////////////////////
void ModelCreator::CreateTheEntity()
{
  if (!this->dataPtr->modelSDF->Root()->HasElement("model"))
  {
    gzerr << "Generated invalid SDF! Cannot create entity." << std::endl;
    return;
  }

  msgs::Factory msg;
  // Create a new name if the model exists
  auto modelElem = this->dataPtr->modelSDF->Root()->GetElement("model");
  std::string modelElemName = modelElem->Get<std::string>("name");
  if (has_entity_name(modelElemName))
  {
    int i = 0;
    while (has_entity_name(modelElemName))
    {
      modelElemName = modelElem->Get<std::string>("name") + "_" +
        boost::lexical_cast<std::string>(i++);
    }
    modelElem->GetAttribute("name")->Set(modelElemName);
  }

  msg.set_sdf(this->dataPtr->modelSDF->ToString());
  msgs::Set(msg.mutable_pose(), this->dataPtr->modelPose);
  this->dataPtr->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelCreator::AddEntity(const sdf::ElementPtr &_sdf)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  this->Stop();

  if (_sdf->GetName() == "model")
  {
    // Create a top-level nested model
    NestedModelData *modelData =
        this->CreateModelFromSDF(_sdf, this->dataPtr->previewVisual, false);

    this->dataPtr->addEntityType = ENTITY_MODEL;
    rendering::VisualPtr entityVisual = modelData->modelVisual;

    this->dataPtr->mouseVisual = entityVisual;
  }
}

/////////////////////////////////////////////////
void ModelCreator::AddLink(const EntityType _type)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  this->Stop();

  this->dataPtr->addEntityType = _type;
  if (_type != ENTITY_NONE)
    this->AddShape(_type);
}

/////////////////////////////////////////////////
void ModelCreator::Stop()
{
  if (this->dataPtr->addEntityType != ENTITY_NONE && this->dataPtr->mouseVisual)
  {
    this->RemoveEntity(this->dataPtr->mouseVisual->GetName());
    this->dataPtr->mouseVisual.reset();
    emit LinkAdded();
  }
  if (this->dataPtr->jointMaker)
    this->dataPtr->jointMaker->Stop();
}

/////////////////////////////////////////////////
void ModelCreator::OnDelete()
{
  if (this->dataPtr->inspectName.empty())
    return;

  this->OnDelete(this->dataPtr->inspectName);
  this->dataPtr->inspectName = "";
}

/////////////////////////////////////////////////
void ModelCreator::OnDelete(const std::string &_entity)
{
  this->RemoveEntity(_entity);
}

/////////////////////////////////////////////////
void ModelCreator::RemoveEntity(const std::string &_entity)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  // if it's a nestedModel
  if (this->dataPtr->allNestedModels.find(_entity) !=
      this->dataPtr->allNestedModels.end())
  {
    this->RemoveNestedModelImpl(_entity);
    return;
  }

  // if it's a link
  if (this->dataPtr->allLinks.find(_entity) != this->dataPtr->allLinks.end())
  {
    if (this->dataPtr->jointMaker)
      this->dataPtr->jointMaker->RemoveJointsByLink(_entity);
    this->RemoveLinkImpl(_entity);
    return;
  }

  // if it's a visual
  rendering::VisualPtr vis =
      gui::get_active_camera()->GetScene()->GetVisual(_entity);
  if (vis)
  {
    rendering::VisualPtr parentLink = vis->GetParent();
    std::string parentLinkName = parentLink->GetName();

    if (this->dataPtr->allLinks.find(parentLinkName) !=
        this->dataPtr->allLinks.end())
    {
      // remove the parent link if it's the only child
      if (parentLink->GetChildCount() == 1)
      {
        if (this->dataPtr->jointMaker)
          this->dataPtr->jointMaker->RemoveJointsByLink(parentLink->GetName());
        this->RemoveLinkImpl(parentLink->GetName());
        return;
      }
    }
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnRemoveModelPlugin(const QString &_name)
{
  this->RemoveModelPlugin(_name.toStdString());
}

/////////////////////////////////////////////////
void ModelCreator::RemoveModelPlugin(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  auto it = this->dataPtr->allModelPlugins.find(_name);
  if (it == this->dataPtr->allModelPlugins.end())
  {
    return;
  }

  ModelPluginData *data = it->second;

  // Remove from map
  this->dataPtr->allModelPlugins.erase(_name);
  delete data;

  // Notify removal
  gui::model::Events::modelPluginRemoved(_name);
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
    if (!this->dataPtr->selectedLinks.empty())
    {
      for (const auto &linkVis : this->dataPtr->selectedLinks)
      {
        this->OnDelete(linkVis->GetName());
      }
      this->DeselectAll();
    }
    else if (!this->dataPtr->selectedModelPlugins.empty())
    {
      for (const auto &plugin : this->dataPtr->selectedModelPlugins)
      {
        this->RemoveModelPlugin(plugin);
      }
      this->DeselectAll();
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

  if (this->dataPtr->jointMaker->GetState() != JointMaker::JOINT_NONE)
  {
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  rendering::VisualPtr vis = userCamera->GetVisual(_event.Pos());
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

  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  if (this->dataPtr->mouseVisual)
  {
    if (_event.Button() == common::MouseEvent::RIGHT)
      return true;

    // set the link data pose
    auto linkIt = this->dataPtr->allLinks.find(
        this->dataPtr->mouseVisual->GetName());
    if (linkIt != this->dataPtr->allLinks.end())
    {
      LinkData *link = linkIt->second;
      link->SetPose(this->dataPtr->mouseVisual->GetWorldPose().Ign() -
          this->dataPtr->modelPose);
      gui::model::Events::linkInserted(this->dataPtr->mouseVisual->GetName());
    }
    else
    {
      auto modelIt = this->dataPtr->allNestedModels.find(
          this->dataPtr->mouseVisual->GetName());
      if (modelIt != this->dataPtr->allNestedModels.end())
      {
        NestedModelData *modelData = modelIt->second;
        modelData->SetPose(this->dataPtr->mouseVisual->GetWorldPose().Ign() -
            this->dataPtr->modelPose);

        this->EmitNestedModelInsertedEvent(this->dataPtr->mouseVisual);
      }
    }

    // reset and return
    emit LinkAdded();
    this->dataPtr->mouseVisual.reset();
    this->AddLink(ENTITY_NONE);
    return true;
  }

  rendering::VisualPtr vis = userCamera->GetVisual(_event.Pos());
  if (vis)
  {
    rendering::VisualPtr topLevelVis = vis->GetNthAncestor(2);
    if (!topLevelVis)
      return false;

    // Is link
    if (this->dataPtr->allLinks.find(topLevelVis->GetName()) !=
        this->dataPtr->allLinks.end())
    {
      // Handle snap from GLWidget
      if (g_snapAct->isChecked())
        return false;

      // trigger link inspector on right click
      if (_event.Button() == common::MouseEvent::RIGHT)
      {
        this->dataPtr->inspectName = topLevelVis->GetName();

        this->ShowContextMenu(this->dataPtr->inspectName);
        return true;
      }

      // Not in multi-selection mode.
      if (!(QApplication::keyboardModifiers() & Qt::ControlModifier))
      {
        this->DeselectAll();
        this->SetSelected(topLevelVis, true);
      }
      // Multi-selection mode
      else
      {
        this->DeselectAllModelPlugins();

        auto it = std::find(this->dataPtr->selectedLinks.begin(),
            this->dataPtr->selectedLinks.end(), topLevelVis);
        // Highlight and select clicked link if not already selected
        if (it == this->dataPtr->selectedLinks.end())
        {
          this->SetSelected(topLevelVis, true);
        }
        // Deselect if already selected
        else
        {
          this->SetSelected(topLevelVis, false);
        }
      }

      if (this->dataPtr->manipMode == "translate" ||
          this->dataPtr->manipMode == "rotate" ||
          this->dataPtr->manipMode == "scale")
      {
        this->OnManipMode(this->dataPtr->manipMode);
      }

      return true;
    }
    // Not link
    else
    {
      this->DeselectAll();

      g_alignAct->setEnabled(false);
      g_copyAct->setEnabled(!this->dataPtr->selectedLinks.empty());

      if (!vis->IsPlane())
        return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
void ModelCreator::EmitNestedModelInsertedEvent(
    const rendering::VisualPtr &_vis) const
{
  if (!_vis)
    return;

  auto modelIt = this->dataPtr->allNestedModels.find(_vis->GetName());
  if (modelIt != this->dataPtr->allNestedModels.end())
    gui::model::Events::nestedModelInserted(_vis->GetName());
  else
    return;

  for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
    this->EmitNestedModelInsertedEvent(_vis->GetChild(i));
}

/////////////////////////////////////////////////
void ModelCreator::ShowContextMenu(const std::string &_link)
{
  auto it = this->dataPtr->allLinks.find(_link);
  if (it == this->dataPtr->allLinks.end())
    return;

  // disable interacting with nested links for now
  LinkData *link = it->second;
  if (link->nested)
    return;

  this->dataPtr->inspectName = _link;
  QMenu menu;
  if (this->dataPtr->inspectAct)
  {
    menu.addAction(this->dataPtr->inspectAct);

    if (this->dataPtr->jointMaker)
    {
      std::vector<JointData *> joints =
          this->dataPtr->jointMaker->GetJointDataByLink(_link);

      if (!joints.empty())
      {
        QMenu *jointsMenu = menu.addMenu(tr("Open Joint Inspector"));

        for (auto joint : joints)
        {
          QAction *jointAct = new QAction(tr(joint->name.c_str()), this);
          connect(jointAct, SIGNAL(triggered()), joint,
              SLOT(OnOpenInspector()));
          jointsMenu->addAction(jointAct);
        }
      }
    }
  }
  QAction *deleteAct = new QAction(tr("Delete"), this);
  connect(deleteAct, SIGNAL(triggered()), this, SLOT(OnDelete()));
  menu.addAction(deleteAct);

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void ModelCreator::ShowModelPluginContextMenu(const std::string &_name)
{
  auto it = this->dataPtr->allModelPlugins.find(_name);
  if (it == this->dataPtr->allModelPlugins.end())
    return;

  // Open inspector
  QAction *inspectorAct = new QAction(tr("Open Model Plugin Inspector"), this);

  // Map signals to pass argument
  QSignalMapper *inspectorMapper = new QSignalMapper(this);

  connect(inspectorAct, SIGNAL(triggered()), inspectorMapper, SLOT(map()));
  inspectorMapper->setMapping(inspectorAct, QString::fromStdString(_name));

  connect(inspectorMapper, SIGNAL(mapped(QString)), this,
      SLOT(OnOpenModelPluginInspector(QString)));

  // Delete
  QAction *deleteAct = new QAction(tr("Delete"), this);

  // Map signals to pass argument
  QSignalMapper *deleteMapper = new QSignalMapper(this);

  connect(deleteAct, SIGNAL(triggered()), deleteMapper, SLOT(map()));
  deleteMapper->setMapping(deleteAct, QString::fromStdString(_name));

  connect(deleteMapper, SIGNAL(mapped(QString)), this,
      SLOT(OnRemoveModelPlugin(QString)));

  // Menu
  QMenu menu;
  menu.addAction(inspectorAct);
  menu.addAction(deleteAct);

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseMove(const common::MouseEvent &_event)
{
  this->dataPtr->lastMouseEvent = _event;
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  if (!this->dataPtr->mouseVisual)
  {
    rendering::VisualPtr vis = userCamera->GetVisual(_event.Pos());
    if (vis && !vis->IsPlane())
    {
      rendering::VisualPtr topLevelVis = vis->GetNthAncestor(2);
      if (!topLevelVis)
        return false;

      // Main window models always handled here
      // Not possible to interact with nested models yet
      if (this->dataPtr->allLinks.find(topLevelVis->GetName()) ==
          this->dataPtr->allLinks.end())
      {
        // Prevent highlighting for snapping
        if (this->dataPtr->manipMode == "snap" ||
            this->dataPtr->manipMode == "select" ||
            this->dataPtr->manipMode == "")
        {
          // Don't change cursor on hover
          QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
          userCamera->HandleMouseEvent(_event);
        }
        // Allow ModelManipulator to work while dragging handle over this
        else if (_event.Dragging())
        {
          ModelManipulator::Instance()->OnMouseMoveEvent(_event);
        }
        return true;
      }
    }
    return false;
  }

  auto pose = this->dataPtr->mouseVisual->GetWorldPose().Ign();
  pose.Pos() = ModelManipulator::GetMousePositionOnPlane(
      userCamera, _event).Ign();

  if (!_event.Shift())
  {
    pose.Pos() = ModelManipulator::SnapPoint(pose.Pos()).Ign();
  }
  pose.Pos().Z(this->dataPtr->mouseVisual->GetWorldPose().Ign().Pos().Z());

  this->dataPtr->mouseVisual->SetWorldPose(pose);

  return true;
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseDoubleClick(const common::MouseEvent &_event)
{
  // open the link inspector on double click
  rendering::VisualPtr vis = gui::get_active_camera()->GetVisual(_event.Pos());
  if (!vis)
    return false;

  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  auto it = this->dataPtr->allLinks.find(vis->GetParent()->GetName());
  if (it != this->dataPtr->allLinks.end())
  {
    this->OpenInspector(vis->GetParent()->GetName());
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
void ModelCreator::OnOpenInspector()
{
  if (this->dataPtr->inspectName.empty())
    return;

  this->OpenInspector(this->dataPtr->inspectName);
  this->dataPtr->inspectName = "";
}

/////////////////////////////////////////////////
void ModelCreator::OpenInspector(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
  auto it = this->dataPtr->allLinks.find(_name);
  if (it == this->dataPtr->allLinks.end())
  {
    gzerr << "Link [" << _name << "] not found." << std::endl;
    return;
  }

  // disable interacting with nested links for now
  LinkData *link = it->second;
  if (link->nested)
    return;

  link->SetPose(link->linkVisual->GetWorldPose().Ign() -
      this->dataPtr->modelPose);
  link->UpdateConfig();
  link->inspector->Open();
}

/////////////////////////////////////////////////
void ModelCreator::OnCopy()
{
  if (!g_editModelAct->isChecked())
    return;

  if (!this->dataPtr->selectedLinks.empty())
  {
    this->dataPtr->copiedLinkNames.clear();
    for (auto vis : this->dataPtr->selectedLinks)
    {
      this->dataPtr->copiedLinkNames.push_back(vis->GetName());
    }
    g_pasteAct->setEnabled(true);
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnPaste()
{
  if (this->dataPtr->copiedLinkNames.empty() || !g_editModelAct->isChecked())
  {
    return;
  }

  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  // For now, only copy the last selected model
  auto it = this->dataPtr->allLinks.find(this->dataPtr->copiedLinkNames.back());
  if (it != this->dataPtr->allLinks.end())
  {
    LinkData *copiedLink = it->second;
    if (!copiedLink)
      return;

    this->Stop();
    this->DeselectAll();

    if (!this->dataPtr->previewVisual)
    {
      this->Reset();
    }

    LinkData* clonedLink = this->CloneLink(it->first);

    auto clonePose = copiedLink->linkVisual->GetWorldPose().Ign();
    rendering::UserCameraPtr userCamera = gui::get_active_camera();
    if (userCamera)
    {
      auto mousePosition = ModelManipulator::GetMousePositionOnPlane(
          userCamera, this->dataPtr->lastMouseEvent).Ign();
      clonePose.Pos().X(mousePosition.X());
      clonePose.Pos().Y(mousePosition.Y());
    }

    clonedLink->linkVisual->SetWorldPose(clonePose);
    this->dataPtr->addEntityType = ENTITY_MESH;
    this->dataPtr->mouseVisual = clonedLink->linkVisual;
  }
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

  this->dataPtr->modelSDF.reset(new sdf::SDF);
  this->dataPtr->modelSDF->SetFromString(ModelData::GetTemplateSDFString());

  modelElem = this->dataPtr->modelSDF->Root()->GetElement("model");

  modelElem->ClearElements();
  modelElem->GetAttribute("name")->Set(this->dataPtr->folderName);

  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  if (this->dataPtr->serverModelName.empty())
  {
    // set center of all links and nested models to be origin
    /// \todo issue #1485 set a better origin other than the centroid
    ignition::math::Vector3d mid;
    int entityCount = 0;
    for (auto &linksIt : this->dataPtr->allLinks)
    {
      LinkData *link = linksIt.second;
      if (link->nested)
        continue;
      mid += link->Pose().Pos();
      entityCount++;
    }
    for (auto &nestedModelsIt : this->dataPtr->allNestedModels)
    {
      NestedModelData *modelData = nestedModelsIt.second;

      // get only top level nested models
      if (modelData->Depth() != 2)
        continue;

      mid += modelData->Pose().Pos();
      entityCount++;
    }

    if (!(this->dataPtr->allLinks.empty() &&
          this->dataPtr->allNestedModels.empty()))
    {
      mid /= entityCount;
    }

    this->dataPtr->modelPose.Pos() = mid;
  }

  // Update poses in case they changed
  this->dataPtr->previewVisual->SetWorldPose(this->dataPtr->modelPose);
  for (auto &linksIt : this->dataPtr->allLinks)
  {
    LinkData *link = linksIt.second;
    if (link->nested)
      continue;
    link->SetPose(link->linkVisual->GetWorldPose().Ign() -
        this->dataPtr->modelPose);
    link->linkVisual->SetPose(link->Pose());
  }
  for (auto &nestedModelsIt : this->dataPtr->allNestedModels)
  {
    NestedModelData *modelData = nestedModelsIt.second;

    if (!modelData->modelVisual)
      continue;

    // get only top level nested models
    if (modelData->Depth() != 2)
      continue;

    modelData->SetPose(modelData->modelVisual->GetWorldPose().Ign() -
        this->dataPtr->modelPose);
    modelData->modelVisual->SetPose(modelData->Pose());
  }

  // generate canonical link sdf first.
  if (!this->dataPtr->canonicalLink.empty())
  {
    auto canonical = this->dataPtr->allLinks.find(this->dataPtr->canonicalLink);
    if (canonical != this->dataPtr->allLinks.end())
    {
      LinkData *link = canonical->second;
      if (!link->nested)
      {
        link->UpdateConfig();
        sdf::ElementPtr newLinkElem = this->GenerateLinkSDF(link);
        modelElem->InsertElement(newLinkElem);
      }
    }
  }

  // loop through rest of all links and generate sdf
  for (auto &linksIt : this->dataPtr->allLinks)
  {
    LinkData *link = linksIt.second;

    if (linksIt.first == this->dataPtr->canonicalLink || link->nested)
      continue;

    link->UpdateConfig();

    sdf::ElementPtr newLinkElem = this->GenerateLinkSDF(link);
    modelElem->InsertElement(newLinkElem);
  }

  // generate canonical model sdf first.
  if (!this->dataPtr->canonicalModel.empty())
  {
    auto canonical = this->dataPtr->allNestedModels.find(
        this->dataPtr->canonicalModel);
    if (canonical != this->dataPtr->allNestedModels.end())
    {
      NestedModelData *nestedModelData = canonical->second;
      modelElem->InsertElement(nestedModelData->modelSDF);
    }
  }

  // loop through rest of all nested models and add sdf
  for (auto &nestedModelsIt : this->dataPtr->allNestedModels)
  {
    NestedModelData *nestedModelData = nestedModelsIt.second;

    if (nestedModelsIt.first == this->dataPtr->canonicalModel ||
        nestedModelData->Depth() != 2)
      continue;

    modelElem->InsertElement(nestedModelData->modelSDF);
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

  // Add plugin elements
  for (auto modelPlugin : this->dataPtr->allModelPlugins)
    modelElem->InsertElement(modelPlugin.second->modelPluginSDF->Clone());
}

/////////////////////////////////////////////////
sdf::ElementPtr ModelCreator::GenerateLinkSDF(LinkData *_link)
{
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;
  visualNameStream.str("");
  collisionNameStream.str("");

  sdf::ElementPtr newLinkElem = _link->linkSDF->Clone();
  newLinkElem->GetElement("pose")->Set(_link->linkVisual->GetWorldPose().Ign()
      - this->dataPtr->modelPose);

  // visuals
  for (auto const &it : _link->visuals)
  {
    rendering::VisualPtr visual = it.first;
    msgs::Visual visualMsg = it.second;
    sdf::ElementPtr visualElem = visual->GetSDF()->Clone();

    visualElem->GetElement("transparency")->Set<double>(
        visualMsg.transparency());
    newLinkElem->InsertElement(visualElem);
  }

  // collisions
  for (auto const &colIt : _link->collisions)
  {
    sdf::ElementPtr collisionElem = msgs::CollisionToSDF(colIt.second);
    newLinkElem->InsertElement(collisionElem);
  }
  return newLinkElem;
}

/////////////////////////////////////////////////
void ModelCreator::OnAlignMode(const std::string &_axis,
    const std::string &_config, const std::string &_target, const bool _preview,
    const bool _inverted)
{
  ModelAlign::Instance()->AlignVisuals(this->dataPtr->selectedLinks, _axis,
      _config, _target, !_preview, _inverted);
}

/////////////////////////////////////////////////
void ModelCreator::DeselectAll()
{
  this->DeselectAllLinks();
  this->DeselectAllModelPlugins();
}

/////////////////////////////////////////////////
void ModelCreator::DeselectAllLinks()
{
  while (!this->dataPtr->selectedLinks.empty())
  {
    rendering::VisualPtr vis = this->dataPtr->selectedLinks[0];
    vis->SetHighlighted(false);
    this->dataPtr->selectedLinks.erase(this->dataPtr->selectedLinks.begin());
    model::Events::setSelectedLink(vis->GetName(), false);
  }
}

/////////////////////////////////////////////////
void ModelCreator::DeselectAllModelPlugins()
{
  while (!this->dataPtr->selectedModelPlugins.empty())
  {
    auto it = this->dataPtr->selectedModelPlugins.begin();
    std::string name = this->dataPtr->selectedModelPlugins[0];
    this->dataPtr->selectedModelPlugins.erase(it);
    model::Events::setSelectedModelPlugin(name, false);
  }
}

/////////////////////////////////////////////////
void ModelCreator::SetSelected(const std::string &_name, const bool _selected)
{
  auto it = this->dataPtr->allLinks.find(_name);
  if (it == this->dataPtr->allLinks.end())
    return;

  this->SetSelected((*it).second->linkVisual, _selected);
}

/////////////////////////////////////////////////
void ModelCreator::SetSelected(const rendering::VisualPtr &_linkVis,
    const bool _selected)
{
  if (!_linkVis)
    return;

  _linkVis->SetHighlighted(_selected);
  auto it = std::find(this->dataPtr->selectedLinks.begin(),
      this->dataPtr->selectedLinks.end(), _linkVis);
  if (_selected)
  {
    if (it == this->dataPtr->selectedLinks.end())
    {
      this->dataPtr->selectedLinks.push_back(_linkVis);
      model::Events::setSelectedLink(_linkVis->GetName(), _selected);
    }
  }
  else
  {
    if (it != this->dataPtr->selectedLinks.end())
    {
      this->dataPtr->selectedLinks.erase(it);
      model::Events::setSelectedLink(_linkVis->GetName(), _selected);
    }
  }
  g_copyAct->setEnabled(!this->dataPtr->selectedLinks.empty());
  g_alignAct->setEnabled(this->dataPtr->selectedLinks.size() > 1);
}

/////////////////////////////////////////////////
void ModelCreator::OnManipMode(const std::string &_mode)
{
  if (!this->dataPtr->active)
    return;

  this->dataPtr->manipMode = _mode;

  if (!this->dataPtr->selectedLinks.empty())
  {
    ModelManipulator::Instance()->SetAttachedVisual(
        this->dataPtr->selectedLinks.back());
  }

  ModelManipulator::Instance()->SetManipulationMode(_mode);
  ModelSnap::Instance()->Reset();

  // deselect 0 to n-1 models.
  if (this->dataPtr->selectedLinks.size() > 1)
  {
    rendering::VisualPtr link =
        this->dataPtr->selectedLinks[this->dataPtr->selectedLinks.size()-1];
    this->DeselectAll();
    this->SetSelected(link, true);
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnSetSelectedEntity(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  this->DeselectAll();
}

/////////////////////////////////////////////////
void ModelCreator::OnSetSelectedLink(const std::string &_name,
    const bool _selected)
{
  this->SetSelected(_name, _selected);
}

/////////////////////////////////////////////////
void ModelCreator::OnSetSelectedModelPlugin(const std::string &_name,
    const bool _selected)
{
  auto plugin = this->dataPtr->allModelPlugins.find(_name);
  if (plugin == this->dataPtr->allModelPlugins.end())
    return;

  auto it = std::find(this->dataPtr->selectedModelPlugins.begin(),
      this->dataPtr->selectedModelPlugins.end(), _name);
  if (_selected && it == this->dataPtr->selectedModelPlugins.end())
  {
    this->dataPtr->selectedModelPlugins.push_back(_name);
  }
  else if (!_selected && it != this->dataPtr->selectedModelPlugins.end())
  {
    this->dataPtr->selectedModelPlugins.erase(it);
  }
}

/////////////////////////////////////////////////
void ModelCreator::ModelChanged()
{
  if (this->dataPtr->currentSaveState != NEVER_SAVED)
    this->dataPtr->currentSaveState = UNSAVED_CHANGES;
}

/////////////////////////////////////////////////
void ModelCreator::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  // Check if any links have been moved or resized and trigger ModelChanged
  for (auto &linksIt : this->dataPtr->allLinks)
  {
    LinkData *link = linksIt.second;
    if (link->Pose() != link->linkVisual->GetPose().Ign())
    {
      link->SetPose(link->linkVisual->GetWorldPose().Ign() -
          this->dataPtr->modelPose);
      this->ModelChanged();
    }
    for (auto &scaleIt : this->dataPtr->linkScaleUpdate)
    {
      if (link->linkVisual->GetName() == scaleIt.first)
        link->SetScale(scaleIt.second);
    }
  }
  if (!this->dataPtr->linkScaleUpdate.empty())
    this->ModelChanged();
  this->dataPtr->linkScaleUpdate.clear();
}

/////////////////////////////////////////////////
void ModelCreator::OnEntityScaleChanged(const std::string &_name,
  const ignition::math::Vector3d &_scale)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
  for (auto linksIt : this->dataPtr->allLinks)
  {
    std::string linkName;
    size_t pos = _name.rfind("::");
    if (pos != std::string::npos)
      linkName = _name.substr(0, pos);
    if (_name == linksIt.first || linkName == linksIt.first)
    {
      this->dataPtr->linkScaleUpdate[linksIt.first] = _scale;
      break;
    }
  }
}

/////////////////////////////////////////////////
void ModelCreator::SetModelVisible(const std::string &_name,
    const bool _visible)
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  rendering::VisualPtr visual = scene->GetVisual(_name);
  if (!visual)
    return;

  this->SetModelVisible(visual, _visible);

  if (_visible)
    visual->SetHighlighted(false);
}

/////////////////////////////////////////////////
void ModelCreator::SetModelVisible(const rendering::VisualPtr &_visual,
    const bool _visible)
{
  if (!_visual)
    return;

  for (unsigned int i = 0; i < _visual->GetChildCount(); ++i)
    this->SetModelVisible(_visual->GetChild(i), _visible);

  if (!_visible)
  {
    // store original visibility
    this->dataPtr->serverModelVisible[_visual->GetId()] = _visual->GetVisible();
    _visual->SetVisible(_visible);
  }
  else
  {
    // restore original visibility
    auto it = this->dataPtr->serverModelVisible.find(_visual->GetId());
    if (it != this->dataPtr->serverModelVisible.end())
    {
      _visual->SetVisible(it->second, false);
    }
  }
}

/////////////////////////////////////////////////
ModelCreator::SaveState ModelCreator::CurrentSaveState() const
{
  return this->dataPtr->currentSaveState;
}

/////////////////////////////////////////////////
void ModelCreator::OnAddModelPlugin(const std::string &_name,
    const std::string &_filename, const std::string &_innerxml)
{
  if (_name.empty() || _filename.empty())
  {
    gzerr << "Cannot add model plugin. Empty name or filename" << std::endl;
    return;
  }

  // Use the SDF parser to read all the inner xml.
  sdf::ElementPtr modelPluginSDF(new sdf::Element);
  sdf::initFile("plugin.sdf", modelPluginSDF);
  std::stringstream tmp;
  tmp << "<sdf version='" << SDF_VERSION << "'>";
  tmp << "<plugin name='" << _name << "' filename='" << _filename << "'>";
  tmp << _innerxml;
  tmp << "</plugin></sdf>";

  if (sdf::readString(tmp.str(), modelPluginSDF))
  {
    this->AddModelPlugin(modelPluginSDF);
    this->ModelChanged();
  }
  else
  {
    gzerr << "Error reading Plugin SDF. Unable to parse Innerxml:\n"
        << _innerxml << std::endl;
  }
}

/////////////////////////////////////////////////
void ModelCreator::AddModelPlugin(const sdf::ElementPtr &_pluginElem)
{
  if (_pluginElem->HasAttribute("name"))
  {
    std::string name = _pluginElem->Get<std::string>("name");

    // Create data
    ModelPluginData *modelPlugin = new ModelPluginData();
    modelPlugin->Load(_pluginElem);

    // Add to map
    {
      boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
      this->dataPtr->allModelPlugins[name] = modelPlugin;
    }

    // Notify addition
    gui::model::Events::modelPluginInserted(name);
  }
}

/////////////////////////////////////////////////
ModelPluginData *ModelCreator::ModelPlugin(const std::string &_name)
{
  auto it = this->dataPtr->allModelPlugins.find(_name);
  if (it != this->dataPtr->allModelPlugins.end())
    return it->second;
  return NULL;
}

/////////////////////////////////////////////////
void ModelCreator::OnOpenModelPluginInspector(const QString &_name)
{
  this->OpenModelPluginInspector(_name.toStdString());
}

/////////////////////////////////////////////////
void ModelCreator::OpenModelPluginInspector(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  auto it = this->dataPtr->allModelPlugins.find(_name);
  if (it == this->dataPtr->allModelPlugins.end())
  {
    gzerr << "Model plugin [" << _name << "] not found." << std::endl;
    return;
  }

  ModelPluginData *modelPlugin = it->second;
  modelPlugin->inspector->move(QCursor::pos());
  modelPlugin->inspector->show();
}
