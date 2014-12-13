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
#include <set>
#include <boost/filesystem.hpp>

#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/math/Quaternion.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"

#include "gazebo/common/SystemPaths.hh"
#ifdef HAVE_GTS
  #include "gazebo/common/Mesh.hh"
  #include "gazebo/common/MeshManager.hh"
  #include "gazebo/common/MeshCSG.hh"
#endif

#include "gazebo/gazebo_config.h"
#include "gazebo/gui/building/FinishBuildingDialog.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/BuildingModelManip.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/BuildingMaker.hh"

using namespace gazebo;
using namespace gui;

double BuildingMaker::conversionScale;

/////////////////////////////////////////////////
// Helper function to generate a valid folder name from a human-readable model
// name.
std::string GetFolderNameFromModelName(const std::string &_modelName)
{
  // Auto-generate folder name based on model name
  std::string foldername = _modelName;

  std::vector<std::pair<std::string, std::string> > replacePairs;
  replacePairs.push_back(std::pair<std::string, std::string>(" ", "_"));

  for (unsigned int i = 0; i < replacePairs.size(); ++i)
  {
    std::string forbiddenChar = replacePairs[i].first;
    std::string replaceChar = replacePairs[i].second;
    size_t index = foldername.find(forbiddenChar);
    while (index != std::string::npos)
    {
      foldername.replace(index, forbiddenChar.size(), replaceChar);
      index = foldername.find(forbiddenChar);
    }
  }

  return foldername;
}

/////////////////////////////////////////////////
// Add the parent folder of _path to the model path represented by SystemPaths,
// notify InsertModelWidget to display the model name in the "Insert Models"
// tab, and write the parent folder filename to gui.ini
void AddDirToModelPaths(const std::string& _path)
{
  std::string parentDirectory = boost::filesystem::path(_path)
                                  .parent_path().string();

  std::list<std::string> modelPaths =
              gazebo::common::SystemPaths::Instance()->GetModelPaths();
  std::list<std::string>::iterator iter;
  for (iter = modelPaths.begin();
       iter != modelPaths.end(); ++iter)
  {
    if (iter->compare(parentDirectory) == 0)
    {
      break;
    }
  }

  gazebo::common::SystemPaths::Instance()->
    AddModelPathsUpdate(parentDirectory);

  std::string additionalProperties =
    gui::getINIProperty<std::string>("model_paths.filenames", "");
  if (additionalProperties.find(parentDirectory) == std::string::npos)
  {
    // Add it to gui.ini
    gui::setINIProperty("model_paths.filenames", parentDirectory);

    // Save any changes that were made to the property tree
    // TODO: check gui.ini env variable
    char *home = getenv("HOME");
    if (home)
    {
      boost::filesystem::path guiINIPath = home;
      guiINIPath  = guiINIPath / ".gazebo" / "gui.ini";
      saveINI(guiINIPath);
    }
  }
}

/////////////////////////////////////////////////
BuildingMaker::BuildingMaker() : EntityMaker()
{
  this->buildingDefaultName = "Untitled";
  this->modelName = this->buildingDefaultName;

  this->conversionScale = 0.01;

  // Counters are only used for giving visuals unique names.
  // FIXME they cannot be reset else gazebo complains about creating,
  // deleting duplicate visuals
  this->wallCounter = 0;
  this->windowCounter = 0;
  this->doorCounter = 0;
  this->stairsCounter = 0;
  this->floorCounter = 0;
  this->currentLevel = 0;

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());

  this->connections.push_back(
    gui::editor::Events::ConnectSaveBuildingEditor(
      boost::bind(&BuildingMaker::OnSave, this, _1)));
  this->connections.push_back(
    gui::editor::Events::ConnectSaveAsBuildingEditor(
      boost::bind(&BuildingMaker::OnSaveAs, this, _1)));
  this->connections.push_back(
    gui::editor::Events::ConnectNewBuildingEditor(
      boost::bind(&BuildingMaker::OnNew, this)));
  this->connections.push_back(
    gui::editor::Events::ConnectExitBuildingEditor(
      boost::bind(&BuildingMaker::OnExit, this)));
  this->connections.push_back(
    gui::editor::Events::ConnectBuildingNameChanged(
      boost::bind(&BuildingMaker::OnNameChanged, this, _1)));
  this->connections.push_back(
  gui::editor::Events::ConnectChangeBuildingLevel(
    boost::bind(&BuildingMaker::OnChangeLevel, this, _1)));
  this->connections.push_back(
      gui::editor::Events::ConnectColorSelected(
      boost::bind(&BuildingMaker::OnColorSelected, this, _1)));
  this->connections.push_back(
      gui::editor::Events::ConnectTextureSelected(
      boost::bind(&BuildingMaker::OnTextureSelected, this, _1)));
  this->connections.push_back(
      gui::editor::Events::ConnectToggleEditMode(
      boost::bind(&BuildingMaker::OnEdit, this, _1)));

  this->saveDialog =
      new FinishBuildingDialog(FinishBuildingDialog::MODEL_SAVE, 0);

  this->Reset();
}

/////////////////////////////////////////////////
BuildingMaker::~BuildingMaker()
{
//  this->camera.reset();
  if (this->saveDialog)
    delete this->saveDialog;
}

/////////////////////////////////////////////////
void BuildingMaker::OnEdit(bool _checked)
{
  if (_checked)
  {
    MouseEventHandler::Instance()->AddPressFilter("building_maker",
        boost::bind(&BuildingMaker::On3dMousePress, this, _1));
    MouseEventHandler::Instance()->AddReleaseFilter("building_maker",
        boost::bind(&BuildingMaker::On3dMouseRelease, this, _1));
    MouseEventHandler::Instance()->AddMoveFilter("building_maker",
        boost::bind(&BuildingMaker::On3dMouseMove, this, _1));
    KeyEventHandler::Instance()->AddPressFilter("building_maker",
        boost::bind(&BuildingMaker::On3dKeyPress, this, _1));
  }
  else
  {
    MouseEventHandler::Instance()->RemovePressFilter("building_maker");
    MouseEventHandler::Instance()->RemoveReleaseFilter("building_maker");
    MouseEventHandler::Instance()->RemoveMoveFilter("building_maker");
    KeyEventHandler::Instance()->RemovePressFilter("building_maker");
  }
}

/////////////////////////////////////////////////
void BuildingMaker::ConnectItem(const std::string &_partName,
    const EditorItem *_item)
{
  BuildingModelManip *manip = this->allItems[_partName];

  // Make sure each connection calls BuildingMaker::BuildingChanged

  // item changes -> manip changes
  QObject::connect(_item, SIGNAL(SizeChanged(double, double, double)),
      manip, SLOT(OnSizeChanged(double, double, double)));
  QObject::connect(_item, SIGNAL(PoseChanged(double, double, double,
      double, double, double)), manip, SLOT(OnPoseChanged(double, double,
      double, double, double, double)));
  QObject::connect(_item, SIGNAL(PoseOriginTransformed(double, double, double,
      double, double, double)), manip, SLOT(OnPoseOriginTransformed(double,
      double, double, double, double, double)));
  QObject::connect(_item, SIGNAL(PositionChanged(double, double, double)),
      manip, SLOT(OnPositionChanged(double, double, double)));
  QObject::connect(_item, SIGNAL(RotationChanged(double, double, double)),
      manip, SLOT(OnRotationChanged(double, double, double)));
  QObject::connect(_item, SIGNAL(LevelChanged(int)),
      manip, SLOT(OnLevelChanged(int)));
  QObject::connect(_item, SIGNAL(ColorChanged(QColor)),
      manip, SLOT(OnColorChanged(QColor)));
  QObject::connect(_item, SIGNAL(TextureChanged(QString)),
      manip, SLOT(OnTextureChanged(QString)));
  QObject::connect(_item, SIGNAL(TransparencyChanged(float)),
      manip, SLOT(OnTransparencyChanged(float)));

  QObject::connect(_item, SIGNAL(WidthChanged(double)),
      manip, SLOT(OnWidthChanged(double)));
  QObject::connect(_item, SIGNAL(HeightChanged(double)),
      manip, SLOT(OnHeightChanged(double)));
  QObject::connect(_item, SIGNAL(DepthChanged(double)),
      manip, SLOT(OnDepthChanged(double)));
  QObject::connect(_item, SIGNAL(PosXChanged(double)),
      manip, SLOT(OnPosXChanged(double)));
  QObject::connect(_item, SIGNAL(PosYChanged(double)),
      manip, SLOT(OnPosYChanged(double)));
  QObject::connect(_item, SIGNAL(PosZChanged(double)),
      manip, SLOT(OnPosZChanged(double)));
  QObject::connect(_item, SIGNAL(YawChanged(double)),
      manip, SLOT(OnYawChanged(double)));
  QObject::connect(_item, SIGNAL(ItemDeleted()), manip, SLOT(OnDeleted()));

  // manip changes -> item changes
  QObject::connect(manip, SIGNAL(ColorChanged(QColor)),
      _item, SLOT(OnColorChanged(QColor)));
  QObject::connect(manip, SIGNAL(TextureChanged(QString)),
      _item, SLOT(OnTextureChanged(QString)));
}

/////////////////////////////////////////////////
void BuildingMaker::AttachManip(const std::string &_child,
    const std::string &_parent)
{
  std::map<std::string, BuildingModelManip *>::const_iterator it =
      this->allItems.find(_child);
  if (it == this->allItems.end())
  {
    gzerr << "Child manip " << _child << " not found." << std::endl;
    return;
  }

  it = this->allItems.find(_parent);
  if (it == this->allItems.end())
  {
    gzerr << "Parent manip " << _parent << " not found." << std::endl;
    return;
  }

  BuildingModelManip *child = this->allItems[_child];
  BuildingModelManip *parent = this->allItems[_parent];
  parent->AttachManip(child);
}

/////////////////////////////////////////////////
void BuildingMaker::DetachManip(const std::string &_child,
    const std::string &_parent)
{
  std::map<std::string, BuildingModelManip *>::const_iterator it =
      this->allItems.find(_child);
  if (it == this->allItems.end())
  {
    gzerr << "Child manip " << _child << " not found." << std::endl;
    return;
  }

  it = this->allItems.find(_parent);
  if (it == this->allItems.end())
  {
    gzerr << "Parent manip " << _parent << " not found." << std::endl;
    return;
  }

  BuildingModelManip *child = this->allItems[_child];
  BuildingModelManip *parent = this->allItems[_parent];
  parent->DetachManip(child);
}

/////////////////////////////////////////////////
void BuildingMaker::DetachAllChildren(const std::string &_manip)
{
  std::map<std::string, BuildingModelManip *>::const_iterator it =
      this->allItems.find(_manip);
  if (it == this->allItems.end())
  {
    gzerr << "Manip " << _manip << " not found." << std::endl;
    return;
  }

  BuildingModelManip *manip = this->allItems[_manip];
  for (int i = manip->GetAttachedManipCount()-1; i >= 0; i--)
  {
    (manip->GetAttachedManip(i))->DetachFromParent();
  }
}

/////////////////////////////////////////////////
std::string BuildingMaker::CreateModel()
{
  this->Reset();
  return this->modelName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddPart(const std::string &_type,
    const QVector3D &_size, const QVector3D &_pos, double _angle)
{
  if (_type == "wall")
    return this->AddWall(_size, _pos, _angle);
  else if (_type == "window")
    return this->AddWindow(_size, _pos, _angle);
  else if (_type == "door")
    return this->AddDoor(_size, _pos, _angle);
  else if (_type == "stairs")
    return this->AddStairs(_size, _pos, _angle, 10);
  return "";
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWall(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->modelVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "Wall_" << this->wallCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem = this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->ClearElements();
  visualElem->GetElement("material")->AddElement("ambient")
      ->Set(gazebo::common::Color(1, 1, 1));
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  BuildingModelManip *wallManip = new BuildingModelManip();
  wallManip->SetMaker(this);
  wallManip->SetName(linkName);
  wallManip->SetVisual(visVisual);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  wallManip->SetLevel(this->currentLevel);
  wallManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = wallManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWindow(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->modelVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "Window_" << this->windowCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")->GetElement("name")
      ->Set("Gazebo/BuildingFrame");
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);

  BuildingModelManip *windowManip = new BuildingModelManip();
  windowManip->SetMaker(this);
  windowManip->SetName(linkName);
  windowManip->SetVisual(visVisual);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  windowManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  windowManip->SetLevel(this->currentLevel);
  this->allItems[linkName] = windowManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddDoor(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->modelVisual)
  {
    this->Reset();
  }

  /// TODO a copy of AddWindow function. FIXME later
  std::ostringstream linkNameStream;
  linkNameStream << "Door_" << this->doorCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")->GetElement("name")
      ->Set("Gazebo/BuildingFrame");
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);

  BuildingModelManip *doorManip = new BuildingModelManip();
  doorManip->SetMaker(this);
  doorManip->SetName(linkName);
  doorManip->SetVisual(visVisual);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  doorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  doorManip->SetLevel(this->currentLevel);
  this->allItems[linkName] = doorManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddStairs(const QVector3D &_size,
    const QVector3D &_pos, double _angle, int _steps)
{
  if (!this->modelVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "Stairs_" << this->stairsCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visVisual->Load(visualElem);
  visVisual->DetachObjects();

  BuildingModelManip *stairsManip = new BuildingModelManip();
  stairsManip->SetMaker(this);
  stairsManip->SetName(linkName);
  stairsManip->SetVisual(visVisual);
  stairsManip->SetLevel(this->currentLevel);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  double dSteps = static_cast<double>(_steps);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  stairsManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = stairsManip;

  std::stringstream visualStepName;
  visualStepName << visualName.str() << "step" << 0;
  rendering::VisualPtr baseStepVisual(new rendering::Visual(
      visualStepName.str(), visVisual));
  visualElem->GetElement("material")->ClearElements();
  visualElem->GetElement("material")->AddElement("ambient")
      ->Set(gazebo::common::Color(1, 1, 1));
  visualElem->AddElement("cast_shadows")->Set(false);
  baseStepVisual->Load(visualElem);

  double rise = 1.0 / dSteps;
  double run = 1.0 / dSteps;
  baseStepVisual->SetScale(math::Vector3(1, run, rise));

  math::Vector3 baseOffset(0, 0.5 - run/2.0,
      -0.5 + rise/2.0);
  baseStepVisual->SetPosition(baseOffset);

  for (int i = 1; i < _steps; ++i)
  {
    visualStepName.str("");
    visualStepName << visualName.str() << "step" << i;
    rendering::VisualPtr stepVisual = baseStepVisual->Clone(
        visualStepName.str(), visVisual);
    stepVisual->SetPosition(math::Vector3(0, baseOffset.y-(run*i),
        baseOffset.z + rise*i));
    stepVisual->SetRotation(baseStepVisual->GetRotation());
  }

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->BuildingChanged();

  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddFloor(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->modelVisual)
  {
    this->Reset();
  }

  /// TODO a copy of AddWindow function. FIXME later
  std::ostringstream linkNameStream;
  linkNameStream << "Floor_" << this->floorCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->ClearElements();
  visualElem->GetElement("material")->AddElement("ambient")
      ->Set(gazebo::common::Color(1, 1, 1));
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);

  BuildingModelManip *floorManip = new BuildingModelManip();
  floorManip->SetMaker(this);
  floorManip->SetName(linkName);
  floorManip->SetVisual(visVisual);
  floorManip->SetLevel(this->currentLevel);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  floorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = floorManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
void BuildingMaker::RemovePart(const std::string &_partName)
{
  std::map<std::string, BuildingModelManip *>::const_iterator it =
      this->allItems.find(_partName);
  if (it == this->allItems.end())
  {
    gzerr << _partName << " does not exist\n";
    return;
  }
  BuildingModelManip *manip = this->allItems[_partName];

  rendering::VisualPtr vis = manip->GetVisual();
  rendering::VisualPtr visParent = vis->GetParent();
  rendering::ScenePtr scene = vis->GetScene();
  scene->RemoveVisual(vis);
  if (visParent)
    scene->RemoveVisual(visParent);
  this->allItems.erase(_partName);
  delete manip;
  this->BuildingChanged();
}

/////////////////////////////////////////////////
void BuildingMaker::RemoveWall(const std::string &_wallName)
{
  this->RemovePart(_wallName);
}


/////////////////////////////////////////////////
void BuildingMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;
}

/////////////////////////////////////////////////
void BuildingMaker::Stop()
{
//  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
//  scene->RemoveVisual(this->modelVisual);
//  this->modelVisual.reset();
//  this->visuals.clear();
//  this->modelSDF.reset();
//  this->modelVisual->SetVisible(false);
}

/////////////////////////////////////////////////
void BuildingMaker::Reset()
{
  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
    return;

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (!scene)
  {
    gzerr << "Couldn't get scene node from BuildingMaker" << std::endl;
    return;
  }

  if (this->modelVisual)
    scene->RemoveVisual(this->modelVisual);

  this->currentSaveState = NEVER_SAVED;
  this->SetModelName(this->buildingDefaultName);
  this->defaultPath = (QDir::homePath() + "/building_editor_models")
                        .toStdString();
  this->saveLocation = defaultPath + "/" +
                        GetFolderNameFromModelName(this->modelName);

  this->modelVisual.reset(new rendering::Visual(this->modelName,
      scene->GetWorldVisual()));

  this->modelVisual->Load();
  this->modelPose = math::Pose::Zero;
  this->modelVisual->SetPose(this->modelPose);
  this->modelVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  scene->AddVisual(this->modelVisual);

  std::map<std::string, BuildingModelManip *>::iterator it;
  for (it = this->allItems.begin(); it != this->allItems.end(); ++it)
    delete (*it).second;
  this->allItems.clear();
}

/////////////////////////////////////////////////
bool BuildingMaker::IsActive() const
{
  return true;
}

/////////////////////////////////////////////////
void BuildingMaker::SetModelName(const std::string &_modelName)
{
  this->modelName = _modelName;
  this->saveDialog->SetModelName(_modelName);
  this->BuildingChanged();
  // send to palette?
}

/////////////////////////////////////////////////
void BuildingMaker::SaveToSDF(const std::string &_savePath)
{
  std::ofstream savefile;
  boost::filesystem::path path(_savePath);
  path = path / "model.sdf";

  // FIXME
  savefile.open(path.string().c_str());
  if (!savefile.is_open())
  {
    gzerr << "Couldn't open file for writing: " << path.string() << std::endl;
    return;
  }
  savefile << this->modelSDF->ToString();
  savefile.close();
  gzdbg << "Saved file to " << path.string() << std::endl;
}

/////////////////////////////////////////////////
void BuildingMaker::FinishModel()
{
  this->CreateTheEntity();
  this->Reset();
}

/////////////////////////////////////////////////
void BuildingMaker::GenerateSDF()
{
  // This function generates an SDF file for the final building model (which
  // will eventually be uploaded to the server). It loops through all the model
  // manip objects (allItems), grabs pose and size data of each model manip, and
  // creates a link element consisting of a visual and a collision element with
  // the same geometry. If the model manip has attached objects, e.g. a wall
  // with windows/doors and a floor with stairs, it creates holes in the
  // surface of the parent by using a rectangular surface subdivision algorithm.
  // TODO Refactor code

  sdf::ElementPtr modelElem;
  sdf::ElementPtr linkElem;
  sdf::ElementPtr visualElem;
  sdf::ElementPtr collisionElem;

  this->modelSDF.reset(new sdf::SDF);
  this->modelSDF->SetFromString(this->GetTemplateSDFString());

  modelElem = this->modelSDF->root->GetElement("model");

  linkElem = modelElem->GetElement("link");
  sdf::ElementPtr templateLinkElem = linkElem->Clone();
  sdf::ElementPtr templateVisualElem = templateLinkElem->GetElement(
      "visual")->Clone();
  sdf::ElementPtr templateCollisionElem = templateLinkElem->GetElement(
      "collision")->Clone();
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(
      GetFolderNameFromModelName(this->modelName));

  std::map<std::string, BuildingModelManip *>::iterator itemsIt;

  // loop through all model manips
  for (itemsIt = this->allItems.begin(); itemsIt != this->allItems.end();
      ++itemsIt)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    std::string name = itemsIt->first;
    BuildingModelManip *buildingModelManip = itemsIt->second;
    rendering::VisualPtr visual = buildingModelManip->GetVisual();
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(buildingModelManip->GetName());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose());

    // Only stairs have children
    if (visual->GetChildCount() == 0)
    {
      // Window / Door not attached to walls
      if (name.find("Window") != std::string::npos
          || name.find("Door") != std::string::npos)
      {
        if (buildingModelManip->IsAttached())
          continue;
        visualElem->GetAttribute("name")->Set(buildingModelManip->GetName()
            + "_Visual");
        collisionElem->GetAttribute("name")->Set(buildingModelManip->GetName()
            + "_Collision");
        visualElem->GetElement("pose")->Set(visual->GetPose());
        collisionElem->GetElement("pose")->Set(visual->GetPose());
        visualElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale());
        collisionElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale());
      }
      // Wall
      else if (name.find("Wall") != std::string::npos)
      {
        // check if walls have attached children, i.e. windows or doors
        if (buildingModelManip->GetAttachedManipCount() != 0 )
        {
          std::vector<QRectF> holes;
          rendering::VisualPtr wallVis = visual;
          math::Pose wallPose = wallVis->GetParent()->GetWorldPose();
          math::Vector3 wallSize = wallVis->GetScale();
          for (unsigned int i = 0; i <
              buildingModelManip->GetAttachedManipCount(); ++i)
          {
            BuildingModelManip *attachedObj =
                buildingModelManip->GetAttachedManip(i);
            std::string objName = attachedObj->GetName();
            if (objName.find("Window") != std::string::npos
                || objName.find("Door") != std::string::npos)
            {
              rendering::VisualPtr attachedVis = attachedObj->GetVisual();
              math::Pose offset = attachedVis->GetParent()->GetWorldPose()
                  - wallPose;
              math::Vector3 size = attachedVis->GetScale();

              offset.pos.z += attachedVis->GetPosition().z
                  - wallVis->GetPosition().z;

              math::Vector3 newOffset = offset.pos - (-wallSize/2.0)
                  - size/2.0;
              QRectF hole(newOffset.x, newOffset.z, size.x, size.z);
              holes.push_back(hole);
            }
          }
          std::vector<QRectF> subdivisions;
          QRectF surface(0, 0, wallVis->GetScale().x, wallVis->GetScale().z);

          // subdivide the wall into mutiple compositing rectangles in order
          // to create holes for the attached children
          this->SubdivideRectSurface(surface, holes, subdivisions);

          newLinkElem->ClearElements();
          newLinkElem->GetAttribute("name")->Set(buildingModelManip->GetName());
          newLinkElem->GetElement("pose")->Set(
              visual->GetParent()->GetWorldPose());
          // create a link element of box geom for each subdivision
          for (unsigned int i = 0; i< subdivisions.size(); ++i)
          {
            visualNameStream.str("");
            collisionNameStream.str("");
            visualElem = templateVisualElem->Clone();
            collisionElem = templateCollisionElem->Clone();
            visualNameStream << buildingModelManip->GetName() << "_Visual_"
                << i;
            visualElem->GetAttribute("name")->Set(visualNameStream.str());
            collisionNameStream << buildingModelManip->GetName()
                << "_Collision_" << i;
            collisionElem->GetAttribute("name")->Set(collisionNameStream.str());

            math::Vector3 newSubPos =
                math::Vector3(-wallSize.x/2.0, 0, -wallSize.z/2.0)
                + math::Vector3(subdivisions[i].x(), 0, subdivisions[i].y())
                + math::Vector3(subdivisions[i].width()/2, 0,
                    subdivisions[i].height()/2);
            newSubPos.z += wallSize.z/2;
            math::Pose newPose(newSubPos, math::Quaternion(0, 0, 0));
            visualElem->GetElement("pose")->Set(newPose);
            collisionElem->GetElement("pose")->Set(newPose);
            math::Vector3 blockSize(subdivisions[i].width(),
                wallVis->GetScale().y, subdivisions[i].height());
            visualElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            collisionElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            visualElem->GetElement("material")->GetElement("ambient")->
                Set(buildingModelManip->GetColor());
            visualElem->GetElement("material")->GetElement("script")
                ->GetElement("name")->Set(buildingModelManip->GetTexture());
            newLinkElem->InsertElement(visualElem);
            newLinkElem->InsertElement(collisionElem);
          }
        }
        // Wall without windows or doors
        else
        {
          visualElem->GetAttribute("name")->Set(buildingModelManip->GetName()
              + "_Visual");
          collisionElem->GetAttribute("name")->Set(buildingModelManip->GetName()
              + "_Collision");
          visualElem->GetElement("pose")->Set(visual->GetPose());
          collisionElem->GetElement("pose")->Set(visual->GetPose());
          visualElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          collisionElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          visualElem->GetElement("material")->GetElement("ambient")->
              Set(buildingModelManip->GetColor());
          visualElem->GetElement("material")->GetElement("script")
              ->GetElement("name")->Set(buildingModelManip->GetTexture());
        }
      }
      // Floor
      else if (name.find("Floor") != std::string::npos)
      {
        // check if floors have attached children, i.e. stairs
        if (buildingModelManip->GetAttachedManipCount() != 0 )
        {
          std::vector<QRectF> holes;
          rendering::VisualPtr floorVis = visual;
          math::Pose floorPose = floorVis->GetWorldPose();
          math::Vector3 floorSize = floorVis->GetScale();
          for (unsigned int i = 0; i <
              buildingModelManip->GetAttachedManipCount(); ++i)
          {
            BuildingModelManip *attachedObj =
                buildingModelManip->GetAttachedManip(i);
            std::string objName = attachedObj->GetName();
            if (objName.find("Stairs") != std::string::npos)
            {
              rendering::VisualPtr attachedVis = attachedObj->GetVisual();
              math::Pose offset = attachedVis->GetParent()->GetWorldPose()
                  - floorPose;
              math::Vector3 size = attachedVis->GetScale();

              QRectF rect(0, 0, size.x, size.y);
              QPolygonF polygon(rect);
              QTransform transform;
              transform.rotate(GZ_RTOD(offset.rot.GetAsEuler().z));
              QRectF bound = transform.map(polygon).boundingRect();
              math::Vector3 newOffset = offset.pos - (-floorSize/2.0)
                  - math::Vector3(bound.width(), bound.height(), size.z)/2.0;
              QRectF hole(newOffset.x, newOffset.y, bound.width(),
                  bound.height());
              holes.push_back(hole);
            }
          }
          std::vector<QRectF> subdivisions;
          QRectF surface(0, 0, floorVis->GetScale().x, floorVis->GetScale().y);

          // subdivide the floor into mutiple compositing rectangles to make an
          // opening for the stairs
          this->SubdivideRectSurface(surface, holes, subdivisions);

          newLinkElem->ClearElements();
          newLinkElem->GetAttribute("name")->Set(buildingModelManip->GetName());
          newLinkElem->GetElement("pose")->Set(
              visual->GetParent()->GetWorldPose());
          // create a link element of box geom for each subdivision
          for (unsigned int i = 0; i< subdivisions.size(); ++i)
          {
            visualNameStream.str("");
            collisionNameStream.str("");
            visualElem = templateVisualElem->Clone();
            collisionElem = templateCollisionElem->Clone();
            visualNameStream << buildingModelManip->GetName() << "_Visual_"
                << i;
            visualElem->GetAttribute("name")->Set(visualNameStream.str());
            collisionNameStream << buildingModelManip->GetName()
                << "_Collision_" << i;
            collisionElem->GetAttribute("name")->Set(collisionNameStream.str());

            math::Vector3 newSubPos =
                math::Vector3(-floorSize.x/2.0, -floorSize.y/2.0, 0)
                + math::Vector3(subdivisions[i].x(), subdivisions[i].y(), 0)
                + math::Vector3(subdivisions[i].width()/2,
                subdivisions[i].height()/2, 0);
            newSubPos.z += floorVis->GetPosition().z;
            math::Pose newPose(newSubPos, visual->GetParent()->GetRotation());
            visualElem->GetElement("pose")->Set(newPose);
            collisionElem->GetElement("pose")->Set(newPose);
            math::Vector3 blockSize(subdivisions[i].width(),
                subdivisions[i].height(), floorVis->GetScale().z);
            visualElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            collisionElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            visualElem->GetElement("material")->GetElement("ambient")->
                Set(buildingModelManip->GetColor());
            visualElem->GetElement("material")->GetElement("script")
                ->GetElement("name")->Set(buildingModelManip->GetTexture());
            newLinkElem->InsertElement(visualElem);
            newLinkElem->InsertElement(collisionElem);
          }
        }
        // Floor without stairs
        else
        {
          visualElem->GetAttribute("name")->Set(buildingModelManip->GetName()
              + "_Visual");
          collisionElem->GetAttribute("name")->Set(buildingModelManip->GetName()
              + "_Collision");
          visualElem->GetElement("pose")->Set(visual->GetPose());
          collisionElem->GetElement("pose")->Set(visual->GetPose());
          visualElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          collisionElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          visualElem->GetElement("material")->GetElement("ambient")->
              Set(buildingModelManip->GetColor());
          visualElem->GetElement("material")->GetElement("script")
              ->GetElement("name")->Set(buildingModelManip->GetTexture());
        }
      }
    }
    // Stairs
    else
    {
      // TODO: This handles the special case for stairs, where
      // there are nested visuals which SDF doesn't support.
      // Should somehow generalize/combine the code above and below
      newLinkElem->ClearElements();
      for (unsigned int i = 0; i< visual->GetChildCount(); ++i)
      {
        visualNameStream.str("");
        collisionNameStream.str("");
        visualElem = templateVisualElem->Clone();
        collisionElem = templateCollisionElem->Clone();
        visualNameStream << buildingModelManip->GetName() << "_Visual_" << i;
        visualElem->GetAttribute("name")->Set(visualNameStream.str());
        collisionNameStream << buildingModelManip->GetName()
            << "_Collision_" << i;
        collisionElem->GetAttribute("name")->Set(collisionNameStream.str());
        rendering::VisualPtr childVisual = visual->GetChild(i);
        math::Pose newPose(childVisual->GetWorldPose().pos,
            visual->GetParent()->GetRotation());
        visualElem->GetElement("pose")->Set(newPose);
        collisionElem->GetElement("pose")->Set(newPose);
        visualElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale()*childVisual->GetScale());
        collisionElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale()*childVisual->GetScale());
        visualElem->GetElement("material")->GetElement("ambient")->
              Set(buildingModelManip->GetColor());
        visualElem->GetElement("material")->GetElement("script")
            ->GetElement("name")->Set(buildingModelManip->GetTexture());
        newLinkElem->InsertElement(visualElem);
        newLinkElem->InsertElement(collisionElem);
      }
    }
    modelElem->InsertElement(newLinkElem);
  }
  (modelElem->AddElement("static"))->Set("true");
  // qDebug() << this->modelSDF->ToString().c_str();
}

/////////////////////////////////////////////////
void BuildingMaker::GenerateSDFWithCSG()
{
#ifdef HAVE_GTS
  // This function generates an SDF file for the final building model in a
  // similar manner to the GenerateSDF function. However instead of using a
  // surface subdivision algorithm for making openings for attached objects,
  // it uses boolean operations to find the difference between one mesh and
  // the other.
  // TODO The function is not yet integrated as custom shapes are not yet
  // supported in Gazebo.
  sdf::ElementPtr modelElem;
  sdf::ElementPtr linkElem;
  sdf::ElementPtr visualElem;
  sdf::ElementPtr collisionElem;

  this->modelSDF.reset(new sdf::SDF);
  this->modelSDF->SetFromString(this->GetTemplateSDFString());

  modelElem = this->modelSDF->root->GetElement("model");
  linkElem = modelElem->GetElement("link");

  sdf::ElementPtr templateLinkElem = linkElem->Clone();
  sdf::ElementPtr templateVisualElem = templateLinkElem->GetElement(
      "visual")->Clone();
  sdf::ElementPtr templateCollisionElem = templateLinkElem->GetElement(
      "collision")->Clone();
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(this->modelName);

  std::map<std::string, BuildingModelManip *>::iterator itemsIt;
  for (itemsIt = this->allItems.begin(); itemsIt != this->allItems.end();
      ++itemsIt)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    std::string name = itemsIt->first;
    BuildingModelManip *buildingModelManip = itemsIt->second;
    rendering::VisualPtr visual = buildingModelManip->GetVisual();
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(buildingModelManip->GetName());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose());

    // create a hole to represent a window/door in the wall
    if (name.find("Window") != std::string::npos
        || name.find("Door") != std::string::npos)
    {
      if (buildingModelManip->IsAttached())
        continue;
    }
    else if (name.find("Wall") != std::string::npos
        && buildingModelManip->GetAttachedManipCount() != 0)
    {
      rendering::VisualPtr wallVis = visual;
      math::Pose wallPose = wallVis->GetWorldPose();
      const common::Mesh *wallMesh = common::MeshManager::Instance()
          ->GetMesh(wallVis->GetMeshName());
      // clone wall mesh to to be used in boolean operation
      common::Mesh *m1 = new common::Mesh();
      for (unsigned int k = 0; k < wallMesh->GetSubMeshCount(); ++k)
      {
        common::SubMesh *m1SubMesh = new common::SubMesh();
        m1->AddSubMesh(m1SubMesh);
        const common::SubMesh *subMesh = wallMesh->GetSubMesh(k);
        if (subMesh->GetVertexCount() <= 2)
          continue;
        for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
        {
          m1SubMesh->AddVertex(subMesh->GetVertex(j));
        }
        for (unsigned int j = 0; j < subMesh->GetIndexCount(); ++j)
        {
          m1SubMesh->AddIndex(subMesh->GetIndex(j));
        }
      }
      m1->SetScale(math::Vector3(wallVis->GetScale()));

      std::string booleanMeshName = buildingModelManip->GetName() + "_Boolean";
      common::Mesh *booleanMesh = NULL;
      for (unsigned int i = 0; i < buildingModelManip->GetAttachedManipCount();
          ++i)
      {
        if (booleanMesh)
        {
          delete m1;
          m1 = booleanMesh;
        }

        BuildingModelManip *attachedObj =
            buildingModelManip->GetAttachedManip(i);
        std::string objName = attachedObj->GetName();
        if (objName.find("Window") != std::string::npos
            || objName.find("Door") != std::string::npos)
        {
          rendering::VisualPtr attachedVis = attachedObj->GetVisual();
          math::Pose offset = attachedVis->GetWorldPose() - wallPose;
          const common::Mesh *attachedMesh = common::MeshManager::Instance()->
              GetMesh(attachedVis->GetMeshName());
          // clone attached object mesh
          common::Mesh *m2 = new common::Mesh();
          for (unsigned int k = 0; k < attachedMesh->GetSubMeshCount(); ++k)
          {
            common::SubMesh *m2SubMesh = new common::SubMesh();
            m2->AddSubMesh(m2SubMesh);
            const common::SubMesh *subMesh = attachedMesh->GetSubMesh(k);
            if (subMesh->GetVertexCount() <= 2)
              continue;
            for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
            {
              m2SubMesh->AddVertex(subMesh->GetVertex(j));
            }
            for (unsigned int j = 0; j < subMesh->GetIndexCount(); ++j)
            {
              m2SubMesh->AddIndex(subMesh->GetIndex(j));
            }
          }
          m2->SetScale(math::Vector3(attachedVis->GetScale()));
          // create csg but don't add to mesh manager just yet
          common::MeshCSG csg;
          booleanMesh = csg.CreateBoolean(m1, m2, common::MeshCSG::DIFFERENCE,
              offset);
        }
      }
      // add to mesh manager after all boolean operations are done
      booleanMesh->SetName(booleanMeshName);
      common::MeshManager::Instance()->AddMesh(booleanMesh);

      // finally create the sdf elements
      visualElem->GetAttribute("name")->Set(buildingModelManip->GetName()
          + "_Visual");
      collisionElem->GetAttribute("name")->Set(buildingModelManip->GetName()
          + "_Collision");
      sdf::ElementPtr visGeomElem = visualElem->GetElement("geometry");
      visGeomElem->ClearElements();
      sdf::ElementPtr meshElem = visGeomElem->AddElement("mesh");
      // TODO create the folder
      std::string uri = "model://" + this->modelName + "/meshes/"
          + booleanMeshName;
      meshElem->GetElement("uri")->Set(uri);
      visualElem->GetElement("pose")->Set(visual->GetPose());
      collisionElem->GetElement("geometry")->GetElement("box")->
          GetElement("size")->Set(visual->GetScale());
      collisionElem->GetElement("pose")->Set(visual->GetPose());
    }
    else if (name.find("Stairs") != std::string::npos
        && visual->GetChildCount() > 0)
    {
      newLinkElem->ClearElements();
      for (unsigned int i = 0; i< visual->GetChildCount(); ++i)
      {
        visualNameStream.str("");
        collisionNameStream.str("");
        visualElem = templateVisualElem->Clone();
        collisionElem = templateCollisionElem->Clone();
        visualNameStream << buildingModelManip->GetName() << "_Visual_" << i;
        visualElem->GetAttribute("name")->Set(visualNameStream.str());
        collisionNameStream << buildingModelManip->GetName() << "_Collision_"
            << i;
        collisionElem->GetAttribute("name")->Set(collisionNameStream.str());
        rendering::VisualPtr childVisual = visual->GetChild(i);
        math::Pose newPose(childVisual->GetWorldPose().pos,
            visual->GetRotation());
        visualElem->GetElement("pose")->Set(newPose);
        collisionElem->GetElement("pose")->Set(newPose);
        visualElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale()*childVisual->GetScale());
        collisionElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale()*childVisual->GetScale());

        newLinkElem->InsertElement(visualElem);
        newLinkElem->InsertElement(collisionElem);
      }
    }
    modelElem->InsertElement(newLinkElem);
  }
  (modelElem->AddElement("static"))->Set("true");
#endif
}

/////////////////////////////////////////////////
void BuildingMaker::CreateTheEntity()
{
  msgs::Factory msg;
  // Create a new name if the model exists
  if (!this->modelSDF->root->HasElement("model"))
  {
    gzerr << "Generated invalid SDF! Cannot create entity." << std::endl;
    return;
  }

  sdf::ElementPtr modelElem = this->modelSDF->root->GetElement("model");
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

  msg.set_sdf(this->modelSDF->ToString());
  this->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
math::Vector3 BuildingMaker::ConvertSize(double _width, double _length,
    double _height)
{
  return math::Vector3(conversionScale*_width, conversionScale*_length,
      conversionScale*_height);
}

/////////////////////////////////////////////////
math::Pose BuildingMaker::ConvertPose(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  return math::Pose(conversionScale*_x, conversionScale*_y, conversionScale*_z,
      GZ_DTOR(_roll), GZ_DTOR(_pitch), GZ_DTOR(_yaw));
}

/////////////////////////////////////////////////
math::Vector3 BuildingMaker::ConvertSize(const QVector3D &_size)
{
  QVector3D scaledSize = conversionScale*_size;
  return math::Vector3(scaledSize.x(), scaledSize.y(), scaledSize.z());
}

/////////////////////////////////////////////////
math::Pose BuildingMaker::ConvertPose(const QVector3D &_pos,
    const QVector3D &_rot)
{
  QVector3D scaledPos = conversionScale*_pos;
  return math::Pose(scaledPos.x(), scaledPos.y(), scaledPos.z(),
      GZ_DTOR(_rot.x()), GZ_DTOR(_rot.y()), GZ_DTOR(_rot.z()));
}

/////////////////////////////////////////////////
double BuildingMaker::Convert(double _value)
{
  return conversionScale*_value;
}

/////////////////////////////////////////////////
double BuildingMaker::ConvertAngle(double _angle)
{
  return GZ_DTOR(_angle);
}

/////////////////////////////////////////////////
std::string BuildingMaker::GetTemplateConfigString()
{
  std::ostringstream newModelStr;
  newModelStr << "<?xml version=\"1.0\"?>"
  << "<model>"
  <<   "<name>building_template_model</name>"
  <<   "<version>1.0</version>"
  <<   "<sdf version=\"1.5\">model.sdf</sdf>"
  <<   "<author>"
  <<     "<name>author_name</name>"
  <<     "<email>author_email</email>"
  <<   "</author>"
  <<   "<description>Made with the Gazebo Building Editor</description>"
  << "</model>";
  return newModelStr.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::GetTemplateSDFString()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='building_template_model'>"
    << "<pose>0 0 0.0 0 0 0</pose>"
    << "<link name ='link'>"
    <<   "<collision name ='collision'>"
    <<     "<geometry>"
    <<       "<box>"
    <<         "<size>1.0 1.0 1.0</size>"
    <<       "</box>"
    <<     "</geometry>"
    <<   "</collision>"
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
bool BuildingMaker::PointCompareY(const QPointF &_a, const QPointF &_b)
{
  return _a.y() < _b.y();
}

/////////////////////////////////////////////////
bool BuildingMaker::RectCompareX(const QRectF &_a, const QRectF &_b)
{
  return _a.x() < _b.x();
}

/////////////////////////////////////////////////
bool BuildingMaker::RectCompareY(const QRectF &_a, const QRectF &_b)
{
  return _a.y() < _b.y();
}

/////////////////////////////////////////////////
void BuildingMaker::SubdivideRectSurface(const QRectF &_surface,
    const std::vector<QRectF> &_holes, std::vector<QRectF> &_subdivisions)
{
  // use multiset for ordered elements
  std::multiset<QRectF, bool (*)(const QRectF &, const QRectF &)>
      filledX(BuildingMaker::RectCompareX);
  std::multiset<QRectF, bool (*)(const QRectF &, const QRectF &)>
      filledY(BuildingMaker::RectCompareY);
  for (unsigned int i = 0; i < _holes.size(); ++i)
  {
    filledX.insert(_holes[i]);
    filledY.insert(_holes[i]);
  }

  std::multiset<QPointF, bool (*)(const QPointF &, const QPointF &)>
      startings(BuildingMaker::PointCompareY);

  QPointF start(_surface.x(), _surface.y());
  startings.insert(start);

  std::multiset<QPointF>::iterator startIt;

  // Surface subdivision algorithm:
  // subdivisions are called blocks here
  // 1. Insert a few starting points found by walking along top and left borders
  // 2. Walk along y from a starting point and stop on first obstacle with
  //    same x, this gives block height
  // 3. Find the next obstacle in x dir, this gives block width
  // 4. Remove starting point from the list
  // 5. Find next starting points by walking along the block's
  //    bottom and right edges
  // 6. Insert new starting points and the new block
  // 7. Repeat 2~6 until there are no more starting points.

  double eps = 0.001;
  QPointF borderStart;
  std::multiset<QRectF>::iterator borderIt = filledX.begin();
  for (borderIt; borderIt != filledX.end(); ++borderIt)
  {
    if (fabs((*borderIt).y() - _surface.y()) < eps
        && ((*borderIt).x() + (*borderIt).width())
        < (_surface.x() + _surface.width()))
    {
      borderStart.setX((*borderIt).x() + (*borderIt).width());
      borderStart.setY(0);
      startings.insert(borderStart);
    }
    if (fabs((*borderIt).x() - _surface.x()) < eps
        && ((*borderIt).y() + (*borderIt).height())
        < (_surface.y() + _surface.height()))
    {
      borderStart.setX(0);
      borderStart.setY((*borderIt).y() + (*borderIt).height());
      startings.insert(borderStart);
    }
  }

  while (!startings.empty())
  {
    startIt = startings.begin();

    // walk along y
    double maxY = _surface.y() + _surface.height();
    std::multiset<QRectF>::iterator it = filledY.begin();
    for (it; it != filledY.end(); ++it)
    {
      if (((*startIt).x() >= (*it).x())
          && ((*startIt).x() < ((*it).x() + (*it).width())))
      {
        if (((*it).y() < maxY) && ((*it).y() > (*startIt).y()))
        {
          maxY = (*it).y();
        }
      }
    }

    // find next obstacle in x dir
    double maxX = _surface.x() + _surface.width();
    it = filledX.begin();
    for (it; it != filledX.end(); ++it)
    {
      if ((maxY > (*it).y()) )
      {
        if (((*it).x() < maxX) && ((*it).x() > (*startIt).x()))
        {
          maxX = (*it).x();
        }
      }
    }
//    std::cout << " next xy " << maxX << " " << maxY << std::endl;

    QRectF block((*startIt).x(), (*startIt).y(),
        maxX - (*startIt).x(), maxY - (*startIt).y());

    // remove current starting point
    startings.erase(startIt);

    // find new starting points
    // walk along bottom and right edges
    // first start with bottom edge
    QPointF tmpStart(block.x(), block.y() + block.height());
    bool walkedEdge = false;
    std::multiset<QRectF>::iterator edgeIt = filledX.begin();

    if (tmpStart.y() >= _surface.y() + _surface.height())
      edgeIt = filledX.end();
    for (edgeIt; edgeIt != filledX.end(); ++edgeIt)
    {
      if (fabs((*edgeIt).y() - (block.y() + block.height())) > eps)
        continue;

      if (fabs((*edgeIt).x() - tmpStart.x()) < eps)
        walkedEdge = false;

      if ((*edgeIt).x() > tmpStart.x())
      {
         startings.insert(tmpStart);
      }
      if (((*edgeIt).x() + (*edgeIt).width()) < (block.x() + block.width()))
      {
        if (((*edgeIt).x() + (*edgeIt).width()) > block.x())
        {
          tmpStart.setX((*edgeIt).x() + (*edgeIt).width());
          walkedEdge = true;
        }
      }
      else
      {
        break;
      }
    }
    if (walkedEdge && (tmpStart.x() < (block.x() + block.width())))
    {
      if ((tmpStart.x() < (_surface.x() + _surface.width()))
          && (tmpStart.y() < (_surface.y() + _surface.height())))
      {
        startings.insert(tmpStart);
      }
    }

    // then look at the right edge
    tmpStart.setX(block.x() + block.width());
    tmpStart.setY(block.y());
    edgeIt = filledY.begin();
    walkedEdge = false;

    if (tmpStart.x() >= (_surface.x() + _surface.width()))
      edgeIt = filledY.end();

    for (edgeIt; edgeIt != filledY.end(); ++edgeIt)
    {
      if (fabs((*edgeIt).x() - (block.x() + block.width())) > eps)
        continue;

      if (fabs((*edgeIt).y() - tmpStart.y()) < eps)
        walkedEdge = false;

      if ((*edgeIt).y() > tmpStart.y())
      {
        startings.insert(tmpStart);
        walkedEdge = false;
      }
      if ( ((*edgeIt).y() + (*edgeIt).height()) < (block.y() + block.height()))
      {
        if (((*edgeIt).y() + (*edgeIt).height()) > block.y())
        {
          tmpStart.setY((*edgeIt).y() + (*edgeIt).height());
          walkedEdge = true;
        }
      }
      else
      {
        break;
      }
    }
    if (walkedEdge && (tmpStart.y() <= (block.y() + block.height())) )
    {
      if ((tmpStart.x() < (_surface.x() + _surface.width()))
          && (tmpStart.y() < (_surface.y() + _surface.height())))
      {
        startings.insert(tmpStart);
      }
     }

    filledX.insert(block);
    filledY.insert(block);
    _subdivisions.push_back(block);
  }
}

/////////////////////////////////////////////////
void BuildingMaker::OnNew()
{
  if (this->allItems.empty())
  {
    this->Reset();
    gui::editor::Events::newBuildingModel();
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
      if (!this->OnSave(this->modelName))
      {
        return;
      }
    }

    this->Reset();
    gui::editor::Events::newBuildingModel();
  }
}

void BuildingMaker::SaveModelFiles()
{
  this->SetModelName(this->modelName);
  this->GenerateConfig();
  this->SaveToConfig(this->saveLocation);
  this->GenerateSDF();
  this->SaveToSDF(this->saveLocation);
  this->currentSaveState = ALL_SAVED;
}

/////////////////////////////////////////////////
void BuildingMaker::GenerateConfig()
{
  // Create an xml config file
  this->modelConfig.Clear();
  this->modelConfig.Parse(this->GetTemplateConfigString().c_str());

  TiXmlElement *modelXML = this->modelConfig.FirstChildElement("model");
  if (!modelXML)
  {
    gzerr << "No model name in default config file\n";
    return;
  }
  TiXmlElement *modelNameXML = modelXML->FirstChildElement("name");
  modelNameXML->FirstChild()->SetValue(this->modelName);

  TiXmlElement *versionXML = modelXML->FirstChildElement("version");
  if (!versionXML)
  {
    gzerr << "Couldn't find model version" << std::endl;
    versionXML->FirstChild()->SetValue("1.0");
  }
  else
  {
    versionXML->FirstChild()->SetValue(this->version);
  }

  TiXmlElement *descriptionXML = modelXML->FirstChildElement("description");
  if (!descriptionXML)
  {
    gzerr << "Couldn't find model description" << std::endl;
    descriptionXML->FirstChild()->SetValue("");
  }
  else
  {
    descriptionXML->FirstChild()->SetValue(this->description);
  }

  // TODO: Multiple authors
  TiXmlElement *authorXML = modelXML->FirstChildElement("author");
  if (!authorXML)
  {
    gzerr << "Couldn't find model author" << std::endl;
  }
  else
  {
    TiXmlElement *authorChild = authorXML->FirstChildElement("name");
    if (!authorChild)
    {
      gzerr << "Couldn't find author name" << std::endl;
      authorChild->FirstChild()->SetValue("");
    }
    else
    {
      authorChild->FirstChild()->SetValue(this->authorName);
    }
    authorChild = authorXML->FirstChildElement("email");
    if (!authorChild)
    {
      gzerr << "Couldn't find author email" << std::endl;
      authorChild->FirstChild()->SetValue("");
    }
    else
    {
      authorChild->FirstChild()->SetValue(this->authorEmail);
    }
  }
}

/////////////////////////////////////////////////
void BuildingMaker::SaveToConfig(const std::string &_savePath)
{
  boost::filesystem::path path(_savePath);
  path = path / "model.config";
  const char* modelConfigString = path.string().c_str();

  this->modelConfig.SaveFile(modelConfigString);
  gzdbg << "Saved file to " << modelConfigString << std::endl;
}

/////////////////////////////////////////////////
bool BuildingMaker::OnSave(const std::string &_saveName)
{
  if (_saveName != "")
    this->SetModelName(_saveName);

  switch (this->currentSaveState)
  {
    case UNSAVED_CHANGES:
    {
      // TODO: Subtle filesystem race condition
      this->SaveModelFiles();
      AddDirToModelPaths(this->saveLocation);
      gui::editor::Events::saveBuildingModel(this->modelName,
          this->saveLocation);
      return true;
    }
    case NEVER_SAVED:
    {
      return this->OnSaveAs(_saveName);
    }
    default:
      return false;
  }
}

/////////////////////////////////////////////////
bool BuildingMaker::OnSaveAs(const std::string &_saveName)
{
  this->saveDialog->SetModelName(_saveName);

  if (this->saveLocation.length() > 0)
  {
    this->saveDialog->SetSaveLocation(this->saveLocation);
  }
  if (this->saveDialog->exec() == QDialog::Accepted)
  {
    if (this->saveDialog->GetModelName().size() == 0)
    {
      QMessageBox msgBox(QMessageBox::Warning, QString("Empty Name"),
                       QString("Please give your model a non-empty name."));

      msgBox.exec();
      return this->OnSaveAs(_saveName);
    }
    if (this->saveDialog->GetSaveLocation().size() == 0)
    {
      QMessageBox msgBox(QMessageBox::Warning, QString("Empty Location"),
             QString("Please give a path to where your model will be saved."));

      msgBox.exec();
      return this->OnSaveAs(_saveName);
    }

    this->modelName = this->saveDialog->GetModelName();
    this->saveLocation = this->saveDialog->GetSaveLocation();
    this->authorName = this->saveDialog->GetAuthorName();
    this->authorEmail = this->saveDialog->GetAuthorEmail();
    this->description = this->saveDialog->GetDescription();
    this->version = this->saveDialog->GetVersion();

    if (this->modelName.compare(this->buildingDefaultName) == 0)
    {
      // Parse saveLocation and set model name
      boost::filesystem::path saveLocPath(this->saveLocation);
      this->SetModelName(saveLocPath.filename().string());
    }

    boost::filesystem::path path;
    path = path / this->saveLocation;
    if (!boost::filesystem::exists(path))
    {
      if (!boost::filesystem::create_directories(path))
      {
        gzerr << "Couldn't create folder for model files." << std::endl;
        return false;
      }
      gzmsg << "Created folder " << path << " for model files." << std::endl;
    }

    boost::filesystem::path modelConfigPath = path / "model.config";

    boost::filesystem::path sdfPath = path / "model.sdf";

    // Before writing
    if (boost::filesystem::exists(sdfPath) ||
          boost::filesystem::exists(modelConfigPath))
    {
      std::string msg = "A model named " + this->modelName +
                        " already exists in folder " + path.string() + ".\n\n"
                        "Do you wish to overwrite the existing model files?\n";

      QMessageBox msgBox(QMessageBox::Warning, QString("Files Exist"),
                         QString(msg.c_str()));

      QPushButton *saveButton = msgBox.addButton("Save",
                                                 QMessageBox::ApplyRole);
      msgBox.addButton(QMessageBox::Cancel);
      msgBox.exec();
      if (msgBox.clickedButton() != saveButton)
      {
        return this->OnSaveAs(this->modelName);
      }
    }

    this->SaveModelFiles();

    AddDirToModelPaths(this->saveLocation);

    gui::editor::Events::saveBuildingModel(this->modelName, this->saveLocation);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void BuildingMaker::OnNameChanged(const std::string &_name)
{
  if (_name.compare(this->modelName) == 0)
  {
    return;
  }
  this->SetModelName(_name);

  if (this->currentSaveState == NEVER_SAVED)
  {
    // Set new saveLocation
    boost::filesystem::path oldPath(this->saveLocation);

    boost::filesystem::path newPath = oldPath.parent_path() /
          GetFolderNameFromModelName(_name);
    this->saveLocation = newPath.string();
  }

  this->BuildingChanged();
}

/////////////////////////////////////////////////
void BuildingMaker::OnExit()
{
  if (this->allItems.empty())
  {
    this->Reset();
    gui::editor::Events::finishBuildingModel();
    return;
  }

  switch (this->currentSaveState)
  {
    case ALL_SAVED:
    {
      QString msg("Once you exit the Building Editor, "
      "your building will no longer be editable.\n\n"
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
          "Note: Once you exit the Building Editor, "
          "your building will no longer be editable.\n\n");

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
        if (!this->OnSave(this->modelName))
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

  gui::editor::Events::newBuildingModel();
  gui::editor::Events::finishBuildingModel();
}

/////////////////////////////////////////////////
void BuildingMaker::OnColorSelected(QColor _color)
{
  this->selectedTexture = "";
  this->selectedColor = _color;
}

/////////////////////////////////////////////////
void BuildingMaker::OnTextureSelected(QString _texture)
{
  this->selectedColor = QColor::Invalid;

  if (_texture != QString(""))
    this->selectedTexture = _texture;
}

/////////////////////////////////////////////////
bool BuildingMaker::On3dMouseMove(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  if (_event.dragging)
  {
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  if (this->selectedTexture == QString("") && !this->selectedColor.isValid())
  {
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);
  // Highlight visual on hover
  if (vis)
  {
    std::string visName = vis->GetParent()->GetName();

    // Stairs have nested visuals
    if (visName.find("Stair") != std::string::npos)
    {
      vis = vis->GetParent();
      visName = vis->GetParent()->GetName();
    }

    if (this->hoverVis && this->hoverVis != vis)
      this->ResetHoverVis();

    // Only handle items from building being edited
    visName = visName.substr(visName.find("::")+2);
    std::map<std::string, BuildingModelManip *>::const_iterator it =
        this->allItems.find(visName);
    if (it == this->allItems.end())
    {
      userCamera->HandleMouseEvent(_event);
      return true;
    }

    if (visName.find("Wall") != std::string::npos ||
        visName.find("Floor") != std::string::npos ||
        visName.find("Stair") != std::string::npos)
    {
      this->hoverVis = vis;
      if (this->selectedColor.isValid())
      {
        common::Color newColor(this->selectedColor.red(),
                               this->selectedColor.green(),
                               this->selectedColor.blue());
        this->hoverVis->SetAmbient(newColor);
      }
      else if (this->selectedTexture != "")
      {
        std::string material = "Gazebo/Grey";
        if (this->selectedTexture == ":wood.jpg")
          material = "Gazebo/Wood";
        else if (this->selectedTexture == ":tiles.jpg")
          material = "Gazebo/CeilingTiled";
        else if (this->selectedTexture == ":bricks.png")
          material = "Gazebo/Bricks";

        this->hoverVis->SetMaterial(material);
      }

      this->hoverVis->SetTransparency(0);
    }
    else
    {
      this->ResetHoverVis();
    }
  }
  else
  {
    this->ResetHoverVis();
  }
  return true;
}

/////////////////////////////////////////////////
bool BuildingMaker::On3dMousePress(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  userCamera->HandleMouseEvent(_event);
  return true;
}

/////////////////////////////////////////////////
bool BuildingMaker::On3dMouseRelease(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
  {
    this->StopMaterialModes();
    return true;
  }

  if (this->hoverVis)
  {
    std::string hoverName = this->hoverVis->GetParent()->GetName();
    hoverName = hoverName.substr(hoverName.find("::")+2);

    std::map<std::string, BuildingModelManip *>::const_iterator it =
        this->allItems.find(hoverName);
    if (it == this->allItems.end())
    {
      gzerr << "Visual " << hoverName << " is not part of the building but "
            << "was hovered. This should never happen." << std::endl;
    }
    else
    {
      BuildingModelManip *manip = this->allItems[hoverName];
      if (this->selectedColor.isValid())
      {
        manip->SetColor(this->selectedColor);
      }
      else if (this->selectedTexture != "")
      {
        manip->SetTexture(this->selectedTexture);
      }
    }
    this->hoverVis.reset();
  }
  else
  {
    rendering::UserCameraPtr userCamera = gui::get_active_camera();
    userCamera->HandleMouseEvent(_event);
    this->StopMaterialModes();
  }
  return true;
}

/////////////////////////////////////////////////
bool BuildingMaker::On3dKeyPress(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    this->StopMaterialModes();
  }
  return false;
}

/////////////////////////////////////////////////
void BuildingMaker::StopMaterialModes()
{
  this->ResetHoverVis();
  this->selectedColor = QColor::Invalid;
  gui::editor::Events::colorSelected(this->selectedColor.convertTo(
      QColor::Invalid));
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingMaker::ResetHoverVis()
{
  if (this->hoverVis)
  {
    std::string hoverName = this->hoverVis->GetParent()->GetName();
    hoverName = hoverName.substr(hoverName.find("::")+2);

    std::map<std::string, BuildingModelManip *>::const_iterator it =
        this->allItems.find(hoverName);
    if (it == this->allItems.end())
    {
      gzerr << "Visual " << hoverName << " is not part of the building but "
            << "was hovered. This should never happen." << std::endl;
    }
    else
    {
      BuildingModelManip *manip = this->allItems[hoverName];
      this->hoverVis->SetAmbient(manip->GetColor());
      this->hoverVis->SetMaterial(manip->GetTexture());
      this->hoverVis->SetTransparency(manip->GetTransparency());
    }
    this->hoverVis.reset();
  }
}

/////////////////////////////////////////////////
void BuildingMaker::OnChangeLevel(int _level)
{
  this->currentLevel = _level;
}

/////////////////////////////////////////////////
void BuildingMaker::BuildingChanged()
{
  if (this->currentSaveState != NEVER_SAVED)
    this->currentSaveState = UNSAVED_CHANGES;
}
