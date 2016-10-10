/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <sstream>
#include <boost/filesystem.hpp>

#include "gazebo/gazebo_config.h"

#include "gazebo/common/Exception.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/SaveEntityDialog.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/BuildingMakerPrivate.hh"
#include "gazebo/gui/building/BuildingModelManip.hh"
#include "gazebo/gui/building/EditorItem.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#ifdef HAVE_GTS
  #include "gazebo/common/Mesh.hh"
  #include "gazebo/common/MeshManager.hh"
  #include "gazebo/common/MeshCSG.hh"
#endif

using namespace gazebo;
using namespace gui;

const double BuildingMaker::conversionScale = 0.01;

/////////////////////////////////////////////////
BuildingMaker::BuildingMaker() : dataPtr(new BuildingMakerPrivate())
{
  this->dataPtr->buildingDefaultName = "Untitled";
  this->dataPtr->previewName = "BuildingPreview";

  // Counters are only used for giving visuals unique names.
  // FIXME they cannot be reset else gazebo complains about creating,
  // deleting duplicate visuals
  this->dataPtr->wallCounter = 0;
  this->dataPtr->windowCounter = 0;
  this->dataPtr->doorCounter = 0;
  this->dataPtr->stairsCounter = 0;
  this->dataPtr->floorCounter = 0;
  this->dataPtr->currentLevel = 0;

  this->dataPtr->modelTemplateSDF.reset(new sdf::SDF);
  this->dataPtr->modelTemplateSDF->SetFromString(this->TemplateSDFString());

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectSaveBuildingEditor(
      std::bind(&BuildingMaker::OnSave, this)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectSaveAsBuildingEditor(
      std::bind(&BuildingMaker::OnSaveAs, this)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectNewBuildingEditor(
      std::bind(&BuildingMaker::OnNew, this)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectExitBuildingEditor(
      std::bind(&BuildingMaker::OnExit, this)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectBuildingNameChanged(
      std::bind(&BuildingMaker::OnNameChanged, this, std::placeholders::_1)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectChangeBuildingLevel(
      std::bind(&BuildingMaker::OnChangeLevel, this, std::placeholders::_1)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectColorSelected(
      std::bind(&BuildingMaker::OnColorSelected, this, std::placeholders::_1)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectTextureSelected(
      std::bind(&BuildingMaker::OnTextureSelected, this,
      std::placeholders::_1)));
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectToggleEditMode(
      std::bind(&BuildingMaker::OnEdit, this, std::placeholders::_1)));

  this->dataPtr->saveDialog.reset(
      new SaveEntityDialog(SaveEntityDialog::BUILDING));

  // Transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->makerPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");

  this->Reset();
}

/////////////////////////////////////////////////
BuildingMaker::~BuildingMaker()
{
  this->dataPtr->modelSDF.reset();

  this->dataPtr->makerPub.reset();
  this->dataPtr->node->Fini();
  this->dataPtr->node.reset();
}

/////////////////////////////////////////////////
void BuildingMaker::OnEdit(bool _checked)
{
  if (_checked)
  {
    MouseEventHandler::Instance()->AddPressFilter("building_maker",
        std::bind(&BuildingMaker::On3dMousePress, this, std::placeholders::_1));
    MouseEventHandler::Instance()->AddReleaseFilter("building_maker",
        std::bind(&BuildingMaker::On3dMouseRelease, this,
        std::placeholders::_1));
    MouseEventHandler::Instance()->AddMoveFilter("building_maker",
        std::bind(&BuildingMaker::On3dMouseMove, this, std::placeholders::_1));
    KeyEventHandler::Instance()->AddPressFilter("building_maker",
        std::bind(&BuildingMaker::On3dKeyPress, this, std::placeholders::_1));
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
  auto manip = this->dataPtr->allItems[_partName];
  if (!manip)
    return;

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
  QObject::connect(_item, SIGNAL(ColorChanged(common::Color)),
      manip, SLOT(OnColorChanged(common::Color)));
  QObject::connect(_item, SIGNAL(TextureChanged(std::string)),
      manip, SLOT(OnTextureChanged(std::string)));
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
  QObject::connect(manip, SIGNAL(ColorChanged(common::Color)),
      _item, SLOT(OnColorChanged(common::Color)));
  QObject::connect(manip, SIGNAL(TextureChanged(std::string)),
      _item, SLOT(OnTextureChanged(std::string)));
}

/////////////////////////////////////////////////
void BuildingMaker::AttachManip(const std::string &_child,
    const std::string &_parent)
{
  auto it = this->dataPtr->attachmentMap.find(_parent);
  if (it != this->dataPtr->attachmentMap.end())
  {
    auto children = it->second;
    if (std::find(children.begin(), children.end(), _child) ==
        children.end())
    {
      it->second.push_back(_child);
    }
  }
  else
  {
    this->dataPtr->attachmentMap[_parent].push_back(_child);
  }
}

/////////////////////////////////////////////////
void BuildingMaker::DetachFromParent(const std::string &_child)
{
  for (auto &parentManip : this->dataPtr->attachmentMap)
  {
    parentManip.second.erase(std::remove(parentManip.second.begin(),
        parentManip.second.end(), _child), parentManip.second.end());

    if (parentManip.second.empty())
      this->DetachAllChildren(parentManip.first);
  }
}

/////////////////////////////////////////////////
void BuildingMaker::DetachAllChildren(const std::string &_parent)
{
  auto it = this->dataPtr->attachmentMap.find(_parent);
  if (it == this->dataPtr->attachmentMap.end())
    return;

  it->second.clear();
  this->dataPtr->attachmentMap.erase(_parent);
}

/////////////////////////////////////////////////
bool BuildingMaker::IsAttached(const std::string &_child) const
{
  for (auto const &parentManip : this->dataPtr->attachmentMap)
  {
    if (std::find(parentManip.second.begin(), parentManip.second.end(), _child)
        != parentManip.second.end())
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
BuildingModelManip *BuildingMaker::ManipByName(const std::string &_name)
{
  auto it = this->dataPtr->allItems.find(_name);
  if (it == this->dataPtr->allItems.end())
    return NULL;
  else
    return this->dataPtr->allItems[_name];
}

/////////////////////////////////////////////////
std::string BuildingMaker::CreateModel()
{
  this->Reset();
  return this->dataPtr->folderName;
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
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "Wall_" << this->dataPtr->wallCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      this->dataPtr->previewName + "::" + linkName,
      this->dataPtr->previewVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->dataPtr->previewName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem = this->dataPtr->modelTemplateSDF->Root()
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->ClearElements();
  visualElem->GetElement("material")->AddElement("ambient")
      ->Set(gazebo::common::Color(1, 1, 1));
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);
  auto scaledSize = BuildingMaker::ConvertSize(_size.x(), _size.y(), _size.z());
  BuildingModelManip *wallManip = new BuildingModelManip();
  wallManip->SetMaker(this);
  wallManip->SetName(linkName);
  wallManip->SetVisual(visVisual);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  wallManip->SetLevel(this->dataPtr->currentLevel);
  wallManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->dataPtr->allItems[linkName] = wallManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWindow(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  std::ostringstream linkNameStream;
  linkNameStream << "Window_" << this->dataPtr->windowCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      this->dataPtr->previewName + "::" + linkName,
      this->dataPtr->previewVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->dataPtr->previewName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->Root()
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")->GetElement("name")
      ->Set("Gazebo/BuildingFrame");
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);

  BuildingModelManip *windowManip = new BuildingModelManip();
  windowManip->SetMaker(this);
  windowManip->SetName(linkName);
  windowManip->SetVisual(visVisual);
  auto scaledSize = BuildingMaker::ConvertSize(_size.x(), _size.y(), _size.z());
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  windowManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  windowManip->SetLevel(this->dataPtr->currentLevel);
  this->dataPtr->allItems[linkName] = windowManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddDoor(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  /// TODO a copy of AddWindow function. FIXME later
  std::ostringstream linkNameStream;
  linkNameStream << "Door_" << this->dataPtr->doorCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      this->dataPtr->previewName + "::" + linkName,
      this->dataPtr->previewVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->dataPtr->previewName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->Root()
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")->GetElement("name")
      ->Set("Gazebo/BuildingFrame");
  visualElem->AddElement("cast_shadows")->Set(false);
  visVisual->Load(visualElem);

  BuildingModelManip *doorManip = new BuildingModelManip();
  doorManip->SetMaker(this);
  doorManip->SetName(linkName);
  doorManip->SetVisual(visVisual);
  auto scaledSize = BuildingMaker::ConvertSize(_size.x(), _size.y(), _size.z());
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  doorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  doorManip->SetLevel(this->dataPtr->currentLevel);
  this->dataPtr->allItems[linkName] = doorManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddStairs(const QVector3D &_size,
    const QVector3D &_pos, double _angle, int _steps)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  if (_steps == 0)
  {
    gzerr << "Can't make stairs with 0 steps" << std::endl;
    return "";
  }

  // Link visual
  std::ostringstream linkNameStream;
  linkNameStream << "Stairs_" << this->dataPtr->stairsCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      this->dataPtr->previewName + "::" + linkName,
      this->dataPtr->previewVisual));
  linkVisual->Load();

  // Size for the whole staircase as one thing
  auto totalSize = BuildingMaker::ConvertSize(_size.x(), _size.y(), _size.z());

  // Parent visual which will act as a container for the all the steps
  std::ostringstream visualName;
  visualName << this->dataPtr->previewName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  visVisual->Load();
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, totalSize.Z()/2.0));
  visVisual->SetScale(totalSize);

  // Visual SDF template (unit box)
  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->Root()
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->ClearElements();
  visualElem->GetElement("material")->AddElement("ambient")
      ->Set(gazebo::common::Color(1, 1, 1));
  visualElem->AddElement("cast_shadows")->Set(false);

  // Relative size of each step within the parent visual
  double dSteps = static_cast<double>(_steps);
  double rise = 1.0 / dSteps;
  double run = 1.0 / dSteps;

  for (int i = 0; i < _steps; ++i)
  {
    std::stringstream visualStepName;
    visualStepName << visualName.str() << "step" << i;
    rendering::VisualPtr stepVisual(new rendering::Visual(
        visualStepName.str(), visVisual));
    stepVisual->Load(visualElem);

    stepVisual->SetPosition(ignition::math::Vector3d(0, 0.5-run*(0.5+i),
        -0.5 + rise*(0.5+i)));
    stepVisual->SetScale(ignition::math::Vector3d(1, run, rise));
  }

  // Stairs manip
  BuildingModelManip *stairsManip = new BuildingModelManip();
  stairsManip->SetMaker(this);
  stairsManip->SetName(linkName);
  stairsManip->SetVisual(visVisual);
  stairsManip->SetLevel(this->dataPtr->currentLevel);
  stairsManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->dataPtr->allItems[linkName] = stairsManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->BuildingChanged();

  return linkName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddFloor(const QVector3D &_size,
    const QVector3D &_pos, double _angle)
{
  if (!this->dataPtr->previewVisual)
  {
    this->Reset();
  }

  /// TODO a copy of AddWindow function. FIXME later
  std::ostringstream linkNameStream;
  linkNameStream << "Floor_" << this->dataPtr->floorCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(
      this->dataPtr->previewName + "::" + linkName,
      this->dataPtr->previewVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->dataPtr->previewName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->dataPtr->modelTemplateSDF->Root()
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
  floorManip->SetLevel(this->dataPtr->currentLevel);
  auto scaledSize = BuildingMaker::ConvertSize(_size.x(), _size.y(), _size.z());
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  floorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->dataPtr->allItems[linkName] = floorManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->BuildingChanged();
  return linkName;
}

/////////////////////////////////////////////////
void BuildingMaker::RemovePart(const std::string &_partName)
{
  std::map<std::string, BuildingModelManip *>::const_iterator it =
      this->dataPtr->allItems.find(_partName);
  if (it == this->dataPtr->allItems.end())
  {
    gzerr << _partName << " does not exist\n";
    return;
  }
  BuildingModelManip *manip = this->dataPtr->allItems[_partName];

  rendering::VisualPtr vis = manip->Visual();
  rendering::VisualPtr visParent = vis->GetParent();
  rendering::ScenePtr scene = vis->GetScene();
  scene->RemoveVisual(vis);
  if (visParent)
    scene->RemoveVisual(visParent);
  this->dataPtr->allItems.erase(_partName);

  this->DetachAllChildren(_partName);
  this->DetachFromParent(_partName);

  delete manip;
  this->BuildingChanged();
}

/////////////////////////////////////////////////
void BuildingMaker::RemoveWall(const std::string &_wallName)
{
  this->RemovePart(_wallName);
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

  if (this->dataPtr->previewVisual)
    scene->RemoveVisual(this->dataPtr->previewVisual);

  this->dataPtr->currentSaveState = BuildingMakerPrivate::NEVER_SAVED;
  this->SetModelName(this->dataPtr->buildingDefaultName);

  this->dataPtr->previewVisual.reset(new rendering::Visual(
      this->dataPtr->previewName, scene->WorldVisual()));

  this->dataPtr->previewVisual->Load();
  this->dataPtr->previewVisual->SetPose(ignition::math::Pose3d::Zero);
  this->dataPtr->previewVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  for (auto it : this->dataPtr->allItems)
    delete it.second;
  this->dataPtr->allItems.clear();

  this->dataPtr->attachmentMap.clear();
}

/////////////////////////////////////////////////
void BuildingMaker::SetModelName(const std::string &_modelName)
{
  this->dataPtr->modelName = _modelName;
  this->dataPtr->saveDialog->SetModelName(_modelName);

  this->dataPtr->folderName = this->dataPtr->saveDialog->
      GetFolderNameFromModelName(this->dataPtr->modelName);

  if (this->dataPtr->currentSaveState == BuildingMakerPrivate::NEVER_SAVED)
  {
    // Set new saveLocation
    boost::filesystem::path oldPath(
        this->dataPtr->saveDialog->GetSaveLocation());

    auto newPath = oldPath.parent_path() / this->dataPtr->folderName;
    this->dataPtr->saveDialog->SetSaveLocation(newPath.string());
  }
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

  this->dataPtr->modelSDF.reset(new sdf::SDF);
  this->dataPtr->modelSDF->SetFromString(this->TemplateSDFString());

  modelElem = this->dataPtr->modelSDF->Root()->GetElement("model");

  linkElem = modelElem->GetElement("link");
  sdf::ElementPtr templateLinkElem = linkElem->Clone();
  sdf::ElementPtr templateVisualElem = templateLinkElem->GetElement(
      "visual")->Clone();
  sdf::ElementPtr templateCollisionElem = templateLinkElem->GetElement(
      "collision")->Clone();
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(this->dataPtr->folderName);
  auto modelOrigin = ignition::math::Pose3d::Zero;
  if (this->dataPtr->previewVisual)
  {
    modelOrigin = ignition::math::Pose3d(
      this->dataPtr->previewVisual->GetBoundingBox().GetCenter().x,
      this->dataPtr->previewVisual->GetBoundingBox().GetCenter().y, 0, 0, 0, 0);
  }
  modelElem->GetElement("pose")->Set(modelOrigin);

  // loop through all model manips
  for (const auto itemsIt : this->dataPtr->allItems)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    std::string name = itemsIt.first;
    BuildingModelManip *buildingModelManip = itemsIt.second;
    rendering::VisualPtr visual = buildingModelManip->Visual();
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(buildingModelManip->Name());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose() -
        modelOrigin);

    // Only stairs have children
    if (visual->GetChildCount() == 0)
    {
      // Window / Door not attached to walls
      if (name.find("Window") != std::string::npos
          || name.find("Door") != std::string::npos)
      {
        if (this->IsAttached(name))
          continue;
        visualElem->GetAttribute("name")->Set(buildingModelManip->Name()
            + "_Visual");
        collisionElem->GetAttribute("name")->Set(buildingModelManip->Name()
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
        auto parentManip = this->dataPtr->attachmentMap.find(name);
        if (parentManip != this->dataPtr->attachmentMap.end() &&
            !parentManip->second.empty())
        {
          std::vector<QRectF> holes;
          rendering::VisualPtr wallVis = visual;
          auto wallPose =
              wallVis->GetParent()->GetWorldPose().Ign() - modelOrigin;
          auto wallSize = wallVis->GetScale().Ign();
          for (auto const &childManip : parentManip->second)
          {
            auto attachedObj = this->ManipByName(childManip);
            if (!attachedObj)
            {
              gzerr << "Manip [" << childManip << "] not found." << std::endl;
              continue;
            }
            std::string objName = attachedObj->Name();
            if (objName.find("Window") != std::string::npos
                || objName.find("Door") != std::string::npos)
            {
              rendering::VisualPtr attachedVis = attachedObj->Visual();
              auto offset = attachedVis->GetParent()->GetWorldPose().Ign() -
                  modelOrigin - wallPose;
              auto size = attachedVis->GetScale().Ign();

              offset.Pos().Z() += attachedVis->GetPosition().z
                  - wallVis->GetPosition().z;

              auto newOffset = offset.Pos() - (-wallSize/2.0) - size/2.0;
              QRectF hole(newOffset.X(), newOffset.Z(), size.X(), size.Z());
              holes.push_back(hole);
            }
          }
          std::vector<QRectF> subdivisions;
          QRectF surface(0, 0, wallVis->GetScale().x, wallVis->GetScale().z);

          // subdivide the wall into mutiple compositing rectangles in order
          // to create holes for the attached children
          this->SubdivideRectSurface(surface, holes, subdivisions);

          newLinkElem->ClearElements();
          newLinkElem->GetAttribute("name")->Set(buildingModelManip->Name());
          newLinkElem->GetElement("pose")->Set(
              visual->GetParent()->GetWorldPose() - modelOrigin);
          // create a link element of box geom for each subdivision
          for (unsigned int i = 0; i< subdivisions.size(); ++i)
          {
            visualNameStream.str("");
            collisionNameStream.str("");
            visualElem = templateVisualElem->Clone();
            collisionElem = templateCollisionElem->Clone();
            visualNameStream << buildingModelManip->Name() << "_Visual_"
                << i;
            visualElem->GetAttribute("name")->Set(visualNameStream.str());
            collisionNameStream << buildingModelManip->Name()
                << "_Collision_" << i;
            collisionElem->GetAttribute("name")->Set(collisionNameStream.str());

            auto newSubPos = ignition::math::Vector3d(
                -wallSize.X()/2.0, 0, -wallSize.Z()/2.0)
                + ignition::math::Vector3d(
                subdivisions[i].x(), 0, subdivisions[i].y())
                + ignition::math::Vector3d(subdivisions[i].width()/2, 0,
                subdivisions[i].height()/2);
            newSubPos.Z() += wallSize.Z()/2;
            ignition::math::Pose3d newPose(newSubPos,
                ignition::math::Quaterniond(0, 0, 0));
            visualElem->GetElement("pose")->Set(newPose);
            collisionElem->GetElement("pose")->Set(newPose);
            ignition::math::Vector3d blockSize(subdivisions[i].width(),
                wallVis->GetScale().y, subdivisions[i].height());
            visualElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            collisionElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            visualElem->GetElement("material")->GetElement("ambient")->
                Set(buildingModelManip->Color());
            visualElem->GetElement("material")->GetElement("script")
                ->GetElement("name")->Set(buildingModelManip->Texture());
            visualElem->GetElement("meta")->GetElement("layer")
                ->Set(buildingModelManip->Level());
            newLinkElem->InsertElement(visualElem);
            newLinkElem->InsertElement(collisionElem);
          }
        }
        // Wall without windows or doors
        else
        {
          visualElem->GetAttribute("name")->Set(buildingModelManip->Name()
              + "_Visual");
          collisionElem->GetAttribute("name")->Set(buildingModelManip->Name()
              + "_Collision");
          visualElem->GetElement("pose")->Set(visual->GetPose());
          collisionElem->GetElement("pose")->Set(visual->GetPose());
          visualElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          collisionElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          visualElem->GetElement("material")->GetElement("ambient")->
              Set(buildingModelManip->Color());
          visualElem->GetElement("material")->GetElement("script")
              ->GetElement("name")->Set(buildingModelManip->Texture());
          visualElem->GetElement("meta")->GetElement("layer")
              ->Set(buildingModelManip->Level());
        }
      }
      // Floor
      else if (name.find("Floor") != std::string::npos)
      {
        // check if floors have attached children, i.e. stairs
        auto parentManip = this->dataPtr->attachmentMap.find(name);
        if (parentManip != this->dataPtr->attachmentMap.end() &&
            !parentManip->second.empty())
        {
          std::vector<QRectF> holes;
          rendering::VisualPtr floorVis = visual;
          auto floorPose = floorVis->GetWorldPose().Ign() - modelOrigin;
          auto floorSize = floorVis->GetScale().Ign();
          for (auto const &childManip : parentManip->second)
          {
            auto attachedObj = this->ManipByName(childManip);
            if (!attachedObj)
            {
              gzerr << "Manip [" << childManip << "] not found." << std::endl;
              continue;
            }
            std::string objName = attachedObj->Name();
            if (objName.find("Stairs") != std::string::npos)
            {
              rendering::VisualPtr attachedVis = attachedObj->Visual();
              auto offset = attachedVis->GetParent()->GetWorldPose().Ign() -
                  modelOrigin - floorPose;
              auto size = attachedVis->GetScale().Ign();

              QRectF rect(0, 0, size.X(), size.Y());
              QPolygonF polygon(rect);
              QTransform transform;
              transform.rotate(IGN_RTOD(offset.Rot().Euler().Z()));
              QRectF bound = transform.map(polygon).boundingRect();
              auto newOffset = offset.Pos() - (-floorSize/2.0)
                  - ignition::math::Vector3d(bound.width(), bound.height(),
                  size.Z())/2.0;
              QRectF hole(newOffset.X(), newOffset.Y(), bound.width(),
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
          newLinkElem->GetAttribute("name")->Set(buildingModelManip->Name());
          newLinkElem->GetElement("pose")->Set(
              visual->GetParent()->GetWorldPose() - modelOrigin);
          // create a link element of box geom for each subdivision
          for (unsigned int i = 0; i< subdivisions.size(); ++i)
          {
            visualNameStream.str("");
            collisionNameStream.str("");
            visualElem = templateVisualElem->Clone();
            collisionElem = templateCollisionElem->Clone();
            visualNameStream << buildingModelManip->Name() << "_Visual_"
                << i;
            visualElem->GetAttribute("name")->Set(visualNameStream.str());
            collisionNameStream << buildingModelManip->Name()
                << "_Collision_" << i;
            collisionElem->GetAttribute("name")->Set(collisionNameStream.str());

            auto newSubPos = ignition::math::Vector3d(-floorSize.X()/2.0,
                -floorSize.Y()/2.0, 0)
                + ignition::math::Vector3d(subdivisions[i].x(),
                subdivisions[i].y(), 0)
                + ignition::math::Vector3d(subdivisions[i].width()/2,
                subdivisions[i].height()/2, 0);
            newSubPos.Z(newSubPos.Z() + floorVis->GetPosition().z);
            ignition::math::Pose3d newPose(newSubPos,
                visual->GetParent()->GetRotation().Ign());
            visualElem->GetElement("pose")->Set(newPose);
            collisionElem->GetElement("pose")->Set(newPose);
            ignition::math::Vector3d blockSize(subdivisions[i].width(),
                subdivisions[i].height(), floorVis->GetScale().z);
            visualElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            collisionElem->GetElement("geometry")->GetElement("box")->
                GetElement("size")->Set(blockSize);
            visualElem->GetElement("material")->GetElement("ambient")->
                Set(buildingModelManip->Color());
            visualElem->GetElement("material")->GetElement("script")
                ->GetElement("name")->Set(buildingModelManip->Texture());
            visualElem->GetElement("meta")->GetElement("layer")
                ->Set(buildingModelManip->Level());
            newLinkElem->InsertElement(visualElem);
            newLinkElem->InsertElement(collisionElem);
          }
        }
        // Floor without stairs
        else
        {
          visualElem->GetAttribute("name")->Set(buildingModelManip->Name()
              + "_Visual");
          collisionElem->GetAttribute("name")->Set(buildingModelManip->Name()
              + "_Collision");
          visualElem->GetElement("pose")->Set(visual->GetPose());
          collisionElem->GetElement("pose")->Set(visual->GetPose());
          visualElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          collisionElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(visual->GetScale());
          visualElem->GetElement("material")->GetElement("ambient")->
              Set(buildingModelManip->Color());
          visualElem->GetElement("material")->GetElement("script")
              ->GetElement("name")->Set(buildingModelManip->Texture());
          visualElem->GetElement("meta")->GetElement("layer")
              ->Set(buildingModelManip->Level());
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
        visualNameStream << buildingModelManip->Name() << "_Visual_" << i;
        visualElem->GetAttribute("name")->Set(visualNameStream.str());
        collisionNameStream << buildingModelManip->Name()
            << "_Collision_" << i;
        collisionElem->GetAttribute("name")->Set(collisionNameStream.str());
        rendering::VisualPtr childVisual = visual->GetChild(i);
        ignition::math::Pose3d newPose(childVisual->GetWorldPose().pos.Ign() -
            modelOrigin.Pos(), visual->GetParent()->GetRotation().Ign());
        visualElem->GetElement("pose")->Set(newPose);
        collisionElem->GetElement("pose")->Set(newPose);
        visualElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale()*childVisual->GetScale());
        collisionElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(visual->GetScale()*childVisual->GetScale());
        visualElem->GetElement("material")->GetElement("ambient")->
              Set(buildingModelManip->Color());
        visualElem->GetElement("material")->GetElement("script")
            ->GetElement("name")->Set(buildingModelManip->Texture());
        visualElem->GetElement("meta")->GetElement("layer")
            ->Set(buildingModelManip->Level());
        newLinkElem->InsertElement(visualElem);
        newLinkElem->InsertElement(collisionElem);
      }
    }
    modelElem->InsertElement(newLinkElem);
  }
  (modelElem->AddElement("static"))->Set("true");
  // qDebug() << this->dataPtr->modelSDF->ToString().c_str();
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

  this->dataPtr->modelSDF.reset(new sdf::SDF);
  this->dataPtr->modelSDF->SetFromString(this->TemplateSDFString());

  modelElem = this->dataPtr->modelSDF->Root()->GetElement("model");
  linkElem = modelElem->GetElement("link");

  sdf::ElementPtr templateLinkElem = linkElem->Clone();
  sdf::ElementPtr templateVisualElem = templateLinkElem->GetElement(
      "visual")->Clone();
  sdf::ElementPtr templateCollisionElem = templateLinkElem->GetElement(
      "collision")->Clone();
  modelElem->ClearElements();
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;

  modelElem->GetAttribute("name")->Set(this->dataPtr->folderName);

  for (auto itemsIt : this->dataPtr->allItems)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    std::string name = itemsIt.first;
    BuildingModelManip *buildingModelManip = itemsIt.second;
    rendering::VisualPtr visual = buildingModelManip->Visual();
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(buildingModelManip->Name());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose());

    // create a hole to represent a window/door in the wall
    auto parentManip = this->dataPtr->attachmentMap.find(name);
    if (name.find("Window") != std::string::npos
        || name.find("Door") != std::string::npos)
    {
      if (!this->IsAttached(name))
        continue;
    }
    else if (name.find("Wall") != std::string::npos
        && !parentManip->second.empty())
    {
      rendering::VisualPtr wallVis = visual;
      auto wallPose = wallVis->GetWorldPose().Ign();
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
          m1SubMesh->AddVertex(subMesh->Vertex(j));
        }
        for (unsigned int j = 0; j < subMesh->GetIndexCount(); ++j)
        {
          m1SubMesh->AddIndex(subMesh->GetIndex(j));
        }
      }
      m1->SetScale(wallVis->GetScale().Ign());

      std::string booleanMeshName = buildingModelManip->Name() + "_Boolean";
      common::Mesh *booleanMesh = NULL;
      for (auto const &childManip : parentManip->second)
      {
        if (booleanMesh)
        {
          delete m1;
          m1 = booleanMesh;
        }

        auto attachedObj = this->ManipByName(childManip);
        if (!attachedObj)
        {
          gzerr << "Manip [" << childManip << "] not found." << std::endl;
          continue;
        }
        std::string objName = attachedObj->Name();
        if (objName.find("Window") != std::string::npos
            || objName.find("Door") != std::string::npos)
        {
          rendering::VisualPtr attachedVis = attachedObj->Visual();
          auto offset = attachedVis->GetWorldPose().Ign() - wallPose;
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
              m2SubMesh->AddVertex(subMesh->Vertex(j));
            }
            for (unsigned int j = 0; j < subMesh->GetIndexCount(); ++j)
            {
              m2SubMesh->AddIndex(subMesh->GetIndex(j));
            }
          }
          m2->SetScale(attachedVis->GetScale().Ign());
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
      visualElem->GetAttribute("name")->Set(buildingModelManip->Name()
          + "_Visual");
      collisionElem->GetAttribute("name")->Set(buildingModelManip->Name()
          + "_Collision");
      sdf::ElementPtr visGeomElem = visualElem->GetElement("geometry");
      visGeomElem->ClearElements();
      sdf::ElementPtr meshElem = visGeomElem->AddElement("mesh");
      // TODO create the folder
      std::string uri = "model://" + this->dataPtr->folderName + "/meshes/"
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
        visualNameStream << buildingModelManip->Name() << "_Visual_" << i;
        visualElem->GetAttribute("name")->Set(visualNameStream.str());
        collisionNameStream << buildingModelManip->Name() << "_Collision_"
            << i;
        collisionElem->GetAttribute("name")->Set(collisionNameStream.str());
        rendering::VisualPtr childVisual = visual->GetChild(i);
        ignition::math::Pose3d newPose(childVisual->GetWorldPose().pos.Ign(),
            visual->GetRotation().Ign());
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
  if (!this->dataPtr->modelSDF->Root()->HasElement("model"))
  {
    gzerr << "Generated invalid SDF! Cannot create entity." << std::endl;
    return;
  }

  msgs::Factory msg;
  // Create a new name if the model exists
  sdf::ElementPtr modelElem =
      this->dataPtr->modelSDF->Root()->GetElement("model");
  std::string modelElemName = modelElem->Get<std::string>("name");
  if (has_entity_name(modelElemName))
  {
    int i = 0;
    while (has_entity_name(modelElemName))
    {
      modelElemName = modelElem->Get<std::string>("name") + "_" +
          std::to_string(i++);
    }
    modelElem->GetAttribute("name")->Set(modelElemName);
  }

  msg.set_sdf(this->dataPtr->modelSDF->ToString());
  this->dataPtr->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
ignition::math::Vector3d BuildingMaker::ConvertSize(const double _width,
    const double _length, const double _height)
{
  return ignition::math::Vector3d(conversionScale*_width,
                                  conversionScale*_length,
                                  conversionScale*_height);
}

/////////////////////////////////////////////////
ignition::math::Pose3d BuildingMaker::ConvertPose(const double _x,
    const double _y, const double _z, const double _roll, const double _pitch,
    const double _yaw)
{
  return ignition::math::Pose3d(conversionScale*_x,
                                conversionScale*_y,
                                conversionScale*_z,
                                IGN_DTOR(_roll),
                                IGN_DTOR(_pitch),
                                IGN_DTOR(_yaw));
}

/////////////////////////////////////////////////
double BuildingMaker::Convert(double _value)
{
  return conversionScale*_value;
}

/////////////////////////////////////////////////
double BuildingMaker::ConvertAngle(double _angle)
{
  return IGN_DTOR(_angle);
}

/////////////////////////////////////////////////
std::string BuildingMaker::TemplateSDFString() const
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
  if (this->dataPtr->allItems.empty())
  {
    this->Reset();
    gui::editor::Events::newBuildingModel();
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
    case BuildingMakerPrivate::ALL_SAVED:
    {
      msg.append("Are you sure you want to close this model and open a new "
                 "canvas?\n\n");
      QPushButton *newButton =
          msgBox.addButton("New Canvas", QMessageBox::AcceptRole);
      msgBox.setDefaultButton(newButton);
      break;
    }
    case BuildingMakerPrivate::UNSAVED_CHANGES:
    case BuildingMakerPrivate::NEVER_SAVED:
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
    gui::editor::Events::newBuildingModel();
  }
}

/////////////////////////////////////////////////
void BuildingMaker::SaveModelFiles()
{
  this->dataPtr->saveDialog->GenerateConfig();
  this->dataPtr->saveDialog->SaveToConfig();
  this->GenerateSDF();
  this->dataPtr->saveDialog->SaveToSDF(this->dataPtr->modelSDF);
  this->dataPtr->currentSaveState = BuildingMakerPrivate::ALL_SAVED;
}

/////////////////////////////////////////////////
std::string BuildingMaker::ModelSDF() const
{
  return this->dataPtr->modelSDF->ToString();
}

/////////////////////////////////////////////////
bool BuildingMaker::OnSave()
{
  switch (this->dataPtr->currentSaveState)
  {
    case BuildingMakerPrivate::UNSAVED_CHANGES:
    {
      // TODO: Subtle filesystem race condition
      this->SaveModelFiles();
      gui::editor::Events::saveBuildingModel(this->dataPtr->modelName);
      return true;
    }
    case BuildingMakerPrivate::NEVER_SAVED:
    {
      return this->OnSaveAs();
    }
    default:
      return false;
  }
}

/////////////////////////////////////////////////
bool BuildingMaker::OnSaveAs()
{
  if (this->dataPtr->saveDialog->OnSaveAs())
  {
    // Prevent changing save location
    this->dataPtr->currentSaveState = BuildingMakerPrivate::ALL_SAVED;
    // Get name set by user
    this->SetModelName(this->dataPtr->saveDialog->GetModelName());
    // Update name on palette
    gui::editor::Events::saveBuildingModel(this->dataPtr->modelName);
    // Generate and save files
    this->SaveModelFiles();
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void BuildingMaker::OnNameChanged(const std::string &_name)
{
  if (_name.compare(this->dataPtr->modelName) == 0)
    return;

  this->SetModelName(_name);
  this->BuildingChanged();
}

/////////////////////////////////////////////////
void BuildingMaker::OnExit()
{
  if (this->dataPtr->allItems.empty())
  {
    this->Reset();
    gui::editor::Events::newBuildingModel();
    gui::editor::Events::finishBuildingModel();
    return;
  }

  switch (this->dataPtr->currentSaveState)
  {
    case BuildingMakerPrivate::ALL_SAVED:
    {
      QString msg("Once you exit the Building Editor, "
      "your building will no longer be editable.\n\n"
      "Are you ready to exit?\n\n");
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
    case BuildingMakerPrivate::UNSAVED_CHANGES:
    case BuildingMakerPrivate::NEVER_SAVED:
    {
      QString msg("Save Changes before exiting?\n\n"
          "Note: Once you exit the Building Editor, "
          "your building will no longer be editable.\n\n");

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
  if (this->dataPtr->currentSaveState != BuildingMakerPrivate::NEVER_SAVED)
    this->FinishModel();

  this->Reset();

  gui::editor::Events::newBuildingModel();
  gui::editor::Events::finishBuildingModel();
}

/////////////////////////////////////////////////
void BuildingMaker::OnColorSelected(QColor _color)
{
  this->dataPtr->selectedTexture = "";
  this->dataPtr->selectedColor = _color;
}

/////////////////////////////////////////////////
void BuildingMaker::OnTextureSelected(QString _texture)
{
  this->dataPtr->selectedColor = QColor::Invalid;
  this->dataPtr->selectedTexture = _texture;
}

/////////////////////////////////////////////////
bool BuildingMaker::On3dMouseMove(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  if (_event.Dragging())
  {
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  if (this->dataPtr->selectedTexture == QString("") &&
      !this->dataPtr->selectedColor.isValid())
  {
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  rendering::VisualPtr vis = userCamera->GetVisual(_event.Pos());
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

    if (this->dataPtr->hoverVis && this->dataPtr->hoverVis != vis)
      this->ResetHoverVis();

    // Only handle items from building being edited
    visName = visName.substr(visName.find("::")+2);
    std::map<std::string, BuildingModelManip *>::const_iterator it =
        this->dataPtr->allItems.find(visName);
    if (it == this->dataPtr->allItems.end())
    {
      userCamera->HandleMouseEvent(_event);
      return true;
    }

    if (visName.find("Wall") != std::string::npos ||
        visName.find("Floor") != std::string::npos ||
        visName.find("Stair") != std::string::npos)
    {
      this->dataPtr->hoverVis = vis;
      if (this->dataPtr->selectedColor.isValid())
      {
        common::Color newColor(this->dataPtr->selectedColor.red(),
                               this->dataPtr->selectedColor.green(),
                               this->dataPtr->selectedColor.blue());
        this->dataPtr->hoverVis->SetAmbient(newColor);
      }
      else if (this->dataPtr->selectedTexture != "")
      {
        std::string material = "Gazebo/Grey";
        if (this->dataPtr->selectedTexture == ":wood.jpg")
          material = "Gazebo/Wood";
        else if (this->dataPtr->selectedTexture == ":tiles.jpg")
          material = "Gazebo/CeilingTiled";
        else if (this->dataPtr->selectedTexture == ":bricks.png")
          material = "Gazebo/Bricks";

        // Must set material before color, otherwise color is overwritten
        this->dataPtr->hoverVis->SetMaterial(material);
        this->dataPtr->hoverVis->SetAmbient((*it).second->Color());
      }

      this->dataPtr->hoverVis->SetTransparency(0);
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
  rendering::UserCameraPtr userCamera = gui::get_active_camera();

  if (_event.Dragging())
  {
    userCamera->HandleMouseEvent(_event);
    return true;
  }

  if (_event.Button() != common::MouseEvent::LEFT)
  {
    this->StopMaterialModes();
    return true;
  }

  if (this->dataPtr->hoverVis)
  {
    std::string hoverName = this->dataPtr->hoverVis->GetParent()->GetName();
    hoverName = hoverName.substr(hoverName.find("::")+2);

    std::map<std::string, BuildingModelManip *>::const_iterator it =
        this->dataPtr->allItems.find(hoverName);
    if (it == this->dataPtr->allItems.end())
    {
      gzerr << "Visual " << hoverName << " is not part of the building but "
            << "was hovered. This should never happen." << std::endl;
    }
    else
    {
      BuildingModelManip *manip = this->dataPtr->allItems[hoverName];
      if (this->dataPtr->selectedColor.isValid())
      {
        manip->SetColor(this->dataPtr->selectedColor);
      }
      else if (this->dataPtr->selectedTexture != "")
      {
        manip->SetTexture(this->dataPtr->selectedTexture);
      }
    }
    this->dataPtr->hoverVis.reset();
  }
  else
  {
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
  this->dataPtr->selectedColor = QColor::Invalid;
  gui::editor::Events::colorSelected(this->dataPtr->selectedColor.convertTo(
      QColor::Invalid));
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingMaker::ResetHoverVis()
{
  if (this->dataPtr->hoverVis)
  {
    std::string hoverName = this->dataPtr->hoverVis->GetParent()->GetName();
    hoverName = hoverName.substr(hoverName.find("::")+2);

    std::map<std::string, BuildingModelManip *>::const_iterator it =
        this->dataPtr->allItems.find(hoverName);
    if (it == this->dataPtr->allItems.end())
    {
      gzerr << "Visual " << hoverName << " is not part of the building but "
            << "was hovered. This should never happen." << std::endl;
    }
    else
    {
      BuildingModelManip *manip = this->dataPtr->allItems[hoverName];
      // Must set material before color, otherwise color is overwritten
      this->dataPtr->hoverVis->SetMaterial(manip->Texture());
      this->dataPtr->hoverVis->SetAmbient(manip->Color());
      this->dataPtr->hoverVis->SetTransparency(manip->Transparency());
    }
    this->dataPtr->hoverVis.reset();
  }
}

/////////////////////////////////////////////////
void BuildingMaker::OnChangeLevel(int _level)
{
  this->dataPtr->currentLevel = _level;
}

/////////////////////////////////////////////////
void BuildingMaker::BuildingChanged()
{
  if (this->dataPtr->currentSaveState != BuildingMakerPrivate::NEVER_SAVED)
    this->dataPtr->currentSaveState = BuildingMakerPrivate::UNSAVED_CHANGES;
}
