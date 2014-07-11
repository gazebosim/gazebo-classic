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
#include <ignition/math/Quaternion.hh>

#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"


#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/EntityMaker.hh"

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
  BuildingMaker::BuildingMaker() : EntityMaker()
{
  this->modelName = "";

  this->conversionScale = 0.01;

  // Counters are only used for giving visuals unique names.
  // FIXME they cannot be reset else gazebo complains about creating,
  // deleting duplicate visuals
  this->wallCounter = 0;
  this->windowCounter = 0;
  this->doorCounter = 0;
  this->stairsCounter = 0;
  this->floorCounter = 0;

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());

  this->connections.push_back(
  gui::editor::Events::ConnectSaveBuildingEditor(
    boost::bind(&BuildingMaker::OnSave, this)));
  this->connections.push_back(
  gui::editor::Events::ConnectDiscardBuildingEditor(
    boost::bind(&BuildingMaker::OnDiscard, this)));
  this->connections.push_back(
  gui::editor::Events::ConnectDoneBuildingEditor(
    boost::bind(&BuildingMaker::OnDone, this)));
  this->connections.push_back(
  gui::editor::Events::ConnectExitBuildingEditor(
    boost::bind(&BuildingMaker::OnExit, this)));

  this->buildingDefaultName = "BuildingDefaultName";

  this->saveDialog =
      new FinishBuildingDialog(FinishBuildingDialog::MODEL_SAVE, 0);
  this->finishDialog =
      new FinishBuildingDialog(FinishBuildingDialog::MODEL_FINISH, 0);
}

/////////////////////////////////////////////////
BuildingMaker::~BuildingMaker()
{
//  this->camera.reset();
  if (this->saveDialog)
    delete this->saveDialog;
  if (this->finishDialog)
    delete this->finishDialog;
}

/////////////////////////////////////////////////
void BuildingMaker::ConnectItem(const std::string &_partName,
    const EditorItem *_item)
{
  BuildingModelManip *manip = this->allItems[_partName];

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
}

/////////////////////////////////////////////////
void BuildingMaker::AttachManip(const std::string &_child,
    const std::string &_parent)
{
  BuildingModelManip *child = this->allItems[_child];
  BuildingModelManip *parent = this->allItems[_parent];
  parent->AttachManip(child);
}

/////////////////////////////////////////////////
void BuildingMaker::DetachManip(const std::string &_child,
    const std::string &_parent)
{
  BuildingModelManip *child = this->allItems[_child];
  BuildingModelManip *parent = this->allItems[_parent];
  parent->DetachManip(child);
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
  linkNameStream << "Wall_" << wallCounter++;
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
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/OrangeTransparent");
  visVisual->Load(visualElem);
  ignition::math::Vector3d scaledSize = BuildingMaker::ConvertSize(_size);
  BuildingModelManip *wallManip = new BuildingModelManip();
  wallManip->SetMaker(this);
  wallManip->SetName(linkName);
  wallManip->SetVisual(visVisual);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  wallManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = wallManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
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
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/BlueTransparent");
  visVisual->Load(visualElem);

  BuildingModelManip *windowManip = new BuildingModelManip();
  windowManip->SetMaker(this);
  windowManip->SetName(linkName);
  windowManip->SetVisual(visVisual);
  ignition::math::Vector3d scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  windowManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = windowManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
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
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/YellowTransparent");
  visVisual->Load(visualElem);

  BuildingModelManip *doorManip = new BuildingModelManip();
  doorManip->SetMaker(this);
  doorManip->SetName(linkName);
  doorManip->SetVisual(visVisual);
  ignition::math::Vector3d scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  doorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = doorManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
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
  ignition::math::Vector3d scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  double dSteps = static_cast<double>(_steps);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  stairsManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = stairsManip;

  std::stringstream visualStepName;
  visualStepName << visualName.str() << "step" << 0;
  rendering::VisualPtr baseStepVisual(new rendering::Visual(
      visualStepName.str(), visVisual));
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/GreenTransparent");
  baseStepVisual->Load(visualElem);

  double rise = 1.0 / dSteps;
  double run = 1.0 / dSteps;
  baseStepVisual->SetScale(ignition::math::Vector3d(1, run, rise));

  ignition::math::Vector3d baseOffset(0, 0.5 - run/2.0,
      -0.5 + rise/2.0);
  baseStepVisual->SetPosition(baseOffset);


  for (int i = 1; i < _steps; ++i)
  {
    visualStepName.str("");
    visualStepName << visualName.str() << "step" << i;
    rendering::VisualPtr stepVisual = baseStepVisual->Clone(
        visualStepName.str(), visVisual);
    stepVisual->SetPosition(ignition::math::Vector3d(0, baseOffset.Y()-(run*i),
        baseOffset.Z() + rise*i));
    stepVisual->SetRotation(baseStepVisual->Rotation());
  }

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
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
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/OrangeTransparent");
  visVisual->Load(visualElem);

  BuildingModelManip *floorManip = new BuildingModelManip();
  floorManip->SetMaker(this);
  floorManip->SetName(linkName);
  floorManip->SetVisual(visVisual);
  ignition::math::Vector3d scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(ignition::math::Vector3d(0, 0, scaledSize.Z()/2.0));
  floorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[linkName] = floorManip;

  linkVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  return linkName;
}

/////////////////////////////////////////////////
void BuildingMaker::RemovePart(const std::string &_partName)
{
  BuildingModelManip *manip = this->allItems[_partName];
  if (!manip)
  {
    gzerr << _partName << " does not exist\n";
    return;
  }
  rendering::VisualPtr vis = manip->GetVisual();
  rendering::VisualPtr visParent = vis->GetParent();
  rendering::ScenePtr scene = vis->GetScene();
  scene->RemoveVisual(vis);
  if (visParent)
    scene->RemoveVisual(visParent);
  this->allItems.erase(_partName);
  delete manip;
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

  if (this->modelVisual)
    scene->RemoveVisual(this->modelVisual);

  this->saved = false;
  this->saveLocation = QDir::homePath().toStdString();
  this->modelName = this->buildingDefaultName;

  this->modelVisual.reset(new rendering::Visual(this->modelName,
      scene->GetWorldVisual()));

  this->modelVisual->Load();
  this->modelPose = ignition::math::Pose3d::Zero;
  this->modelVisual->SetPose(this->modelPose);
  this->modelVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
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
}

/////////////////////////////////////////////////
void BuildingMaker::SaveToSDF(const std::string &_savePath)
{
  this->saveLocation = _savePath;
  std::ofstream savefile;
  boost::filesystem::path path;
  path = boost::filesystem::operator/(this->saveLocation,
      this->modelName + ".sdf");
  savefile.open(path.string().c_str());
  savefile << this->modelSDF->ToString();
  savefile.close();
}

/////////////////////////////////////////////////
void BuildingMaker::FinishModel()
{
  this->CreateTheEntity();
  this->Stop();
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

  modelElem->GetAttribute("name")->Set(this->modelName);

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

    if (visual->GetChildCount() == 0)
    {
      // subdivide wall surface to create holes for representing
      // window/doors
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
      // check if walls have attached children, i.e. windows or doors
      else if (name.find("Wall") != std::string::npos)
      {
        if (buildingModelManip->GetAttachedManipCount() != 0 )
        {
          std::vector<QRectF> holes;
          rendering::VisualPtr wallVis = visual;
          ignition::math::Pose3d wallPose =
            wallVis->GetParent()->GetWorldPose();
          ignition::math::Vector3d wallSize = wallVis->GetScale();
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
              ignition::math::Pose3d offset =
                attachedVis->GetParent()->GetWorldPose() - wallPose;
              ignition::math::Vector3d size = attachedVis->GetScale();

              offset.Pos().Z() += attachedVis->GetPosition().Z()
                  - wallVis->GetPosition().Z();

              ignition::math::Vector3d newOffset =
                offset.Pos() - (-wallSize/2.0) - size/2.0;
              QRectF hole(newOffset.X(), newOffset.Z(), size.X(), size.Z());
              holes.push_back(hole);
            }
          }
          std::vector<QRectF> subdivisions;
          QRectF surface(0, 0,
              wallVis->GetScale().X(), wallVis->GetScale().Z());

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

            ignition::math::Vector3d newSubPos =
              ignition::math::Vector3d(-wallSize.X()/2.0, 0, -wallSize.Z()/2.0)
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
                wallVis->GetScale().Y(), subdivisions[i].height());
            visualElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(blockSize);
            collisionElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(blockSize);
            newLinkElem->InsertElement(visualElem);
            newLinkElem->InsertElement(collisionElem);
          }
        }
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
        }
      }
      // check if floors have attached children, i.e. stairs
      else if (name.find("Floor") != std::string::npos)
      {
        if (buildingModelManip->GetAttachedManipCount() != 0 )
        {
          std::vector<QRectF> holes;
          rendering::VisualPtr floorVis = visual;
          ignition::math::Pose3d floorPose = floorVis->GetWorldPose();
          ignition::math::Vector3d floorSize = floorVis->GetScale();
          for (unsigned int i = 0; i <
              buildingModelManip->GetAttachedManipCount(); ++i)
          {
            BuildingModelManip *attachedObj =
                buildingModelManip->GetAttachedManip(i);
            std::string objName = attachedObj->GetName();
            if (objName.find("Stairs") != std::string::npos)
            {
              rendering::VisualPtr attachedVis = attachedObj->GetVisual();
              ignition::math::Pose3d offset =
                attachedVis->GetParent()->GetWorldPose() - floorPose;
              ignition::math::Vector3d size = attachedVis->GetScale();

              QRectF rect(0, 0, size.X(), size.Y());
              QPolygonF polygon(rect);
              QTransform transform;
              transform.rotate(IGN_RTOD(offset.Rot().Euler().Z()));
              QRectF bound = transform.map(polygon).boundingRect();
              ignition::math::Vector3d newOffset =
                offset.Pos() - (-floorSize/2.0)
                - ignition::math::Vector3d(
                    bound.width(), bound.height(), size.Z())/2.0;

              QRectF hole(newOffset.X(), newOffset.Y(), bound.width(),
                  bound.height());
              holes.push_back(hole);
            }
          }
          std::vector<QRectF> subdivisions;
          QRectF surface(0, 0, floorVis->GetScale().X(),
              floorVis->GetScale().Y());

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

            ignition::math::Vector3d newSubPos =
              ignition::math::Vector3d(
                  -floorSize.X()/2.0, -floorSize.Y()/2.0, 0)
              + ignition::math::Vector3d(
                  subdivisions[i].x(), subdivisions[i].y(), 0)
              + ignition::math::Vector3d(subdivisions[i].width()/2,
                  subdivisions[i].height()/2, 0);

            newSubPos.Z() += floorVis->GetPosition().Z();
            ignition::math::Pose3d newPose(newSubPos,
                visual->GetParent()->Rotation());

            visualElem->GetElement("pose")->Set(newPose);
            collisionElem->GetElement("pose")->Set(newPose);
            ignition::math::Vector3d blockSize(subdivisions[i].width(),
                subdivisions[i].height(), floorVis->GetScale().Z());
            visualElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(blockSize);
            collisionElem->GetElement("geometry")->GetElement("box")->
              GetElement("size")->Set(blockSize);
            newLinkElem->InsertElement(visualElem);
            newLinkElem->InsertElement(collisionElem);
          }
        }
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
        }
      }
    }
    else
    {
      // TODO: This handles the special case for stairs where
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
        ignition::math::Pose3d newPose(childVisual->GetWorldPose().Pos(),
            visual->GetParent()->Rotation());
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
      ignition::math::Pose3d wallPose = wallVis->GetWorldPose();
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
      m1->SetScale(ignition::math::Vector3d(wallVis->GetScale()));

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
          ignition::math::Pose3d offset = attachedVis->GetWorldPose() -
            wallPose;
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
          m2->SetScale(ignition::math::Vector3d(attachedVis->GetScale()));
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
        ignition::math::Pose3d newPose(childVisual->GetWorldPose().Pos(),
            visual->Rotation());
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
  this->GenerateSDF();
  msgs::Factory msg;
  msg.set_sdf(this->modelSDF->ToString());
  this->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
ignition::math::Vector3d BuildingMaker::ConvertSize(
    double _width, double _length, double _height)
{
  return ignition::math::Vector3d(conversionScale*_width,
      conversionScale*_length, conversionScale*_height);
}

/////////////////////////////////////////////////
ignition::math::Pose3d BuildingMaker::ConvertPose(
    double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  return ignition::math::Pose3d(
      conversionScale*_x, conversionScale*_y, conversionScale*_z,
      IGN_DTOR(_roll), IGN_DTOR(_pitch), IGN_DTOR(_yaw));
}

/////////////////////////////////////////////////
ignition::math::Vector3d BuildingMaker::ConvertSize(const QVector3D &_size)
{
  QVector3D scaledSize = conversionScale * _size;
  return ignition::math::Vector3d(
      scaledSize.x(), scaledSize.y(), scaledSize.z());
}

/////////////////////////////////////////////////
ignition::math::Pose3d BuildingMaker::ConvertPose(const QVector3D &_pos,
    const QVector3D &_rot)
{
  QVector3D scaledPos = conversionScale*_pos;
  return ignition::math::Pose3d(scaledPos.x(), scaledPos.y(), scaledPos.z(),
      IGN_DTOR(_rot.x()), IGN_DTOR(_rot.y()), IGN_DTOR(_rot.z()));
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
void BuildingMaker::OnDiscard()
{
  int ret = QMessageBox::warning(0, QString("Discard"),
      QString("Are you sure you want to discard\n"
      "your model? All of your work will\n"
      "be lost."),
      QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Cancel);

  switch (ret)
  {
    case QMessageBox::Discard:
      gui::editor::Events::discardBuildingModel();
      this->modelName = this->buildingDefaultName;
      this->saveLocation = QDir::homePath().toStdString();
      this->saved = false;
      break;
    case QMessageBox::Cancel:
    // Do nothing
    break;
    default:
    break;
  }
}

/////////////////////////////////////////////////
void BuildingMaker::OnSave()
{
  if (this->saved)
  {
    this->SetModelName(this->modelName);
    this->GenerateSDF();
    this->SaveToSDF(this->saveLocation);
  }
  else
  {
    this->saveDialog->SetModelName(this->modelName);
    this->saveDialog->SetSaveLocation(this->saveLocation);
    if (this->saveDialog->exec() == QDialog::Accepted)
    {
      this->modelName = this->saveDialog->GetModelName();
      this->saveLocation = this->saveDialog->GetSaveLocation();
      this->SetModelName(this->modelName);
      this->GenerateSDF();
      this->SaveToSDF(this->saveLocation);
      this->saved = true;
    }
  }
  gui::editor::Events::saveBuildingModel(this->modelName, this->saveLocation);
}

/////////////////////////////////////////////////
void BuildingMaker::OnDone()
{
  this->finishDialog->SetModelName(this->modelName);
  this->finishDialog->SetSaveLocation(this->saveLocation);
  if (this->finishDialog->exec() == QDialog::Accepted)
  {
    this->modelName = this->finishDialog->GetModelName();
    this->saveLocation = this->finishDialog->GetSaveLocation();
    this->SetModelName(this->modelName);
    this->GenerateSDF();
    this->SaveToSDF(this->saveLocation);
    this->FinishModel();
    gui::editor::Events::discardBuildingModel();
    gui::editor::Events::finishBuildingModel();
  }
}

/////////////////////////////////////////////////
void BuildingMaker::OnExit()
{
  int ret = QMessageBox::warning(0, QString("Exit"),
      QString("Save Changes before exiting? If you do not\n"
      "save, all of your work will be lost!\n\n"
      "Note: Building Editor state is not maintained\n"
      "between Gazebo sessions. Once you quit\n"
      "Gazebo, your building will no longer be editable.\n\n"),
//      "If you are done editing your model, select Done\n"),
      QMessageBox::Discard | QMessageBox::Cancel | QMessageBox::Save,
      QMessageBox::Save);

  switch (ret)
  {
    case QMessageBox::Discard:
      gui::editor::Events::discardBuildingModel();
      this->modelName = this->buildingDefaultName;
      this->saveLocation = QDir::homePath().toStdString();
      this->saved = false;
      gui::editor::Events::finishBuildingModel();
      break;
    case QMessageBox::Cancel:
      break;
    case QMessageBox::Save:
      this->OnSave();
      gui::editor::Events::finishBuildingModel();
      break;
    default:
      break;
  }
}
