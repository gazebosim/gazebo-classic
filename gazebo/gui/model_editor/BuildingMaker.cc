/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "msgs/msgs.hh"

#include "common/Console.hh"
#include "common/MouseEvent.hh"
#include "common/Exception.hh"

#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "rendering/Scene.hh"

#include "math/Quaternion.hh"

#include "transport/Publisher.hh"
#include "transport/Node.hh"

#include "gui/Gui.hh"
#include "gui/EntityMaker.hh"

#ifdef HAVE_GTS
  #include "gazebo/common/Mesh.hh"
  #include "gazebo/common/MeshManager.hh"
  #include "gazebo/common/MeshCSG.hh"
#endif

#include "gui/model_editor/EditorEvents.hh"
#include "gui/model_editor/BuildingMaker.hh"

#include "gui/model_editor/EditorItem.hh"
#include "gazebo_config.h"



using namespace gazebo;
using namespace gui;

double BuildingMaker::conversionScale;

/////////////////////////////////////////////////
ModelManip::ModelManip()
{
  this->parent = NULL;
}

/////////////////////////////////////////////////
ModelManip::~ModelManip()
{
}

/////////////////////////////////////////////////
void ModelManip::SetName(std::string _name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
void ModelManip::SetVisual(rendering::VisualPtr _visual)
{
  this->visual = _visual;
}

/////////////////////////////////////////////////
std::string ModelManip::GetName()
{
  return this->name;
}

/////////////////////////////////////////////////
rendering::VisualPtr ModelManip::GetVisual()
{
  return this->visual;
}

/////////////////////////////////////////////////
void ModelManip::SetMaker(BuildingMaker *_maker)
{
  this->maker = _maker;
}

/////////////////////////////////////////////////
void ModelManip::OnSizeChanged(double _width, double _depth, double _height)
{
  this->size = BuildingMaker::ConvertSize(_width, _depth, _height);
  double dScaleZ = this->visual->GetScale().z - this->size.z;
  this->visual->SetScale(this->size);
  math::Vector3 originalPos = this->visual->GetPosition();
  math::Vector3 newPos = originalPos
      - math::Vector3(0, 0, dScaleZ/2.0);
  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void ModelManip::AttachObject(ModelManip *_object)
{
  if (!_object->IsAttached())
  {
    _object->SetAttachedTo(this);
    this->attachedObjects.push_back(_object);
  }
}

/////////////////////////////////////////////////
void ModelManip::DetachObject(ModelManip *_object)
{

  std::remove(this->attachedObjects.begin(), this->attachedObjects.end(),
      _object);
}

/////////////////////////////////////////////////
void ModelManip::SetAttachedTo(ModelManip *_parent)
{
  if (this->IsAttached())
  {
    gzerr << this->name << " is already attached to a parent \n";
    return;
  }
  this->parent = _parent;
}

/////////////////////////////////////////////////
ModelManip *ModelManip::GetAttachedObject(unsigned int _index)
{
  if (_index >= this->attachedObjects.size())
    gzthrow("Index too large");

  return this->attachedObjects[_index];
}

/////////////////////////////////////////////////
unsigned int ModelManip::GetAttachedObjectCount()
{
  return this->attachedObjects.size();
}

/////////////////////////////////////////////////
bool ModelManip::IsAttached()
{
  return (this->parent != NULL);
}

/////////////////////////////////////////////////
void ModelManip::OnPoseChanged(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  this->SetPose(_x, _y, _z, _roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void ModelManip::OnPoseOriginTransformed(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  // Handle translations, currently used by polylines
  math::Pose trans = BuildingMaker::ConvertPose(_x, -_y, _z, _roll, _pitch,
      _yaw);

  math::Pose oldPose = this->visual->GetParent()->GetWorldPose();

  this->visual->GetParent()->SetWorldPose(oldPose + trans);
}

/////////////////////////////////////////////////
void ModelManip::OnPositionChanged(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);

  this->visual->GetParent()->SetWorldPosition(math::Vector3(
      scaledX, scaledY, scaledZ));
}

/////////////////////////////////////////////////
void ModelManip::OnWidthChanged(double _width)
{
  double scaledWidth = BuildingMaker::Convert(_width);
  this->size = this->visual->GetScale();
  this->size.x = scaledWidth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void ModelManip::OnDepthChanged(double _depth)
{
  double scaledDepth = BuildingMaker::Convert(_depth);
  this->size = this->visual->GetScale();
  this->size.y = scaledDepth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void ModelManip::OnHeightChanged(double _height)
{
  double scaledHeight = BuildingMaker::Convert(_height);
  this->size = this->visual->GetScale();
  this->size.z = scaledHeight;
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetScale(this->size);

  math::Vector3 newPos = originalPos
      - math::Vector3(0, 0, dScale.z/2.0);

  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void ModelManip::OnPosXChanged(double _posX)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledX = BuildingMaker::Convert(_posX);
  visualPose.pos.x = scaledX;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void ModelManip::OnPosYChanged(double _posY)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledY = BuildingMaker::Convert(_posY);
  visualPose.pos.y = -scaledY;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void ModelManip::OnPosZChanged(double _posZ)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledZ = BuildingMaker::Convert(_posZ);
  visualPose.pos.z = scaledZ;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void ModelManip::OnYawChanged(double _yaw)
{
  // Rotation around the object's local center;
  // TODO this is different from ModelManip::SetRotation which rotates the
  // object around the parent's center (mainly needed by polylines due to the
  // way user draws walls). Need to change this for consistency
  double newYaw = BuildingMaker::ConvertAngle(_yaw);
  math::Vector3 angles = this->visual->GetRotation().GetAsEuler();
  angles.z = -newYaw;
  this->visual->SetRotation(angles);
}

/////////////////////////////////////////////////
void ModelManip::OnRotationChanged(double _roll, double _pitch, double _yaw)
{
  this->SetRotation(_roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void ModelManip::OnItemDeleted()
{
  this->maker->RemovePart(this->visual->GetName());
}

/////////////////////////////////////////////////
void ModelManip::SetPose(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  this->SetPosition(_x, _y, _z);
  this->SetRotation(_roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void ModelManip::SetPosition(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);
  this->visual->GetParent()->SetWorldPosition(math::Vector3(scaledX, scaledY,
      scaledZ));
}

/////////////////////////////////////////////////
void ModelManip::SetRotation(double _roll, double _pitch, double _yaw)
{
  double rollRad = BuildingMaker::ConvertAngle(_roll);
  double pitchRad = BuildingMaker::ConvertAngle(_pitch);
  double yawRad = BuildingMaker::ConvertAngle(_yaw);

  this->visual->GetParent()->SetRotation(
      math::Quaternion(rollRad, pitchRad, yawRad));
}

/////////////////////////////////////////////////
void ModelManip::SetSize(double _width, double _depth, double _height)
{
  this->size = BuildingMaker::ConvertSize(_width, _depth, _height);

  math::Vector3 dScale = this->visual->GetScale() - this->size;

  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetPosition(math::Vector3(0, 0, 0));
  this->visual->SetScale(this->size);

  // adjust position due to difference in pivot points
  math::Vector3 newPos = originalPos
      - math::Vector3(dScale.x/2.0, dScale.y/2.0, dScale.z/2.0);

  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
  BuildingMaker::BuildingMaker() : EntityMaker()
{
  this->modelName = "";

  this->conversionScale = 0.01;

  this->CreateModel(math::Pose(0, 0, 0, 0, 0, 0));

  this->wallCounter = 0;
  this->windowCounter = 0;
  this->doorCounter = 0;
  this->stairsCounter = 0;

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());

}

/////////////////////////////////////////////////
BuildingMaker::~BuildingMaker()
{
//  this->camera.reset();
}

/////////////////////////////////////////////////
void BuildingMaker::ConnectItem(std::string _partName, EditorItem *_item)
{
  ModelManip *manip = this->allItems[_partName];

  QObject::connect(_item, SIGNAL(sizeChanged(double, double, double)),
      manip, SLOT(OnSizeChanged(double, double, double)));
  QObject::connect(_item, SIGNAL(poseChanged(double, double, double,
      double, double, double)), manip, SLOT(OnPoseChanged(double, double,
      double, double, double, double)));
  QObject::connect(_item, SIGNAL(poseOriginTransformed(double, double, double,
      double, double, double)), manip, SLOT(OnPoseOriginTransformed(double,
      double, double, double, double, double)));
  QObject::connect(_item, SIGNAL(positionChanged(double, double, double)),
      manip, SLOT(OnPositionChanged(double, double, double)));
  QObject::connect(_item, SIGNAL(rotationChanged(double, double, double)),
      manip, SLOT(OnRotationChanged(double, double, double)));

  QObject::connect(_item, SIGNAL(widthChanged(double)),
      manip, SLOT(OnWidthChanged(double)));
  QObject::connect(_item, SIGNAL(heightChanged(double)),
      manip, SLOT(OnHeightChanged(double)));
  QObject::connect(_item, SIGNAL(depthChanged(double)),
      manip, SLOT(OnDepthChanged(double)));
  QObject::connect(_item, SIGNAL(posXChanged(double)),
      manip, SLOT(OnPosXChanged(double)));
  QObject::connect(_item, SIGNAL(posYChanged(double)),
      manip, SLOT(OnPosYChanged(double)));
  QObject::connect(_item, SIGNAL(posZChanged(double)),
      manip, SLOT(OnPosZChanged(double)));
  QObject::connect(_item, SIGNAL(yawChanged(double)),
      manip, SLOT(OnYawChanged(double)));
  QObject::connect(_item, SIGNAL(itemDeleted()), manip, SLOT(OnItemDeleted()));

}

/////////////////////////////////////////////////
std::string BuildingMaker::CreateModel(math::Pose _pose)
{
  this->modelPose = _pose;
  this->modelName = this->node->GetTopicNamespace() + "::" + "building";
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  this->modelVisual.reset(new rendering::Visual(this->modelName,
                          scene->GetWorldVisual()));
  this->modelVisual->Load();
  this->modelVisual->SetPose(modelPose);
  this->modelName = this->modelVisual->GetName();

  scene->AddVisual(this->modelVisual);
  return modelName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddPart(std::string _type, QVector3D _size,
    QVector3D _pos, double _angle)
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
std::string BuildingMaker::AddWall(QVector3D _size, QVector3D _pos,
    double _angle)
{
  std::ostringstream linkNameStream;
  linkNameStream << "Wall_" << wallCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
//  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/Grey");
  visVisual->Load(visualElem);
  //this->visuals.push_back(visVisual);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  ModelManip *wallManip = new ModelManip();
  wallManip->SetMaker(this);
  wallManip->SetName(linkName);
  wallManip->SetVisual(visVisual);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  wallManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[visualName.str()] = wallManip;
  //this->walls[visualName.str()] = wallManip;

  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWindow(QVector3D _size, QVector3D _pos,
    double _angle)
{
  std::ostringstream linkNameStream;
  linkNameStream << "Window_" << this->windowCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
//  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  /// TODO for the moment, just draw a box to represent a window
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/BlueTransparent");
  visVisual->Load(visualElem);

  ModelManip *windowManip = new ModelManip();
  windowManip->SetMaker(this);
  windowManip->SetName(linkName);
  windowManip->SetVisual(visVisual);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  windowManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[visualName.str()] = windowManip;

  // TODO remove me after testing
  std::map<std::string, ModelManip *>::iterator it = this->allItems.begin();
  (*it).second->AttachObject(windowManip);

  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddDoor(QVector3D _size, QVector3D _pos,
    double _angle)
{
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
      ->GetElement("name")->Set("Gazebo/Wood");
  visVisual->Load(visualElem);

  ModelManip *doorManip = new ModelManip();
  doorManip->SetMaker(this);
  doorManip->SetName(linkName);
  doorManip->SetVisual(visVisual);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  doorManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[visualName.str()] = doorManip;

  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddStairs(QVector3D _size, QVector3D _pos,
    double _angle, int _steps)
{
  std::ostringstream linkNameStream;
  linkNameStream << "Stairs_" << this->stairsCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
//  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visVisual->Load(visualElem);
  visVisual->DetachObjects();
  // this->visuals.push_back(visVisual);

  ModelManip *stairsManip = new ModelManip();
  stairsManip->SetMaker(this);
  stairsManip->SetName(linkName);
  stairsManip->SetVisual(visVisual);
  math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
  visVisual->SetScale(scaledSize);
  double dSteps = static_cast<double>(_steps);
//  math::Vector3 offset = scaledSize/2.0;
//  offset.y = -offset.y;
  visVisual->SetPosition(math::Vector3(0, 0, scaledSize.z/2.0));
  stairsManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
  this->allItems[visualName.str()] = stairsManip;
  // this->stairs[visualName.str()] = stairsManip;

  std::stringstream visualStepName;
  visualStepName << visualName.str() << "step" << 0;
  rendering::VisualPtr baseStepVisual(new rendering::Visual(
      visualStepName.str(), visVisual));
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/Red");
  baseStepVisual->Load(visualElem);

  double rise = 1.0 / dSteps;
  double run = 1.0 / dSteps;
  baseStepVisual->SetScale(math::Vector3(1, run, rise));

  math::Vector3 baseOffset(0, 0.5 - run/2.0,
      -0.5 + rise/2.0);
  baseStepVisual->SetPosition(baseOffset);

  for ( int i = 1; i < _steps; ++i)
  {
    visualStepName.str("");
    visualStepName << visualName.str() << "step" << i;
    rendering::VisualPtr stepVisual = baseStepVisual->Clone(
        visualStepName.str(), visVisual);
    stepVisual->SetPosition(math::Vector3(0, baseOffset.y-(run*i),
        baseOffset.z + rise*i));
  }
  return visualName.str();
}

/////////////////////////////////////////////////
void BuildingMaker::RemovePart(std::string _partName)
{
  ModelManip *manip = this->allItems[_partName];
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
void BuildingMaker::RemoveWall(std::string _wallName)
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

  this->modelVisual->SetVisible(false);
}

/////////////////////////////////////////////////
bool BuildingMaker::IsActive() const
{
  return true;
}

/////////////////////////////////////////////////
void BuildingMaker::SetModelName(std::string _modelName)
{
  this->modelName = _modelName;
}

/////////////////////////////////////////////////
void BuildingMaker::SaveToSDF(std::string _savePath)
{
  this->savePath = _savePath;
  std::ofstream savefile;
  std::string saveFilePath = this->savePath + "/" + this->modelName + ".sdf";
  savefile.open(saveFilePath.c_str());
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

  std::map<std::string, ModelManip *>::iterator itemsIt;
  for(itemsIt = this->allItems.begin(); itemsIt != this->allItems.end();
      itemsIt++)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    std::string name = itemsIt->first;
    ModelManip *modelManip = itemsIt->second;
    rendering::VisualPtr visual = modelManip->GetVisual();
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(modelManip->GetName());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose());

    if (visual->GetChildCount() == 0)
    {

#if 0
      // subdivide wall surface to create holes for representing
      // window/doors
      if (name.find("Window") != std::string::npos
          || name.find("Door") != std::string::npos)
      {
        if (modelManip->IsAttached())
          continue;
      }
      else if (name.find("Wall") != std::string::npos
          && modelManip->GetAttachedObjectCount() != 0)
      {
        std::vector<QRectF> holes;
        rendering::VisualPtr wallVis = visual;
        math::Pose wallPose = wallVis->GetWorldPose();
        for (unsigned int i = 0; i < modelManip->GetAttachedObjectCount(); ++i)
        {
          ModelManip *attachedObj = modelManip->GetAttachedObject(i);
          std::string objName = attachedObj->GetName();
          if (objName.find("Window") != std::string::npos
              || objName.find("Door") != std::string::npos)
          {
            rendering::VisualPtr attachedVis = attachedObj->GetVisual();
            math::Pose offset = attachedVis->GetWorldPose() - wallPose;
            math::Vector3 size = attachedVis->GetScale();
            math::Vector3 newOffset = offset.pos - (-wallVis->GetScale()/2.0)
                - size/2.0;
            QRectF hole(newOffset.x, newOffset.z, size.x, size.z);
            holes.push_back(hole);
          }
        }
        std::vector<QRectF> subdivisions;
        QRectF surface(0, 0, wallVis->GetScale().x, wallVis->GetScale().z);

        //QRectF host(0, 0, 1.0, 1.0);
        //QRectF hole1(0.2,0.3,0.2,0.2);;
        //QRectF hole2(0.5,0.2,0.2,0.2);
        //QRectF hole3(0.4,0.6,0.3,0.4);
        //std::vector<QRectF> holess;
        //holess.push_back(hole1);
        //holess.push_back(hole2);
        //holess.push_back(hole3);
        //std::vector<QRectF> subs;
        //this->SubdivideRectSurface(host, holess, subs);

        this->SubdivideRectSurface(surface, holes, subdivisions);

        newLinkElem->ClearElements();
        for (unsigned int i = 0; i< subdivisions.size(); ++i)
        {
          visualNameStream.str("");
          collisionNameStream.str("");
          visualElem = templateVisualElem->Clone();
          collisionElem = templateCollisionElem->Clone();
          visualNameStream << modelManip->GetName() << "_Visual_" << i;
          visualElem->GetAttribute("name")->Set(visualNameStream.str());
          collisionNameStream << modelManip->GetName() << "_Collision_" << i;
          collisionElem->GetAttribute("name")->Set(collisionNameStream.str());


          math::Vector3 newSubPos = wallPose.pos + (-wallVis->GetScale()/2.0)
              + math::Vector3(subdivisions[i].x(), 0, subdivisions[i].y())
              + math::Vector3(subdivisions[i].width()/2, 0,
                  subdivisions[i].height()/2);
          math::Pose newPose(newSubPos, wallPose.rot);
        visualElem->GetElement("pose")->Set(newPose);
        collisionElem->GetElement("pose")->Set(newPose);
          math::Vector3 blockSize(subdivisions[i].width(),
              wallVis->GetScale().y, subdivisions[i].height());
          visualElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(blockSize);
          collisionElem->GetElement("geometry")->GetElement("box")->
            GetElement("size")->Set(blockSize);
          newLinkElem->InsertElement(visualElem);
          newLinkElem->InsertElement(collisionElem);
        }
      }
#endif
      visualElem->GetAttribute("name")->Set(modelManip->GetName() + "_Visual");
      collisionElem->GetAttribute("name")->Set(modelManip->GetName()
          + "_Collision");
      visualElem->GetElement("pose")->Set(visual->GetPose());
      collisionElem->GetElement("pose")->Set(visual->GetPose());
      visualElem->GetElement("geometry")->GetElement("box")->
          GetElement("size")->Set(visual->GetScale());
      collisionElem->GetElement("geometry")->GetElement("box")->
          GetElement("size")->Set(visual->GetScale());
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
        visualNameStream << modelManip->GetName() << "_Visual_" << i;
        visualElem->GetAttribute("name")->Set(visualNameStream.str());
        collisionNameStream << modelManip->GetName() << "_Collision_" << i;
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
//  qDebug() << this->modelSDF->ToString().c_str();
}

/////////////////////////////////////////////////
void BuildingMaker::GenerateSDFWithCSG()
{
#ifdef HAVE_GTS
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

  std::map<std::string, ModelManip *>::iterator itemsIt;
  for(itemsIt = this->allItems.begin(); itemsIt != this->allItems.end();
      itemsIt++)
  {
    visualNameStream.str("");
    collisionNameStream.str("");

    std::string name = itemsIt->first;
    ModelManip *modelManip = itemsIt->second;
    rendering::VisualPtr visual = modelManip->GetVisual();
    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");
    newLinkElem->GetAttribute("name")->Set(modelManip->GetName());
    newLinkElem->GetElement("pose")->Set(visual->GetParent()->GetWorldPose());

    // create a hole to represent a window/door in the wall
    if (name.find("Window") != std::string::npos
        || name.find("Door") != std::string::npos)
    {
      if (modelManip->IsAttached())
        continue;
    }
    else if (name.find("Wall") != std::string::npos
        && modelManip->GetAttachedObjectCount() != 0)
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

      std::string booleanMeshName = modelManip->GetName() + "_Boolean";
      common::Mesh *booleanMesh = NULL;
      for (unsigned int i = 0; i < modelManip->GetAttachedObjectCount(); ++i)
      {
        if (booleanMesh)
        {
          delete m1;
          m1 = booleanMesh;
        }

        ModelManip *attachedObj = modelManip->GetAttachedObject(i);
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
      visualElem->GetAttribute("name")->Set(modelManip->GetName() + "_Visual");
      collisionElem->GetAttribute("name")->Set(modelManip->GetName()
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
        visualNameStream << modelManip->GetName() << "_Visual_" << i;
        visualElem->GetAttribute("name")->Set(visualNameStream.str());
        collisionNameStream << modelManip->GetName() << "_Collision_" << i;
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
math::Vector3 BuildingMaker::ConvertSize(QVector3D _size)
{
  QVector3D scaledSize = conversionScale*_size;
  return math::Vector3(scaledSize.x(), scaledSize.y(), scaledSize.z());
}

/////////////////////////////////////////////////
math::Pose BuildingMaker::ConvertPose(QVector3D _pos, QVector3D _rot)
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
std::string BuildingMaker::GetTemplateSDFString()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='1.3'>"
    << "<model name='building_template_model'>"
    << "<pose>0 0 0.0 0 0 0</pose>"
    << "<link name ='link'>"
    <<   "<pose>0 0 0.0 0 0 0</pose>"
    <<   "<collision name ='collision'>"
    <<     "<pose>0 0 0.0 0 0 0</pose>"
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
    << "</model>"
    << "</sdf>";

  return newModelStr.str();
}


/////////////////////////////////////////////////
struct PointCompareY
{
  bool operator()(QPointF a, QPointF b) const
  {
    return a.y() < b.y();
  }
};

struct RectCompareX
{
  bool operator()(QRectF a, QRectF b) const
  {
    return a.x() < b.x();
  }
};

struct RectCompareY
{
  bool operator()(QRectF a, QRectF b) const
  {
    return a.y() < b.y();
  }
};


/////////////////////////////////////////////////
void BuildingMaker::SubdivideRectSurface(const QRectF _surface,
    const std::vector<QRectF> _holes, std::vector<QRectF> &_subdivisions)
{
  // use multiset for ordered elements
  std::multiset<QRectF, RectCompareX> filledX;
  std::multiset<QRectF, RectCompareY> filledY;
  for (unsigned int i = 0; i < _holes.size(); ++i)
  {
    filledX.insert(_holes[i]);
    filledY.insert(_holes[i]);
  }

  std::multiset<QPointF, PointCompareY> startings;

  QPointF start(_surface.x(), _surface.y());
  startings.insert(start);

  std::multiset<QPointF>::iterator startIt;
  std::multiset<QRectF>::iterator filledIt;

  // Surface subdivision algorithm:
  // subdivisions are called blocks here
  // 1. Start from top left corner
  // 2. Walk along y from starting point and stop on first obstacle with
  //    same x, this gives block height
  // 3. Find the next obstacle in x dir, this gives block width
  // 4. Remove starting point from the list
  // 5. Find next starting points by walking along the block's
  //    bottom and right edges
  // 6. Insert new starting points and the new block
  // 7. Repeat 2~6 until there are no more starting points.

  double eps = 0.001;
  while (!startings.empty())
  {
    startIt = startings.begin();
      std::cout << " start xy " << (*startIt).x()
          << " " << (*startIt).y() << std::endl;

    // walk along y
    double maxY = _surface.y() + _surface.height();
    std::multiset<QRectF>::iterator it = filledY.begin();
    for (it; it!=filledY.end(); ++it)
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
    for (it; it!=filledX.end(); ++it)
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
    for (edgeIt; edgeIt!=filledX.end(); ++edgeIt)
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
      else break;
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

    for (edgeIt; edgeIt!=filledY.end(); ++edgeIt)
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
      else break;
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

/*    std::cout << "================================================="<<std::endl;
    for (unsigned int i = 0; i < _subdivisions.size(); ++i)
    {
      std::cout << "subs " << i << " " << _subdivisions[i].x() << " "
          << _subdivisions[i].y() << " " << _subdivisions[i].width() << " "
          << _subdivisions[i].height() << std::endl;
    }*/
  }
}
