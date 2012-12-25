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
#include "gui/BoxMaker.hh"

#include "gui/model_editor/EditorEvents.hh"
#include "gui/model_editor/BuildingMaker.hh"

#include "gui/model_editor/EditorItem.hh"


using namespace gazebo;
using namespace gui;

double BuildingMaker::conversionScale;

/////////////////////////////////////////////////
ModelManip::ModelManip()
{
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
void ModelManip::OnSizeChanged(double _width, double _depth, double _height)
{
  // TODO verify the result of this function
  this->size = BuildingMaker::ConvertSize(_width, _depth, _height);
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetScale(this->size);

  double angle = this->visual->GetRotation().GetAsEuler().z;
  double xdx = cos(angle)*dScale.x/2.0;
  double xdy = sin(angle)*dScale.x/2.0;
  double ydx = sin(angle)*dScale.y/2.0;
  double ydy = -cos(angle)*dScale.y/2.0;

  // adjust position due to difference in pivot points
  math::Vector3 newPos = originalPos -
      - math::Vector3(xdx + ydx, xdy + ydy, 0);

  this->visual->SetPosition(newPos);
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
void ModelManip::OnDepthChanged(double _depth)
{
  double scaledDepth = BuildingMaker::Convert(_depth);
  this->size = this->visual->GetScale();
  this->size.y = scaledDepth;
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetScale(this->size);

  double angle = this->visual->GetRotation().GetAsEuler().z;
  double dx = sin(angle)*dScale.y/2.0;
  double dy = -cos(angle)*dScale.y/2.0;

  math::Vector3 newPos = originalPos
      - math::Vector3(dx, dy, 0);

  this->visual->SetPosition(newPos);
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
void ModelManip::OnWidthChanged(double _width)
{
  double scaledWidth = BuildingMaker::Convert(_width);
  this->size = this->visual->GetScale();
  this->size.x = scaledWidth;

  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetScale(this->size);

  double angle = this->visual->GetRotation().GetAsEuler().z;
  double dx = cos(angle)*dScale.x/2.0;
  double dy = sin(angle)*dScale.x/2.0;

  math::Vector3 newPos = originalPos
      - math::Vector3(dx , dy, 0);

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

  this->boxMaker = new BoxMaker;

  this->conversionScale = 0.01;

  this->CreateModel(math::Pose(0, 0, 0, 0, 0, 0));
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

//  QObject::connect(_item, SIGNAL(itemDeleted()), this, SLOT(OnItemDeleted());

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
//  else if (_type == "stairs")
//    return this->AddStairs(_size, _pos, _angle);
  return "";
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWall(QVector3D _size, QVector3D _pos,
    double _angle)
{
  std::ostringstream linkNameStream;
  linkNameStream << "Wall_" << this->walls.size();
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual_"
    << this->walls.size();
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  std::string boxString = this->boxMaker->GetSDFString();

  sdf::ElementPtr visualElem;
  sdf::SDF sdf;
  sdf.SetFromString(boxString);
  if (sdf.root->HasElement("model"))
  {
    sdf::ElementPtr modelElem = sdf.root->GetElement("model");
    if (modelElem->HasElement("link"))
    {
      sdf::ElementPtr linkElem = modelElem->GetElement("link");
      if (linkElem->HasElement("visual"))
      {
        visualElem = linkElem->GetElement("visual");
        visVisual->Load(visualElem);
        this->visuals.push_back(visVisual);
        ModelManip *wallManip = new ModelManip();
        math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
        wallManip->SetName(linkName);
        wallManip->SetVisual(visVisual);
        visVisual->SetScale(scaledSize);
        visVisual->SetPosition(math::Vector3(scaledSize.x/2.0,
          -scaledSize.y/2.0, scaledSize.z/2.0));
        wallManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
        this->allItems[visualName.str()] = wallManip;
        this->walls[visualName.str()] = wallManip;
      }
    }
  }

  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWindow(QVector3D _size, QVector3D _pos,
    double _angle)
{
  std::ostringstream linkNameStream;
  linkNameStream << "Window_" << this->windows.size();
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual_"
    << this->windows.size();
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  /// TODO for the moment, just draw a box to represent a window
  std::string boxString = this->boxMaker->GetSDFString();

  sdf::ElementPtr visualElem;
  sdf::SDF sdf;
  sdf.SetFromString(boxString);
  if (sdf.root->HasElement("model"))
  {
    sdf::ElementPtr modelElem = sdf.root->GetElement("model");
    if (modelElem->HasElement("link"))
    {
      sdf::ElementPtr linkElem = modelElem->GetElement("link");
      if (linkElem->HasElement("visual"))
      {
        visualElem = linkElem->GetElement("visual");
        visVisual->Load(visualElem);
        this->visuals.push_back(visVisual);

        ModelManip *windowManip = new ModelManip();
        windowManip->SetName(linkName);
        windowManip->SetVisual(visVisual);
        math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
        visVisual->SetScale(scaledSize);
        visVisual->SetPosition(math::Vector3(scaledSize.x/2, -scaledSize.y/2, 0));
        windowManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
        this->allItems[visualName.str()] = windowManip;
        this->windows[visualName.str()] = windowManip;
      }
    }
  }
  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddStairs(QVector3D _size, QVector3D _pos,
    double _angle, int _steps)
{
  std::ostringstream linkNameStream;
  linkNameStream << "Stairs_" << this->walls.size();
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual_"
    << this->stairs.size();
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  std::string boxString = this->boxMaker->GetSDFString();

  sdf::ElementPtr visualElem;
  sdf::SDF sdf;
  sdf.SetFromString(boxString);
  if (sdf.root->HasElement("model"))
  {
    sdf::ElementPtr modelElem = sdf.root->GetElement("model");
    if (modelElem->HasElement("link"))
    {
      sdf::ElementPtr linkElem = modelElem->GetElement("link");
      if (linkElem->HasElement("visual"))
      {
        visualElem = linkElem->GetElement("visual");
        visVisual->Load(visualElem);
        visVisual->DetachObjects();
        this->visuals.push_back(visVisual);

        ModelManip *stairsManip = new ModelManip();
        stairsManip->SetName(linkName);
        stairsManip->SetVisual(visVisual);
        math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
        visVisual->SetScale(scaledSize);
        double dSteps = static_cast<double>(_steps);
        math::Vector3 offset = scaledSize/2.0;
        offset.y = -offset.y;
        visVisual->SetPosition(offset);
        stairsManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
        this->allItems[visualName.str()] = stairsManip;
        this->stairs[visualName.str()] = stairsManip;

        std::stringstream visualStepName;
        visualStepName << visualName.str() << "step" << 0;
        rendering::VisualPtr baseStepVisual(new rendering::Visual(
            visualStepName.str(), visVisual));
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
      }
    }
  }
  GenerateSDF();
  return visualName.str();
}

/////////////////////////////////////////////////
void BuildingMaker::RemoveWall(std::string wallName)
{
  ModelManip *manip = this->walls[wallName];
  rendering::VisualPtr vis = manip->GetVisual();
  vis->GetScene()->RemoveVisual(vis);
  this->walls.erase(wallName);
  this->allItems.erase(wallName);
  delete manip;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddDoor(QVector3D /*_size*/, QVector3D /*_pos*/,
    double /*_angle*/)
{
  std::string doorVisualName = "";
  return doorVisualName;
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
  this->modelSDF.reset();
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
//  this->Stop();
}

/////////////////////////////////////////////////
void BuildingMaker::GenerateSDF()
{
  std::string boxString = this->boxMaker->GetSDFString();

  sdf::ElementPtr modelElem;
  sdf::ElementPtr linkElem;
  sdf::ElementPtr visualElem;
  sdf::ElementPtr collisionElem;

  this->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->modelSDF);
  sdf::readString(boxString, this->modelSDF);

  //qDebug() <<  this->modelSDF->ToString().c_str();

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

    ModelManip *modelManip = itemsIt->second;
    rendering::VisualPtr visual = modelManip->GetVisual();

    sdf::ElementPtr newLinkElem = templateLinkElem->Clone();
    visualElem = newLinkElem->GetElement("visual");
    collisionElem = newLinkElem->GetElement("collision");

    newLinkElem->GetAttribute("name")->Set(modelManip->GetName());

    if (!newLinkElem->HasElement("pose"))
      newLinkElem->AddElement("pose");
    newLinkElem->GetElement("pose")->Set(
        visual->GetParent()->GetWorldPose());

    if (visual->GetChildCount() == 0)
    {
      visualElem->GetAttribute("name")->Set(modelManip->GetName() + "_Visual");
      collisionElem->GetAttribute("name")->Set(modelManip->GetName()
          + "_Collision");

      if (!visualElem->HasElement("pose"))
        visualElem->AddElement("pose");
      visualElem->GetElement("pose")->Set(visual->GetPose());

      if (!collisionElem->HasElement("pose"))
        collisionElem->AddElement("pose");
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
        if (!visualElem->HasElement("pose"))
          visualElem->AddElement("pose");

        math::Pose newPose(childVisual->GetWorldPose().pos,
            visual->GetRotation());

        if (!visualElem->HasElement("pose"))
          visualElem->AddElement("pose");
        visualElem->GetElement("pose")->Set(newPose);

        if (!collisionElem->HasElement("pose"))
          collisionElem->AddElement("pose");
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
  //qDebug() << this->modelSDF->ToString().c_str();
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
/*void BuildingMaker::OnItemDeleteted()
{
  ModelManip *manip = this->walls[wallName];
  rendering::VisualPtr vis = manip->GetVisual();
  vis->GetScene()->RemoveVisual(vis);
  this->walls.erase(wallName);
  this->allItems.erase(wallName);
  delete manip;
}*/

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
