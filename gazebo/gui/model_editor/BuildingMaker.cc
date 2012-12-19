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
//  this->transformOrigin = math::Pose(0, 0, 0, 0, 0, 0);
}

/////////////////////////////////////////////////
ModelManip::~ModelManip()
{
}

/////////////////////////////////////////////////
void ModelManip::SetVisual(rendering::VisualPtr _visual)
{
  this->visual = _visual;
}

/////////////////////////////////////////////////
rendering::VisualPtr ModelManip::GetVisual()
{
  return this->visual;
}

/////////////////////////////////////////////////
void ModelManip::OnSizeChanged(double _width, double _length, double _height)
{
  this->size = BuildingMaker::ConvertSize(_length, _width, _height);
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  this->visual->SetScale(this->size);

  // adjust position due to difference in pivot points
  math::Vector3 newPos = this->visual->GetPosition()
      - math::Vector3(dScale.x/2.0 + dScale.y/2.0, 0, dScale.z/2.0);

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
  math::Pose trans = BuildingMaker::ConvertPose(_x, -_y, _z, _roll, _pitch,
      _yaw);

  math::Pose oldPose = this->visual->GetParent()->GetWorldPose();

  this->visual->GetParent()->SetWorldPose(oldPose + trans);
}

/////////////////////////////////////////////////
void ModelManip::OnWidthChanged(double _width)
{
  double scaledWidth = BuildingMaker::Convert(_width);
  this->size = this->visual->GetScale();
  this->size.y = scaledWidth;
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetPosition(math::Vector3(0, 0, 0));
  this->visual->SetScale(this->size);

  math::Vector3 newPos = originalPos
      + math::Vector3(0, dScale.y/2.0, 0);

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
  this->visual->SetPosition(math::Vector3(0, 0, 0));
  this->visual->SetScale(this->size);

  math::Vector3 newPos = originalPos
      - math::Vector3(0, 0, dScale.z/2.0);

  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void ModelManip::OnLengthChanged(double _length)
{
  double scaledLength = BuildingMaker::Convert(_length);
  this->size = this->visual->GetScale();
  this->size.x = scaledLength;
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetPosition(math::Vector3(0, 0, 0));
  this->visual->SetScale(this->size);

  math::Vector3 newPos = originalPos
      - math::Vector3(dScale.x/2.0 , 0, 0);

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
  double newYaw = BuildingMaker::ConvertAngle(_yaw);
  math::Vector3 angles = this->visual->GetRotation().GetAsEuler();
  angles.z = -newYaw;
  // TODO change this to be consistent with SetRotation
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
void ModelManip::SetSize(double _width, double _length, double _height)
{
  this->size = BuildingMaker::ConvertSize(_length, _width, _height);

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
/*  this->connections.push_back(
  gui::Events::ConnectCreateBuildingPart(
    boost::bind(&BuildingMaker::OnCreateBuildingPart, this, _1)));

  this->connections.push_back(
  gui::Events::ConnectSetBuildingPartPose(
    boost::bind(&BuildingMaker::OnSetBuildingPartPose, this, _1, _2)));

  this->connections.push_back(
  gui::Events::ConnectSetBuildingPartSize(
    boost::bind(&BuildingMaker::OnSetBuildingPartSize, this, _1, _2)));*/

  this->boxMaker = new BoxMaker;

  this->conversionScale = 0.01;

  math::Pose modelPose;
  modelPose.Set(0, 0, 0, 0, 0, 0);
  this->MakeModel(modelPose);
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

  QObject::connect(_item, SIGNAL(widthChanged(double)),
      manip, SLOT(OnWidthChanged(double)));
  QObject::connect(_item, SIGNAL(heightChanged(double)),
      manip, SLOT(OnHeightChanged(double)));
  QObject::connect(_item, SIGNAL(lengthChanged(double)),
      manip, SLOT(OnLengthChanged(double)));
  QObject::connect(_item, SIGNAL(posXChanged(double)),
      manip, SLOT(OnPosXChanged(double)));
  QObject::connect(_item, SIGNAL(posYChanged(double)),
      manip, SLOT(OnPosYChanged(double)));
  QObject::connect(_item, SIGNAL(posZChanged(double)),
      manip, SLOT(OnPosZChanged(double)));
  QObject::connect(_item, SIGNAL(yawChanged(double)),
      manip, SLOT(OnYawChanged(double)));

}

/////////////////////////////////////////////////
std::string BuildingMaker::MakeModel(math::Pose _pose)
{
  this->modelName = "";

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  math::Pose modelPose;

  modelPose = _pose;

  this->modelName = this->node->GetTopicNamespace() + "::" + "building";

  this->modelVisual.reset(new rendering::Visual(modelName,
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
  std::string linkName = "Wall";

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << modelName << "::" << linkName << "::Visual_"
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
        wallManip->SetVisual(visVisual);
        visVisual->SetScale(scaledSize);
        visVisual->SetPosition(math::Vector3(scaledSize.x/2.0, 0, 0));
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
  std::string linkName = "Window";

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << modelName << "::" << linkName << "::Visual_"
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
        windowManip->SetVisual(visVisual);
        math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
        visVisual->SetScale(scaledSize);
        visVisual->SetPosition(math::Vector3(scaledSize.x/2, scaledSize.y/2, 0));
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
  std::string linkName = "Stairs";

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << modelName << "::" << linkName << "::Visual_"
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
        //visVisual->Load(visualElem);
        this->visuals.push_back(visVisual);



        ModelManip *stairsManip = new ModelManip();
        stairsManip->SetVisual(visVisual);
        math::Vector3 scaledSize = BuildingMaker::ConvertSize(_size);
        visVisual->SetScale(scaledSize);
//        visVisual->SetPosition(math::Vector3(scaledSize.x/2, scaledSize.y/2, 0));
        stairsManip->SetPose(_pos.x(), _pos.y(), _pos.z(), 0, 0, _angle);
        this->allItems[visualName.str()] = stairsManip;
        this->stairs[visualName.str()] = stairsManip;

        std::stringstream visualStepName;
        visualStepName << visualName.str() << "step" << 0;
        rendering::VisualPtr baseStepVisual(new rendering::Visual(
            visualStepName.str(), visVisual));
        baseStepVisual->Load(visualElem);

        double dSteps = static_cast<double>(_steps);
        double rise = scaledSize.z / (dSteps*scaledSize.z);
        double run = scaledSize.y / (dSteps*scaledSize.y);
        math::Vector3 stepSize = scaledSize;
        stepSize.x = scaledSize.x / scaledSize.x;
        stepSize.y = scaledSize.y / (dSteps*scaledSize.y);
        stepSize.z = scaledSize.z / (dSteps*scaledSize.z);
        baseStepVisual->SetScale(stepSize);

        math::Vector3 offset = stepSize/2.0;
        baseStepVisual->SetPosition(offset);

        for ( int i = 1; i < _steps; ++i)
        {
          visualStepName.str("");
          visualStepName << visualName.str() << "step" << i;
          rendering::VisualPtr stepVisual = baseStepVisual->Clone(
              visualStepName.str(), visVisual);
          stepVisual->SetPosition(math::Vector3(offset.x, run*i + offset.y,
              rise*i + offset.z));
        }

      }
    }
  }
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
}

/////////////////////////////////////////////////
bool BuildingMaker::IsActive() const
{
  return true;
}

/////////////////////////////////////////////////
void BuildingMaker::CreateTheEntity()
{
  /*msgs::Factory msg;
  if (!this->clone)
  {
    rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
    sdf::ElementPtr modelElem;
    bool isModel = false;
    bool isLight = false;
    if (this->modelSDF->root->HasElement("model"))
    {
      modelElem = this->modelSDF->root->GetElement("model");
      isModel = true;
    }
    else if (this->modelSDF->root->HasElement("light"))
    {
      modelElem = this->modelSDF->root->GetElement("light");
      isLight = true;
    }

    std::string modelNameStr = modelElem->GetValueString("name");

    // Automatically create a new name if the model exists
    int i = 0;
    while ((isModel && has_entity_name(modelNameStr)) ||
        (isLight && scene->GetLight(modelNameStr)))
    {
      modelNameStr = modelElem->GetValueString("name") + "_" +
        boost::lexical_cast<std::string>(i++);
    }

    // Remove the topic namespace from the model name. This will get re-inserted
    // by the World automatically
    modelNameStr.erase(0, this->node->GetTopicNamespace().size()+2);

    // The the SDF model's name
    modelElem->GetAttribute("name")->Set(modelNameStr);
    modelElem->GetElement("pose")->Set(
        this->modelVisual->GetWorldPose());

    // Spawn the model in the physics server
    msg.set_sdf(this->modelSDF->ToString());
  }
  else
  {
    msgs::Set(msg.mutable_pose(), this->modelVisual->GetWorldPose());
    msg.set_clone_model_name(this->modelVisual->GetName().substr(0,
          this->modelVisual->GetName().find("_clone_tmp")));
  }

  this->makerPub->Publish(msg);*/
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
