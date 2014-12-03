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

#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Inertial.hh"

#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/model/PartVisualConfig.hh"
#include "gazebo/gui/model/PartGeneralConfig.hh"
#include "gazebo/gui/model/PartCollisionConfig.hh"

#include "gazebo/gui/model/ModelData.hh"

using namespace gazebo;
using namespace gui;


/////////////////////////////////////////////////
CollisionData::CollisionData()
{
  this->collisionSDF.reset(new sdf::Element);
  sdf::initFile("collision.sdf", this->collisionSDF);
}


/////////////////////////////////////////////////
PartData::PartData()
{
  this->partSDF.reset(new sdf::Element);
  sdf::initFile("link.sdf", this->partSDF);

  this->inspector = new PartInspector;
  this->inspector->setModal(false);
  connect(this->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));
  connect(this->inspector->GetVisualConfig(),
      SIGNAL(VisualAdded(const std::string &)),
      this, SLOT(OnAddVisual(const std::string &)));
  connect(this->inspector->GetVisualConfig(),
      SIGNAL(VisualRemoved(const std::string &)), this,
      SLOT(OnRemoveVisual(const std::string &)));

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&PartData::Update, this)));
  this->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
PartData::~PartData()
{
  event::Events::DisconnectPreRender(this->connections[0]);
}

/////////////////////////////////////////////////
std::string PartData::GetName() const
{
  return this->partSDF->Get<std::string>("name");
}

/////////////////////////////////////////////////
void PartData::SetName(const std::string &_name)
{
  this->partSDF->GetAttribute("name")->Set(_name);
  this->inspector->SetName(_name);
}

/////////////////////////////////////////////////
math::Pose PartData::GetPose() const
{
  return this->partSDF->Get<math::Pose>("pose");
}

/////////////////////////////////////////////////
void PartData::SetPose(const math::Pose &_pose)
{
  this->partSDF->GetElement("pose")->Set(_pose);

  PartGeneralConfig *generalConfig = this->inspector->GetGeneralConfig();
  generalConfig->SetPose(_pose);
}

/////////////////////////////////////////////////
void PartData::AddVisual(rendering::VisualPtr _visual)
{
  PartVisualConfig *visualConfig = this->inspector->GetVisualConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_visual->GetSDF());

  // some of the default values do not transfer to the visualMsg
  // so set them here and find a better way to fix this in the future.
  visualMsg.set_transparency(1.0);
  visualMsg.mutable_material()->set_lighting(true);

  this->visuals[_visual] = visualMsg;

/*  std::cerr << "_visual->GetSDF() "  <<
      _visual->GetSDF()->ToString("") << std::endl;;

  std::cerr << "===================================" << std::endl;
  std::cerr << "visualMsg "  <<
      visualMsg.DebugString() << std::endl;;*/

  std::string partName = this->partVisual->GetName();
  std::string visName = _visual->GetName();
  std::string leafName =
      visName.substr(visName.find(partName)+partName.size()+1);

  visualConfig->AddVisual(leafName, &visualMsg);

}

/////////////////////////////////////////////////
void PartData::OnApply()
{
  PartGeneralConfig *generalConfig = this->inspector->GetGeneralConfig();

  this->partSDF = msgs::LinkToSDF(*generalConfig->GetData(), this->partSDF);

  // set visual properties
  if (!this->visuals.empty())
  {
    this->partVisual->SetWorldPose(this->GetPose());

    PartVisualConfig *visualConfig = this->inspector->GetVisualConfig();
    std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
    for (it = this->visuals.begin(); it != this->visuals.end(); ++it)
    {
      std::string name = it->first->GetName();
      std::string partName = this->partVisual->GetName();
      std::string leafName =
          name.substr(name.find(partName)+partName.size()+1);

      msgs::Visual *updateMsg = visualConfig->GetData(leafName);
      if (updateMsg)
      {
        // std::cerr << " updateMsg " << updateMsg->DebugString() << std::endl;

        msgs::Visual visualMsg = it->second;
        visualMsg.set_transparency(updateMsg->transparency());
        updateMsg->clear_transparency();

        updateMsg->mutable_material()->clear_ambient();
        updateMsg->mutable_material()->clear_diffuse();
        updateMsg->mutable_material()->clear_specular();
        updateMsg->mutable_material()->clear_emissive();

//        msgs::Visual msg;
//        msg.CopyFrom(*updateMsg);

        boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
        this->updateMsgs.push_back(updateMsg);

        //it->first->UpdateFromMsg(ConstVisualPtr(updateMsg));

      }
/*      if (this->visuals[i]->GetMeshName() != visual->GetGeometry(i))
      {
        this->visuals[i]->DetachObjects();
        this->visuals[i]->AttachMesh(visual->GetGeometry(i));
      }
      if (this->visuals[i]->GetMaterialName() != visual->GetMaterial(i))
      {
        this->visuals[i]->SetMaterial(visual->GetMaterial(i), false);
      }

      this->visuals[i]->SetPose(visual->GetPose(i));
      this->visuals[i]->SetTransparency(visual->GetTransparency(i));
      this->visuals[i]->SetScale(visual->GetGeometryScale(i));*/
    }
  }
}

/////////////////////////////////////////////////
void PartData::OnAddVisual(const std::string &_name)
{
  // add a visual when the user adds a visual via the inspector's visual tab
  PartVisualConfig *visualConfig = this->inspector->GetVisualConfig();
  if (this->visuals.size() != visualConfig->GetVisualCount())
  {
    std::ostringstream visualName;
    visualName << this->partVisual->GetName() << "_" << _name;

    //visualConfig->SetName(visualConfig->GetVisualCount()-1, visualName.str());

    // add a box for now
    rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        this->partVisual));
    visVisual->Load();
    this->partVisual->GetScene()->AddVisual(visVisual);
    visVisual->AttachMesh("unit_box");
    visVisual->SetMaterial("Gazebo/Grey");
    visVisual->SetTransparency(0.5);
  }
}

/////////////////////////////////////////////////
void PartData::OnRemoveVisual(const std::string &_name)
{
  // find and remove visual when the user removes it in the
  // inspector's visual tab
  std::ostringstream name;
  name << this->partVisual->GetName() << "_" << _name;
  std::string visualName = name.str();

  std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
  for (it = this->visuals.begin(); it != this->visuals.end(); ++it)
  {
    if (visualName == it->first->GetName())
    {
      this->partVisual->DetachVisual(it->first);
      this->partVisual->GetScene()->RemoveVisual(it->first);
      this->visuals.erase(it);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PartData::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);

  while (!this->updateMsgs.empty())
  {
    boost::shared_ptr<gazebo::msgs::Visual> updateMsgPtr;
    updateMsgPtr.reset(new msgs::Visual);
    updateMsgPtr->CopyFrom(*this->updateMsgs.front());

    this->updateMsgs.erase(this->updateMsgs.begin());
    std::map<rendering::VisualPtr, msgs::Visual>::iterator it;
    for (it = this->visuals.begin(); it != this->visuals.end(); ++it)
    {
      if (it->second.name() == updateMsgPtr->name())
      {
        it->first->UpdateFromMsg(updateMsgPtr);
      }
    }
  }
}
