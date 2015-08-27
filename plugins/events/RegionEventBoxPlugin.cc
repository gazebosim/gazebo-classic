/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "RegionEventBoxPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RegionEventBoxPlugin)

//////////////////////////////////////////////////
RegionEventBoxPlugin::RegionEventBoxPlugin()
  : ModelPlugin(), eventPub(0)
{
  this->hasStaleSizeAndPose = true;
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->modelName = _parent->GetName();
  this->world = _parent->GetWorld();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->modelSub = this->node->Subscribe("~/model/info",
      &RegionEventBoxPlugin::OnModelMsg, this);

  sdf::ElementPtr linkEl = this->model->GetSDF()->GetElement("link");
  sdf::ElementPtr visualEl = linkEl->GetElement("visual");
  sdf::ElementPtr geometryEl = visualEl->GetElement("geometry");
  sdf::ElementPtr boxEl = geometryEl->GetElement("box");
  this->boxSize = boxEl->Get<ignition::math::Vector3d>("size");
  this->boxScale = ignition::math::Vector3d::One;
  this->boxPose = this->model->GetWorldPose().Ign();
  this->UpdateRegion(this->boxSize * this->boxScale, this->boxPose);

  if (_sdf->HasElement("event"))
  {
    sdf::ElementPtr event = _sdf->GetElement("event");
    std::string eventType = event->Get<std::string>("type");

    if (eventType == "inclusion")
    {
      this->eventPub =
          this->node->Advertise<gazebo::msgs::SimEvent>("/gazebo/sim_events");
      this->eventSource = gazebo::EventSourcePtr(
          new EventSource(eventPub, eventType, this->world));
      this->eventSource->Load(event);
    }
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RegionEventBoxPlugin::OnUpdate, this, _1));
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::OnModelMsg(ConstModelPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->receiveMutex);
  if (_msg->has_name() && _msg->name() == this->modelName && _msg->has_scale())
  {
    this->boxScale = msgs::ConvertIgn(_msg->scale());
    this->hasStaleSizeAndPose = true;
  }
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  {
    boost::mutex::scoped_lock lock(this->receiveMutex);
    if (this->boxPose != this->model->GetWorldPose().Ign())
    {
      this->boxPose = this->model->GetWorldPose().Ign();
      this->hasStaleSizeAndPose = true;
    }

    if (this->hasStaleSizeAndPose)
    {
      this->UpdateRegion(this->boxSize * this->boxScale, this->boxPose);
      this->hasStaleSizeAndPose = false;
    }
  }

  for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
  {
    physics::ModelPtr m = this->world->GetModel(i);
    std::string name = m->GetName();

    if (name == "ground_plane" || name == this->modelName)
      continue;

    auto it = this->insiders.find(m->GetName());

    if (this->PointInRegion(m->GetWorldPose().pos.Ign(), this->box,
        this->boxPose))
    {
      if (it == this->insiders.end())
      {
        this->insiders[m->GetName()] = _info.simTime;
        if (this->eventPub)
          this->SendEnteringRegionEvent(m);
      }
    }
    else
    {
      if (it != this->insiders.end())
      {
        if (this->eventPub)
          this->SendExitingRegionEvent(m);

        this->insiders.erase(m->GetName());
      }
    }
  }
}

//////////////////////////////////////////////////
bool RegionEventBoxPlugin::PointInRegion(const ignition::math::Vector3d &_point,
    const ignition::math::Box &_box, const ignition::math::Pose3d &_pose) const
{
  // transform box extents into local space
  // box extents are already axis-aligned (see UpdateRegion) so no need to
  // apply inverse rotation.
  ignition::math::Box localBox(_box.Min() - _pose.Pos(),
      _box.Max() - _pose.Pos());

  // transform point into box space
  ignition::math::Vector3d p =
      _pose.Rot().Inverse() * (_point - _pose.Pos());

  return localBox.Contains(p);
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::UpdateRegion(const ignition::math::Vector3d &_size,
    const ignition::math::Pose3d &_pose)
{
  ignition::math::Vector3d vmin(_pose.Pos().X() - (_size.X() * 0.5),
      _pose.Pos().Y() - (_size.Y() * 0.5), _pose.Pos().Z() - (_size.Z() * 0.5));
  ignition::math::Vector3d vmax(_pose.Pos().X() + (_size.X() * 0.5),
      _pose.Pos().Y() + (_size.Y() * 0.5), _pose.Pos().Z() + (_size.Z() * 0.5));

  this->box = ignition::math::Box(vmin, vmax);
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::SendEnteringRegionEvent(physics::ModelPtr _model)
    const
{
  std::string json = "{";
  json += "\"state\":\"inside\",";
  json += "\"region\":\"" + this->modelName + "\", ";
  json += "\"model\":\"" + _model->GetName() + "\"";
  json += "}";

  this->eventSource->Emit(json);
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::SendExitingRegionEvent(physics::ModelPtr _model)
    const
{
  std::string json = "{";
  json += "\"state\":\"outside\",";
  json += "\"region\":\"" + this->modelName + "\", ";
  json += "\"model\":\"" + _model->GetName() + "\"";
  json += "}";

  this->eventSource->Emit(json);
}
