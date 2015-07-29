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
  this->receiveMutex = new boost::mutex();
  this->hasStaleSizeAndPose = true;
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::cout << "RegionEventBoxPlugin::Load(): model='" << _parent->GetName()
      << "'" << std::endl << std::flush;

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
  this->boxSize = boxEl->Get<math::Vector3>("size");
  this->boxScale = math::Vector3::One;
  this->boxPose = this->model->GetWorldPose();
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
void RegionEventBoxPlugin::OnModelMsg(ConstModelPtr & _msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  if (_msg->has_name() && _msg->name() == this->modelName && _msg->has_scale())
  {
    this->boxScale = msgs::Convert(_msg->scale());
    this->hasStaleSizeAndPose = true;
  }
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::OnUpdate(const common::UpdateInfo & _info)
{
  {
    boost::mutex::scoped_lock lock(*this->receiveMutex);
    if (this->boxPose != this->model->GetWorldPose())
    {
      this->boxPose = this->model->GetWorldPose();
      this->hasStaleSizeAndPose = true;
    }

    if (this->hasStaleSizeAndPose)
    {
      if (!this->UpdateRegion(this->boxSize * this->boxScale, this->boxPose))
      {
        std::cerr << "RegionEventPlugin::OnUpdate(): "
            << "failed to update size and pose for model '" << this->modelName
            << "'" << std::endl << std::flush;
        return;
      }
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

    if (this->PointInRegion(m->GetWorldPose().pos, this->box, this->boxPose))
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
bool RegionEventBoxPlugin::PointInRegion(const math::Vector3 &_point,
    const math::Box &_box, const math::Pose &_pose)
{
  // transfrom box extents into local space
  // box extents are already axis-aligned (see UpdateRegion) so no need to
  // apply inverse rotation.
  math::Box localBox(_box.min - _pose.pos, _box.max - _pose.pos);

  // transform point into box space
  math::Vector3 p = _pose.rot.GetInverse() * (_point - _pose.pos);

  return localBox.Contains(p);
}

//////////////////////////////////////////////////
bool RegionEventBoxPlugin::UpdateRegion(const math::Vector3 &_size,
    const math::Pose &_pose)
{
  std::cout << "RegionEventBoxPlugin::UpdateSizeAndPose(): model='"
      << this->modelName << "'" << std::endl << std::flush;

  math::Vector3 vmin(_pose.pos.x - (_size.x * 0.5),
      _pose.pos.y - (_size.y * 0.5), _pose.pos.z - (_size.z * 0.5));
  math::Vector3 vmax(_pose.pos.x + (_size.x * 0.5),
      _pose.pos.y + (_size.y * 0.5), _pose.pos.z + (_size.z * 0.5));

  this->box = math::Box(vmin, vmax);

  return true;
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::SendEnteringRegionEvent(physics::ModelPtr _model)
{
  std::cout << "RegionEventBoxPlugin::SendEnteringRegionEvent(): model='"
      << _model->GetName() << "' region='" << this->modelName << "'"
      << std::endl << std::flush;

  std::string json = "{";
  json += "\"state\":\"inside\",";
  json += "\"region\":\"" + this->modelName + "\", ";
  json += "\"model\":\"" + _model->GetName() + "\"";
  json += "}";

  eventSource->Emit(json);
}

//////////////////////////////////////////////////
void RegionEventBoxPlugin::SendExitingRegionEvent(physics::ModelPtr _model)
{
  std::cout << "RegionEventBoxPlugin::SendExitingRegionEvent(): model='"
      << _model->GetName() << "' region=\"" << this->modelName << "'"
      << std::endl << std::flush;

  std::string json = "{";
  json += "\"state\":\"outside\",";
  json += "\"region\":\"" + this->modelName + "\", ";
  json += "\"model\":\"" + _model->GetName() + "\"";
  json += "}";

  eventSource->Emit(json);
}
