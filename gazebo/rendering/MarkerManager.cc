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

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/MarkerVisual.hh"
#include "gazebo/rendering/MarkerManagerPrivate.hh"
#include "gazebo/rendering/MarkerManager.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
MarkerManager::MarkerManager()
  : dataPtr(new MarkerManagerPrivate)
{
}

/////////////////////////////////////////////////
MarkerManager::~MarkerManager()
{
  this->dataPtr->node->Fini();
  delete this->dataPtr;
}

/////////////////////////////////////////////////
void MarkerManager::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->scene = rendering::get_scene();

  // Subscribe to the marker topic
  this->dataPtr->markerSub = this->dataPtr->node->Subscribe("~/marker",
      &MarkerManager::OnMarkerMsg, this);

  // Process markers on PreRender
  this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
      std::bind(&MarkerManager::OnPreRender, this));
}

/////////////////////////////////////////////////
void MarkerManager::OnPreRender()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // Process the marker messages.
  for (auto markerIter = this->dataPtr->markerMsgs.begin();
       markerIter != this->dataPtr->markerMsgs.end();)
  {
    if (this->ProcessMarkerMsg(**markerIter))
      this->dataPtr->markerMsgs.erase(markerIter++);
    else
      ++markerIter;
  }
}

//////////////////////////////////////////////////
bool MarkerManager::ProcessMarkerMsg(const msgs::Marker &_msg)
{
  std::cout << "PRocess marker message:" << _msg.DebugString() << "]\n";

  Marker_M::iterator nsIter = this->dataPtr->markers.find(_msg.ns());

  if (nsIter != this->dataPtr->markers.end())
  {
    std::cout << " Namespace exists\n";
    std::map<uint64_t, MarkerVisualPtr>::iterator markerIter =
      nsIter->second.find(_msg.id());

    if (markerIter != nsIter->second.end())
    {
      std::cout << "Marker exists\n";
      markerIter->second->Load(_msg);
    }
    else
    {
      std::cout << "New marker created1\n";
      std::string name = "__GZ_MARKER_VISUAL_NS_" + _msg.ns() + "_" +
        std::to_string(_msg.id());
      MarkerVisualPtr marker(new MarkerVisual(name,
            this->dataPtr->scene->GetWorldVisual()));
      marker->Load(_msg);
      nsIter->second[_msg.id()] = marker;
    }
  }
  else
  {
    std::cout << "New marker created0\n";
    std::string name = "__GZ_MARKER_VISUAL_NS_" + _msg.ns() + "_" +
      std::to_string(_msg.id());

    MarkerVisualPtr marker(new MarkerVisual(name,
          this->dataPtr->scene->GetWorldVisual()));
    marker->Load(_msg);
    this->dataPtr->markers[_msg.layer()][_msg.id()] = marker;
  }

  return true;
}

/////////////////////////////////////////////////
void MarkerManager::OnMarkerMsg(ConstMarkerPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->markerMsgs.push_back(_msg);
}
