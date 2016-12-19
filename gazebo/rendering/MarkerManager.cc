/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <mutex>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/MarkerVisual.hh"
#include "gazebo/rendering/MarkerManager.hh"

using namespace gazebo;
using namespace rendering;

/// Private data for the MarkerManager class
class gazebo::rendering::MarkerManagerPrivate
{
  /// \def MarkerVisualPtr
  /// \brief Shared pointer to MarkerVisual
  typedef std::shared_ptr<MarkerVisual> MarkerVisualPtr;

  /// \def Marker_M
  /// \brief Map of markers. The key is a marker namespace, the
  /// value is the map of markers in the namespace and their ids.
  typedef std::map<std::string, std::map<uint64_t, MarkerVisualPtr>> Marker_M;

  /// \def MarkerMsgs_L
  /// \brief List of marker messages.
  typedef std::list<ignition::msgs::Marker> MarkerMsgs_L;

  /// \brief Process a marker message.
  /// \param[in] _msg The message data.
  /// \return True if the marker was processed successfully.
  public: bool ProcessMarkerMsg(const ignition::msgs::Marker &_msg);

  /// \brief Update the markers. This function is called on
  /// the PreRender event.
  public: void OnPreRender();

  /// \brief Callback that receives marker messages.
  /// \param[in] _req The marker message.
  public: void OnMarkerMsg(const ignition::msgs::Marker &_req);

  /// \brief Service callback that returns a list of markers.
  /// \param[out] _rep Service reply
  /// \param[out] _result True on success.
  public: void OnList(ignition::msgs::Marker_V &_rep, bool &_result);

  /// \brief Mutex to protect message list.
  public: std::mutex mutex;

  /// \brief Map of markers
  public: Marker_M markers;

  /// \brief List of marker message to process.
  public: MarkerMsgs_L markerMsgs;

  /// \brief Pointer to the scene
  public: Scene *scene = nullptr;

  /// \brief Ignition node
  public: ignition::transport::Node node;

  /// \brief Connect to the prerender signal
  public: event::ConnectionPtr preRenderConnection;
};

/////////////////////////////////////////////////
MarkerManager::MarkerManager()
: dataPtr(new MarkerManagerPrivate)
{
}

/////////////////////////////////////////////////
MarkerManager::~MarkerManager()
{
}

/////////////////////////////////////////////////
bool MarkerManager::Init(Scene *_scene)
{
  if (!_scene)
  {
    gzerr << "Scene pointer is invalid\n";
    return false;
  }

  this->dataPtr->scene = _scene;

  // Advertise the list service
  if (!this->dataPtr->node.Advertise("/marker/list",
      &MarkerManagerPrivate::OnList, this->dataPtr.get()))
  {
    gzerr << "Unable to advertise to the /marker/list service.\n";
  }

  // Advertise to the marker service
  if (!this->dataPtr->node.Advertise("/marker",
        &MarkerManagerPrivate::OnMarkerMsg, this->dataPtr.get()))
  {
    gzerr << "Unable to advertise to the /marker service.\n";
  }

  // Process markers on PreRender
  this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
      std::bind(&MarkerManagerPrivate::OnPreRender, this->dataPtr.get()));

  return true;
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::OnPreRender()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Process the marker messages.
  for (auto markerIter = this->markerMsgs.begin();
       markerIter != this->markerMsgs.end();)
  {
    if (this->ProcessMarkerMsg(*markerIter))
      this->markerMsgs.erase(markerIter++);
    else
      ++markerIter;
  }

  // Erase any markers that have a lifetime.
  for (auto mit = this->markers.begin();
       mit != this->markers.end();)
  {
    for (auto it = mit->second.cbegin();
         it != mit->second.cend();)
    {
      // Erase a marker if it has a lifetime and it's expired.
      if (it->second->Lifetime() != common::Time::Zero &&
          it->second->Lifetime() <= this->scene->SimTime())
      {
        it->second->Fini();
        this->scene->RemoveVisual(it->second);
        it = mit->second.erase(it);
      }
      else
        ++it;
    }

    // Erase a namespace if it's empty
    if (mit->second.empty())
      mit = this->markers.erase(mit);
    else
      ++mit;
  }
}

//////////////////////////////////////////////////
bool MarkerManagerPrivate::ProcessMarkerMsg(const ignition::msgs::Marker &_msg)
{
  // Get the namespace, if it exists
  std::string ns;
  if (_msg.has_ns())
    ns = _msg.ns();

  // Get the namespace that the marker belongs to
  Marker_M::iterator nsIter = this->markers.find(ns);

  // Add/modify a marker
  if (_msg.action() == ignition::msgs::Marker::ADD_MODIFY)
  {
    std::map<uint64_t, MarkerVisualPtr>::iterator markerIter;

    // Add the marker to an existing namespace, if the namespace exists.
    if (nsIter != this->markers.end() &&
        (markerIter = nsIter->second.find(_msg.id())) != nsIter->second.end())
    {
      markerIter->second->Load(_msg);
    }
    else
    {
      // Create the name for the marker
      std::string name = "__GZ_MARKER_VISUAL_" + ns + "_" +
        std::to_string(_msg.id());

      // Create the new marker
      MarkerVisualPtr marker(new MarkerVisual(name,
            this->scene->WorldVisual()));

      // Load the marker
      marker->Load(_msg);

      // Store the marker
      this->markers[ns][_msg.id()] = marker;
    }
  }
  // Remove a single marker
  else if (_msg.action() == ignition::msgs::Marker::DELETE_MARKER)
  {
    std::map<uint64_t, MarkerVisualPtr>::iterator markerIter;

    // Remove the marker if it can be found.
    if (nsIter != this->markers.end() &&
        (markerIter = nsIter->second.find(_msg.id())) != nsIter->second.end())
    {
      markerIter->second->Fini();
      this->scene->RemoveVisual(markerIter->second);
      this->markers[ns].erase(markerIter);
    }
    else
    {
      gzerr << "Unable to delete marker with id[" << _msg.id() << "] "
        << "in namespace[" << ns << "]\n";
    }
  }
  // Remove all markers, or all markers in a namespace
  else if (_msg.action() == ignition::msgs::Marker::DELETE_ALL)
  {
    // Remove all markers in the specified namespace
    if (nsIter != this->markers.end())
    {
      for (auto it = nsIter->second.begin(); it != nsIter->second.end(); ++it)
      {
        it->second->Fini();
        this->scene->RemoveVisual(it->second);
      }
      nsIter->second.clear();
    }
    // Remove all markers in all namespaces.
    else
    {
      for (nsIter = this->markers.begin();
           nsIter != this->markers.end(); ++nsIter)
      {
        for (auto it = nsIter->second.begin(); it != nsIter->second.end(); ++it)
        {
          it->second->Fini();
          this->scene->RemoveVisual(it->second);
        }
      }
      this->markers.clear();
    }
  }
  else
  {
    gzerr << "Unknown marker action[" << _msg.action() << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::OnMarkerMsg(const ignition::msgs::Marker &_req)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->markerMsgs.push_back(_req);
}

/////////////////////////////////////////////////
void MarkerManagerPrivate::OnList(ignition::msgs::Marker_V &_rep, bool &_result)
{
  _result = true;

  // Create the list of markers
  for (Marker_M::const_iterator mIter = this->markers.begin();
       mIter != this->markers.end(); ++mIter)
  {
    for (std::map<uint64_t, MarkerVisualPtr>::const_iterator iter =
        mIter->second.begin(); iter != mIter->second.end(); ++iter)
    {
      ignition::msgs::Marker *markerMsg = _rep.add_marker();
      markerMsg->set_ns(mIter->first);
      markerMsg->set_id(iter->first);
      iter->second->FillMsg(*markerMsg);
    }
  }
}
