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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gz_marker.hh"

using namespace gazebo;

/////////////////////////////////////////////////
MarkerCommand::MarkerCommand()
  : Command("marker", "Add, modify, or delete visual markers")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("add,a", "Add or modify a visual marker")
    ("namespace,n", po::value<std::string>(),"Namespace for the visual marker")
    ("id,i", po::value<unsigned int>(),
     "Positive integer value of a visual marker")
    ("type,t", po::value<std::string>(),
     "Type of geometry: cube, cylinder, sphere, line_list, line_strip, "
     "points, text, triangle_fan, triangle_list, triangle_strip")
    ("lifetime,f", po::value<double>(),
     "Time the marker should last before deletion")
    ("delete,d", "Delete an existing visual marker")
    ("delete-all,x", "Delete all the visual markers")
    ("list,l", "Get a list of the visual markers");
}

/////////////////////////////////////////////////
void MarkerCommand::HelpDetailed()
{
  std::cerr <<
    "\tAdd, modify, or delete visual markers. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool MarkerCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  this->node.reset(new transport::Node());
  this->node->Init(worldName);
  this->pub = this->node->Advertise<gazebo::msgs::Marker>("~/marker");
  this->pub->WaitForConnection();

  std::string ns = "default";
  unsigned int id = 0;
  std::string type = "none";
  common::Time lifetime;

  if (this->vm.count("namespace"))
    ns = this->vm["namespace"].as<std::string>();
  if (this->vm.count("id"))
    id = this->vm["id"].as<unsigned int>();
  if (this->vm.count("type"))
    type = this->vm["type"].as<std::string>();
  if (this->vm.count("lifetime"))
    lifetime.Set(this->vm["lifetime"].as<double>());

  if (this->vm.count("list"))
    this->List();
  else if (this->vm.count("add"))
    this->Add(ns, id, type, lifetime);
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
void MarkerCommand::List() const
{
}

/////////////////////////////////////////////////
void MarkerCommand::Add(const std::string &_ns, const unsigned int _id,
    const std::string &_type, const common::Time _lifetime) const
{
  gazebo::msgs::Marker msg;
  msg.set_ns(_ns);
  msg.set_id(_id);

  msgs::Set(msg.mutable_lifetime(), _lifetime);

  msg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
  if (_type == "cube" || _type == "box")
    msg.set_type(gazebo::msgs::Marker::CUBE);
  else if (_type == "sphere")
    msg.set_type(gazebo::msgs::Marker::SPHERE);
  else if (_type == "cylinder")
    msg.set_type(gazebo::msgs::Marker::CYLINDER);
  else if (_type == "line_list" || _type == "line-list")
    msg.set_type(gazebo::msgs::Marker::LINE_LIST);
  else if (_type == "line_strip" || _type == "line-strip")
    msg.set_type(gazebo::msgs::Marker::LINE_STRIP);
  else if (_type == "points")
    msg.set_type(gazebo::msgs::Marker::POINTS);
  else if (_type == "text")
    msg.set_type(gazebo::msgs::Marker::TEXT);
  else if (_type == "triangle_fan" || _type == "triangle-fan")
    msg.set_type(gazebo::msgs::Marker::TRIANGLE_FAN);
  else if (_type == "triangle_strip" || _type == "triangle-strip")
    msg.set_type(gazebo::msgs::Marker::TRIANGLE_STRIP);

  std::cout << msg.DebugString() << std::endl;
  this->pub->Publish(msg);
}
