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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <tuple>

#include <google/protobuf/text_format.h>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>

#include "gz_marker.hh"

using namespace gazebo;

/////////////////////////////////////////////////
MarkerCommand::MarkerCommand()
: Command("marker", "Add, modify, or delete visual markers")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("add,a", "Add or modify a visual marker")
    ("args,r", "Arguments to add")
    ("namespace,n", po::value<std::string>(), "Namespace for the visual marker")
    ("id,i", po::value<unsigned int>(),
     "Positive integer value of a visual marker")
    ("type,t", po::value<std::string>(),
     "Type of geometry: cube, cylinder, sphere, line_list, line_strip, "
     "points, text, triangle_fan, triangle_list, triangle_strip")
    ("parent,p", po::value<std::string>(),
     "Name of a visual to which this marker should be attached.")
    ("lifetime,f", po::value<double>(),
     "Simulation time the marker should last before deletion")
    ("delete,d", "Delete an existing visual marker")
    ("delete-all,x", "Delete all the visual markers")
    ("list,l", "Get a list of the visual markers");
}

/////////////////////////////////////////////////
void MarkerCommand::HelpDetailed()
{
  std::cerr << "\tAdd, modify, or delete visual markers." << std::endl;
}

/////////////////////////////////////////////////
bool MarkerCommand::RunImpl()
{
  std::vector<std::string> serviceList;
  node.ServiceList(serviceList);

  if (std::find(serviceList.begin(), serviceList.end(), "/marker")
      == serviceList.end())
  {
    std::cerr << "Error: /marker service not present on network.\n";
    return false;
  }

  node.Advertise<ignition::msgs::Marker>("/marker");

  std::string ns = "default";
  unsigned int id = 0;
  std::string type = "none";
  std::string parent;
  std::string args;
  common::Time lifetime;

  if (this->vm.count("namespace"))
    ns = this->vm["namespace"].as<std::string>();
  if (this->vm.count("id"))
    id = this->vm["id"].as<unsigned int>();
  if (this->vm.count("type"))
    type = this->vm["type"].as<std::string>();
  if (this->vm.count("parent"))
    parent = this->vm["parent"].as<std::string>();
  if (this->vm.count("lifetime"))
    lifetime.Set(this->vm["lifetime"].as<double>());
  if (this->vm.count("args"))
    args = this->vm["args"].as<std::string>();

  if (this->vm.count("list"))
    this->List();
  else if (this->vm.count("add"))
    this->Add(ns, id, type, lifetime, parent, args);
  else if (this->vm.count("delete"))
    this->Delete(ns, id);
  else if (this->vm.count("delete-all"))
    this->DeleteAll();
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
void MarkerCommand::List()
{
  ignition::msgs::StringMsg req;
  req.set_data("list");

  ignition::msgs::Marker_V rep;
  bool result;
  bool executed = this->node.Request("/marker/list", req, 5000u, rep, result);

  if (executed)
  {
    if (result)
    {
      std::map<std::string,
        std::vector<std::tuple<uint64_t, std::string, std::string>>> data;

      // Organize the data
      for (int i = 0; i < rep.marker_size(); ++i)
      {
        std::string ns = rep.marker(i).has_ns() ? rep.marker(i).ns() : "";
        uint64_t id = rep.marker(i).has_id() ? rep.marker(i).id() : 0;
        std::string layer = rep.marker(i).layer();
        std::string type;
        switch (rep.marker(i).type())
        {
          case ignition::msgs::Marker::NONE:
            type = "none";
            break;
          case ignition::msgs::Marker::BOX:
            type = "box";
            break;
          case ignition::msgs::Marker::CYLINDER:
            type = "cylinder";
            break;
          case ignition::msgs::Marker::LINE_LIST:
            type = "line_list";
            break;
          case ignition::msgs::Marker::LINE_STRIP:
            type = "line_strip";
            break;
          case ignition::msgs::Marker::POINTS:
            type = "points";
            break;
          case ignition::msgs::Marker::SPHERE:
            type = "sphere";
            break;
          case ignition::msgs::Marker::TEXT:
            type = "text";
            break;
          case ignition::msgs::Marker::TRIANGLE_FAN:
            type = "triangle_fan";
            break;
          case ignition::msgs::Marker::TRIANGLE_LIST:
            type = "triangle_list";
            break;
          case ignition::msgs::Marker::TRIANGLE_STRIP:
            type = "triangle_strip";
            break;
          default:
            type = "unknown";
            break;
        };

        data[ns].push_back(std::make_tuple(id, layer, type));
      }

      for (auto const d : data)
      {
        std::cout << "NAMESPACE " << d.first << std::endl;
        for (auto const m : d.second)
        {
          uint64_t id = std::get<0>(m);
          std::cout << "  ID " << id;
          if (id < 10)
            std::cout << "    ";
          else if (id < 100)
            std::cout << "   ";
          else if (id < 1000)
            std::cout << "  ";
          else
            std::cout << " ";
          std::string type = std::get<2>(m) ;
          std::cout << "TYPE " << type;
          for (unsigned int i = 0; i < 15 - type.size(); ++i)
            std::cout << " ";
          std::cout << "LAYER " << std::get<1>(m) << std::endl;
        }
      }
    }
    else
    {
      std::cerr << "Error when getting the list of visual markers\n";
    }
  }
  else
  {
    std::cerr << "Failed to get the list of visual markers.\n";
  }
}

/////////////////////////////////////////////////
void MarkerCommand::Add(const std::string &_ns, const unsigned int _id,
    const std::string &_type, const common::Time _lifetime,
    const std::string &_parent, const std::string &_args)
{
  // Construct the marker message
  ignition::msgs::Marker msg;
  msg.set_ns(_ns);
  msg.set_id(_id);
  msg.set_parent(_parent);
  msg.set_action(ignition::msgs::Marker::ADD_MODIFY);

  // Set lifetime if non-zero
  if (_lifetime > common::Time::Zero)
  {
    msg.mutable_lifetime()->set_sec(_lifetime.sec);
    msg.mutable_lifetime()->set_nsec(_lifetime.nsec);
  }

  if (_type == "cube" || _type == "box")
  {
    msg.set_type(ignition::msgs::Marker::BOX);
  }
  else if (_type == "sphere")
  {
    msg.set_type(ignition::msgs::Marker::SPHERE);
  }
  else if (_type == "cylinder")
  {
    msg.set_type(ignition::msgs::Marker::CYLINDER);
  }
  else if (_type == "line_list" || _type == "line-list" ||
           _type == "linelist" || _type == "lineList")
  {
    msg.set_type(ignition::msgs::Marker::LINE_LIST);
  }
  else if (_type == "line_strip" || _type == "line-strip" ||
           _type == "linestrip" || _type == "lineStrip")
  {
    msg.set_type(ignition::msgs::Marker::LINE_STRIP);
  }
  else if (_type == "points")
  {
    msg.set_type(ignition::msgs::Marker::POINTS);
  }
  else if (_type == "text")
  {
    msg.set_type(ignition::msgs::Marker::TEXT);
  }
  else if (_type == "triangle_fan" || _type == "triangle-fan" ||
           _type == "trianglefan" || _type == "triangleFan")
  {
    msg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  }
  else if (_type == "triangle_strip" || _type == "triangle-strip" ||
           _type == "trianglestrip" || _type == "triangleStrip")
  {
    msg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  }

  if (!_args.empty())
  {
    google::protobuf::TextFormat::ParseFromString(_args, &msg);
  }

  std::cout << "Args[" << _args << "]\n";
  std::cout << msg.DebugString() << std::endl;
  bool result;
  ignition::msgs::StringMsg rep;
  this->node.Request("/marker", msg, 5000u, rep, result);
}

/////////////////////////////////////////////////
void MarkerCommand::Delete(const std::string &_ns, const unsigned int _id)
{
  ignition::msgs::Marker msg;
  msg.set_ns(_ns);
  msg.set_id(_id);
  msg.set_action(ignition::msgs::Marker::DELETE_MARKER);

  bool result;
  ignition::msgs::StringMsg rep;

  this->node.Request("/marker", msg, 5000u, rep, result);
}

/////////////////////////////////////////////////
void MarkerCommand::DeleteAll()
{
  ignition::msgs::Marker msg;
  msg.set_action(ignition::msgs::Marker::DELETE_ALL);

  bool result;
  ignition::msgs::StringMsg rep;

  this->node.Request("/marker", msg, 5000u, rep, result);
}
