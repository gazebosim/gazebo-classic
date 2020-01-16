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
    ("msg,m", po::value<std::string>(), "Specify and send a marker message.")
    ("namespace,n", po::value<std::string>(), "Namespace for the visual marker")
    ("id,i", po::value<unsigned int>(),
     "Positive integer value of a visual marker")
    ("type,t", po::value<std::string>(),
     "Type of geometry: box, cylinder, sphere, line_list, line_strip, "
     "points, text, triangle_fan, triangle_list, triangle_strip")
    ("parent,p", po::value<std::string>(),
     "Name of a visual to which this marker should be attached.")
    ("lifetime,f", po::value<double>(),
     "Simulation time the marker should last before deletion.")
    ("delete,d", "Delete an existing visual marker.")
    ("delete-all,x", "Delete all markers, or all markers in a namespace.")
    ("list,l", "Get a list of the visual markers.")
    ("layer,y", po::value<int32_t>(),
     "Add or move a marker to the specified layer.");
}

/////////////////////////////////////////////////
void MarkerCommand::HelpDetailed()
{
  std::cerr << "\tAdd, modify, or delete visual markers.\n\n";
  std::cerr << "Option Details\n\n"

    << "-a, --add: No argument\n\n"
    << "  This option indicates that a marker should be added or modified.\n"
    << "  Use this in conjunction with -t to specify a marker type,\n"
    << "  -i to specify a marker id, -p to specify a parent, -f to specify \n"
    << "  a lifetime for the marker, or -n to specify a namespace.\n\n"

    << "-n, --namespace: string argument\n\n"
    << "  Namespaces allow markers to be grouped. This option can be used\n"
    << "  the -a and -x command. The default namespace is empty string.\n\n"

    << "-i, --id: integer argument\n\n"
    << "  Each marker has a unique id. Use this option with the -a command\n"
    << "  to assign an id to a marker. If -i is not specified, a value of\n"
    << "  zero will be used.\n\n"

    << "-t, --type: string argument\n\n"
    << "  Use this command with -a to specify a marker type. The string \n"
    << "  argument must be one of: sphere, box, cylinder, line_list, \n"
    << "  line_strip, points, sphere, text, triangle_fan, triangle_list, \n"
    << "  triangle_strip.\n\n"

    << "-p, --parent: string argument\n\n"
    << "  A marker can be attached to an existing visual. Use this command\n"
    << "  with -a to specify a parent visual. By default a marker is not\n"
    << "  attached to a parent visual.\n\n"

    << "-f, --lifetime: double argument\n\n"
    << "  A marker's lifetime is the number of seconds that it will exist.\n"
    << "  Time starts counting when the marker is created. By default a \n"
    << "  marker has an infite lifetime.\n\n"

    << "-d, --delete: integer argument\n\n"
    << "  This option will delete a single marker, if a marker exists with\n"
    << "  the specified id. The integer argument is the id of the marker to\n"
    << "  delete.\n\n"

    << "-x, --delete-all: no argument\n\n"
    << "   Delete all markers.\n\n"

    << "-l, --list: no argument\n\n"
    << "   List all markers.\n\n"

    << "-y, --layer: integer argument\n\n"
    << "   Add or move a marker to the specified layer. Use this argument with"
    << "   the -a argument.\n\n"

    << "-m, --msg: string argument\n\n"
    << "  Use this option to send a custom marker message. This option will\n"
    << "  override all other command line options. Details about the marker\n"
    << "  message can be found using: \n\n"
    << "     $ ign msg -i ign_msgs.Marker\n\n"
    << "  Example:\n\n"
    << "     $ gz marker -m 'action: ADD_MODIFY, type: SPHERE, id: 2,"
    << " scale: {x:0.2, y:0.4, z:1.2}'\n"
    << std::endl;
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

  std::string ns = "";
  unsigned int id = 0;
  int32_t layer = 0;
  std::string type = "none";
  std::string parent;
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
  if (this->vm.count("layer"))
    layer = this->vm["layer"].as<int32_t>();

  if (this->vm.count("msg"))
    this->Msg(this->vm["msg"].as<std::string>());
  else if (this->vm.count("list"))
    this->List();
  else if (this->vm.count("add"))
    this->Add(ns, id, type, lifetime, parent, layer);
  else if (this->vm.count("delete"))
    this->Delete(ns, id);
  else if (this->vm.count("delete-all"))
    this->DeleteAll(ns);
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
void MarkerCommand::List()
{
  ignition::msgs::Marker_V rep;
  bool result;
  bool executed = this->node.Request("/marker/list", 5000u, rep, result);

  if (executed)
  {
    if (result)
    {
      std::map<std::string,
        std::vector<std::tuple<uint64_t, int32_t, std::string> > > data;

      // Organize the data
      for (int i = 0; i < rep.marker_size(); ++i)
      {
        std::string ns = rep.marker(i).has_ns() ? rep.marker(i).ns() : "";
        uint64_t id = rep.marker(i).has_id() ? rep.marker(i).id() : 0;
        int32_t layer = rep.marker(i).layer();
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
          std::string type = std::get<2>(m);
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
    const std::string &_type, const common::Time &_lifetime,
    const std::string &_parent, const int32_t _layer)
{
  // Construct the marker message
  ignition::msgs::Marker msg;
  msg.set_ns(_ns);
  msg.set_id(_id);
  msg.set_parent(_parent);
  msg.set_layer(_layer);
  msg.set_action(ignition::msgs::Marker::ADD_MODIFY);

  // Set lifetime if non-zero
  if (_lifetime > common::Time::Zero)
  {
    msg.mutable_lifetime()->set_sec(_lifetime.sec);
    msg.mutable_lifetime()->set_nsec(_lifetime.nsec);
  }

  if (_type == "box")
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
  else
  {
    std::cerr << "Invalid type[" << _type << "]\n";
    return;
  }

  if (!this->node.Request("/marker", msg))
    std::cerr << "Error adding a marker\n";
}

/////////////////////////////////////////////////
void MarkerCommand::Msg(const std::string &_msg)
{
  if (!_msg.empty())
  {
    ignition::msgs::Marker msg;
    if (google::protobuf::TextFormat::ParseFromString(_msg, &msg))
    {
      if (!this->node.Request("/marker", msg))
        std::cerr << "Unable to send marker request.\n ";
    }
    else
    {
      std::cerr << "Invalid string message: " << _msg << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void MarkerCommand::Delete(const std::string &_ns, const unsigned int _id)
{
  ignition::msgs::Marker msg;
  msg.set_ns(_ns);
  msg.set_id(_id);
  msg.set_action(ignition::msgs::Marker::DELETE_MARKER);

  if (!this->node.Request("/marker", msg))
    std::cerr << "Failed to delete the marker.";
}

/////////////////////////////////////////////////
void MarkerCommand::DeleteAll(const std::string &_ns)
{
  ignition::msgs::Marker msg;
  msg.set_ns(_ns);
  msg.set_action(ignition::msgs::Marker::DELETE_ALL);

  if (!this->node.Request("/marker", msg))
    std::cerr << "Failed to delete all the markers.";
}
