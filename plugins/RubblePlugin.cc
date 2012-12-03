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

#include "gazebo/math/Rand.hh"
#include "RubblePlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(RubblePlugin)

/////////////////////////////////////////////////
RubblePlugin::RubblePlugin()
{
}

/////////////////////////////////////////////////
void RubblePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

  math::Vector3 bottomRight = _sdf->GetValueVector3("bottom_right");
  math::Vector3 topLeft = _sdf->GetValueVector3("top_left");
  math::Vector3 minSize = _sdf->GetValueVector3("min_size");
  math::Vector3 maxSize = _sdf->GetValueVector3("max_size");
  double minMass = _sdf->GetValueDouble("min_mass");
  double maxMass = _sdf->GetValueDouble("max_mass");
  unsigned int count = _sdf->GetValueUInt("count");

  std::vector<CompoundObj> objects;
  std::vector<CompoundObj>::iterator iter;

  for (unsigned int i = 0; i < count; ++i)
  {
    int rubbleType = math::Rand::GetIntUniform(0, 2);
    double mass = math::Rand::GetDblUniform(minMass, maxMass);

    Obj obj;

    obj.pos.x = math::Rand::GetDblUniform(bottomRight.x, topLeft.x);
    obj.pos.y = math::Rand::GetDblUniform(bottomRight.y, topLeft.y);
    obj.pos.z = math::Rand::GetDblUniform(bottomRight.z, topLeft.z);

    obj.type = rubbleType;

    // Make a box
    if (rubbleType == 0)
    {
      obj.size.x = math::Rand::GetDblUniform(minSize.x, maxSize.x);
      obj.size.y = math::Rand::GetDblUniform(minSize.y, maxSize.y);
      obj.size.z = math::Rand::GetDblUniform(minSize.z, maxSize.z);
    }
    // Make  a sphere
    else if (rubbleType == 1)
    {
      obj.size.x = math::Rand::GetDblUniform(minSize.x, maxSize.x);
      obj.size.y = obj.size.z = obj.size.x;
    }
    // Make a cylinder
    else
    {
      obj.size.x = math::Rand::GetDblUniform(minSize.x, maxSize.x);
      obj.size.z = math::Rand::GetDblUniform(minSize.z, maxSize.z);
      obj.size.y = obj.size.x;
    }

    // Make sure the bottom of the rubble piece is above the bottomRight.z
    // This will prevent ground penetration.
    if (obj.pos.z - obj.size.z * 0.5 < bottomRight.z)
      obj.pos.z = bottomRight.z + obj.size.z * 0.5;

    std::ostringstream name;
    name << "rubble_" << i;

    if (rubbleType == 0)
      this->MakeBox(name.str(), obj.pos, obj.size, mass);
    else if (rubbleType == 1)
      this->MakeSphere(name.str(), obj.pos, obj.size, mass);
    else
      this->MakeCylinder(name.str(), obj.pos, obj.size, mass);

    // Disable compound objects for now.
    // bool merged = false;
    /*for (iter = objects.begin(); iter != objects.end(); ++iter)
    {
      bool x = fabs(obj.pos.x - (*iter).pos.x) <=
        (*iter).size.x * 0.5 + obj.size.x * 0.5;
      bool y = fabs(obj.pos.y - (*iter).pos.y) <=
        (*iter).size.y * 0.5 + obj.size.y * 0.5;
      bool z = fabs(obj.pos.z - (*iter).pos.z) <=
        (*iter).size.z * 0.5 + obj.size.z * 0.5;

      if (x && y && z)
      {
        (*iter).objects.push_back(obj);
        std::vector<Obj>::iterator objIter;

        (*iter).pos.Set(0, 0, 0);
        math::Vector3 min, max;
        min.x = min.y = min.z = std::numeric_limits<double>::max();
        max.x = max.y = max.z = std::numeric_limits<double>::min();

        for (objIter = (*iter).objects.begin();
             objIter != (*iter).objects.end(); ++objIter)
        {
          (*iter).pos += (*objIter).pos;
          min.x = std::min(min.x, (*objIter).pos.x - (*objIter).size.x * 0.5);
          min.y = std::min(min.y, (*objIter).pos.y - (*objIter).size.y * 0.5);
          min.z = std::min(min.z, (*objIter).pos.z - (*objIter).size.z * 0.5);

          max.x = std::max(max.x, (*objIter).pos.x + (*objIter).size.x * 0.5);
          max.y = std::max(max.y, (*objIter).pos.y + (*objIter).size.y * 0.5);
          max.z = std::max(max.z, (*objIter).pos.z + (*objIter).size.z * 0.5);
        }

        // Recalculate the middle position of the compound object
        (*iter).pos /= (*iter).objects.size();

        // Recalculate the total size of the compound object
        (*iter).size.Set(max.x - min.x, max.y - min.y, max.z - min.z);

        merged = true;
        break;
      }
    }

    if (!merged)
    {
      RubblePlugin::CompoundObj co;
      co.pos = obj.pos;
      co.size = obj.size;
      co.objects.push_back(obj);
      objects.push_back(co);
    }*/
  }

  // Disable compound objects for now.
  /*
  int i =0;
  for (iter = objects.begin(); iter != objects.end(); ++iter, ++i)
  {
    std::ostringstream name;
    name << "rubble_" << i;
    this->MakeCompound(name.str(), *iter);
  }*/
}

/////////////////////////////////////////////////
void RubblePlugin::Init()
{
}

/////////////////////////////////////////////////
void RubblePlugin::MakeSphere(const std::string &_name, math::Vector3 &_pos,
                              math::Vector3 &_size, double _mass)
{
  std::ostringstream newModelStr;

  float radius = _size.x;

  newModelStr << "<gazebo version='1.2'>"
    "<model name='" << _name << "'>"
    "<pose>" << _pos << " 0 0 0</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.1</linear>"
        "<angular>0.1</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
          "<ixx>" << (2.0/5.0)* _mass * radius << "</ixx>"
          "<iyy>" << (2.0/5.0)* _mass * radius << "</iyy>"
          "<izz>" << (2.0/5.0)* _mass * radius << "</izz>"
          "<ixy>" << 0.0 << "</ixy>"
          "<ixz>" << 0.0 << "</ixz>"
          "<iyz>" << 0.0 << "</iyz>"
        "</inertia>"
      "</inertial>"
      "<collision name='collision'>"
        "<geometry>"
          "<sphere><radius>" << _size.x * 0.5 << "</radius></sphere>"
        "</geometry>"
      "</collision>"
      "<visual name='visual'>"
        "<geometry>"
          "<sphere><radius>" << _size.x * 0.5 << "</radius></sphere>"
        "</geometry>"
      "</visual>"
    "</link>"
  "</model>"
  "</gazebo>";

  this->world->InsertModelString(newModelStr.str());
}

/////////////////////////////////////////////////
void RubblePlugin::MakeBox(const std::string &_name, math::Vector3 &_pos,
                           math::Vector3 &_size, double _mass)
{
  std::ostringstream newModelStr;

  float w = _size.y;
  float h = _size.z;
  float d = _size.x;
  newModelStr << "<gazebo version='1.2'>"
    "<model name='" << _name << "'>"
    "<allow_auto_disable>true</allow_auto_disable>"
    "<pose>" << _pos << " 0 0 0</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.1</linear>"
        "<angular>0.1</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
        "<ixx>" << (1.0/12.0) * _mass * (h*h + d*d) << "</ixx>"
        "<iyy>" << (1.0/12.0) * _mass * (w*w + d*d) << "</iyy>"
        "<izz>" << (1.0/12.0) * _mass * (w*w + h*h) << "</izz>"
        "<ixy>" << 0.0 << "</ixy>"
        "<ixz>" << 0.0 << "</ixz>"
        "<iyz>" << 0.0 << "</iyz>"
        "</inertia>"
      "</inertial>"
      "<collision name='collision'>"
        "<geometry>"
          "<box><size>" << _size << "</size></box>"
        "</geometry>"
      "</collision>"
      "<visual name='visual'>"
        "<geometry>"
          "<box><size>" << _size << "</size></box>"
        "</geometry>"
      "</visual>"
    "</link>"
  "</model>"
  "</gazebo>";

  this->world->InsertModelString(newModelStr.str());
}

/////////////////////////////////////////////////
void RubblePlugin::MakeCylinder(const std::string &_name, math::Vector3 &_pos,
                                math::Vector3 &_size, double _mass)
{
  std::ostringstream newModelStr;

  float r = _size.x * 0.5;
  float h = _size.z;

  newModelStr << "<gazebo version='1.2'>"
    "<model name='" << _name << "'>"
    "<pose>" << _pos << " 0 0 0</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.1</linear>"
        "<angular>0.1</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
          "<ixx>" << (1.0/12.0) * _mass * (3*r*r + h*h) << "</ixx>"
          "<iyy>" << (1.0/12.0) * _mass * (3*r*r + h*h) << "</iyy>"
          "<izz>" << (1.0/12.0) * _mass * r * r << "</izz>"
          "<ixy>" << 0.0 << "</ixy>"
          "<ixz>" << 0.0 << "</ixz>"
          "<iyz>" << 0.0 << "</iyz>"
        "</inertia>"
      "</inertial>"
      "<collision name ='collision'>"
        "<geometry>"
          "<cylinder><radius>" << r << "</radius>"
          "<length>" << h << "</length></cylinder>"
        "</geometry>"
      "</collision>"
      "<visual name='visual'>"
        "<geometry>"
          "<cylinder><radius>" << r << "</radius>"
          "<length>" << h << "</length></cylinder>"
        "</geometry>"
      "</visual>"
    "</link>"
  "</model>"
  "</gazebo>";

  this->world->InsertModelString(newModelStr.str());
}

/////////////////////////////////////////////////
void RubblePlugin::MakeCompound(const std::string &_name, CompoundObj &_obj)
{
  std::ostringstream newModelStr, geomStr, inertiaStr;

  newModelStr << "<gazebo version ='1.2'>"
      << "<model name='" << _name << "'>"
      << "  <pose>" << _obj.pos << " 0 0 0</pose>"
      << "  <link name='link'>";

  float mass = 0.1;

  inertiaStr << "<inertial><mass>" << mass << "</mass><inertial>";

  int i = 0;
  for (std::vector<Obj>::iterator iter = _obj.objects.begin();
       iter != _obj.objects.end(); ++iter, ++i)
  {
    if ((*iter).type == 0)
    {
      float h = (*iter).size.z;
      float w = (*iter).size.y;
      float d = (*iter).size.x;

      inertiaStr << "<ixx>" << (1.0/12.0) * mass * (h*h + d*d) << "</ixx>"
                 << "<iyy>" << (1.0/12.0) * mass * (w*w + d*d) << "</iyy>"
                 << "<izz>" << (1.0/12.0) * mass * (w*w + h*h) << "</izz>";

      geomStr << "<box><size>" << (*iter).size << "</size></box>";
    }
    else if ((*iter).type == 1)
    {
      float radius = (*iter).size.x * 0.5;

      inertiaStr << "<ixx>" << (2.0/5.0)* mass * radius << "</ixx>"
                 << "<iyy>" << (2.0/5.0)* mass * radius << "</iyy>"
                 << "<izz>" << (2.0/5.0)* mass * radius << "</izz>";

      geomStr << "<sphere><radius>" << radius << "</radius></sphere>";
    }
    else
    {
      float r = (*iter).size.x * 0.5;
      float h = (*iter).size.z;
      inertiaStr << "<ixx>" << (1.0/12.0) * mass * (3*r*r + h*h) << "</ixx>"
                 << "<iyy>" << (1.0/12.0) * mass * (3*r*r + h*h) << "</iyy>"
                 << "<izz>" << (1.0/12.0) * mass * r * r << "</izz>";

      geomStr << "<cylinder><radius>" << r << "</radius>"
              << "<length>" << h << "</length></cylinder>";
    }

    inertiaStr << "<ixy>" << 0.0 << "</ixy>"
               << "<ixz>" << 0.0 << "</ixz>"
               << "<iyz>" << 0.0 << "</iyz>"
               << "</inertial>";

    newModelStr << inertiaStr;

    newModelStr << "    <collision name ='collision_" << i << "'>"
                << "      <pose>" << (*iter).pos << " 0 0 0</pose>"
                << "      <geometry>"
                << "        " << geomStr.str()
                << "      </geometry>"
                << "    </collision>"
                << "    <visual name ='visual_" << i << "'>"
                << "      <pose>" << (*iter).pos << " 0 0 0</pose>"
                << "      <geometry>"
                << "        " << geomStr.str()
                << "      </geometry>"
                << "    </visual>";
  }

  newModelStr << "  </link>"
              << "</model>"
              << "</gazebo>";

  this->world->InsertModelString(newModelStr.str());
}
