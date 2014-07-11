/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <ignition/math/Rand.hh>
#include <gazebo/physics/World.hh>
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

  ignition::math::Vector3d bottomRight =
    _sdf->Get<ignition::math::Vector3d>("bottom_right");
  ignition::math::Vector3d topLeft =
    _sdf->Get<ignition::math::Vector3d>("top_left");
  ignition::math::Vector3d minSize =
    _sdf->Get<ignition::math::Vector3d>("min_size");
  ignition::math::Vector3d maxSize =
    _sdf->Get<ignition::math::Vector3d>("max_size");
  double minMass = _sdf->Get<double>("min_mass");
  double maxMass = _sdf->Get<double>("max_mass");
  unsigned int count = _sdf->Get<unsigned int>("count");

  for (unsigned int i = 0; i < count; ++i)
  {
    int rubbleType = ignition::math::Rand::IntUniform(0, 1);
    double mass = ignition::math::Rand::DblUniform(minMass, maxMass);

    Obj obj;

    obj.pose.Pos().X() =
      ignition::math::Rand::DblUniform(bottomRight.X(), topLeft.X());
    obj.pose.Pos().Y() =
      ignition::math::Rand::DblUniform(bottomRight.Y(), topLeft.Y());
    obj.pose.Pos().Z() =
      ignition::math::Rand::DblUniform(bottomRight.Z(), topLeft.Z());

    obj.pose.Rot().Euler(ignition::math::Vector3d(
        ignition::math::Rand::DblUniform(0.0, 3.1415),
        ignition::math::Rand::DblUniform(-0.1, 0.1),
        ignition::math::Rand::DblUniform(0.0, 3.1415)));


    obj.type = rubbleType;

    // Make a 2x4
    if (rubbleType == 0)
    {
      // between 2 and 8 feet.
      obj.size.X() = ignition::math::Rand::DblUniform(0.6096, 2.4384);

      // 4 inches
      obj.size.Y() = 0.1016;

      // 2 inches
      obj.size.Z() = 0.0508;
    }
    // Make a cinder block
    else if (rubbleType == 1)
    {
      // Standard US cinder block size
      obj.size.X() = 0.2032;
      obj.size.Y() = 0.2032;
      obj.size.Z() = 0.4064;
    }
    // Make a cylinder
    else
    {
      obj.size.X() =
        ignition::math::Rand::DblUniform(minSize.X(), maxSize.X());
      obj.size.Z() =
        ignition::math::Rand::DblUniform(minSize.Z(), maxSize.Z());
      obj.size.Y() = obj.size.X();
    }

    // Make sure the bottom of the rubble piece is above the bottomRight.z
    // This will prevent ground penetration.
    if (obj.pose.Pos().Z() - obj.size.Z() * 0.5 < bottomRight.Z())
      obj.pose.Pos().Z() = bottomRight.Z() + obj.size.Z() * 0.5;

    std::ostringstream name;
    name << "rubble_" << i;

    if (rubbleType == 0)
      this->MakeBox(name.str(), obj.pose, obj.size, mass);
    else if (rubbleType == 1)
      this->MakeCinderBlock(name.str(), obj.pose, obj.size, mass);
    /*else
      this->MakeCylinder(name.str(), obj.pos, obj.size, mass);
      */

    // Disable compound objects for now.
    // bool merged = false;
    /* std::vector<CompoundObj> objects;
       std::vector<CompoundObj>::iterator iter;
       for (iter = objects.begin(); iter != objects.end(); ++iter)
    {
      bool x = fabs(obj.Pos().X() - (*iter).Pos().X()) <=
        (*iter).size.X() * 0.5 + obj.size.X() * 0.5;
      bool y = fabs(obj.Pos().Y() - (*iter).Pos().Y()) <=
        (*iter).size.Y() * 0.5 + obj.size.Y() * 0.5;
      bool z = fabs(obj.Pos().Z() - (*iter).Pos().Z()) <=
        (*iter).size.Z() * 0.5 + obj.size.Z() * 0.5;

      if (x && y && z)
      {
        (*iter).objects.push_back(obj);
        std::vector<Obj>::iterator objIter;

        (*iter).Pos().Set(0, 0, 0);
        ignition::math::Vector3d min, max;
        min.X() = min.Y() = min.Z() = std::numeric_limits<double>::max();
        max.X() = max.Y() = max.Z() = std::numeric_limits<double>::min();

        for (objIter = (*iter).objects.begin();
             objIter != (*iter).objects.end(); ++objIter)
        {
          (*iter).pos += (*objIter).pos;
          min.X() = std::min(min.X(),
          (*objIter).Pos().X() - (*objIter).size.X() * 0.5);
          min.Y() = std::min(min.Y(),
          (*objIter).Pos().Y() - (*objIter).size.Y() * 0.5);
          min.Z() = std::min(min.Z(),
          (*objIter).Pos().Z() - (*objIter).size.Z() * 0.5);

          max.X() = std::max(max.X(),
          (*objIter).Pos().X() + (*objIter).size.X() * 0.5);
          max.Y() = std::max(max.Y(),
          (*objIter).Pos().Y() + (*objIter).size.Y() * 0.5);
          max.Z() = std::max(max.Z(),
          (*objIter).Pos().Z() + (*objIter).size.Z() * 0.5);
        }

        // Recalculate the middle position of the compound object
        (*iter).pos /= (*iter).objects.size();

        // Recalculate the total size of the compound object
        (*iter).size.Set(max.X() -min.X(), max.Y() -
        min.Y(), max.Z() - min.Z());

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
void RubblePlugin::MakeCinderBlock(const std::string &_name,
    ignition::math::Pose3d &_pose,
    ignition::math::Vector3d &_size, double _mass)
{
  std::ostringstream newModelStr;

  float sx = _size.X();
  float sy = _size.Y();
  float sz = _size.Z();

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    "<model name='" << _name << "'>"
    "<pose>" << _pose << "</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.01</linear>"
        "<angular>0.01</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
        "<ixx>" << (1.0/12.0) * _mass * (sy*sy + sz*sz) << "</ixx>"
        "<iyy>" << (1.0/12.0) * _mass * (sz*sz + sx*sx) << "</iyy>"
        "<izz>" << (1.0/12.0) * _mass * (sx*sx + sy*sy) << "</izz>"
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
  "</sdf>";

  this->world->InsertModelString(newModelStr.str());
}

/////////////////////////////////////////////////
void RubblePlugin::MakeBox(const std::string &_name,
    ignition::math::Pose3d &_pose,
    ignition::math::Vector3d &_size, double _mass)
{
  std::ostringstream newModelStr;

  float sx = _size.X();
  float sy = _size.Y();
  float sz = _size.Z();

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    "<model name='" << _name << "'>"
    "<allow_auto_disable>true</allow_auto_disable>"
    "<pose>" << _pose << "</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.01</linear>"
        "<angular>0.01</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
        "<ixx>" << (1.0/12.0) * _mass * (sy*sy + sz*sz) << "</ixx>"
        "<iyy>" << (1.0/12.0) * _mass * (sz*sz + sx*sx) << "</iyy>"
        "<izz>" << (1.0/12.0) * _mass * (sx*sx + sy*sy) << "</izz>"
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
  "</sdf>";

  this->world->InsertModelString(newModelStr.str());
}

/////////////////////////////////////////////////
void RubblePlugin::MakeCylinder(const std::string &_name,
    ignition::math::Vector3d &_pos,
    ignition::math::Vector3d &_size, double _mass)
{
  std::ostringstream newModelStr;

  float r = _size.X() * 0.5;
  float h = _size.Z();

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    "<model name='" << _name << "'>"
    "<pose>" << _pos << " 0 0 0</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.01</linear>"
        "<angular>0.01</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
          "<ixx>" << (1.0/12.0) * _mass * (3*r*r + h*h) << "</ixx>"
          "<iyy>" << (1.0/12.0) * _mass * (3*r*r + h*h) << "</iyy>"
          "<izz>" << (1.0/2.0)  * _mass * r * r << "</izz>"
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
  "</sdf>";

  this->world->InsertModelString(newModelStr.str());
}

/////////////////////////////////////////////////
void RubblePlugin::MakeCompound(const std::string &_name, CompoundObj &_obj)
{
  std::ostringstream newModelStr, geomStr, inertiaStr;

  newModelStr << "<sdf version ='1.3'>"
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
      float h = (*iter).size.Z();
      float w = (*iter).size.Y();
      float d = (*iter).size.X();

      inertiaStr << "<ixx>" << (1.0/12.0) * mass * (h*h + d*d) << "</ixx>"
                 << "<iyy>" << (1.0/12.0) * mass * (w*w + d*d) << "</iyy>"
                 << "<izz>" << (1.0/12.0) * mass * (w*w + h*h) << "</izz>";

      geomStr << "<box><size>" << (*iter).size << "</size></box>";
    }
    else if ((*iter).type == 1)
    {
      float radius = (*iter).size.X() * 0.5;

      inertiaStr << "<ixx>" << (2.0/5.0)* mass * radius << "</ixx>"
                 << "<iyy>" << (2.0/5.0)* mass * radius << "</iyy>"
                 << "<izz>" << (2.0/5.0)* mass * radius << "</izz>";

      geomStr << "<sphere><radius>" << radius << "</radius></sphere>";
    }
    else
    {
      float r = (*iter).size.X() * 0.5;
      float h = (*iter).size.Z();
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
                << "      <pose>" << (*iter).pose << "</pose>"
                << "      <geometry>"
                << "        " << geomStr.str()
                << "      </geometry>"
                << "    </collision>"
                << "    <visual name ='visual_" << i << "'>"
                << "      <pose>" << (*iter).pose << "</pose>"
                << "      <geometry>"
                << "        " << geomStr.str()
                << "      </geometry>"
                << "    </visual>";
  }

  newModelStr << "  </link>"
              << "</model>"
              << "</sdf>";

  this->world->InsertModelString(newModelStr.str());
}
