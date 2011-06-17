/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */


#include <fstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>

#include "link.h"

using namespace sdf;

boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *_g)
{
  boost::shared_ptr<Geometry> geom;
  if (!_g) 
    return geom;

  TiXmlElement *shape = _g->FirstChildElement();
  if (!shape)
  {
    printf("Geometry tag contains no child element.\n");
    return geom;
  }

  std::string typeName = shape->ValueStr();
  if (typeName == "sphere")
    geom.reset(new Sphere);
  else if (typeName == "box")
    geom.reset(new Box);
  else if (typeName == "cylinder")
    geom.reset(new Cylinder);
  else if (typeName == "mesh")
    geom.reset(new Mesh);
  else
  {
    printf("Unknown geometry type '%s'\n", typeName.c_str());
    return geom;
  }

  // clear geom object when fails to initialize
  if (!geom->InitXml(shape)){
    printf("Geometry failed to parse\n");
    geom.reset();
  }

  return geom;
}

bool Visual::InitXml(TiXmlElement *_config)
{
  this->Clear();

  this->name = _config->Attribute("name");

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("Origin tag not present for visual element, using default (Identity)");
    this->origin.Clear();
  }
  else if (!this->origin.InitXml(o))
  {
    printf("Visual has a malformed origin tag");
    this->origin.Clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = _config->FirstChildElement("geometry");
  geometry = parseGeometry(geom);
  if (!geometry)
  {
    printf("Malformed geometry for Visual element");
    return false;
  }

  // Material
  TiXmlElement *mat = _config->FirstChildElement("material");
  if (!mat)
  {
    printf("visual element has no material tag.");
  }
  else
  {
    // try to parse material element in place
    this->material.reset(new Material);
    if (!this->material->InitXml(mat))
    {
      printf("Could not parse material element in Visual block, maybe defined outside.");
      this->material.reset();
    }
    else
    {
      printf("Parsed material element in Visual block.");
    }
  }

  return true;
}

bool Collision::InitXml(TiXmlElement *_config)
{
  this->Clear();

  this->name = _config->Attribute("name");

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("Origin tag not present for collision element, using default (Identity)");
    this->origin.Clear();
  }
  else if (!this->origin.InitXml(o))
  {
    printf("Collision has a malformed origin tag");
    this->origin.Clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = _config->FirstChildElement("geometry");
  geometry = parseGeometry(geom);
  if (!geometry)
  {
    printf("Malformed geometry for Collision element");
    return false;
  }

  return true;
}

bool Sphere::InitXml(TiXmlElement *c)
{
  this->Clear();

  this->type = SPHERE;
  if (!c->Attribute("radius"))
  {
    printf("Sphere shape must have a radius attribute");
    return false;
  }

  try
  {
    radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("radius (%s) is not a valid float",c->Attribute("radius"));
    return false;
  }

  return true;
}

bool Box::InitXml(TiXmlElement *_config)
{
  this->Clear();

  this->type = BOX;
  if (!_config->Attribute("size"))
  {
    printf("Box shape has no size attribute");
    return false;
  }

  if (!size.Init(_config->Attribute("size")))
  {
    printf("Box shape has malformed size attribute");
    size.Clear();
    return false;
  }

  return true;
}

bool Cylinder::InitXml(TiXmlElement *_config)
{
  this->Clear();

  this->type = CYLINDER;

  if (!_config->Attribute("length") ||
      !_config->Attribute("radius"))
  {
    printf("Cylinder shape must have both length and radius attributes");
    return false;
  }

  try
  {
    length = boost::lexical_cast<double>(_config->Attribute("length"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("length (%s) is not a valid float",_config->Attribute("length"));
    return false;
  }

  try
  {
    radius = boost::lexical_cast<double>(_config->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("radius (%s) is not a valid float",_config->Attribute("radius"));
    return false;
  }

  return true;
}

bool Mesh::FileExists(std::string _filename)
{
  printf("Warning: Need to implement a gazebo_config hook\n");
  std::string fullname = _filename;

  std::ifstream fin; 
  fin.open(fullname.c_str(), std::ios::in); fin.close();

  if (fin.fail()) {
    printf("Mesh [%s] does not exist\n",_filename.c_str());
    return false;
  }
  
  return true;
}

bool Mesh::InitXml(TiXmlElement *_config)
{
  this->Clear();

  this->type = MESH;
  if (!_config->Attribute("filename"))
  {
    printf("Mesh must contain a filename attribute\n");
    return false;
  }

  filename = _config->Attribute("filename");

  // check if filename exists, is this really necessary?
  if (!this->FileExists(filename))
    printf("filename referred by mesh [%s] does not appear to exist.\n", filename.c_str());

  if (_config->Attribute("scale"))
  {
    if (!this->scale.Init(_config->Attribute("scale")))
    {
      printf("Mesh scale was specified, but could not be parsed\n");
      this->scale.Clear();
      return false;
    }
  }
  else
    printf("Mesh scale was not specified, default to (1,1,1)\n");

  return true;
}


bool Link::InitXml(TiXmlElement *_config)
{
  this->Clear();

  const char *nameChar = _config->Attribute("name");
  if (!nameChar)
  {
    printf("No name given for the link.\n");
    return false;
  }
  this->name = std::string(nameChar);

  // Inertial (optional)
  TiXmlElement *i = _config->FirstChildElement("inertial");
  if (i)
  {
    inertial.reset(new Inertial);
    if (!inertial->InitXml(i))
    {
      printf("Could not parse inertial element for Link '%s'\n", this->name.c_str());
      return false;
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* visXml = _config->FirstChildElement("visual"); 
       visXml; visXml = visXml->NextSiblingElement("visual"))
  {
    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual);

    if (vis->InitXml(visXml))
    {
      //  add Visual to the vector
      this->visuals.push_back(vis);
      printf("successfully added a new visual\n");
    }
    else
    {
      printf("Could not parse visual element for Link '%s'\n", this->name.c_str());
      vis.reset();
      return false;
    }
  }

  // Multiple Collisions (optional)
  for (TiXmlElement* colXml = _config->FirstChildElement("collision"); 
       colXml; colXml = colXml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision);

    if (col->InitXml(colXml))
    {
      // group exists, add Collision to the vector in the map
      this->collisions.push_back(col);
      printf("successfully added a new collision\n");
    }
    else
    {
      printf("Could not parse collision element for Link '%s'\n", this->name.c_str());
      col.reset();
      return false;
    }
  }

  // Get all sensor elements
  for (TiXmlElement* sensorXml = _config->FirstChildElement("sensor"); 
       sensorXml; sensorXml = _config->NextSiblingElement("sensor"))
  {
    boost::shared_ptr<Sensor> sensor;
    sensor.reset(new Sensor);

    if (sensor->InitXml(sensorXml))
    {
      if (this->GetSensor(sensor->name))
      {
        printf("sensor '%s' is not unique.\n", sensor->name.c_str());
        sensor.reset();
        return false;
      }
      else
      {
        this->sensors.insert(make_pair(sensor->name,sensor));
        printf("successfully added a new sensor '%s'\n", sensor->name.c_str());
      }
    }
    else
    {
      printf("sensor xml is not initialized correctly\n");
      sensor.reset();
      return false;
    }
  }

  return true;
}

void Link::AddVisual(boost::shared_ptr<Visual> _visual)
{
  // group exists, add Visual to the vector in the map
  std::vector<boost::shared_ptr<Visual > >::iterator vis_it = find(this->visuals.begin(),this->visuals.end(), _visual);

  if (vis_it != this->visuals.end())
    printf("attempted to add a visual that already exists, skipping.\n");
  else
    this->visuals.push_back(_visual);
  printf("successfully added a new visual\n");
}

void Link::GetVisuals(std::vector<boost::shared_ptr<Visual > > &_vis) const
{
  _vis = this->visuals;
}

void Link::AddCollision(boost::shared_ptr<Collision> _collision)
{
  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<Collision > >::iterator vis_it = find(this->collisions.begin(),this->collisions.end(),_collision);

  if (vis_it != this->collisions.end())
    printf("attempted to add a collision that already exists, skipping.\n");
  else
    this->collisions.push_back(_collision);

  printf("successfully added a new collision\n");
}

void Link::GetCollisions(std::vector<boost::shared_ptr<Collision > > &_col) const
{
  _col = this->collisions;
}

boost::shared_ptr<const Sensor> Link::GetSensor(const std::string& _name) const
{
  boost::shared_ptr<const Sensor> ptr;

  if (this->sensors.find(_name) == this->sensors.end())
    ptr.reset();
  else
    ptr = this->sensors.find(_name)->second;

  return ptr;
}
