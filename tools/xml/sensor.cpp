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


#include "link.h"
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>

namespace gdf{

boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g)
{
  boost::shared_ptr<Geometry> geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    printf("Geometry tag contains no child element.\n");
    return geom;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
    geom.reset(new Sphere);
  else if (type_name == "box")
    geom.reset(new Box);
  else if (type_name == "cylinder")
    geom.reset(new Cylinder);
  else if (type_name == "mesh")
    geom.reset(new Mesh);
  else
  {
    printf("Unknown geometry type '%s'\n", type_name.c_str());
    return geom;
  }

  // clear geom object when fails to initialize
  if (!geom->initXml(shape)){
    printf("Geometry failed to parse\n");
    geom.reset();
  }

  return geom;
}

bool Material::initXml(TiXmlElement *config)
{
  bool has_rgb = false;

  this->clear();

  this->script = config->Attribute("script");

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba"))
    {
      if (!this->color.init(c->Attribute("rgba")))
      {
        printf("Material has malformed color rgba values.\n");
        this->color.clear();
        return false;
      }
      else
        has_rgb = true;
    }
    else
    {
      printf("Material color has no rgba\n");
    }
  }

  return !this->script.empty() || has_rgb;
}

bool Inertial::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for inertial element, using default (Identity)\n\n");
    this->origin.clear();
  }
  else
  {
    if (!this->origin.initXml(o))
    {
      printf("Inertial has a malformed origin tag\n");
      this->origin.clear();
      return false;
    }
  }

  if (!config->Attribute("mass"))
  {
    printf("Inertial element must have mass attribute.");
    return false;
  }

  try
  {
    mass = boost::lexical_cast<double>(config->Attribute("mass"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("mass (%s) is not a float",config->Attribute("mass"));
    return false;
  }

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    printf("Inertial element must have inertia element");
    return false;
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    printf("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
    return false;
  }
  try
  {
    ixx  = boost::lexical_cast<double>(inertia_xml->Attribute("ixx"));
    ixy  = boost::lexical_cast<double>(inertia_xml->Attribute("ixy"));
    ixz  = boost::lexical_cast<double>(inertia_xml->Attribute("ixz"));
    iyy  = boost::lexical_cast<double>(inertia_xml->Attribute("iyy"));
    iyz  = boost::lexical_cast<double>(inertia_xml->Attribute("iyz"));
    izz  = boost::lexical_cast<double>(inertia_xml->Attribute("izz"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("one of the inertia elements: ixx (%s) ixy (%s) ixz (%s) iyy (%s) iyz (%s) izz (%s) is not a valid double",
              inertia_xml->Attribute("ixx"),
              inertia_xml->Attribute("ixy"),
              inertia_xml->Attribute("ixz"),
              inertia_xml->Attribute("iyy"),
              inertia_xml->Attribute("iyz"),
              inertia_xml->Attribute("izz"));
    return false;
  }

  return true;
}

bool Visual::initXml(TiXmlElement *config)
{
  this->clear();

  this->name = config->Attribute("name");

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    printf("Origin tag not present for visual element, using default (Identity)");
    this->origin.clear();
  }
  else if (!this->origin.initXml(o))
  {
    printf("Visual has a malformed origin tag");
    this->origin.clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry = parseGeometry(geom);
  if (!geometry)
  {
    printf("Malformed geometry for Visual element");
    return false;
  }

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (!mat)
  {
    printf("visual element has no material tag.");
  }
  else
  {
    // try to parse material element in place
    this->material.reset(new Material);
    if (!this->material->initXml(mat))
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

bool Collision::initXml(TiXmlElement* config)
{
  this->clear();

  this->name = config->Attribute("name");

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    printf("Origin tag not present for collision element, using default (Identity)");
    this->origin.clear();
  }
  else if (!this->origin.initXml(o))
  {
    printf("Collision has a malformed origin tag");
    this->origin.clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry = parseGeometry(geom);
  if (!geometry)
  {
    printf("Malformed geometry for Collision element");
    return false;
  }

  return true;
}

bool Sphere::initXml(TiXmlElement *c)
{
  this->clear();

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

bool Box::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = BOX;
  if (!c->Attribute("size"))
  {
    printf("Box shape has no size attribute");
    return false;
  }
  if (!size.init(c->Attribute("size")))
  {
    printf("Box shape has malformed size attribute");
    size.clear();
    return false;
  }
  return true;
}

bool Cylinder::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    printf("Cylinder shape must have both length and radius attributes");
    return false;
  }

  try
  {
    length = boost::lexical_cast<double>(c->Attribute("length"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("length (%s) is not a valid float",c->Attribute("length"));
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

bool Mesh::fileExists(std::string filename)
{
  printf("Warning: Need to implement a gazebo_config hook\n");
  std::string fullname = filename;

  std::ifstream fin; 
  fin.open(fullname.c_str(), std::ios::in); fin.close();

  if (fin.fail()) {
    printf("Mesh [%s] does not exist\n",filename.c_str());
    return false;
  }
  
  return true;
}

bool Mesh::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = MESH;
  if (!c->Attribute("filename"))
  {
    printf("Mesh must contain a filename attribute\n");
    return false;
  }

  filename = c->Attribute("filename");

  // check if filename exists, is this really necessary?
  if (!fileExists(filename))
    printf("filename referred by mesh [%s] does not appear to exist.\n",filename.c_str());

  if (c->Attribute("scale"))
  {
    if (!this->scale.init(c->Attribute("scale")))
    {
      printf("Mesh scale was specified, but could not be parsed\n");
      this->scale.clear();
      return false;
    }
  }
  else
    printf("Mesh scale was not specified, default to (1,1,1)\n");

  return true;
}


bool Link::initXml(TiXmlElement* config)
{
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    printf("No name given for the link.\n");
    return false;
  }
  this->name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    inertial.reset(new Inertial);
    if (!inertial->initXml(i))
    {
      printf("Could not parse inertial element for Link '%s'\n", this->name.c_str());
      return false;
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {
    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual);

    if (vis->initXml(vis_xml))
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
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision);

    if (col->initXml(col_xml))
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

  return true;
}

void Link::addVisual(boost::shared_ptr<Visual> visual)
{
  // group exists, add Visual to the vector in the map
  std::vector<boost::shared_ptr<Visual > >::iterator vis_it = find(this->visuals.begin(),this->visuals.end(),visual);

  if (vis_it != this->visuals.end())
    printf("attempted to add a visual that already exists, skipping.\n");
  else
    this->visuals.push_back(visual);
  printf("successfully added a new visual\n");
}

void Link::getVisuals(std::vector<boost::shared_ptr<Visual > > &vis) const
{
  vis = this->visuals;
}

void Link::addCollision(boost::shared_ptr<Collision> collision)
{
  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<Collision > >::iterator vis_it = find(this->collisions.begin(),this->collisions.end(),collision);
  if (vis_it != this->collisions.end())
    printf("attempted to add a collision that already exists, skipping.\n");
  else
    this->collisions.push_back(collision);
  printf("successfully added a new collision\n");
}

void Link::getCollisions(std::vector<boost::shared_ptr<Collision > > &col) const
{
  col = this->collisions;
}

}
