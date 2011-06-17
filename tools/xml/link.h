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

#ifndef URDF_LINK_H
#define URDF_LINK_H

#include <string>
#include <vector>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>

#include "joint.h"
#include "color.h"

namespace gdf{

class Geometry
{
public:
  enum {SPHERE, BOX, CYLINDER, MESH} type;

  virtual bool initXml(TiXmlElement *) = 0;
};

class Sphere : public Geometry
{
public:
  Sphere() { this->clear(); };
  double radius;

  void clear()
  {
    radius = 0;
  };
  bool initXml(TiXmlElement *);
};

class Box : public Geometry
{
public:
  Box() { this->clear(); };
  Vector3 size;

  void clear()
  {
    size.clear();
  };
  bool initXml(TiXmlElement *);
};

class Cylinder : public Geometry
{
public:
  Cylinder() { this->clear(); };
  double length;
  double radius;

  void clear()
  {
    length = 0;
    radius = 0;
  };
  bool initXml(TiXmlElement *);
};

class Mesh : public Geometry
{
public:
  Mesh() { this->clear(); };
  std::string filename;
  Vector3 scale;

  void clear()
  {
    filename.clear();
    // default scale
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;
  };
  bool initXml(TiXmlElement *);
  bool fileExists(std::string filename);
};

class Material
{
public:
  Material() { this->clear(); };
  std::string script;
  Color color;

  void clear()
  {
    color.clear();
    script.clear();
  };
  bool initXml(TiXmlElement* config);
};

class Inertial
{
public:
  Inertial() { this->clear(); };
  Pose origin;
  double mass;
  double ixx,ixy,ixz,iyy,iyz,izz;

  void clear()
  {
    origin.clear();
    mass = 0;
    ixx = ixy = ixz = iyy = iyz = izz = 0;
  };
  bool initXml(TiXmlElement* config);
};

class Visual
{
public:
  Visual() { this->clear(); };
  Pose origin;
  std::string name;
  boost::shared_ptr<Geometry> geometry;

  boost::shared_ptr<Material> material;

  void clear()
  {
    origin.clear();
    name.clear();
    material.reset();
    geometry.reset();
  };
  bool initXml(TiXmlElement* config);
};

class Collision
{
public:
  Collision() { this->clear(); };
  Pose origin;
  std::string name;
  boost::shared_ptr<Geometry> geometry;

  void clear()
  {
    origin.clear();
    name.clear();
    geometry.reset();
  };
  bool initXml(TiXmlElement* config);
};


class Link
{
public:
  Link() { this->clear(); };

  std::string name;

  /// inertial element
  boost::shared_ptr<Inertial> inertial;

  /// a collection of visual elements
  std::vector<boost::shared_ptr<Visual> > visuals;

  /// a collection of collision elements
  std::vector<boost::shared_ptr<Collision> > collisions;

  bool initXml(TiXmlElement* config);

  void clear()
  {
    this->name.clear();
    this->inertial.reset();
    this->visuals.clear();
    this->collisions.clear();
  };

  void addVisual(boost::shared_ptr<Visual> visual);
  void getVisuals(std::vector<boost::shared_ptr<Visual > > &vis) const;

  void addCollision(boost::shared_ptr<Collision> collision);
  void getCollisions(std::vector<boost::shared_ptr<Collision > > &col) const;
};




}

#endif
