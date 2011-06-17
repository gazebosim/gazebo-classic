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
#include <map>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "sensor.h"
#include "joint.h"
#include "color.h"

namespace sdf
{
  class Geometry
  {
    public: enum {SPHERE, BOX, CYLINDER, MESH} type;
  };
  
  class Sphere : public Geometry
  {
    public: Sphere() { this->Clear(); };
    public: double radius;

    public: void Clear()
            {
              radius = 0;
            };
  };
  
  class Box : public Geometry
  {
    public: Box() { this->Clear(); };
    public: Vector3 size;

    public: void Clear()
            {
              size.Clear();
            };
  };
  
  class Cylinder : public Geometry
  {
    public: Cylinder() { this->Clear(); };
    public: double length;
    public: double radius;

    public: void Clear()
            {
              length = 0;
              radius = 0;
            };
  };
  
  class Mesh : public Geometry
  {
    public: Mesh() { this->Clear(); };
    public: std::string filename;
    public: Vector3 scale;

    public: void Clear()
            {
              filename.clear();
              // default scale
              scale.x = 1;
              scale.y = 1;
              scale.z = 1;
            };
    public: bool FileExists(std::string filename);
  };
  
  class Material
  {
    public: Material() { this->Clear(); };
    public: std::string script;
    public: Color color;

    public: void Clear()
            {
              color.Clear();
              script.clear();
            };
  };
  
  class Inertial
  {
    public: Inertial() { this->Clear(); };
    public: Pose origin;
    public: double mass;
    public: double ixx,ixy,ixz,iyy,iyz,izz;

    public: void Clear()
            {
              origin.Clear();
              mass = 0;
              ixx = ixy = ixz = iyy = iyz = izz = 0;
            };

    public: friend std::ostream &operator<<(std::ostream &out, const Inertial &inertial)
            {
              out << "Inertial: Mass[" << inertial.mass << "]\n";
              return out;
            }
  };
  
  class Visual
  {
    public: Visual() { this->Clear(); };
    public: Pose origin;
    public: std::string name;
    public: boost::shared_ptr<Geometry> geometry;

    public: boost::shared_ptr<Material> material;

    public: void Clear()
            {
              origin.Clear();
              name.clear();
              material.reset();
              geometry.reset();
            };

    public: friend std::ostream &operator<<(std::ostream &out, const Visual &vis)
            {
              out << "Visual Name[" << vis.name << "]";
              return out;
            }
  };
  
  class Collision
  {
    public: Collision() { this->Clear(); };
    public: Pose origin;
    public: std::string name;
    public: boost::shared_ptr<Geometry> geometry;

    public: void Clear()
            {
              origin.Clear();
              name.clear();
              geometry.reset();
            };


    public: friend std::ostream &operator<<(std::ostream &out, const Collision &col)
            {
              out << "Collision Name[" << col.name << "]";
              return out;
            }

  };
  
  class Link
  {
    public: Link() { this->Clear(); };

    public: std::string name;

              /// inertial element
    public: boost::shared_ptr<Inertial> inertial;

            /// a collection of visual elements
    public: std::vector<boost::shared_ptr<Visual> > visuals;

            /// a collection of collision elements
    public: std::vector<boost::shared_ptr<Collision> > collisions;

            // a collection of the sensors
    public: std::map<std::string, boost::shared_ptr<Sensor> > sensors;

    public: void Clear()
            {
              this->name.clear();
              this->inertial.reset();
              this->visuals.clear();
              this->collisions.clear();
            };

    public: void AddVisual(boost::shared_ptr<Visual> _visual);
    public: void GetVisuals(std::vector<boost::shared_ptr<Visual > > &_vis) const;

    public: void AddCollision(boost::shared_ptr<Collision> _collision);
    public: void GetCollisions(std::vector<boost::shared_ptr<Collision > > &_col) const;

    public: boost::shared_ptr<const Sensor> GetSensor(const std::string &_name) const;

    public: friend std::ostream &operator<<(std::ostream &out, const Link &link)
  {
    out << "Link: Name[" << link.name << "]\n";

    std::vector<boost::shared_ptr<Visual> >::const_iterator viter;
    for (viter = link.visuals.begin(); viter != link.visuals.end(); viter++)
    {
      out << "  " << *((*viter).get()) << "\n";
    }

    std::vector<boost::shared_ptr<Collision> >::const_iterator citer;
    for (citer = link.collisions.begin(); citer != link.collisions.end(); citer++)
    {
      out << "  " << *((*citer).get()) << "\n";
    }


    // Print sensors
    std::map<std::string, boost::shared_ptr<Sensor> >::const_iterator iter;
    for (iter = link.sensors.begin(); iter != link.sensors.end(); iter++)
    {
      out << "  " << *(iter->second.get()) << "\n";
    }

    return out;
  }


  };

}

#endif
