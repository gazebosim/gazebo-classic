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

#ifndef SDF_LINK_HH
#define SDF_LINK_HH

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "sdf/interface/Param.hh"
#include "sdf/interface/sensor.hh"
#include "sdf/interface/joint.hh"
#include "sdf/interface/color.hh"

namespace sdf
{
  class Geometry
  {
    public: enum Type{UNKNOWN, SPHERE, BOX, CYLINDER, MESH};
    public: Geometry(Type _type = UNKNOWN) : type(_type) {}
    public: Type type;
  };
  
  class Sphere : public Geometry
  {
    public: Sphere() : Geometry(SPHERE), radius("radius", 1.0) { this->Clear(); }
    public: ParamT<double, true> radius;

    public: void Clear()
            {
              this->radius.Reset();
            };
  };
  
  class Box : public Geometry
  {
    public: Box() : Geometry(BOX), size("size", Vector3(0,0,0)) { this->Clear(); };
    public: ParamT<Vector3, true> size;

    public: void Clear()
            {
              size.Reset();
            };
  };
  
  class Cylinder : public Geometry
  {
    public: Cylinder() : Geometry(CYLINDER), 
            length("length", 1.0), radius("radius", 1.0) 
            { this->Clear(); };
    public: ParamT<double, true> length;
    public: ParamT<double, true> radius;

    public: void Clear()
            {
              this->length.Reset();
              this->radius.Reset();
            };
  };
  
  class Mesh : public Geometry
  {
    public: Mesh() : Geometry(MESH), filename("filename", ""), scale("scale", Vector3(1,1,1)) 
            { this->Clear(); }
    public: ParamT<std::string, true> filename;
    public: ParamT<Vector3, false> scale;

    public: void Clear()
            {
              this->filename.Reset();
              this->scale.Reset();
            };
    public: bool FileExists(std::string filename);
  };
  
  class Material
  {
    public: Material(): script("script", ""), color("color",Color())
            { this->Clear(); }
    public: ParamT<std::string, false> script;
    public: ParamT<Color, false> color;

    public: void Clear()
            {
              this->color.Reset();
              this->script.Reset();
            };
  };
  
  class Inertial
  {
    public: Inertial() : origin("origin", Pose()), 
            mass("mass",1.0), ixx("ixx",0.0), ixy("ixy",0.0), ixz("ixz",0.0), 
            iyy("iyy",0.0), iyz("iyz",0.0), izz("izz",0.0) 
    { this->Clear(); }

    public: ParamT<Pose, false> origin;
    public: ParamT<double, true> mass;
    public: ParamT<double, true> ixx,ixy,ixz,iyy,iyz,izz;

    public: void Clear()
            {
              this->origin.Reset();
              this->mass.Reset();
              this->ixx.Reset();
              this->ixy.Reset();
              this->ixz.Reset();
              this->iyy.Reset();
              this->iyz.Reset();
              this->izz.Reset();
            };

    public: void Print(const std::string &prefix)
            {
              std::cout << prefix << "Inertial: Mass[" << this->mass << "]\n";
            }
  };
  
  class Visual
  {
    public: Visual() :origin("origin", Pose()), 
                      name("name","") { this->Clear(); };
    public: ParamT<Pose, false> origin;
    public: ParamT<std::string, true> name;

    public: boost::shared_ptr<Geometry> geometry;
    public: boost::shared_ptr<Material> material;

    public: void Clear()
            {
              this->origin.Reset();
              this->name.Reset();
              this->material.reset();
              this->geometry.reset();
            };

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Visual Name[" << this->name << "]\n";
            }
  };
  
  class Collision
  {
    public: Collision(): origin("origin", Pose()), name("name","") 
    { this->Clear(); };

    public: ParamT<Pose, false> origin;
    public: ParamT<std::string, true> name;
    public: boost::shared_ptr<Geometry> geometry;

    public: void Clear()
            {
              this->origin.Reset();
              this->name.Reset();
              geometry.reset();
            };


    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Collision Name[" << this->name << "]\n";
            }

  };
  
  class Link
  {
    public: Link() : name("name", "") { this->Clear(); };

    public: ParamT<std::string, true> name;

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
              this->name.Reset();
              this->inertial.reset();
              this->visuals.clear();
              this->collisions.clear();
            };

    public: void AddVisual(boost::shared_ptr<Visual> _visual);
    public: void GetVisuals(std::vector<boost::shared_ptr<Visual > > &_vis) const;

    public: void AddCollision(boost::shared_ptr<Collision> _collision);
    public: void GetCollisions(std::vector<boost::shared_ptr<Collision > > &_col) const;

    public: boost::shared_ptr<const Sensor> GetSensor(const std::string &_name) const;

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Link: Name[" << this->name << "]\n";

              std::vector<boost::shared_ptr<Visual> >::const_iterator viter;
              for (viter = this->visuals.begin(); viter != this->visuals.end(); viter++)
              {
                (*viter)->Print(_prefix + "  ");
              }

              std::vector<boost::shared_ptr<Collision> >::const_iterator citer;
              for (citer = this->collisions.begin(); citer != this->collisions.end(); citer++)
              {
                (*citer)->Print(_prefix + "  ");
              }


              // Print sensors
              std::map<std::string, boost::shared_ptr<Sensor> >::const_iterator iter;
              for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
              {
                iter->second->Print( _prefix + "  " );
              }
            }


  };

}

#endif
