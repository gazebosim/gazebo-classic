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

#include "common/Color.hh"
#include "math/Pose.hh"
#include "math/Vector3.hh"

#include "sdf/interface/SDFBase.hh"
#include "sdf/interface/Sensor.hh"
#include "sdf/interface/Joint.hh"

namespace sdf
{
  class Geometry : public SDFBase
  {
    public: enum Type{UNKNOWN, PLANE, SPHERE, BOX, CYLINDER, MESH};
    public: Geometry(Type _type = UNKNOWN) : type(_type) {}
    public: Type type;
  };
  
  class Plane : public Geometry
  {
    public: Plane() : Geometry(PLANE), 
            normal("normal", gazebo::math::Vector3(0,0,1), true) 
    { 
      Param::End();
      this->xmlTree = "{geometry:{plane:normal}}";
      this->Clear(); 
    }
    public: ParamT<gazebo::math::Vector3> normal;
   };
  
  class Sphere : public Geometry
  {
    public: Sphere() : Geometry(SPHERE), 
            radius("radius", 1.0, true) 
    { 
      Param::End();
      this->xmlTree = "{geometry:{sphere:radius}}";
      this->Clear(); 
    }
    public: ParamT<double> radius;
 };
  
  class Box : public Geometry
  {
    public: Box() : Geometry(BOX), 
            size("size", gazebo::math::Vector3(0,0,0), true) 
    { 
      Param::End();
      this->xmlTree = "{geometry:{box:size}}";
      this->Clear(); 
    }
    public: ParamT<gazebo::math::Vector3> size;
  };
  
  class Cylinder : public Geometry
  {
    public: Cylinder() : Geometry(CYLINDER), 
            length("length", 1.0, true), 
            radius("radius", 1.0, true) 
            { 
              Param::End();
              this->xmlTree = "{geometry:{cylinder:length,radius}}";
            }
    public: ParamT<double> length;
    public: ParamT<double> radius;
  };
  
  class Mesh : public Geometry
  {
    public: Mesh() : Geometry(MESH), 
            filename("filename", "", true), 
            scale("scale", gazebo::math::Vector3(1,1,1), false) 
            { 
              Param::End();
              this->xmlTree = "{geometry:{mesh:filename,scale}}";
            }

    public: ParamT<std::string> filename;
    public: ParamT<gazebo::math::Vector3> scale;

    public: bool FileExists(std::string filename);
  };
  
  class Material : public SDFBase
  {
    public: Material(): 
            script("script", "", false), 
            color("rgba",gazebo::common::Color(), false)
            { 
              Param::End();
              this->xmlTree = "{material: script,{color:rgba}}";
            }

    public: ParamT<std::string> script;
    public: ParamT<gazebo::common::Color> color;
  };
  
  class Inertial : public SDFBase
  {
    public: Inertial() : 
            origin("origin", gazebo::math::Pose(), false), 
            mass("mass",1.0, true), 
            ixx("ixx",0.0, true), 
            ixy("ixy",0.0, true), 
            ixz("ixz",0.0, true), 
            iyy("iyy",0.0, true), 
            iyz("iyz",0.0, true), 
            izz("izz",0.0, true) 
    { 
      Param::End();
      this->xmlTree = "{inertial:mass,{origin:pose},{inertia:ixx,ixy,ixz,iyy,iyz,izz}}";
    }

    public: ParamT<gazebo::math::Pose> origin;
    public: ParamT<double> mass;
    public: ParamT<double> ixx,ixy,ixz,iyy,iyz,izz;

    public: void Print(const std::string &prefix)
            {
              std::cout << prefix << "Inertial: Mass[" << this->mass << "]\n";
            }
  };
  
  class Visual : public SDFBase
  {
    public: Visual() :
            origin("origin", gazebo::math::Pose(), false), 
            name("name","", true),
            castShadows("cast_shadows", true, false)
            { 
              Param::End();
              this->xmlTree = "{visual:name,cast_shadows,{origin:pose}}";
            }

    public: ParamT<gazebo::math::Pose> origin;
    public: ParamT<std::string> name;
    public: ParamT<bool> castShadows;

    public: boost::shared_ptr<Geometry> geometry;
    public: boost::shared_ptr<Material> material;

    public: void Clear()
            {
              SDFBase::Clear();
              this->material.reset();
              this->geometry.reset();
            };

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Visual Name[" << this->name << "]\n";
            }
  };

  class Friction : public SDFBase
  {
    public: enum Type {UNKNOWN, ODE};
    public: Friction(Type _t) : type(_t) {}
    public: Type type;
  };

  class ODEFriction : public Friction
  {
    public: ODEFriction() : Friction(ODE), 
            mu("mu",0.0, true), 
            mu2("mu2",0.0, false), 
            fdir1("fdir1",0.0,false), 
            slip1("slip1",0.0, false), 
            slip2("slip2",0.0, false)
            { 
              Param::End();
              this->xmlTree = "{friction:{ode:mu,mu1,fdir1,slip1,slip2}}";
            }

    public: ParamT<double> mu;
    public: ParamT<double> mu2;
    public: ParamT<double> fdir1;
    public: ParamT<double> slip1;
    public: ParamT<double> slip2;
  };

  class SurfaceContact : public SDFBase
  {
    public: enum Type {UNKNOWN, ODE};
    public: SurfaceContact(Type _t) : type(_t) {}
    public: Type type;
  };

  class ODESurfaceContact : public SurfaceContact
  {
    public: ODESurfaceContact() : SurfaceContact(ODE), 
            softCFM("soft_cfm",0, false), 
            kp("kp",0, false), 
            kd("kd",0, false), 
            maxVel("max_vel",0, false), 
            minDepth("min_depth",0,false)
    { 
      Param::End();
      this->xmlTree = "{contact:{ode:soft_cfm,kp,kd,max_vel,min_depth}}";
      this->Clear(); 
    }

    public: ParamT<double> softCFM;
    public: ParamT<double> kp;
    public: ParamT<double> kd;
    public: ParamT<double> maxVel;
    public: ParamT<double> minDepth;

  };

  class Surface : public SDFBase
  {
    public: Surface() : 
            bounceRestCoeff("restitution_coefficient",0, false), 
            bounceThreshold("threshold",0,false) 
    { 
      Param::End();
      this->xmlTree = "{surface:{bounce:restitution_coefficient,threshold}}";
    }

    public: ParamT<double> bounceRestCoeff;
    public: ParamT<double> bounceThreshold;
    public: std::vector< boost::shared_ptr<SurfaceContact> > contacts;
    public: std::vector< boost::shared_ptr<Friction> > frictions;

    public: void Clear()
            {
              SDFBase::Clear();
              this->contacts.clear();
              this->frictions.clear();
            }
  };
  
  class Collision : public SDFBase
  {
    public: Collision() : 
            origin("origin", gazebo::math::Pose(), false), 
            name("name","", true)
    { 
      Param::End(); 
      this->xmlTree = "{collision:name,{origin:pose}}";
    }

    public: ParamT<gazebo::math::Pose> origin;
    public: ParamT<std::string> name;

    public: boost::shared_ptr<Geometry> geometry;
    public: boost::shared_ptr<Surface> surface;

    public: void Clear()
            {
              SDFBase::Clear();
              this->geometry.reset();
              this->surface.reset();
            };

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Collision Name[" << this->name << "]\n";
            }

  };
  
  class Link : public SDFBase
  {
    public: Link() : 
            origin("origin", gazebo::math::Pose(), false), 
            name("name", "", true),
            selfCollide("self_collide", false, false), 
            gravity("gravity",true, false) 
    { this->Clear(); };

    public: ParamT<gazebo::math::Pose> origin;
    public: ParamT<std::string> name;
    public: ParamT<bool> selfCollide;
    public: ParamT<bool> gravity;

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
              SDFBase::Clear();
              this->inertial.reset();
              this->visuals.clear();
              this->collisions.clear();
            };

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
