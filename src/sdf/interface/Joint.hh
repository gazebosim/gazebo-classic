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

#ifndef SDF_JOINT_HH
#define SDF_JOINT_HH

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "sdf/interface/Param.hh"
#include "math/Pose3d.hh"

namespace sdf
{
  class Link;
  
  class JointDynamics
  {
    public: JointDynamics() : damping("damping",0), friction("friction",0) 
            { this->Clear(); }

    public: ParamT<double, true> damping;
    public: ParamT<double, true> friction;

    public: void Clear()
            {
              damping.Reset();
              friction.Reset();
            };
  };
  
  class JointLimits
  {
    public: JointLimits() : lower("lower",0), upper("upper",0),
            effort("effort",0), velocity("velocity",0) 
    { this->Clear(); }

    public: ParamT<double,true> lower;
    public: ParamT<double,true> upper;
    public: ParamT<double,true> effort;
    public: ParamT<double,true> velocity;

    public: void Clear()
            {
              this->lower.Reset();
              this->upper.Reset();
              this->effort.Reset();
              this->velocity.Reset();
            };
  };
  
  
  class Joint
  {
    public: enum
            {
              UNKNOWN, REVOLUTE, REVOLUTE2, PRISMATIC, PISTON, BALL, UNIVERSAL
            } typeEnum;

    public: Joint() : name("name",""), type("type",""), 
            axis("axis",gazebo::math::Vector3()), 
            axis2("axis2", gazebo::math::Vector3()), 
            childLinkName("link", ""), 
            parentLinkName("link", ""), origin("origin", gazebo::math::Pose3d())
    { this->Clear(); };

    public: ParamT<std::string,true> name;
    public: ParamT<std::string,true> type;

    /// \brief     type_       meaning of axis_
    /// ------------------------------------------------------
    ///            UNKNOWN    unknown type
    ///            REVOLUTE   rotation axis
    ///            REVOLUTE2  rotation axis
    ///            PRISMATIC  translation axis
    ///            PISTON     N/A
    ///            BALL       N/A
    ///            UNIVERSAL  N/A
    public: ParamT<gazebo::math::Vector3, false> axis;
    public: ParamT<gazebo::math::Vector3, false> axis2;

    /// child Link element
    ///   child link frame is the same as the Joint frame
    public: ParamT<std::string, true> childLinkName;

    /// parent Link element
    ///   origin specifies the transform from Parent Link to Joint Frame
    public: ParamT<std::string, true> parentLinkName;

    /// transform from Child Link frame to Joint frame
    public: ParamT<gazebo::math::Pose3d, true> origin;

    /// Joint Dynamics
    public: boost::shared_ptr<JointDynamics> dynamics;

    /// Joint Limits
    public: boost::shared_ptr<JointLimits> limits;

    public: void Clear()
            {
              this->axis.Reset();
              this->axis2.Reset();
              this->childLinkName.Reset();
              this->parentLinkName.Reset();
              this->origin.Reset();
              this->dynamics.reset();
              this->limits.reset();
              this->typeEnum = UNKNOWN;
            };

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Joint: Name[" << this->name << "] Type[" 
                  << this->type << "]\n";
              std::cout <<  _prefix << "  Parent[" << this->parentLinkName << "]\n";
              std::cout << _prefix << "  Child[" << this->childLinkName << "]\n";
            }



  };
}

#endif
