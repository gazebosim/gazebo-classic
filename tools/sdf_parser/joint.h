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

#ifndef URDF_JOINT_H
#define URDF_JOINT_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "pose.h"

namespace sdf
{
  class Link;
  
  class JointDynamics
  {
    public: JointDynamics() { this->Clear(); };
    public: double damping;
    public: double friction;

    public: void Clear()
            {
              damping = 0;
              friction = 0;
            };
  };
  
  class JointLimits
  {
    public: JointLimits() { this->Clear(); };
    public: double lower;
    public: double upper;
    public: double effort;
    public: double velocity;

    public: void Clear()
            {
              lower = 0;
              upper = 0;
              effort = 0;
              velocity = 0;
            };
  };
  
  
  class Joint
  {
    public: enum
            {
              UNKNOWN, REVOLUTE, REVOLUTE2, PRISMATIC, PISTON, BALL, UNIVERSAL
            } type;

    public: Joint() { this->Clear(); };

    public: std::string name;

    /// \brief     type_       meaning of axis_
    /// ------------------------------------------------------
    ///            UNKNOWN    unknown type
    ///            REVOLUTE   rotation axis
    ///            REVOLUTE2  rotation axis
    ///            PRISMATIC  translation axis
    ///            PISTON     N/A
    ///            BALL       N/A
    ///            UNIVERSAL  N/A
    public: Vector3 axis;
    public: Vector3 axis2;

    /// child Link element
    ///   child link frame is the same as the Joint frame
    public: std::string childLinkName;

    /// parent Link element
    ///   origin specifies the transform from Parent Link to Joint Frame
    public: std::string parentLinkName;

    /// transform from Child Link frame to Joint frame
    public: Pose origin;

    /// Joint Dynamics
    public: boost::shared_ptr<JointDynamics> dynamics;

    /// Joint Limits
    public: boost::shared_ptr<JointLimits> limits;


    public: void Clear()
            {
              this->axis.Clear();
              this->axis2.Clear();
              this->childLinkName.clear();
              this->parentLinkName.clear();
              this->origin.Clear();
              this->dynamics.reset();
              this->limits.reset();
              this->type = UNKNOWN;
            };

    public: friend std::ostream &operator<<(std::ostream &out, const Joint &joint)
            {
              out << "Joint Name[" << joint.name << "] Type[" << joint.type << "] Parent[" << joint.parentLinkName << "] Child[" << joint.childLinkName << "]";
              return out;
            }

  };

}

#endif
