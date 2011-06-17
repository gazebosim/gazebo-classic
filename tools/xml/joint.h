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
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>

#include "pose.h"

namespace gdf{

class Link;

class JointDynamics
{
public:
  JointDynamics() { this->clear(); };
  double damping;
  double friction;

  void clear()
  {
    damping = 0;
    friction = 0;
  };
  bool initXml(TiXmlElement* config);
};

class JointLimits
{
public:
  JointLimits() { this->clear(); };
  double lower;
  double upper;
  double effort;
  double velocity;

  void clear()
  {
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
  };
  bool initXml(TiXmlElement* config);
};


class Joint
{
public:

  Joint() { this->clear(); };

  std::string name;
  enum
  {
    UNKNOWN, REVOLUTE, REVOLUTE2, PRISMATIC, PISTON, BALL, UNIVERSAL
  } type;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN    unknown type
  ///            REVOLUTE   rotation axis
  ///            REVOLUTE2  rotation axis
  ///            PRISMATIC  translation axis
  ///            PISTON     N/A
  ///            BALL       N/A
  ///            UNIVERSAL  N/A
  Vector3 axis;
  Vector3 axis2;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;

  /// transform from Child Link frame to Joint frame
  Pose origin;

  /// Joint Dynamics
  boost::shared_ptr<JointDynamics> dynamics;

  /// Joint Limits
  boost::shared_ptr<JointLimits> limits;

  bool initXml(TiXmlElement* xml);
  void clear()
  {
    this->axis.clear();
    this->axis2.clear();
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->origin.clear();
    this->dynamics.reset();
    this->limits.reset();
    this->type = UNKNOWN;
  };
};

}

#endif
