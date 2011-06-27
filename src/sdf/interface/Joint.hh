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

#include "sdf/interface/SDFBase.hh"
#include "math/Pose.hh"

namespace sdf
{
  class Joint : public SDFBase
  {
    public: Joint() : 
            name("name","", true), 
            type("type","", true), 
            axis("xyz",gazebo::math::Vector3(), false), 
            axis2("xyz", gazebo::math::Vector3(), false), 
            childLinkName("link", "", true), 
            parentLinkName("link", "", true), 
            origin("origin", gazebo::math::Pose(), true),
            damping("damping",0, false), 
            friction("friction",0, false),
            limitLower("lower",0, false), 
            limitUpper("upper",0, false),
            limitEffort("effort",0, false), 
            limitVelocity("velocity",0, false)
    { 
      Param::End();
      this->xmlTree = "{joint:type,name, {axis:xyz}, {axis2:xyz},\
                              {child:link}, {parent:link}, {origin:pose},\
                              {dynamics:damping,friction},\
                              {limit:lower,upper,effort,velocity}}";
      this->Clear(); 
    }

    public: ParamT<std::string> name;

    /// \brief     type_       meaning of axis_
    /// ------------------------------------------------------
    ///            UNKNOWN    unknown type
    ///            REVOLUTE   rotation axis
    ///            REVOLUTE2  rotation axis
    ///            PRISMATIC  translation axis
    ///            PISTON     N/A
    ///            BALL       N/A
    ///            UNIVERSAL  N/A
    public: ParamT<std::string> type;

    public: ParamT<gazebo::math::Vector3> axis;
    public: ParamT<gazebo::math::Vector3> axis2;

    /// child Link element
    ///   child link frame is the same as the Joint frame
    public: ParamT<std::string> childLinkName;

    /// parent Link element
    ///   origin specifies the transform from Parent Link to Joint Frame
    public: ParamT<std::string> parentLinkName;

    /// transform from Child Link frame to Joint frame
    public: ParamT<gazebo::math::Pose> origin;

    public: ParamT<double> damping;
    public: ParamT<double> friction;

    public: ParamT<double> limitLower;
    public: ParamT<double> limitUpper;
    public: ParamT<double> limitEffort;
    public: ParamT<double> limitVelocity;

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
