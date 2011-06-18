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

#ifndef ROBOT_WORLD_URDF_H
#define ROBOT_WORLD_URDF_H

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "plugin.h"
#include "scene.h"
#include "physics.h"
#include "model.h"
#include "joint.h"

namespace sdf
{
  class World
  {
    public: World();
  
    public: boost::shared_ptr<const Model> GetModel(const std::string &_name) const;
    public: const std::string& GetName() const {return this->name;};
  
    public: boost::shared_ptr<const Joint> GetJoint(const std::string &_name) const;
    public: void GetModels(std::vector<boost::shared_ptr<Model> > &_models) const;
  
    public: std::string name;
    public: boost::shared_ptr<Scene> scene;
    public: boost::shared_ptr<Physics> physics;

    /// \brief complete list of models
    public: std::map<std::string, boost::shared_ptr<Model> > models;

    /// \brief complete list of plugins
    public: std::map<std::string, boost::shared_ptr<Plugin> > plugins;

    /// \brief complete list of joints
    public: std::map<std::string, boost::shared_ptr<Joint> > joints;

    public: void Clear();
  
    /// non-const getLink()
    private: void GetModel(const std::string &_name, 
                          boost::shared_ptr<Model> &_model) const;
  
    public: friend std::ostream &operator<<(std::ostream &out, const World &world)
  {
    out << "World: Name[" << world.name << "]\n";

    out << *(world.scene.get()) << "\n";
    out << *(world.physics.get()) << "\n";

    // Print models
    std::map<std::string, boost::shared_ptr<Model> >::const_iterator miter;
    for (miter = world.models.begin(); miter != world.models.end(); miter++)
    {
      out << "  " << *(miter->second.get()) << "\n";
    }

    // Print joints
    std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
    for (jiter = world.joints.begin(); jiter != world.joints.end(); jiter++)
    {
      out << "  " << *(jiter->second.get()) << "\n";
    }

    // Print plugins
    std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator iter;
    for (iter = world.plugins.begin(); iter != world.plugins.end(); iter++)
    {
      out << "  " << *(iter->second.get()) << "\n";
    }

    return out;
  }

  };
}

#endif
