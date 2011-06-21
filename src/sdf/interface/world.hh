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

#ifndef SDF_WORLD_HH
#define SDF_WORLD_HH

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "sdf/interface/Param.hh"
#include "sdf/interface/plugin.hh"
#include "sdf/interface/scene.hh"
#include "sdf/interface/physics.hh"
#include "sdf/interface/model.hh"
#include "sdf/interface/joint.hh"

namespace sdf
{
  class World
  {
    public: World();
  
    public: boost::shared_ptr<const Model> GetModel(const std::string &_name) const;
    public: std::string GetName() const {return this->name.GetValue();}
  
    public: boost::shared_ptr<const Joint> GetJoint(const std::string &_name) const;
    public: void GetModels(std::vector<boost::shared_ptr<Model> > &_models) const;
  
    public: ParamT<std::string, true> name;
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
 
    public: void Print( const std::string &_prefix) 
            {
              std::cout << _prefix << "World: Name[" << this->name << "]\n";

              this->scene->Print( _prefix + "  " );

              this->physics->Print( _prefix + "  " );

              // Print models
              std::map<std::string, boost::shared_ptr<Model> >::const_iterator miter;
              for (miter = this->models.begin(); miter != this->models.end(); miter++)
              {
                miter->second->Print( _prefix + "  " );
              }

              // Print joints
              std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
              for (jiter = this->joints.begin(); jiter != this->joints.end(); jiter++)
              {
                jiter->second->Print( _prefix + "  " );
              }

              // Print plugins
              std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator iter;
              for (iter = this->plugins.begin(); iter != this->plugins.end(); iter++)
              {
                iter->second->Print( _prefix + "  " );
              }
            }

  };
}

#endif
