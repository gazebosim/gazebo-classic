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

#ifndef SDF_MODEL_HH
#define SDF_MODEL_HH

#include <string>
#include <map>
#include <iostream>
#include <boost/shared_ptr.hpp>


#include "sdf/interface/Param.hh"
#include "sdf/interface/plugin.hh"
#include "sdf/interface/link.hh"

namespace sdf
{
  class Model
  {
    public: Model();
  
    /// \brief Load Model given a filename
    public: bool InitFile(const std::string &_filename);

    /// \brief Load Model from a XML-string
    public: bool InitString(const std::string &_xmlstring);
  
    public: boost::shared_ptr<const Link> GetLink(const std::string &_name) const;
    public: boost::shared_ptr<const Joint> GetJoint(const std::string &_name) const;
    public: std::string GetName() const {return this->name.GetValue();}
  
    public: void GetLinks(std::vector<boost::shared_ptr<Link> > &_links) const;
  
    /// \brief complete list of Links
    public: std::map<std::string, boost::shared_ptr<Link> > links;

    /// \brief complete list of Joints
    public: std::map<std::string, boost::shared_ptr<Joint> > joints;

    /// \brief complete list of plugins
    public: std::map<std::string, boost::shared_ptr<Plugin> > plugins;

    public: ParamT<std::string,true> name;

    public: void Clear();
  
    /// non-const getLink()
    private: void GetLink(const std::string &_name, 
                          boost::shared_ptr<Link> &_link) const;
  
    /// non-const getMaterial()
    private: boost::shared_ptr<Material> GetMaterial(const std::string &_name) const;

    public: void Print(const std::string &prefix)
            {
              std::cout << prefix << "Model: Name[" << this->name << "]\n";

              // Print plugins
              std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator iter;
              for (iter = this->plugins.begin(); iter != this->plugins.end(); iter++)
              {
                iter->second->Print(prefix + "  ");
              }

              // Print links
              std::map<std::string, boost::shared_ptr<Link> >::const_iterator liter;
              for (liter = this->links.begin(); liter != this->links.end(); liter++)
              {
                liter->second->Print(prefix + "  ");
              }

              // Print joints
              std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
              for (jiter = this->joints.begin(); jiter != this->joints.end(); jiter++)
              {
                jiter->second->Print(prefix + "  ");
              }
            }

  };
}

#endif
