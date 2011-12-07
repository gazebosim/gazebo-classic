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

/* Author: Nate Koenig, John Hsu */

#ifndef SDF_PLUGIN_HH
#define SDF_PLUGIN_HH

#include <string>

#include "sdf/interface/SDFBase.hh"

namespace sdf
{
  class Plugin : public SDFBase
  {
    public: Plugin() : 
            name("name", "", true), 
            filename("filename", "", true)
            { 
              Param::End();
              this->xmlTree = "{plugin:name,filename}";
            }
  
    public: ParamT<std::string> name;
    public: ParamT<std::string> filename;
    public: std::vector<ParamT<std::string> > data;
 
    public: void Clear()
    {
      SDFBase::Clear();
      std::vector<ParamT<std::string> >::iterator iter;
      for (iter = this->data.begin(); iter != this->data.end(); iter++)
        iter->Reset();
      this->data.clear();
    }

    public: void Print(const std::string &prefix)
            {
              std::cout << prefix  << "Plugin: Name[" << this->name 
                        << "] File[" << this->filename << "]\n";
            }
  };
}

#endif
