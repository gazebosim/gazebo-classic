
/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Author: Nate Koenig, John Hsu */

#ifndef SDF_PLUGIN_HH
#define SDF_PLUGIN_HH

#include <string>
#include <vector>

#include "gazebo/sdf/interface/SDFBase.hh"

namespace sdf
{
  class Plugin : public SDFBase
  {
    public: Plugin() :
            name("name", "", true),
            filename("filename", "", true)
            {
              Param::End();
              this->xmlTree = "{plugin:name, filename}";
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


