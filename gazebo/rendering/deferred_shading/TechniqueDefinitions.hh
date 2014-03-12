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
#ifndef _TECHNIQUEDEFINITIONS_HH_
#define _TECHNIQUEDEFINITIONS_HH_

#include <string>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class GAZEBO_VISIBLE NullTechnique
    {
      protected: std::string GetMaterialPrefix() const
                 {return "NullTechnique";}
      protected: int GetNumInputs() const
                 {return 0;}
      protected: bool UseMaterialProperties() const
                 {return true;}
    };
    class GAZEBO_VISIBLE DeferredShading
    {
      protected: std::string GetMaterialPrefix() const
                 {return "DeferredShading";}
      protected: int GetGBufferSize() const
                 {return 2;}
      protected: bool UseMaterialProperties() const
                 {return true;}
    };
    class GAZEBO_VISIBLE DeferredLighting
    {
      protected: std::string GetMaterialPrefix() const
                 {return "DeferredLighting";}
      protected: int GetGBufferSize() const
                 {return 1;}
      protected: bool UseMaterialProperties() const
                 {return false;}
    };
    class GAZEBO_VISIBLE InferredLighting
    {
      protected: std::string GetMaterialPrefix() const
                 {return "InferredLighting";}
      protected: int GetGBufferSize() const
                 {return 1;}
      protected: bool UseMaterialProperties() const
                 {return false;}
    };
  }
}
#endif
