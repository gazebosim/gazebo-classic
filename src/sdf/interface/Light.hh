/*
 * Copyright 2011 Nate Koenig & John Hsu
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

#ifndef SDF_LIGHT_HH
#define SDF_LIGHT_HH

#include <string>

#include "sdf/interface/SDFBase.hh"
#include "common/Color.hh"
#include "math/Pose.hh"
#include "math/Pose.hh"

namespace sdf
{
  class Light : public SDFBase
  {
    public: Light() :
              origin("pose", gazebo::math::Pose(), true),
              type("type","", false),
              name("name","", false), 
              cast_shadows("cast_shadows", false, false),
              diffuseColor("rgba", gazebo::common::Color(1,1,1,1), false),
              specularColor("rgba", gazebo::common::Color(.1,.1,.1,1), false),
              range("range", 10, false),
              constantAttenuation("constant", 1.0, false),
              linearAttenuation("linear", 1.0, false),
              quadraticAttenuation("quadratic", 0.0, false),
              spotInnerAngle("inner_angle", 0.0, false),
              spotOuterAngle("outer_angle", 0.0, false),
              spotFalloff("falloff", 0.0, false)
            {
              Param::End();
              this->Clear();
              this->xmlTree = "{ light : type,name,cast_shadows,\
                                 {origin: pose}, {diffuse : rgba},\
                                 {attenuation : range, constant, linear,\
                                                quadratic},\
                                 {spot: inner_angle, outer_angle, falloff}}"; 
            }


    public: ParamT<gazebo::math::Pose> origin;
    public: ParamT<std::string> type;
    public: ParamT<std::string> name;
    public: ParamT<bool> cast_shadows;
    public: ParamT<gazebo::common::Color> diffuseColor;
    public: ParamT<gazebo::common::Color> specularColor;

    public: ParamT<double> range;
    public: ParamT<double> constantAttenuation;
    public: ParamT<double> linearAttenuation;
    public: ParamT<double> quadraticAttenuation;

    public: ParamT<double> spotInnerAngle;
    public: ParamT<double> spotOuterAngle;
    public: ParamT<double> spotFalloff;

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Light name[" << this->name << "]\n";
            }
  };
}


#endif
