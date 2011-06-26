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

#include "common/Color.hh"
#include "math/Pose.hh"
#include "sdf/interface/Param.hh"
#include "math/Pose.hh"

namespace sdf
{
  class Base
  {
    public: Base()
            {
              Param::Begin(&this->parameters);
            }

    public: std::vector<Param*> parameters;
  };

  class Light : public Base
  {
    public: Light() :
              origin("pose", gazebo::math::Pose()),
              type("type",""),
              name("name",""), cast_shadows("cast_shadows", false),
              diffuseColor("rgba", gazebo::common::Color(1,1,1,1)),
              specularColor("rgba", gazebo::common::Color(.1,.1,.1,1)),
              range("range", 10),
              constantAttenuation("constant", 1.0),
              linearAttenuation("linear", 1.0),
              quadraticAttenuation("quadratic", 0.0),
              spotInnerAngle("inner_angle", 0.0),
              spotOuterAngle("outer_angle", 0.0),
              spotFalloff("falloff", 0.0)
            {
              Param::End();
              this->tree = "{ light : type,name,cast_shadows,\
                                   {origin: pose}, {diffuse : rgba},\
                                   {attenuation : range, constant, linear,\
                                                  quadratic},\
                                   {spot: inner_angle, outer_angle, falloff}}"; 
              this->Clear();
            }


    public: std::string tree;
    public: ParamT<gazebo::math::Pose, true> origin;
    public: ParamT<std::string, false> type;
    public: ParamT<std::string, false> name;
    public: ParamT<bool, false> cast_shadows;
    public: ParamT<gazebo::common::Color, false> diffuseColor;
    public: ParamT<gazebo::common::Color, false> specularColor;
    public: ParamT<double, false> range;
    public: ParamT<double, false> constantAttenuation;
    public: ParamT<double, false> linearAttenuation;
    public: ParamT<double, false> quadraticAttenuation;

    public: ParamT<double, false> spotInnerAngle;
    public: ParamT<double, false> spotOuterAngle;
    public: ParamT<double, false> spotFalloff;

    public: void Clear()
            {
              this->origin.Reset();
              this->name.Reset();
              this->diffuseColor.Reset();
              this->specularColor.Reset();
              this->range.Reset();
              this->constantAttenuation.Reset();
              this->linearAttenuation.Reset();
              this->quadraticAttenuation.Reset();
              this->spotInnerAngle.Reset();
              this->spotOuterAngle.Reset();
              this->spotFalloff.Reset();
            }

    public: void Print(const std::string &_prefix)
            {
              std::cout << _prefix << "Light name[" << this->name << "]\n";
            }
  };
}


#endif
