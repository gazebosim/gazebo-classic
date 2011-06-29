/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef SDF_HH
#define SDF_HH

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "sdf/interface/Param.hh"

namespace sdf
{
  class SDF;
  class Element;
  typedef boost::shared_ptr<SDF> SDFPtr;
  typedef boost::shared_ptr<Element> ElementPtr;
  typedef std::vector< ElementPtr > ElementPtr_V;

  class Element
  {

    public: boost::shared_ptr<Element> Clone() const;

    public: void SetName(const std::string &_name);
    public: const std::string &GetName() const;

    public: void SetRequired(const std::string &_req);
    public: const std::string &GetRequired() const;

    public: void PrintDescription(std::string _prefix);
    public: void PrintValues(std::string _prefix);


    public: void AddAttribute(const std::string &_key, 
                              const std::string &_type, 
                              const std::string &_defaultvalue,
                              bool _required);

    public: ParamPtr GetAttribute(const std::string &_key);

    public: bool GetValueBool(const std::string &_key);
    public: int GetValueInt(const std::string &_key);
    public: float GetValueFloat(const std::string &_key);
    public: double GetValueDouble(const std::string &_key);
    public: unsigned int GetValueUInt(const std::string &_key);
    public: char GetValueChar(const std::string &_key);
    public: std::string GetValueString(const std::string &_key);
    public: gazebo::math::Vector3 GetValueVector3(const std::string &_key);
    public: gazebo::math::Quaternion GetValueQuaternion(const std::string &_key);
    public: gazebo::math::Pose GetValuePose(const std::string &_key);
    public: gazebo::common::Color GetValueColor(const std::string &_key);
 
    public: bool HasElement(const std::string &_name) const;

    public: boost::shared_ptr<Element> GetElement(const std::string &_name) const;
    public: boost::shared_ptr<Element> GetOrCreateElement(const std::string &_name);
    public: boost::shared_ptr<Element> AddElement(const std::string &_name);

    private: std::string name;
    private: std::string required;

    public: Param_V attributes;
    public: ElementPtr_V elements;
    public: ElementPtr_V elementDescriptions;
  };


  class SDF
  {
    public: SDF();
    public: void PrintDescription();
    public: void PrintValues();

    public: ElementPtr root;
  };
}
#endif
