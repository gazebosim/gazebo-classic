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
#include <boost/enable_shared_from_this.hpp>

#include "sdf/interface/Param.hh"

namespace sdf
{
  class SDF;
  class Element;
  typedef boost::shared_ptr<SDF> SDFPtr;
  typedef boost::shared_ptr<Element> ElementPtr;
  typedef std::vector< ElementPtr > ElementPtr_V;

  class Element : public boost::enable_shared_from_this<Element>
  {
    public: boost::shared_ptr<Element> Clone() const;

    public: ElementPtr GetParent() const;
    public: void SetParent(const ElementPtr &_parent);

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

    public: void AddValue(const std::string &_type, 
                       const std::string &_defaultValue, bool _required);

    /// \brief Get the param of an attribute.
    /// \param _key the name of the attribute
    public: ParamPtr GetAttribute(const std::string &_key);

    /// \brief Get the param of the elements value
    public: ParamPtr GetValue();

    public: bool GetValueBool(const std::string &_key = "");
    public: int GetValueInt(const std::string &_key = "");
    public: float GetValueFloat(const std::string &_key = "");
    public: double GetValueDouble(const std::string &_key = "");
    public: unsigned int GetValueUInt(const std::string &_key = "");
    public: char GetValueChar(const std::string &_key = "" );
    public: std::string GetValueString(const std::string &_key = "");
    public: gazebo::math::Vector3 GetValueVector3(const std::string &_key = "");
    public: gazebo::math::Quaternion GetValueQuaternion(const std::string &_key = "");
    public: gazebo::math::Pose GetValuePose(const std::string &_key = "");
    public: gazebo::common::Color GetValueColor(const std::string &_key = "");
 
    public: bool HasElement(const std::string &_name) const;

    public: ElementPtr GetElement(const std::string &_name) const;
    public: ElementPtr GetFirstElement() const;
    public: ElementPtr GetNextElement(const std::string &_name, 
                                       const ElementPtr &_elem) const;

    public: ElementPtr GetOrCreateElement(const std::string &_name);
    public: ElementPtr AddElement(const std::string &_name);
    public: void ClearElements();

    private: boost::shared_ptr<Param> CreateParam(const std::string &_key, 
                 const std::string &_type, const std::string &_defaultValue, 
                 bool _required);

    private: std::string name;
    private: std::string required;

    private: ElementPtr parent;

    // Attributes of this element
    public: Param_V attributes;

    // Value of this element
    public: ParamPtr value;

    // The existing child elements
    public: ElementPtr_V elements;

    // The possible child elements
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
