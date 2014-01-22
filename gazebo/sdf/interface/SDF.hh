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
#ifndef SDF_HH
#define SDF_HH

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "gazebo/sdf/interface/Param.hh"

#define SDF_VERSION "1.4"

/// \ingroup gazebo_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  class SDF;
  class Element;
  typedef boost::shared_ptr<SDF> SDFPtr;
  typedef boost::shared_ptr<Element> ElementPtr;
  typedef std::vector< ElementPtr > ElementPtr_V;

  /// A function that is used in the external SDF library. This is here to
  /// make the build work if the external SDF library is not present.
  void addURIPath(const std::string &_uri, const std::string &_path);

  /// A function that is used in the external SDF library. This is here to
  /// make the build work if the external SDF library is not present.
  void setFindCallback(boost::function<std::string (const std::string &)> _cb);

  /// \addtogroup gazebo_parser
  /// \{

  /// \brief SDF Element class
  class Element : public boost::enable_shared_from_this<Element>
  {
    public: Element();
    public: virtual ~Element() GAZEBO_DEPRECATED(1.6);

    public: boost::shared_ptr<Element> Clone() const GAZEBO_DEPRECATED(1.6);

    /// \brief Copy values from an Element
    public: void Copy(const ElementPtr _elem) GAZEBO_DEPRECATED(1.6);

    public: ElementPtr GetParent() const GAZEBO_DEPRECATED(1.6);
    public: void SetParent(const ElementPtr _parent) GAZEBO_DEPRECATED(1.6);

    public: void SetName(const std::string &_name) GAZEBO_DEPRECATED(1.6);
    public: const std::string &GetName() const GAZEBO_DEPRECATED(1.6);

    public: void SetRequired(const std::string &_req) GAZEBO_DEPRECATED(1.6);
    public: const std::string &GetRequired() const GAZEBO_DEPRECATED(1.6);

    public: void SetCopyChildren(bool _value) GAZEBO_DEPRECATED(1.6);
    public: bool GetCopyChildren() const GAZEBO_DEPRECATED(1.6);

    public: void PrintDescription(std::string _prefix) GAZEBO_DEPRECATED(1.6);
    public: void PrintValues(std::string _prefix) GAZEBO_DEPRECATED(1.6);
    public: void PrintWiki(std::string _prefix) GAZEBO_DEPRECATED(1.6);

    /// \brief Helper function for SDF::PrintDoc
    ///
    /// This generates the SDF html documentation.
    /// \param[out] _html Accumulated HTML for output.
    /// \param[in] _spacing Amount of spacing for this element.
    /// \param[in] _index Unique index for this element.
    public: void PrintDocLeftPane(std::string &_html,
                int _spacing, int &_index) GAZEBO_DEPRECATED(1.6);

    /// \brief Helper function for SDF::PrintDoc
    ///
    /// This generates the SDF html documentation.
    /// \param[out] _html Accumulated HTML for output
    /// \param[in] _spacing Amount of spacing for this element.
    public: void PrintDocRightPane(std::string &_html,
                int _spacing, int &_index) GAZEBO_DEPRECATED(1.6);

    private: void ToString(const std::string &_prefix,
                 std::ostringstream &_out) const GAZEBO_DEPRECATED(1.6);
    public: std::string ToString(
                const std::string &_prefix) const GAZEBO_DEPRECATED(1.6);

    public: void AddAttribute(const std::string &_key,
                              const std::string &_type,
                              const std::string &_defaultvalue,
                              bool _required,
                              const std::string &_description="")
                              GAZEBO_DEPRECATED(1.6);

    public: void AddValue(const std::string &_type,
                          const std::string &_defaultValue, bool _required,
                          const std::string &_description="")
                          GAZEBO_DEPRECATED(1.6);

    /// \brief Get the param of an attribute.
    /// \param _key the name of the attribute
    public: ParamPtr GetAttribute(
                const std::string &_key) GAZEBO_DEPRECATED(1.6);

    /// \brief Get the number of attributes
    public: unsigned int GetAttributeCount() const GAZEBO_DEPRECATED(1.6);

    /// \brief Get an attribute using an index
    public: ParamPtr GetAttribute(
                unsigned int _index) const GAZEBO_DEPRECATED(1.6);

    /// \brief Get the number of element descriptions
    public: unsigned int GetElementDescriptionCount() const
            GAZEBO_DEPRECATED(1.6);

    /// \brief Get an element description using an index
    public: ElementPtr GetElementDescription(unsigned int _index) const
            GAZEBO_DEPRECATED(1.6);

    /// \brief Get an element descriptio using a key
    public: ElementPtr GetElementDescription(const std::string &_key) const
            GAZEBO_DEPRECATED(1.6);

    /// \brief Return true if an element description exists
    public: bool HasElementDescription(
                const std::string &_name) GAZEBO_DEPRECATED(1.6);

    public: bool HasAttribute(const std::string &_key) GAZEBO_DEPRECATED(1.6);

    /// \brief Return true if the attribute was set (i.e. not default value)
    public: bool GetAttributeSet(const std::string &_key)
            GAZEBO_DEPRECATED(1.6);

    /// \brief Get the param of the elements value
    public: ParamPtr GetValue() GAZEBO_DEPRECATED(1.6);

    public: bool GetValueBool(
                const std::string &_key = "") GAZEBO_DEPRECATED(1.6);
    public: int GetValueInt(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: float GetValueFloat(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: double GetValueDouble(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: unsigned int GetValueUInt(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: char GetValueChar(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: std::string GetValueString(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: gazebo::math::Vector3 GetValueVector3(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: gazebo::math::Vector2d GetValueVector2d(
                const std::string &_key = "") GAZEBO_DEPRECATED(1.6);
    public: gazebo::math::Quaternion GetValueQuaternion(
                const std::string &_key = "") GAZEBO_DEPRECATED(1.6);
    public: gazebo::math::Pose GetValuePose(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: gazebo::common::Color GetValueColor(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);
    public: gazebo::common::Time GetValueTime(const std::string &_key = "")
            GAZEBO_DEPRECATED(1.6);

    public: template<typename T>
            T Get(const std::string &_key = "")
            {
              T result = T();

              if (_key.empty() && this->value)
                this->value->Get<T>(result);
              else if (!_key.empty())
              {
                ParamPtr param = this->GetAttribute(_key);
                if (param)
                  param->Get(result);
                else if (this->HasElement(_key))
                  result = this->GetElementImpl(_key)->Get<T>();
                else if (this->HasElementDescription(_key))
                  result = this->GetElementDescription(_key)->Get<T>();
                else
                  gzerr << "Unable to find value for key[" << _key << "]\n";
              }
              return result;
            }

    public: bool Set(const bool &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const int &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const unsigned int &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const float &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const double &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const char &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const std::string &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(const char *_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::math::Vector3 &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::math::Vector2i &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::math::Vector2d &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::math::Quaternion &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::math::Pose &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::common::Color &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Set(
                const gazebo::common::Time &_value) GAZEBO_DEPRECATED(1.6);

    public: bool HasElement(
                const std::string &_name) const GAZEBO_DEPRECATED(1.6);

    public: ElementPtr GetElement(
                const std::string &_name) const GAZEBO_DEPRECATED(1.6);
    public: ElementPtr GetFirstElement() const GAZEBO_DEPRECATED(1.6);

    public: ElementPtr GetNextElement(
                const std::string &_name = "") const GAZEBO_DEPRECATED(1.6);

    public: ElementPtr GetElement(
                const std::string &_name) GAZEBO_DEPRECATED(1.6);
    public: ElementPtr AddElement(
                const std::string &_name) GAZEBO_DEPRECATED(1.6);
    public: void InsertElement(ElementPtr _elem) GAZEBO_DEPRECATED(1.6);

    /// \brief Remove this element from its parent.
    public: void RemoveFromParent() GAZEBO_DEPRECATED(1.6);

    /// \brief Remove a child element.
    /// \param[in] _child Pointer to the child to remove.
    public: void RemoveChild(ElementPtr _child) GAZEBO_DEPRECATED(1.6);

    /// \brief Remove all child elements.
    public: void ClearElements() GAZEBO_DEPRECATED(1.6);

    public: void Update() GAZEBO_DEPRECATED(1.6);
    public: void Reset() GAZEBO_DEPRECATED(1.6);

    public: void SetInclude(
                const std::string &_filename) GAZEBO_DEPRECATED(1.6);
    public: std::string GetInclude() const GAZEBO_DEPRECATED(1.6);

    /// \brief Get a text description of the element
    public: std::string GetDescription() const GAZEBO_DEPRECATED(1.6);

    /// \brief Set a text description for the element
    public: void SetDescription(
                const std::string &_desc) GAZEBO_DEPRECATED(1.6);

    /// \brief Add a new element description
    public: void AddElementDescription(ElementPtr _elem) GAZEBO_DEPRECATED(1.6);

    private: boost::shared_ptr<Param> CreateParam(const std::string &_key,
                 const std::string &_type, const std::string &_defaultValue,
                 bool _required,
                 const std::string &_description="") GAZEBO_DEPRECATED(1.6);

    public: ElementPtr GetElementImpl(
                const std::string &_name) const GAZEBO_DEPRECATED(1.6);

    private: std::string name;
    private: std::string required;
    private: std::string description;
    private: bool copyChildren;

    private: ElementPtr parent;

    // Attributes of this element
    private: Param_V attributes;

    // Value of this element
    private: ParamPtr value;

    // The existing child elements
    private: ElementPtr_V elements;

    // The possible child elements
    private: ElementPtr_V elementDescriptions;

    /// name of the include file that was used to create this element
    private: std::string includeFilename;
  };


  /// \brief Base SDF class
  class SDF
  {
    public: SDF() GAZEBO_DEPRECATED(1.6);
    public: void PrintDescription() GAZEBO_DEPRECATED(1.6);
    public: void PrintValues() GAZEBO_DEPRECATED(1.6);
    public: void PrintWiki() GAZEBO_DEPRECATED(1.6);
    public: void PrintDoc() GAZEBO_DEPRECATED(1.6);
    public: void Write(const std::string &_filename) GAZEBO_DEPRECATED(1.6);
    public: std::string ToString() const GAZEBO_DEPRECATED(1.6);

    /// \brief Set SDF values from a string
    public: void SetFromString(
                const std::string &_sdfData) GAZEBO_DEPRECATED(1.6);

    public: ElementPtr root;

    public: static std::string version;
  };
  /// \}
}

#endif
