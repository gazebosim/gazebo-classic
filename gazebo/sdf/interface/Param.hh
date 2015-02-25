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
/* Desc: A parameter
 * Author: Nate Koenig
 * Date: 14 Aug 2008
 */

#ifndef SDF_PARAM_HH
#define SDF_PARAM_HH

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <boost/lexical_cast.hpp>
#endif
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/function.hpp>
#include <typeinfo>
#include <string>
#include <vector>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2i.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Quaternion.hh"

namespace sdf
{
  class Param;
  typedef boost::shared_ptr< Param > ParamPtr;
  typedef std::vector< ParamPtr > Param_V;

  /// \brief A parameter class
  class Param
  {
    /// \brief Constructor
    public: Param(Param *_newParam) GAZEBO_DEPRECATED(1.6);

    /// \brief Destructor
    public: virtual  ~Param() GAZEBO_DEPRECATED(1.6);

    /// \brief Get the type
    public: virtual std::string GetAsString() const GAZEBO_DEPRECATED(1.6)
            {return std::string();}
    public: virtual std::string GetDefaultAsString() const
            GAZEBO_DEPRECATED(1.6)
            {return std::string();}

    /// \brief Set the parameter value from a string
    public: virtual bool SetFromString(const std::string &)
            GAZEBO_DEPRECATED(1.6)
            {return true;}
    /// \brief Reset the parameter
    public: virtual void Reset() GAZEBO_DEPRECATED(1.6) = 0;

    public: const std::string &GetKey() const GAZEBO_DEPRECATED(1.6)
            {return this->key;}
    public: std::string GetTypeName() const GAZEBO_DEPRECATED(1.6);
    public: bool GetRequired() const  GAZEBO_DEPRECATED(1.6)
            { return this->required; }
    /// \brief Return true if the parameter has been set
    public: bool GetSet() const  GAZEBO_DEPRECATED(1.6)
            { return this->set; }
    public: virtual boost::shared_ptr<Param> Clone() const
            GAZEBO_DEPRECATED(1.6) = 0;

    /// \brief Update function
    public: template<typename T> void SetUpdateFunc(T _updateFunc)
            { this->updateFunc = _updateFunc; }
    public: virtual void Update() GAZEBO_DEPRECATED(1.6) = 0;

    /// \brief Get the value of the parameter.
    /// \param[out] _value The value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool Get(T &_value)
            {
              try
              {
                _value = boost::lexical_cast<T>(this->GetAsString());
              }
              catch(...)
              {
                gzerr << "Unable to convert parameter[" << this->key << "] "
                      << "whose type is[" << this->typeName << "], to "
                      << "type[" << typeid(T).name() << "]\n";
                return false;
              }
              return true;
            }

    public: bool IsBool() const GAZEBO_DEPRECATED(1.6);
    public: bool IsInt() const GAZEBO_DEPRECATED(1.6);
    public: bool IsUInt() const GAZEBO_DEPRECATED(1.6);
    public: bool IsFloat() const GAZEBO_DEPRECATED(1.6);
    public: bool IsDouble() const GAZEBO_DEPRECATED(1.6);
    public: bool IsChar() const GAZEBO_DEPRECATED(1.6);
    public: bool IsStr() const GAZEBO_DEPRECATED(1.6);
    public: bool IsVector3() const GAZEBO_DEPRECATED(1.6);
    public: bool IsVector2i() const GAZEBO_DEPRECATED(1.6);
    public: bool IsVector2d() const GAZEBO_DEPRECATED(1.6);
    public: bool IsQuaternion() const GAZEBO_DEPRECATED(1.6);
    public: bool IsPose() const GAZEBO_DEPRECATED(1.6);
    public: bool IsColor() const GAZEBO_DEPRECATED(1.6);
    public: bool IsTime() const GAZEBO_DEPRECATED(1.6);

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

    public: bool Get(bool &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(int &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(unsigned int &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(float &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(double &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(char &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(std::string &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::math::Vector3 &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::math::Vector2i &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::math::Vector2d &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::math::Quaternion &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::math::Pose &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::common::Color &_value) GAZEBO_DEPRECATED(1.6);
    public: bool Get(gazebo::common::Time &_value) GAZEBO_DEPRECATED(1.6);

    /// \brief Set the description of the parameter
    public: void SetDescription(
                const std::string &_desc) GAZEBO_DEPRECATED(1.6);

    /// \brief Get the description of the parameter
    public: std::string GetDescription() const GAZEBO_DEPRECATED(1.6);

    /// List of created parameters
    private: static std::vector<Param*> *params GAZEBO_DEPRECATED(1.6);

    protected: std::string key;
    protected: bool required;
    protected: bool set;
    protected: std::string typeName;
    protected: std::string description;

    protected: boost::function<boost::any ()> updateFunc;
  };

  /// \brief Templatized parameter class
  template< typename T>
  class ParamT : public Param
  {
    /// \brief Constructor
    public: ParamT(const std::string &_key, const std::string &_default,
                   bool _required, const std::string &_typeName = "",
                   const std::string &_description = "")
            GAZEBO_DEPRECATED(1.6)
            : Param(this)
    {
      this->key = _key;
      this->required = _required;
      if (_typeName.empty())
        this->typeName = typeid(T).name();
      else
        this->typeName = _typeName;
      this->description = _description;

      this->Set(_default);
      this->defaultValue = this->value;
      this->set = false;
    }

    /// \brief Destructor
    public: virtual ~ParamT() GAZEBO_DEPRECATED(1.6) {}

    /// \brief Update param value
    public: virtual void Update() GAZEBO_DEPRECATED(1.6)
            {
              if (this->updateFunc)
              {
                try
                {
                  const T v = boost::any_cast<T>(this->updateFunc());
                  Param::Set(v);
                }
                catch(boost::bad_any_cast &e)
                {
                  gzerr << "boost any_cast error:" << e.what() << "\n";
                }
              }
            }

    /// \brief Get the parameter value as a string
    public: virtual std::string GetAsString() const GAZEBO_DEPRECATED(1.6)
    {
      std::ostringstream stream;
      stream << std::fixed << this->value;
      return stream.str();
    }

    /// \brief Set the parameter value from a string
    public: virtual bool SetFromString(const std::string &_value)
            GAZEBO_DEPRECATED(1.6)
    { return this->Set(_value); }

    public: virtual std::string GetDefaultAsString() const
            GAZEBO_DEPRECATED(1.6)
    {
      return boost::lexical_cast<std::string>(this->defaultValue);
    }

    /// \brief Set the parameter value from a string
    public: virtual bool Set(const std::string &_str)
            GAZEBO_DEPRECATED(1.6)
    {
      std::string str = _str;
      boost::trim(str);
      if (str.empty() && this->required)
      {
        gzerr << "Empty string used when setting a required parameter. Key["
              << this->GetKey() << "]\n";
        return false;
      }
      else if (str.empty())
      {
        this->value = this->defaultValue;
        return true;
      }

      std::string tmp(str);
      std::string lowerTmp(str);
      boost::to_lower(lowerTmp);

      // "true" and "false" doesn't work properly
      if (lowerTmp == "true")
        tmp = "1";
      else if (lowerTmp == "false")
        tmp = "0";

      try
      {
        this->value = boost::lexical_cast<T>(tmp);
      }
      catch(boost::bad_lexical_cast &e)
      {
        if (str == "inf" || str == "-inf")
        {
          // in this case, the parser complains, but seems to assign the
          // right values
          gzmsg << "INFO [sdf::Param]: boost throws when lexical casting "
            << "inf's, but the values are usually passed through correctly\n";
        }
        else
        {
          gzerr << "Unable to set value [" <<  str
                << "] for key[" << this->key << "]\n";
          return false;
        }
      }

      this->set = true;
      return this->set;
    }

    /// \brief Get the value
    public: T GetValue() const GAZEBO_DEPRECATED(1.6)
    {
      return this->value;
    }

    /// \brief Get the value
    public: T GetDefaultValue() const GAZEBO_DEPRECATED(1.6)
    {
      return this->defaultValue;
    }

    /// \brief Set the value of the parameter
    public: void SetValue(const T &_value) GAZEBO_DEPRECATED(1.6)
    {
      this->value = _value;
      this->set = true;
    }

    /// \brief Reset to default value
    public: virtual void Reset() GAZEBO_DEPRECATED(1.6)
    {
      this->value = this->defaultValue;
      this->set = false;
    }

    public: virtual boost::shared_ptr<Param> Clone() const
            GAZEBO_DEPRECATED(1.6)
            {
              boost::shared_ptr<ParamT<T> > clone(
                  new ParamT<T>(this->GetKey(), this->GetAsString(),
                                this->required, this->typeName,
                                this->description));
              return clone;
            }

    public: inline T operator*() const GAZEBO_DEPRECATED(1.6) {return value;}
    public: friend std::ostream &operator<<(std::ostream &_out,
                const ParamT<T> &_p) GAZEBO_DEPRECATED(1.6)
            {
              _out << _p.value;
              return _out;
            }

    protected: T value;
    protected: T defaultValue;
  };
}
#endif
