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
/* Desc: A parameter
 * Author: Nate Koenig
 * Date: 14 Aug 2008
 */

#ifndef PARAM_HH
#define PARAM_HH

#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <typeinfo>
#include <string>

#include "common/Console.hh"
#include "sdf/interface/pose.hh"
#include "sdf/interface/color.hh"

namespace sdf
{
  class Param;
  typedef std::vector<sdf::Param*> Param_V;

    class Param
    {
      /// \brief Constructor
      public: Param(Param *_newParam);
  
      /// \brief Destructor
      public: virtual  ~Param();

      /// \brief Begin a block of "new Param<*>"
      public: static void Begin(Param_V *_params);
  
      /// \brief End a block of "new Param<*>"
      public: static void End();

      /// \brief Get the type
      public: virtual std::string GetAsString() const {return std::string();}
  
      public: virtual std::string GetDefaultAsString() const 
              {return std::string();}

      /// \brief Set the parameter value from a string
      public: virtual bool Set(const char * /*_string*/) {return true;}

      public: const std::string &GetKey() const {return this->key;} 

      public: std::string GetTypeName() const;

      public: bool IsBool() const;
      public: bool IsInt() const;
      public: bool IsUInt() const;
      public: bool IsFloat() const;
      public: bool IsDouble() const;
      public: bool IsChar() const;
      public: bool IsStr() const;
      public: bool IsVector3() const;
      public: bool IsRotation() const;
  
      /// List of created parameters
      private: static std::vector<Param*> *params;
  
      protected: std::string key; 
      protected: bool required;
      protected: std::string typeName;
    };
  
  
    template< typename T, bool _required>
    class ParamT : public Param
    {
      /// \brief Constructor
      public: ParamT(const std::string &_key, const T &_default)
              : Param(this), value(_default), defaultValue(_default)
      {
        this->key = _key;
        this->required = _required;
        this->typeName = typeid(T).name();
      }
 
      /// \brief Destructor
      public: virtual ~ParamT() {}
 
      /// \brief Get the parameter value as a string
      public: virtual std::string GetAsString() const
      {
         return boost::lexical_cast<std::string>(this->value);
      }

      public: virtual std::string GetDefaultAsString() const
      {
        return boost::lexical_cast<std::string>(this->defaultValue);
      }
 

      /// \brief Set the parameter value from a string
      public: virtual bool Set( const char *_cstr )
      {
        if (_cstr == NULL && this->required)
        {
          gzerr << "Empty string used when setting a required parameter\n";
          return false;
        }
        else if (_cstr == NULL)
        {
          this->value = this->defaultValue;
          return true;
        }

        std::string _str = _cstr;

        std::string tmp = _str;
        std::string lowerTmp = _str;
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
        catch (boost::bad_lexical_cast &e)
        {
          if (_str == "inf" || _str == "-inf")
          {
            // in this case, the parser complains, but seems to assign the 
            // right values
            gzmsg << "INFO [sdf::Param]: boost throws when lexical casting inf's, but the values are usually passed through correctly\n";
          }
          else
          {
            gzerr << "Unable to read value [" <<  _str << "]\n";
            return false;
          }
        }

        return true;
      } 

      /// \brief Get the value
      public: T GetValue() const
      {
        return this->value;
      }

      /// \brief Set the value of the parameter
      public: void SetValue(const T &_value)
      {
        this->value = _value;
      }

      /// \brief Reset to default value
      public: void Reset()
      {
        this->value = this->defaultValue;
      }

      public: inline T operator*() const {return value;}
  
      public: friend std::ostream &operator<<( std::ostream &_out, 
                                               const ParamT<T, _required> &_p)
              {
                _out << _p.value;
                return _out;
              }
  
      private: T value;
      private: T defaultValue;
    };

    template<bool _required>
    class ParamT<Vector3, _required> : public Param
    {
      /// \brief Constructor
      public: ParamT(const std::string &_key, const Vector3 &_default)
              : Param(this), value(_default), defaultValue(_default)
      {
        this->key = _key;
        this->required = _required;
        this->typeName = typeid(Vector3).name();
      }
 
      /// \brief Destructor
      public: virtual ~ParamT() {}
 
      /// \brief Get the parameter value as a string
      public: virtual std::string GetAsString() const
      {
        std::ostringstream ostream;
        ostream << this->value;
        return ostream.str();
      }

      public: virtual std::string GetDefaultAsString() const
      {
        std::ostringstream ostream;
        ostream << this->defaultValue;
        return ostream.str();
      }

      /// \brief Set the parameter value from a string
      public: virtual bool Set( const char *_str )
      {
        if (_str == NULL && this->required)
        {
          gzerr << "Empty string used when setting a required parameter\n";
          return false;
        }
        else if (_str == NULL)
        {
          this->value = this->defaultValue;
          return true;
        }

        return this->value.Init(_str);
      } 

      /// \brief Get the value
      public: Vector3 GetValue() const
      {
        return this->value;
      }

      /// \brief Set the value of the parameter
      public: void SetValue(const Vector3 &_value)
      {
        this->value = _value;
      }

      /// \brief Reset to default value
      public: void Reset()
      {
        this->value = this->defaultValue;
      }

      public: inline Vector3 operator*() const {return this->value;}
  
      public: friend std::ostream &operator<<( std::ostream &_out, 
                                         const ParamT<Vector3, _required> &_p)
              {
                _out << _p.value;
                return _out;
              }
  
      private: Vector3 value;
      private: Vector3 defaultValue;
    };

    template<bool _required>
    class ParamT<Color, _required> : public Param
    {
      /// \brief Constructor
      public: ParamT(const std::string &_key, const Color &_default)
              : Param(this), value(_default), defaultValue(_default)
      {
        this->key = _key;
        this->required = _required;
        this->typeName = typeid(Color).name();
      }
 
      /// \brief Destructor
      public: virtual ~ParamT() {}
 
      /// \brief Get the parameter value as a string
      public: virtual std::string GetAsString() const
      {
        std::ostringstream ostream;
        ostream << this->value;
        return ostream.str();
      }

      public: virtual std::string GetDefaultAsString() const
      {
        std::ostringstream ostream;
        ostream << this->defaultValue;
        return ostream.str();
      }

      /// \brief Set the parameter value from a string
      public: virtual bool Set( const char *_str )
      {
        if (_str == NULL && this->required)
        {
          gzerr << "Empty string used when setting a required parameter\n";
          return false;
        }
        else if (_str == NULL)
        {
          this->value = this->defaultValue;
          return true;
        }

        return this->value.Init(_str);
      } 

      /// \brief Get the value
      public: Color GetValue() const
      {
        return this->value;
      }

      /// \brief Set the value of the parameter
      public: void SetValue(const Color &_value)
      {
        this->value = _value;
      }

      /// \brief Reset to default value
      public: void Reset()
      {
        this->value = this->defaultValue;
      }

      public: inline Color operator*() const {return this->value;}
  
      public: friend std::ostream &operator<<( std::ostream &_out, 
                                         const ParamT<Color, _required> &_p)
              {
                _out << _p.value;
                return _out;
              }
  
      private: Color value;
      private: Color defaultValue;
    };

    template<bool _required>
    class ParamT<Pose, _required> : public Param
    {
      /// \brief Constructor
      public: ParamT(const std::string &_key, const Pose &_default)
              : Param(this), value(_default), defaultValue(_default)
      {
        this->key = _key;
        this->required = _required;
        this->typeName = typeid(Pose).name();
      }
 
      /// \brief Destructor
      public: virtual ~ParamT() {}
 
      /// \brief Get the parameter value as a string
      public: virtual std::string GetAsString() const
      {
        std::ostringstream ostream;
        ostream << "xyz=\"" << this->value.position << "\" ";
        ostream << "rpy=\"" << this->value.rotation << "\"";
        return ostream.str();
      }

      public: virtual std::string GetDefaultAsString() const
      {
        std::ostringstream ostream;
        ostream << "xyz=\"" << this->defaultValue.position << "\" ";
        ostream << "rpy=\"" << this->defaultValue.rotation << "\"";
        return ostream.str();
      }

      /// \brief Set the parameter value from a string
      public: virtual bool Set( const char *_xyz, 
                                const char *_rpy )
      {
        if ( (!_xyz && !_rpy) && this->required)
        {
          gzerr << "Empty string used when setting a required parameter\n";
          return false;
        }

        if (_xyz && !this->value.position.Init(_xyz))
        {
          gzerr << "Unable to set position value\n";
          return false;
        }
        else
          this->value.position = this->defaultValue.position;

        if (_rpy && !this->value.rotation.Init(_rpy))
        {
          gzerr << "Unable to set rotation value\n";
          return false;
        }
        else
          this->value.rotation = this->defaultValue.rotation;


        return true;
      } 

      /// \brief Get the value
      public: Pose GetValue() const
      {
        return this->value;
      }

      /// \brief Set the value of the parameter
      public: void SetValue(const Pose &_value)
      {
        this->value = _value;
      }

      /// \brief Reset to default value
      public: void Reset()
      {
        this->value = this->defaultValue;
      }

      public: inline Pose operator*() const {return this->value;}
  
      public: friend std::ostream &operator<<( std::ostream &_out, 
                                               const ParamT<Pose, _required> &_p)
              {
                _out << _p.value.x << " " << _p.value.y << " " << _p.value.z;
                return _out;
              }
  
      private: Pose value;
      private: Pose defaultValue;
    };
}
#endif
