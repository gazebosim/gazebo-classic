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
#include "common/Event.hh"
#include "common/CommonTypes.hh"

namespace gazebo
{
	namespace common
  {
    class XMLConfigNode;

    class Param
    {
      /// \brief Constructor
      public: Param(Param *newParam);
  
      /// \brief Destructor
      public: virtual  ~Param();

      /// \brief Begin a block of "new Param<*>"
      public: static void Begin(Param_V *_params);
  
      /// \brief End a block of "new Param<*>"
      public: static void End();

       /// \brief Load the param from an XML config file
      public: void Load(XMLConfigNode *node);
  
      /// \brief The name of the key
      public: std::string GetKey() const;
  
      /// \brief Get the name of the param's data type
      public: std::string GetTypename() const;
             
      /// \brief Get the type
      public: virtual std::string GetAsString() const {return std::string();}
  
      public: virtual std::string GetDefaultAsString() const 
              {return std::string();}

      /// \brief Set the parameter value from a string
      public: virtual void SetFromString(const std::string &, bool /*callback_*/=false) {}
  
      /// \brief Set the help string
      public: void SetHelp(const std::string &h) {this->help = h;}
  
      /// \brief Get the help string
      public: std::string GetHelp() const {return this->help;}
  
      public: bool IsBool() const;
      public: bool IsInt() const;
      public: bool IsUInt() const;
      public: bool IsFloat() const;
      public: bool IsDouble() const;
      public: bool IsChar() const;
      public: bool IsStr() const;
      public: bool IsVector3() const;
      public: bool IsVector4() const;
      public: bool IsQuatern() const;
      public: bool IsPose3d() const;
  
      /// List of created parameters
      private: static std::vector<Param*> *params;
  
      protected: std::string key;
      protected: std::string typeName;
      protected: std::string help;
      protected: int required;
    };
  
  
    template< typename T>
    class ParamT : public Param
    {
      /// \brief Constructor
      public: ParamT(std::string key, T defValue, int required);
    
      /// \brief Destructor
      public: virtual ~ParamT();
 
      /// \brief Get the parameter value as a string
      public: virtual std::string GetAsString() const;

      public: virtual std::string GetDefaultAsString() const;
  
      /// \brief Set the parameter value from a string
      public: virtual void SetFromString( const std::string &str, bool callback=false );
  
      /// \brief Get the value
      public: T GetValue() const;
  
      /// \brief Set the value of the parameter
      public: void SetValue(const T &value, bool callback=false);

      /// \brief Set the default value
      public: void SetDefaultValue(const T &value);
  
      public: inline T operator*() const {return value;}
  
      public: friend std::ostream &operator<<( std::ostream &out, const ParamT<T> &p)
              {
                out << "<" << p.key << ">" << p.value << "</" << p.key << ">";
  
                return out;
              }
  
      public: template< typename C>
              void Callback( void (C::*func)(const T &), C *c)
              {
                connection = changeSignal.Connect( boost::bind( func, c, _1) ); 
              }
  
   
      private: T value;
    
      private: T defaultValue;
   
      private: event::ConnectionPtr connection; 
      private: event::EventT<void (T)> changeSignal;
    };
   
  
    //////////////////////////////////////////////////////////////////////////////
    // Constructor
    template< typename T>
    ParamT<T>::ParamT(std::string key_, T defValue_, int required_)
      : Param(this)
    {
      this->key = key_;
      this->defaultValue = defValue_;
      this->required = required_;
      this->value = this->defaultValue;
  
      this->typeName = typeid(T).name();
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Destructor
    template<typename T>
    ParamT<T>::~ParamT()
    {
      if (this->connection)
        this->changeSignal.Disconnect(this->connection);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Get value as string
    template<typename T>
    std::string ParamT<T>::GetAsString() const
    {
      return boost::lexical_cast<std::string>(this->value);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Get default value as string
    template<typename T>
    std::string ParamT<T>::GetDefaultAsString() const
    {
      return boost::lexical_cast<std::string>(this->defaultValue);
    }
 
    //////////////////////////////////////////////////////////////////////////////
    // Set value from string
    template<typename T>
    void ParamT<T>::SetFromString(const std::string &str, bool callback)
    {
      std::string tmp = str;
      std::string lower_tmp = str;
      boost::to_lower(lower_tmp);
  
      // "true" and "false" doesn't work properly
      if (lower_tmp == "true")
        tmp = "1";
      else if (lower_tmp == "false")
        tmp = "0";
  
      try
      {
        this->value = boost::lexical_cast<T>(tmp);
      }
      catch (boost::bad_lexical_cast &e)
      {
        if (str == "inf" || str == "-inf")
        {
          // int this case, the parser complains, but seems to assign the right values
          gzmsg << "INFO [gazebo::Param]: boost throws when lexical casting inf's, but the values are usually passed through correctly (key[" << this->key << "], value[" << str << "])\n";
        }
        else
        {
          gzerr << "Unable to read value with key[" << this->key << "] and value[" << str << "]\n";
        }
      }
  
      if (callback)
        this->changeSignal(this->value);
    }
  
    //////////////////////////////////////////////////////////////////////////////
    ///Get the value
    template<typename T>
    T ParamT<T>::GetValue() const
    {
      return this->value;
    }
  
    //////////////////////////////////////////////////////////////////////////////
    /// Set the value of the parameter
    template<typename T>
    void ParamT<T>::SetValue(const T &v, bool callback)
    {
      this->value = v;
  
      if (callback)
        this->changeSignal(this->value);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Set the default value of the parameter
    template<typename T>
    void ParamT<T>::SetDefaultValue(const T &v)
    {
      this->defaultValue = v;
    }
  
  }
}
#endif
