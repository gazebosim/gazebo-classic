/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: A parameter
 * Author: Nate Koenig
 * Date: 14 Aug 2008
 * SVN: $Id$
 */

#ifndef PARAM_HH
#define PARAM_HH

#include <iostream>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/any.hpp>
#include <boost/bind.hpp>
#include <boost/signal.hpp>
#include <typeinfo>
#include <string>

#include "XMLConfig.hh"

namespace gazebo
{
  class Param
  {
    /// \brief Constructor
    public: Param(Param *newParam);

    /// \brief Destructor
    public: virtual  ~Param();

    /// \brief Begin a block of "new Param<*>"
    public: static void Begin(std::vector<Param*> *_params);

    /// \brief End a block of "new Param<*>"
    public: static void End();

    /// \brief The name of the key
    public: std::string GetKey() const;

    /// \brief Get the name of the param's data type
    public: std::string GetTypename() const;
           
    /// \brief Get the type
    public: virtual std::string GetAsString() const {return std::string();}

    /// \brief Set the parameter value from a string
    public: virtual void SetFromString(const std::string &, bool callback=false) {}

    /// List of created parameters
    private: static std::vector<Param*> *params;

    protected: std::string key;
    protected: std::string typeName;
  };


  template< typename T>
  class ParamT : public Param
  {
    /// \brief Constructor
    public: ParamT(std::string key, T defValue, int required);
  
    /// \brief Destructor
    public: virtual ~ParamT();
  
    /// \brief Load the param from an XML config file
    public: void Load(XMLConfigNode *node);

    /// \brief Get the parameter value as a string
    public: virtual std::string GetAsString() const;

    /// \brief Set the parameter value from a string
    public: virtual void SetFromString( const std::string &str, bool callback=false );

    /// \brief Get the value
    public: T GetValue() const;

    /// \brief Set the value of the parameter
    public: void SetValue(const T &value);

    public: inline T operator*() const {return value;}

    public: friend std::ostream &operator<<( std::ostream &out, const ParamT<T> &p)
            {
              out << "<" << p.key << ">" << p.value << "</" << p.key << ">";

              return out;
            }

    public: template< typename C>
            void Callback( void (C::*func)(const T &), C *c)
            {
              changeSignal.connect( boost::bind( func, c, _1) ); 
            }

 
    private: T value;
  
    private: T defaultValue;
    private: int required;
  
    private: boost::signal<void (T)> changeSignal;
  };
 

  //////////////////////////////////////////////////////////////////////////////
  // Constructor
  template< typename T>
  ParamT<T>::ParamT(std::string key, T defValue, int required)
    : Param(this)
  {
    this->key = key;
    this->defaultValue = defValue;
    this->required = required;
    this->value = this->defaultValue;

    this->typeName = typeid(T).name();
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Destructor
  template<typename T>
  ParamT<T>::~ParamT()
  {
  }
 
  //////////////////////////////////////////////////////////////////////////////
  /// Load the param from an XML config file
  template<typename T>
  void ParamT<T>::Load(XMLConfigNode *node)
  {
    std::ostringstream stream;
    stream << this->defaultValue;

    std::string input = node->GetString(this->key, stream.str(), 
                                        this->required);

    this->SetFromString( input );
  }

  //////////////////////////////////////////////////////////////////////////////
  // Get value as string
  template<typename T>
  std::string ParamT<T>::GetAsString() const
  {
    return boost::lexical_cast<std::string>(this->value);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Set value from string
  template<typename T>
  void ParamT<T>::SetFromString(const std::string &str, bool callback)
  {
    std::string tmp = str;

    // "true" and "false" doesn't work properly
    if (tmp == "true")
      tmp = "1";
    else if (str == "false")
      tmp = "0";

    try
    {
      this->value = boost::lexical_cast<T>(tmp);
    }
    catch (boost::bad_lexical_cast &e)
    {
      std::cerr << "Unable to read value with key[" << this->key << "]\n";
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
  void ParamT<T>::SetValue(const T &v)
  {
    this->value = v;
  }

}
#endif
