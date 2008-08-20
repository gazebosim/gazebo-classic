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
 * SVN: $Id:$
 */

#ifndef PARAM_HH
#define PARAM_HH

#include <iostream>
#include <boost/lexical_cast.hpp>
#include <string>

#include "XMLConfig.hh"

namespace gazebo
{
  template< typename T >
  class Param
  {
    /// \brief Constructor
    public: Param(std::string key, T defValue, int required);
  
    /// \brief Destructor
    public: virtual ~Param();
  
    /// \brief Load the param from an XML config file
    public: void Load(XMLConfigNode *node);

    /// \brief Get the value
    public: T GetValue() const;

    /// \brief Set the value of the parameter
    public: void SetValue(const T &value);

    public: inline T operator*() const {return value;}

    public: friend std::ostream &operator<<( std::ostream &out, const Param<T> &p)
            {
              out << "<" << p.key << ">" << p.value << "</" << p.key << ">";

              return out;
            }
  
    private: T value;
  
    private: std::string key;
    private: T defaultValue;
    private: int required;
  
  };
 

  //////////////////////////////////////////////////////////////////////////////
  // Constructor
  template< typename T>
  Param<T>::Param(std::string key, T defValue, int required)
  {
    this->key = key;
    this->defaultValue = defValue;
    this->required = required;
    this->value = this->defaultValue;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Destructor
  template<typename T>
  Param<T>::~Param()
  {
  }
 
  //////////////////////////////////////////////////////////////////////////////
  /// Load the param from an XML config file
  template<typename T>
  void Param<T>::Load(XMLConfigNode *node)
  {
    std::ostringstream stream;
    stream << this->defaultValue;

    std::string input = node->GetString(this->key, stream.str(), 
                                        this->required);

    // "true" and "false" doesn't work properly
    if (input == "true")
      input = "1";
    else if (input == "false")
      input = "0";

    try
    {
      this->value = boost::lexical_cast<T>(input);
    }
    catch (boost::bad_lexical_cast &e)
    {
      std::cerr << "Unable to read value with key[" << this->key << "]\n";
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  ///Get the value
  template<typename T>
  T Param<T>::GetValue() const
  {
    return this->value;
  }

  //////////////////////////////////////////////////////////////////////////////
  /// Set the value of the parameter
  template<typename T>
  void Param<T>::SetValue(const T &v)
  {
    this->value = v;
  }

}
#endif
