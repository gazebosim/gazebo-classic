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

/* Desc: Base class shared by all classes in Gazebo.
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * SVN: $Id$
 */

#ifndef COMMON_HH
#define COMMON_HH

#include <vector>
#include <string>

#include "Param.hh"

namespace gazebo
{
  class Common
  {
    /// \brief Constructor
    public: Common();

    /// \brief Destructor
    public: virtual ~Common();

    /// \brief Set the name of the entity
    /// \param name Body name
    public: void SetName(const std::string &name);
  
    /// \brief Return the name of the entity
    /// \return Name of the entity
    public: std::string GetName() const;

    /// \brief Get the count of the parameters
    public: unsigned int GetParamCount() const;

    /// \brief Get a param by index
    public: Param *GetParam(unsigned int index) const;

    /// \brief Get a parameter by name
    public: Param *GetParam(const std::string &key) const;

     /// \brief Set a parameter by name
    public: void SetParam(const std::string &key, const std::string &value);
   
    /// \brief Return the ID of this entity. This id is unique
    /// \return Integer ID
    public: int GetId() const;

    /// \brief Set whether the object should be "saved", when the user
    ///        selects to save the world to xml
    public: void SetSaveable(bool v);

    /// \brief Get whether the object should be "saved", when the user
    ///        selects to save the world to xml
    public: bool GetSaveable() const;
   
    /// \brief This entities ID
    private: unsigned int id;
  
    /// \brief Used to automaticaly chose a unique ID on creation
    private: static unsigned int idCounter;
 
    ///  Name of the entity
    protected: ParamT<std::string> *nameP;

    /// List of all the parameters
    protected: std::vector<Param*> parameters;

    /// \brief Set to true if the object should be saved.
    protected: bool saveable;
  };
}

#endif

