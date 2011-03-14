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

/* Desc: Base class shared by all classes in Gazebo.
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * SVN: $Id$
 */

#ifndef COMMON_HH
#define COMMON_HH

#include <vector>
#include <string>

#include "Global.hh"
#include "Param.hh"

namespace gazebo
{
	namespace common
{
  class Common
  {
    /// \brief Constructor
    public: Common(Common *parent);

    /// \brief Destructor
    public: virtual ~Common();

    /// \brief Load 
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Set the name of the entity
    /// \param name Body name
    public: virtual void SetName(const std::string &name);
  
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

    /// \brief Return the ID of the parent
    /// \return Integer ID
    public: int GetParentId() const;
  
    /// \brief Set the parent
    /// \param parent Parent entity
    public: void SetParent(Common *parent);
  
    /// \brief Get the parent
    /// \return Pointer to the parent entity
    public: Common *GetParent() const;

    /// \brief Add a child to this entity
    /// \param child Child entity
    public: void AddChild(Common *child);

    /// \brief Remove a child from this entity
    /// \param child Child to remove
    public: virtual void RemoveChild(Common *child);
 
    /// \brief Get the number of children
    public: unsigned int GetChildCount() const;

    /// \brief Get by name 
    public: Common *GetByName(const std::string &name);

    /// \brief Get a child by index
    public: Common *GetChild(unsigned int i) const;

    /// \brief Get a child by name
    public: Common *GetChild(const std::string &name );

    /// \brief Add a type specifier
    public: void AddType( EntityType type );

    /// \brief Get the type
    public: bool HasType(const EntityType &t) const;

    /// \brief Get the number of types
    public: unsigned int GetTypeCount() const;

    /// \brief Get a type by index
    public: EntityType GetType(unsigned int index) const;

    /// \brief Get the leaf type (last type set)
    public: EntityType GetLeafType() const;

    /// \brief Return the name of this entity with the model scope
    ///        model1::...::modelN::entityName
    public: std::string GetScopedName() const;

    /// \brief Return the name of this entity with the model+body+geom scope
    ///        model1::...::modelN::bodyN::entityName
    public: std::string GetCompleteScopedName() const;

    public: void Print(std::string prefix);

    /// \brief True == show parameters in the gui
    public: bool GetShowInGui() const;

    /// \brief True == show parameters in the gui
    public: void SetShowInGui(bool v);

    /// \brief Set whether this entity has been selected by the user through 
    //         the gui
    public: virtual bool SetSelected( bool s );
  
    /// \brief True if the entity is selected by the user
    public: bool IsSelected() const;

    /// \brief Returns true if the entities are the same. Checks only the name
    public: bool operator==(const Common &ent) const;

    /// \brief Parent of this entity
    protected: Common *parent;

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
 
    /// \brief Children of this entity
    protected: std::vector< Common* > children;
 
    private: std::vector< EntityType > type;

    private: bool selected;

    private: bool showInGui;
  };
}

}
#endif
