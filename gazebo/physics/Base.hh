/*
 * Copyright 2011 Nate Koenig
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
 */

#ifndef GAZEBO_PHYSICS_BASE_HH
#define GAZEBO_PHYSICS_BASE_HH

#include <boost/enable_shared_from_this.hpp>

#include <string>

#include "sdf/sdf.hh"
#include "common/CommonTypes.hh"
#include "physics/PhysicsTypes.hh"

namespace gazebo
{
  /// \brief namespace for physics
  namespace physics
  {
    /// \addtogroup gazebo_physics Classes for physics and dynamics
    /// \{
    static std::string EntityTypename[] = {
      "common",
      "entity",
      "model",
      "actor",
      "link",
      "collision",
      "light",
      "visual",
      "joint",
      "ball",
      "hinge2",
      "hinge",
      "slider",
      "universal",
      "shape",
      "box",
      "cylinder",
      "heightmap",
      "map",
      "multiray",
      "ray",
      "plane",
      "sphere",
      "trimesh"
    };

    /// \class Base Base.hh physics/Base.hh
    /// \brief Base class for most physics classes
    class Base : public boost::enable_shared_from_this<Base>
    {
      /// \enum EntityType
      /// \brief Unique identifiers for all entity types.
      public: enum EntityType {
                /// \brief Base type
                BASE            = 0x00000000,
                /// \brief Entity type
                ENTITY          = 0x00000001,
                /// \brief Model type
                MODEL           = 0x00000002,
                /// \brief Link type
                LINK            = 0x00000004,
                /// \brief Collision type
                COLLISION       = 0x00000008,
                /// \brief Actor type
                ACTOR           = 0x00000016,
                /// \brief Light type
                LIGHT           = 0x00000010,
                /// \brief Visual type
                VISUAL          = 0x00000020,

                /// \brief Joint type
                JOINT           = 0x00000040,
                /// \brief BallJoint type
                BALL_JOINT      = 0x00000080,
                /// \brief Hing2Joint type
                HINGE2_JOINT    = 0x00000100,
                /// \brief HingeJoint type
                HINGE_JOINT     = 0x00000200,
                /// \brief SliderJoint type
                SLIDER_JOINT    = 0x00000400,
                /// \brief ScrewJoint type
                SCREW_JOINT     = 0x00000800,
                /// \brief UniversalJoint type
                UNIVERSAL_JOINT = 0x00001000,

                /// \brief Shape type
                SHAPE           = 0x00002000,
                /// \brief BoxShape type
                BOX_SHAPE       = 0x00004000,
                /// \brief CylinderShape type
                CYLINDER_SHAPE  = 0x00008000,
                /// \brief HeightmapShape type
                HEIGHTMAP_SHAPE = 0x00010000,
                /// \brief MapShape type
                MAP_SHAPE       = 0x00020000,
                /// \brief MultiRayShape type
                MULTIRAY_SHAPE  = 0x00040000,
                /// \brief RayShape type
                RAY_SHAPE       = 0x00080000,
                /// \brief PlaneShape type
                PLANE_SHAPE     = 0x00100000,
                /// \brief SphereShape type
                SPHERE_SHAPE    = 0x00200000,
                /// \brief TrimeshShape type
                TRIMESH_SHAPE   = 0x00400000
              };

      /// \brief Constructor
      /// \param parent Parent of this object
      public: Base(BasePtr parent);

      /// \brief Destructor
      public: virtual ~Base();

      /// \brief Load
      /// \param node Pointer to an SDF parameters
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Finialize the object
      public: virtual void Fini();

      /// \brief virtual
      public: virtual void Init() {}

      /// \brief empty
      public: virtual void Reset();

      /// \brief calls recursive Reset on one of the Base::EntityType's
      public: virtual void Reset(Base::EntityType _resetType);

      /// \brief empty
      public: virtual void Update() {}

      /// \brief Update the parameters using new sdf values
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the name of the entity
      /// \param name Link name
      public: virtual void SetName(const std::string &name);

      /// \brief Return the name of the entity
      /// \return Name of the entity
      public: std::string GetName() const;

      /// \brief Return the ID of this entity. This id is unique
      /// \return Integer ID
      public: unsigned int GetId() const;

      /// \brief Set whether the object should be "saved", when the user
      ///        selects to save the world to xml
      /// \param v Set to True if the object should be saved.
      public: void SetSaveable(bool v);

      /// \brief Get whether the object should be "saved", when the user
      ///        selects to save the world to xml
      /// \return True if the object is saveable
      public: bool GetSaveable() const;

      /// \brief Return the ID of the parent
      /// \return Integer ID
      public: int GetParentId() const;

      /// \brief Set the parent
      /// \param parent Parent object
      public: void SetParent(BasePtr parent);

      /// \brief Get the parent
      /// \return Pointer to the parent entity
      public: BasePtr GetParent() const;

      /// \brief Add a child to this entity
      /// \param child Child entity
      public: void AddChild(BasePtr child);

      /// \brief Remove a child from this entity
      /// \param child Child to remove
      public: virtual void RemoveChild(unsigned int id);

      /// \brief Remove all children
      public: void RemoveChildren();

      /// \brief Get the number of children
      /// \return The number of children
      public: unsigned int GetChildCount() const;

      /// \brief Get by id
      public: BasePtr GetById(unsigned int _id) const;

      /// \brief Get by name
      /// \param name Get a child (or self) object by name
      /// \return A pointer to the object, NULL if not found
      public: BasePtr GetByName(const std::string &name);

      /// \brief Get a child by index
      public: BasePtr GetChild(unsigned int i) const;

      /// \brief Get a child by name
      public: BasePtr GetChild(const std::string &name);

      /// \brief Remove a child by name
      public: void RemoveChild(const std::string &_name);

      /// \brief Add a type specifier
      public: void AddType(EntityType _type);

      /// \brief Get the type
      public: bool HasType(const EntityType &_t) const;

      /// \brief Get a type by index
      public: unsigned int GetType() const;

      /// \brief Return the name of this entity with the model scope
      ///        world::model1::...::modelN::entityName
      public: std::string GetScopedName() const;

      /// \brief Return the name of this entity with the
      ///        model+body+collision scope
      ///        model1::...::modelN::bodyN::entityName
      public: std::string GetCompleteScopedName() const GAZEBO_DEPRECATED;

      /// \brief print this object to screen via gzmsg
      public: void Print(std::string prefix);

      /// \brief True == show parameters in the gui
      public: bool GetShowInGui() const;

      /// \brief True == show parameters in the gui
      public: void SetShowInGui(bool v);

      /// \brief Set whether this entity has been selected by the user through
      //         the gui
      public: virtual bool SetSelected(bool s);

      /// \brief True if the entity is selected by the user
      public: bool IsSelected() const;

      /// \brief Returns true if the entities are the same. Checks only the name
      public: bool operator ==(const Base &ent) const;

      /// \brief Set the world this object belongs to. This will also
      ///        set the world for all children
      public: void SetWorld(const WorldPtr &_newWorld);

      /// \brief Get the world this object is in
      public: const WorldPtr &GetWorld() const;

      /// \brief Get the SDF values for the model
      public: virtual const sdf::ElementPtr GetSDF();

      protected: sdf::ElementPtr sdf;

      /// \brief Parent of this entity
      protected: BasePtr parent;

      /// \brief Children of this entity
      protected: Base_V children;
      protected: Base_V::iterator childrenEnd;

      protected: WorldPtr world;

      /// \brief Set to true if the object should be saved.
      private: bool saveable;

      /// \brief This entities ID
      private: unsigned int id;

      /// \brief Used to automaticaly chose a unique ID on creation
      private: static unsigned int idCounter;

      private: unsigned int type;

      private: bool selected;
    };
    /// \}
  }
}
#endif

