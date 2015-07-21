/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_PHYSICS_BASE_HH_
#define _GAZEBO_PHYSICS_BASE_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/enable_shared_from_this.hpp>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief namespace for physics
  namespace physics
  {
    /// \addtogroup gazebo_physics Classes for physics and dynamics
    /// \{

    /// \brief String names for the different entity types.
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
      "trimesh",
      "polyline"
    };

    /// \class Base Base.hh physics/physics.hh
    /// \brief Base class for most physics classes
    class GZ_PHYSICS_VISIBLE Base : public boost::enable_shared_from_this<Base>
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
                /// \brief GearboxJoint type
                GEARBOX_JOINT   = 0x00002000,
                /// \brief FixedJoint type
                FIXED_JOINT     = 0x00004000,

                /// \brief Shape type
                SHAPE           = 0x00010000,
                /// \brief BoxShape type
                BOX_SHAPE       = 0x00020000,
                /// \brief CylinderShape type
                CYLINDER_SHAPE  = 0x00040000,
                /// \brief HeightmapShape type
                HEIGHTMAP_SHAPE = 0x00080000,
                /// \brief MapShape type
                MAP_SHAPE       = 0x00100000,
                /// \brief MultiRayShape type
                MULTIRAY_SHAPE  = 0x00200000,
                /// \brief RayShape type
                RAY_SHAPE       = 0x00400000,
                /// \brief PlaneShape type
                PLANE_SHAPE     = 0x00800000,
                /// \brief SphereShape type
                SPHERE_SHAPE    = 0x01000000,
                /// \brief MeshShape type
                MESH_SHAPE      = 0x02000000,
                /// \brief PolylineShape type
                POLYLINE_SHAPE  = 0x04000000,

                /// \brief Indicates a collision shape used for sensing
                SENSOR_COLLISION = 0x10000000
              };

      /// \brief Constructor
      /// \param[in] _parent Parent of this object
      public: explicit Base(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Base();

      /// \brief Load
      /// \param[in] node Pointer to an SDF parameters
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Finialize the object
      public: virtual void Fini();

      /// \brief Initialize the object
      public: virtual void Init() {}

      /// \brief Reset the object
      public: virtual void Reset();

      /// \brief Calls recursive Reset on one of the Base::EntityType's
      /// \param[in] _resetType The type of reset operation
      public: virtual void Reset(Base::EntityType _resetType);

      /// \brief Update the object
      public: virtual void Update() {}

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf Update the object's parameters based on SDF
      /// values.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the name of the entity.
      /// \param[in] _name New name.
      public: virtual void SetName(const std::string &_name);

      /// \brief Return the name of the entity.
      /// \return Name of the entity.
      public: std::string GetName() const;

      /// \brief Return the ID of this entity. This id is unique.
      /// \return Integer ID.
      public: uint32_t GetId() const;

      /// \brief Set whether the object should be "saved", when the user
      /// selects to save the world to xml
      /// \param[in] _v Set to True if the object should be saved.
      public: void SetSaveable(bool _v);

      /// \brief Get whether the object should be "saved", when the user
      /// selects to save the world to xml.
      /// \return True if the object is saveable.
      public: bool GetSaveable() const;

      /// \brief Return the ID of the parent.
      /// \return Integer ID.
      public: int GetParentId() const;

      /// \brief Set the parent.
      /// \param[in] _parent Parent object.
      public: void SetParent(BasePtr _parent);

      /// \brief Get the parent.
      /// \return Pointer to the parent entity.
      public: BasePtr GetParent() const;

      /// \brief Add a child to this entity.
      /// \param[in] _child Child entity.
      public: void AddChild(BasePtr _child);

      /// \brief Remove a child from this entity.
      /// \param[in] _id ID of the child to remove.
      public: virtual void RemoveChild(unsigned int _id);

      /// \brief Remove all children.
      public: void RemoveChildren();

      /// \brief Get the number of children.
      /// \return The number of children.
      public: unsigned int GetChildCount() const;

      /// \cond
      /// This is an internal function.
      /// \brief Get a child or self by id.
      /// \param[in] _id ID of the object to retreive.
      /// \return A pointer to the object, NULL if not found
      public: BasePtr GetById(unsigned int _id) const;
      /// \endcond

      /// \brief Get by name.
      /// \param[in] _name Get a child (or self) object by name
      /// \return A pointer to the object, NULL if not found
      public: BasePtr GetByName(const std::string &_name);

      /// \brief Get a child by index.
      /// \param[in] _i Index of the child to retreive.
      /// \return A pointer to the object, NULL if the index is invalid.
      public: BasePtr GetChild(unsigned int _i) const;

      /// \brief Get a child by name.
      /// \param[in] _name Name of the child.
      /// \return A pointer to the object, NULL if not found
      public: BasePtr GetChild(const std::string &_name);

      /// \brief Remove a child by name.
      /// \param[in] _name Name of the child.
      public: void RemoveChild(const std::string &_name);

      /// \brief Add a type specifier.
      /// \param[in] _type New type to append to this objects type
      /// definition.
      public: void AddType(EntityType _type);

      /// \brief Returns true if this object's type definition has the
      /// given type.
      /// \param[in] _t Type to check.
      /// \return True if this object's type definition has the.
      public: bool HasType(const EntityType &_t) const;

      /// \brief Get the full type definition.
      /// \return The full type definition.
      public: unsigned int GetType() const;

      /// \brief Return the name of this entity with the model scope
      /// model1::...::modelN::entityName
      /// \param[in] _prependWorldName True to prended the returned string
      /// with the world name. The result will be
      /// world::model1::...::modelN::entityName.
      /// \return The scoped name.
      public: std::string GetScopedName(bool _prependWorldName = false) const;

      /// \brief Print this object to screen via gzmsg.
      /// \param[in] _prefix Usually a set of spaces.
      public: void Print(const std::string &_prefix);

      /// \brief Set whether this entity has been selected by the user through
      /// the gui.
      /// \param[in] _show True to set this entity as selected.
      public: virtual bool SetSelected(bool _show);

      /// \brief True if the entity is selected by the user.
      /// \return True if the entity is selected.
      public: bool IsSelected() const;

      /// \brief Returns true if the entities are the same.
      /// Checks only the name.
      /// \param[in] _ent Base object to compare with.
      /// \return True if the entities are the same.
      public: bool operator ==(const Base &_ent) const;

      /// \brief Set the world this object belongs to. This will also
      /// set the world for all children.
      /// \param[in] _newWorld The new World this object is part of.
      public: void SetWorld(const WorldPtr &_newWorld);

      /// \brief Get the World this object is in.
      /// \return The World this object is part of.
      public: const WorldPtr &GetWorld() const;

      /// \brief Get the SDF values for the object.
      /// \return The SDF values for the object.
      public: virtual const sdf::ElementPtr GetSDF();

      /// \brief Compute the scoped name of this object based on its
      /// parents.
      /// \sa Base::GetScopedName
      protected: void ComputeScopedName();

      /// \brief The SDF values for this object.
      protected: sdf::ElementPtr sdf;

      /// \brief Parent of this entity.
      protected: BasePtr parent;

      /// \brief Children of this entity.
      protected: Base_V children;

      /// \brief Pointer to the world.
      protected: WorldPtr world;

      /// \brief Set to true if the object should be saved.
      private: bool saveable;

      /// \brief This entities ID.
      private: uint32_t id;

      /// \brief The type of this object.
      private: unsigned int type;

      /// \brief True if selected.
      private: bool selected;

      /// \brief Local copy of the sdf name.
      private: std::string name;

      /// \brief Local copy of the scoped name.
      private: std::string scopedName;

      protected: friend class Entity;
    };
    /// \}
  }
}
#endif
