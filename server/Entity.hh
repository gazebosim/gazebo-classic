#ifndef ENTITY_HH
#define ENTITY_HH

#include <vector>
#include <string>
#include <ode/ode.h>

namespace Ogre
{
  class SceneNode;
}

/// Base class for all objects in Gazebo
/*
 * Facilitates meshing of physics engine with rendering engine
 */
class Entity
{
  /// Constructor
  /// \param parent Parent of the entity.
  public: Entity(Entity *parent=NULL);

   /// Destructor
  public: virtual ~Entity();

  /// Return the ID of this entity. This id is unique
  /// \return Integer ID
  public: int GetId() const;

  /// Return the ID of the parent
  /// \return Integer ID
  public: int GetParentId() const;

  /// Set the parent
  /// \param parent Parent entity
  public: void SetParent(Entity* parent);

  /// Get the parent
  /// \return Pointer to the parent entity
  public: Entity *GetParent() const;

  /// Add a child to this entity
  /// \param child Child entity
  public: void AddChild(Entity *child);

  /// Get all children
  /// \return Vector of children entities
  public: std::vector<Entity*> GetChildren();

  /// Return this entity's sceneNode
  /// \return Ogre scene node
  public: Ogre::SceneNode *GetSceneNode() const;

  /// Set the scene node
  /// \param sceneNode Ogre scene node
  public: void SetSceneNode(Ogre::SceneNode *sceneNode);

  /// Set the name of the body
  /// \param name Body name
  public: void SetName(const std::string &name);

  /// Return the name of the body
  /// \return Name of the body
  public: std::string GetName() const;

  /// Set whether this entity is static: immovable
  /// \param s Bool, true = static
  public: void SetStatic(bool s);

  /// Return whether this entity is static
  /// \return bool True = static
  public: bool IsStatic() const;

  /// Parent of this entity
  private: Entity *parent;

  /// Children of this entity
  private: std::vector<Entity*> children;

  /// This entities ID
  private: unsigned int id;

  /// Used to automaticaly chose a unique ID on creation
  private: static unsigned int idCounter;

  /// OGRE stuff
  protected: Ogre::SceneNode *sceneNode;

  /// ODE Stuff
  public: dSpaceID spaceId;

  /// Name of the entity
  private: std::string name;

  private: bool isStatic;
};

#endif
