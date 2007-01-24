#ifndef ENTITY_HH
#define ENTITY_HH

#include <vector>
#include <string>
#include <ode/ode.h>

namespace Ogre
{
  class SceneNode;
}

class Entity
{
  public: Entity(Entity *parent=NULL);
  public: virtual ~Entity();

  // Return the ID of this entity. This id is unique
  public: int GetId() const;

  // Return the ID of the parent
  public: int GetParentId() const;

  // Set the parent
  public: void SetParent(Entity* parent);

  // Get the parent
  public: Entity *GetParent() const;

  // Add a child to this entity
  public: void AddChild(Entity *child);

  // Get all children
  public: std::vector<Entity*> GetChildren();

  // Return this entitie's sceneNode
  public: Ogre::SceneNode *GetSceneNode() const;

  // Set the scene node
  public: void SetSceneNode(Ogre::SceneNode *sceneNode);

  //! Set the name of the body
  /*!
   * \param name Body name
   */
  public: void SetName(const std::string &name);

  //! Return the name of the body
  /*!
   * \return Name of the body
   */
  public: std::string GetName() const;

  //! Set whether this entity is static: immovable
  /*!
   * \param s Bool, true = static
   */
  public: void SetStatic(bool s);

  //! Return whether this entity is static
  /*!
   * \return bool True = static
   */
  public: bool IsStatic() const;

  // Parent of this entity
  private: Entity *parent;

   // Children of this entity
  private: std::vector<Entity*> children;

  private: unsigned int id;
  private: static unsigned int idCounter;

  // OGRE stuff
  protected: Ogre::SceneNode *sceneNode;

  // ODE Stuff
  public: dSpaceID spaceId;

  // Name of the entity
  private: std::string name;

  private: bool isStatic;
};
#endif
