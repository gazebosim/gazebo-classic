#include <Ogre.h>
#include "Global.hh"
#include "OgreAdaptor.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "Entity.hh"

unsigned int Entity::idCounter = 0;


Entity::Entity(Entity *parent)
{
  // Set the parent and the id
  this->parent = parent;
  this->id = idCounter++;
  this->isStatic = false;

  if (this->parent)
  {
    this->parent->AddChild(this);
  }
  else
  {
    this->sceneNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode();
  }


  // Add this to the phyic's engine
  World::Instance()->GetPhysicsEngine()->AddEntity(this);
}

Entity::~Entity()
{
}

int Entity::GetId() const
{
  return this->id;
}

// Return the ID of the parent
int Entity::GetParentId() const
{
  return this->parent == NULL ? 0 : this->parent->GetId();
}


// Set the parent
void Entity::SetParent(Entity* parent)
{
  this->parent = parent;
}

// Get the parent
Entity *Entity::GetParent() const
{
  return this->parent;
}

// Add a child to this entity
void Entity::AddChild(Entity *child)
{
  // Set the child's parent
  child->SetParent(this);

  // The the child's scene node
  child->SetSceneNode(this->sceneNode->createChildSceneNode());

  child->SetStatic(this->IsStatic());

  // Add this child to our list
  this->children.push_back(child);
}

// Get all children
std::vector<Entity*> Entity::GetChildren()
{
  return this->children;
}

// Return this entitie's sceneNode
Ogre::SceneNode *Entity::GetSceneNode() const
{
  return this->sceneNode;
}

// Set the scene node
void Entity::SetSceneNode(Ogre::SceneNode *sceneNode)
{
  this->sceneNode = sceneNode;
}

////////////////////////////////////////////////////////////////////////////////
// Set the name of the body
void Entity::SetName(const std::string &name)
{
  this->name = name;
}

////////////////////////////////////////////////////////////////////////////////
// Return the name of the body
std::string Entity::GetName() const
{
  return this->name;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this entity is static: immovable
void Entity::SetStatic(bool s)
{
  this->isStatic = s;
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this entity is static
bool Entity::IsStatic() const
{
  return this->isStatic;
}

