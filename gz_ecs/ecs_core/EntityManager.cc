#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/ecs_core/EntityManagerPrivate.hh"
#include "gazebo/ecs_core/EntityQueryResultPrivate.hh"


namespace gazebo
{
namespace ecs_core
{

  EntityManager::EntityManager()
{
  this->impl.reset(new EntityManagerPrivate);
}

EntityManager::~EntityManager()
{
}

Entity EntityManager::CreateEntity()
{
  // TODO Reuse ids, This will run out of entities eventually
  Entity e = (this->impl->nextEntity)++;
  this->impl->entities[e];
  return e;
}

bool EntityManager::AddQuery(const EntityQuery &_query)
{
  auto iter = this->impl->queries.begin();
  auto iterEnd = this->impl->queries.end();
  for (; iter != iterEnd; ++iter)
  {
    if (_query == *iter)
    {
      // Already have this query, bail
      return false;
    }
  }
  this->impl->queries.push_back(_query);
}

void EntityManager::RemoveQuery(const EntityQuery &_query)
{
  auto iter = this->impl->queries.begin();
  auto iterEnd = this->impl->queries.end();
  for (; iter != iterEnd; ++iter)
  {
    if (_query == *iter)
    {
      this->impl->queries.erase(iter);
      break;
    }
  }
}

void* EntityManager::AddComponent(Entity _id, ComponentType _hash, std::size_t _size)
{
  void *ret = nullptr;
  // Make some space to store the components
  if (this->GetComponent(_id, _hash, _size) == nullptr)
  {
    char *comp = new char[_size];
    this->impl->entities[_id][_hash].reset(comp);
    ret = static_cast<void*>(comp);
  }

  return ret;
}

void* EntityManager::GetComponent(Entity _id, ComponentType _hash, std::size_t _size)
{
  // TODO map.find() better than count() here
  void *ret = nullptr;
  if (this->impl->entities.count(_id))
  {
    auto componentsOfOneEntity = this->impl->entities[_id];
    if (componentsOfOneEntity.count(_hash))
    {
      char *component = &*componentsOfOneEntity[_hash];
      ret = static_cast<void*>(component);
    }
  }
  return ret;
}

void EntityManager::Update()
{
  // Update EntityQuery results
  auto queryIter = this->impl->queries.begin();
  auto queryIterEnd = this->impl->queries.end();
  for (; queryIter != queryIterEnd; ++queryIter)
  {
    std::vector<Entity> &results = queryIter->Results()->impl->results;
    std::vector<ComponentType> components = queryIter->ComponentTypes();

    // Check every entity
    // TODO this could be faster if which entities had components added
    // or removed was tracked somehow, then the queries would only be
    // updated with those entities that changed
    auto entityIter = this->impl->entities.begin();
    auto entityIterEnd = this->impl->entities.end();
    for (; entityIter != entityIterEnd; ++entityIter)
    {
      auto componentIter = components.begin();
      auto componentIterEnd = components.end();
      bool foundAll = true;
      for (; componentIter != componentIterEnd; ++componentIter)
      {
        if (entityIter->second.find(*componentIter) == entityIter->second.end())
        {
          // Nope, this entity is missing a component
          foundAll = false;
          break;
        }
      }
      if (foundAll)
      {
        results.push_back(entityIter->first);
      }
    }
  }
}

}
}

