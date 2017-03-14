#include "gazebo/ecs_core/System.hh"
#include "gazebo/ecs_core/SystemPrivate.hh"

namespace gazebo
{
namespace ecs_core
{

System::System()
{
  // The SystemManager is a friend who will populate this structure
  this->impl.reset(new SystemPrivate());
}

System::~System()
{
}

SystemManager* System::GetSystemManager() const
{
  return this->impl->sm;
}

EntityManager* System::GetEntityManager() const
{
  return this->impl->em;
}

}
}
