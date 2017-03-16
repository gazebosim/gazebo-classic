#include <iostream>
#include "gazebo/components/Triplet.hh"
#include "gazebo/ecs_core/Entity.hh"
#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/EntityQueryResult.hh"
#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/plugin/RegisterMacros.hh"
#include "gazebo/private/systems/AddAndPrintResult.hh"

namespace gazebo
{
namespace systems
{

void AddAndPrintResult::Init(ecs_core::EntityQuery &_query)
{
  // Add components which are required
  _query.AddComponent<components::Triplet>();
}

void AddAndPrintResult::Update(
    double _dt, const ecs_core::EntityQueryResult &_result)
{
  // Loop through all of the entities which have the required components
  auto em = this->GetEntityManager();
  for (int i = 0; i < _result.NumResults(); i++)
  {
    ecs_core::Entity entity = _result.At(i);
    auto numbers = em->GetComponent<components::Triplet>(entity);

    std::cout << "Adding " << entity << ":" << 
      numbers->first + numbers->second + numbers->third << std::endl;
  }
}

}
}

GZ_REGISTER_SINGLE_PLUGIN(gazebo::systems::AddAndPrintResult,
                          gazebo::ecs_core::System)
