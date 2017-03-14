#include <iostream>
#include "gazebo/components/Fraction.hh"
#include "gazebo/ecs_core/Entity.hh"
#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/EntityQueryResult.hh"
#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/private/systems/DivideAndPrintResult.hh"

namespace gazebo
{
namespace systems
{

void DivideAndPrintResult::Init(ecs_core::EntityQuery &_query)
{
  // First things first, tell the system manager what components
  // this system requires. This sytem only requires the Fraction component
  _query.AddComponent<components::Fraction>();
}

void DivideAndPrintResult::Update(
    double _dt, const ecs_core::EntityQueryResult &_result)
{
  // Loop through all of the entities which have the required components
  auto em = this->GetEntityManager();
  for (int i = 0; i < _result.NumResults(); i++)
  {
    ecs_core::Entity entity = _result.At(i);
    auto fraction = em->GetComponent<components::Fraction>(entity);

    std::cout << entity << ":" << fraction->numerator / fraction->denominator << std::endl;
  }
}

}
}
