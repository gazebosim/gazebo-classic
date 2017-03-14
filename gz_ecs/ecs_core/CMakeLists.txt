set(sources
  EntityManager.cc
  EntityQuery.cc
  EntityQueryResult.cc
  SystemManager.cc
  System.cc
)

add_library(GazeboECSCore SHARED ${sources})
