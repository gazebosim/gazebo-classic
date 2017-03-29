set(sources
  Entity.cc
  EntityQuery.cc
  Component.cc
  ComponentFactory.cc
  Manager.cc
  System.cc
)

add_library(GazeboECSCore SHARED ${sources})
