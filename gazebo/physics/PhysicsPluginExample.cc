#include "gazebo/physics/PhysicsPlugin.hh"

void hello()
{
  printf("Plugin implementation\n");
}

PhysicsPlugin *create_engine()
{
  PhysicsPlugin *engine = new PhyiscsPlugin;
  engine->hello = &hello();
}
