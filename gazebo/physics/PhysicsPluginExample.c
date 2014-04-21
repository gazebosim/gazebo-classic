#include <stdio.h>
#include <stdlib.h>
#include "gazebo/physics/PhysicsPlugin.h"

void app_hello(void)
{
  printf("Plugin implementation\n");
}

PhysicsPlugin *create_engine()
{
  printf("create_engine");

  PhysicsPlugin *engine = (PhysicsPlugin*)malloc(sizeof(PhysicsPlugin));
  engine->hello = app_hello;
  return engine;
}
