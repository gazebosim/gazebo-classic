/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 *
 * gzserver -l libPhysicsPluginExample.so -e plugin
 *
*/
#include <stdio.h>
#include <stdlib.h>
#include "gazebo/physics/PhysicsPlugin.h"
#include <sdf.hh>

PhysicsPlugin *g_plugin = NULL;

/************************************************/
int load_sdf(sdf::ElementPtr _sdf)
{
  /* Functions required to initialize the model upon load completion. */
  printf("Initialize models\n");

  return 0;
}

/************************************************/
int init_physics(void)
{
  /* Add in any functions required to initialize the physics engine. */
  printf("Initialize the engine\n");

  // what in physics engine needs to be initialized?

  return 0;
}

/************************************************/
int update_collision(void)
{
  /* Add in any functions required to update collision engine. */
  printf("Check collisions\n");

  return 0;
}

/************************************************/
int update_constraints(void)
{
  /* update constraints for collision or custom constraints. */
  printf("update constraints\n");

  return 0;
}

/************************************************/
int update_physics(void)
{
  /* Add in any functions required to update/step physics engine. */
  printf("Update/step physcis engine\n");

  // get data from gazebo (e.g. gui commands, sensor data, other commands?)

  // send data back to gazebo

  return 0;
}

/************************************************/
int set_state(const gazebo::physics::WorldState &_state)
{
  /* Add in any functions required to update/step physics engine. */
  printf("get state\n");

  // get data from gazebo (e.g. gui commands, sensor data, other commands?)

  // send data back to gazebo

  return 0;
}

/************************************************/
int get_state(gazebo::physics::WorldState &_state)
{
  /* Add in any functions required to update/step physics engine. */
  printf("get state\n");

  // get data from gazebo (e.g. gui commands, sensor data, other commands?)

  // send data back to gazebo

  return 0;
}

/************************************************/
int destroy_physics(void)
{
  /* Add in any functions required to destroy the physics engine. */
  printf("Destroy the engine\n");
  free(g_plugin);

  return 0;
}

/************************************************/
PhysicsPlugin *create()
{
  // To run this example, try
  //
  // gazebo -l libPhysicsPluginExample.so

  // create a pointer to physics plugin
  g_plugin = (PhysicsPlugin*)malloc(sizeof(PhysicsPlugin));

  // load sdf
  g_plugin->loadSDF = load_sdf;

  // link initPhysics
  g_plugin->initPhysics = init_physics;

  // set state, can call during load or at runtime
  g_plugin->setState = set_state;

  // update collision
  g_plugin->updateCollision = update_collision;

  // update constraints
  g_plugin->updateConstraints = update_constraints;

  // update physics, or effectively step physics
  g_plugin->updatePhysics = update_physics;

  // get state, can call during load or at runtime
  g_plugin->getState = get_state;

  // link destroyPhysics to destroy_physics, this function is called
  // at the beginning of World::Fini
  g_plugin->destroyPhysics = destroy_physics;

  /* Add in any functions required to create the physics engine. */

  return g_plugin;
}
