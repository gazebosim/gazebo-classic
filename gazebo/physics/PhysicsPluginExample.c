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
#include <stdio.h>
#include <stdlib.h>
#include "gazebo/physics/PhysicsPlugin.h"

PhysicsPlugin *g_plugin = NULL;

/************************************************/
int example_init(void)
{
  /* Add in any functions required to initialize the physics engine. */
  printf("Initialize the engine\n");

  return 0;
}

/************************************************/
int example_destroy(void)
{
  /* Add in any functions required to destroy the physics engine. */
  printf("Destroy the engine\n");
  free(g_plugin);

  return 0;
}

/************************************************/
PhysicsPlugin *create()
{
  g_plugin = (PhysicsPlugin*)malloc(sizeof(PhysicsPlugin));
  g_plugin->init = example_init;
  g_plugin->destroy = example_destroy;

  /* Add in any functions required to create the physics engine. */

  return g_plugin;
}
