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
#ifndef _GAZEBO_PHYSICS_PLUGIN_H_
#define _GAZEBO_PHYSICS_PLUGIN_H_

// #include <gazebo/physics/physics.hh>
#include <gazebo/physics/WorldState.hh>
#include <sdf/sdf.hh>

#ifdef __cplusplus
extern "C" {
#endif

/* Basic functions needed to use the C-API for loading a physics engine
 *
 *  * - optional function
 *
 *  1. Load World
 *    - LoadSDF - load world / models sdf
 *    - InitPhysics* - custom initialization for physics engine
 *    - SetState* - custom call to set state of the world/models
 *  2. Simulation Update
 *    - SetState* - custom call to set state of the world/models
 *    - UpdateCollision
 *    - UpdateConstraints - for contacts or anything else
 *    - UpdatePhysics
 *    - GetState - get model states from physics engine, update Gazebo
 * 10. DestroyPhysics
 */

/* \brief LoadModel function pointer.
 * This function is called after the physics has been initialized (InitPhysics).
 * Use this function to instantiate or set the model in the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*LoadSDFFnPtr)(sdf::ElementPtr _sdf);

/* \brief InitPhysics function pointer.
 * This function is called after the plugin has been created. Use this
 * function to initialize the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*InitPhysicsFnPtr)(void);

/* \brief SetState function pointer.
 * This function is called after the model has been loaded (LoadModel).
 * Use this function to instantiate or set the state for the model.
 * \return 0 on success, -1 on error.
 */
typedef int (*SetStateFnPtr)(const gazebo::physics::WorldState &_state);

/* \brief UpdateConstraints function pointer.
 * This function is called after 
 * Use this function to 
 * \return 0 on success, -1 on error.
 */
typedef int (*UpdateCollisionFnPtr)(void);

/* \brief UpdateConstraints function pointer.
 * This function is called after 
 * Use this function to 
 * \return 0 on success, -1 on error.
 */
typedef int (*UpdateConstraintsFnPtr)(void);

/* \brief UpdatePhysics function pointer.
 * This function is called after 
 * Use this function to 
 * \return 0 on success, -1 on error.
 */
typedef int (*UpdatePhysicsFnPtr)(void);

/* \brief GetState function pointer.
 * This function is called after the model has been loaded (LoadModel).
 * Use this function to get the state for the model.
 * \return 0 on success, -1 on error.
 */
typedef int (*GetStateFnPtr)(gazebo::physics::WorldState &_state);

/* \brief DestroyPhysics function pointer.
 * This function is called when the plugin is deleted. Use this function
 * to destroy physics and cleanup the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*DestroyPhysicsFnPtr)(void);

struct _Quaternion
{
  double w;
  double x;
  double y;
  double z;
};

struct _Vector3
{
  double x;
  double y;
  double z;
};

struct _Pose
{
  struct _Vector3 pos;
  struct _Quaternion rot;
};

/* \brief The physics plugin structure
 * This structure defines all the functions necessary to create and control
 * a physics engine.
 */
struct _PhysicsPlugin
{
  /*
   *
   */
  LoadSDFFnPtr loadSDF;

  /* \brief Pointer to the initialize function.
   * This function must be implemented by a physics plugin.
   */
  InitPhysicsFnPtr initPhysics;

  /*
   *
   */
  SetStateFnPtr setState;

  /*
   *
   */
  UpdateCollisionFnPtr updateCollision;

  /*
   *
   */
  UpdateConstraintsFnPtr updateConstraints;

  /*
   *
   */
  UpdatePhysicsFnPtr updatePhysics;

  /*
   *
   */
  GetStateFnPtr getState;

  /* \brief Pointer to the destroy physics function.
   * This function must be implemented by a physics plugin.
   */
  DestroyPhysicsFnPtr destroyPhysics;

  /* \brief data structure holding model information
   * that needs to be passed between gazebo and plugin.
   */
  int lotOfModelData;

  /* \brief data structure holding state information
   * that needs to be passed between gazebo and plugin.
   */
  struct _Pose *pose;
};
typedef struct _PhysicsPlugin PhysicsPlugin;

/* \brief Create the physics engine.
 * This function is called when the plugin is instantiated.
 * \return Pointer to the physics plugin. NULL on error. The plugin will
 * maintain ownership of the PhysicsPlugin pointer. The calling application
 * should not delete/free this pointer.
 */
PhysicsPlugin *create(void);

#ifdef __cplusplus
}
#endif

#endif
