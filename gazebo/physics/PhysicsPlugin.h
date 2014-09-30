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

#ifdef __cplusplus
extern "C" {
#endif

/* Basic functions needed to use the C-API for loading a physics engine
 *
 *  0. Gazebo should have loaded params via sdf.
 *  1. InitPhysics - initialize physics engine
 *  2. [LoadModel] - initialize model(s)
 *  3. [SetState] - initialize states(s)
 *  4. Loop below if necessary:
 *  5.     UpdatePhysics - advance in time
 *  6.     GetState - get model states from physics engine, update Gazebo
 *  7.     [SetPhysicsParams] - modify physics solver
 *  8.     [LoadModel] - modify model
 *  9.     [SetState] - modify state, set force
 * 10. DestroyPhysics
 */

/* \brief InitPhysics function pointer.
 * This function is called after the plugin has been created. Use this
 * function to initialize the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*InitPhysicsFnPtr)(void);

/* \brief LoadModel function pointer.
 * This function is called after the physics has been initialized (InitPhysics).
 * Use this function to instantiate or set the model in the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*LoadModelFnPtr)(void);

/* \brief SetState function pointer.
 * This function is called after the model has been loaded (LoadModel).
 * Use this function to instantiate or set the state for the model.
 * \return 0 on success, -1 on error.
 */
typedef int (*SetStateFnPtr)(void);

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
typedef int (*GetStateFnPtr)(void);

/* \brief SetPhysicsParams function pointer.
 * This function is called after the InitPhysics.
 * Use this function to modify physics parameters.
 * \return 0 on success, -1 on error.
 */
typedef int (*SetPhysicsParamsFnPtr)(void);

/* \brief DestroyPhysics function pointer.
 * This function is called when the plugin is deleted. Use this function
 * to destroy physics and cleanup the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*DestroyPhysicsFnPtr)(void);

/* \brief The physics plugin structure
 * This structure defines all the functions necessary to create and control
 * a physics engine.
 */
struct _PhysicsPlugin
{
  /* \brief Pointer to the initialize function.
   * This function must be implemented by a physics plugin.
   * \sa InitPhysicsFnPtr
   */
  InitPhysicsFnPtr initPhysics;

  /*
   *
   */
  LoadModelFnPtr initModel;

  /*
   *
   */
  SetStateFnPtr setState;

  /*
   *
   */
  UpdatePhysicsFnPtr updatePhysics;

  /*
   *
   */
  GetStateFnPtr getState;

  /*
   *
   */
  SetPhysicsParamsFnPtr setPhysicsParams;

  /* \brief Pointer to the destroy physics function.
   * This function must be implemented by a physics plugin.
   * \sa DestroyPhysicsFnPtr
   */
  DestroyPhysicsFnPtr destroyPhysics;
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
