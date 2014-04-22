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

#ifdef __cplusplus
extern "C" {
#endif

/* \brief Init function pointer.
 * This function is called after the plugin has been created. Use this
 * function to initialize the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*InitFnPtr)(void);

/* \brief Destroy function pointer.
 * This function is called when the plugin is deleted. Use this function
 * to destroy and cleanup the physics engine.
 * \return 0 on success, -1 on error.
 */
typedef int (*DestroyFnPtr)(void);

/* \brief The physics plugin structure
 * This structure defines all the functions necessary to create and control
 * a physics engine.
 */
struct _PhysicsPlugin
{
  /* \brief Pointer to the initialize function.
   * This function must be implemented by a physics plugin.
   * \sa InitFnPtr
   */
  InitFnPtr init;

  /* \brief Pointer to the destroy function.
   * This function must be implemented by a physics plugin.
   * \sa DestroyFnPtr
   */
  DestroyFnPtr destroy;
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
