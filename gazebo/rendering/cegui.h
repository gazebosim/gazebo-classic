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
#ifndef _GAZEBO_CEGUI_H_
#define _GAZEBO_CEGUI_H_

// This disables warning messages
#pragma GCC system_header

#include "gazebo/gazebo_config.h"

#ifdef HAVE_CEGUI
#include <CEGUI/CEGUI.h>
#include <CEGUI/CEGUIEventArgs.h>
// For ogre-1.9 or greater while CEGUI is lower than 0.8
// check issue: https://bitbucket.org/osrf/gazebo/issue/994
#if ((OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0)) && \
    (CEGUI_VERSION < ((0 << 16) | (8 << 8) | 3)))
#include "gazebo/rendering/CEGUIOgreRenderer.h"
#else
#include <CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h>
#endif

#endif
#endif
