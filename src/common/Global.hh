/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Generic Typedefs, macros, functions, etc
 * Author: Nate Koenig
 * Date: 16 June 2003
 * CVS: $Id$
 */

#ifndef GZGLOBAL_HH
#define GZGLOBAL_HH

#include <string>

/////////////////////////////////////////////////////////////////////////////
// Defines
/////////////////////////////////////////////////////////////////////////////
#ifndef NULL
#define NULL 0
}
#endif

#ifndef GZ_COLLIDE_BITS

#define GZ_ALL_COLLIDE 0x0FFFFFFF
#define GZ_NONE_COLLIDE 0x00000000
#define GZ_FIXED_COLLIDE 0x00000001
#define GZ_SENSOR_COLLIDE 0x00000002
#define GZ_GHOST_COLLIDE 0x10000000

}
#endif

#ifndef GZ_CAMERA_BITS

#define GZ_ALL_CAMERA 0xFFFFFFFF
#define GZ_LASER_CAMERA 0x00000001

}
#endif

/////////////////////////////////////////////////////////////////////////////
// Macros
/////////////////////////////////////////////////////////////////////////////

#if defined(__GNUC__)
#define GAZEBO_DEPRECATED __attribute__((deprecated))
#define GAZEBO_FORCEINLINE __attribute__((always_inline))
#elif defined(MSVC)
#define GAZEBO_DEPRECATED
#define GAZEBO_FORCEINLINE __forceinline
#else
#define GAZEBO_DEPRECATED
#define GAZEBO_FORCEINLINE
}
#endif

// Convert radians to degrees
#define RTOD(r) ((r) * 180 / M_PI)

// Convert degrees to radians
#define DTOR(d) ((d) * M_PI / 180)

// Normalize an angle in the range -Pi to Pi
#define NORMALIZE(a) (atan2(sin(a), cos(a)))

// Odd and even macros
#define ISEVEN(x) ( ((x) % 2) == 0)
#define ISODD(x) ( ((x) % 2) != 0)

#define ROUND(x) ( (int)( floor((x)+0.5) ) )

enum EntityType{COMMON, ENTITY, MODEL, BODY, GEOM, BALL_JOINT, BOX_SHAPE, CYLINDER_SHAPE, HEIGHTMAP_SHAPE, HINGE2_JOINT, HINGE_JOINT, JOINT, MAP_SHAPE, MULTIRAY_SHAPE, RAY_SHAPE, PLANE_SHAPE, SHAPE, SLIDER_JOINT, SPHERE_SHAPE, TRIMESH_SHAPE, UNIVERSAL_JOINT, LIGHT, VISUAL};

static std::string EntityTypename[] = { "common", "entity", "model", "body", "geom", "ball", "box", "cylinder", "heightmap", "hinge2", "hinge", "joint", "map", "multiray", "ray", "plane", "shape", "slider", "sphere", "trimesh", "universal", "light","visual" };

}
#endif
