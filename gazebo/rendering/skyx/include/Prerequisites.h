/*
--------------------------------------------------------------------------------
This source file is part of SkyX.
Visit http://www.paradise-studios.net/products/skyx/

Copyright (C) 2009-2012 Xavier Verguín González <xavyiy@gmail.com>

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
--------------------------------------------------------------------------------
*/

#ifndef _SkyX_Prerequisites_H_
#define _SkyX_Prerequisites_H_

#ifdef _MSC_VER
  // conversion from 'xxx' to 'yyy', possible loss of data
  #pragma warning(disable:4244)
  // 'this' : used in base member initializer list
  #pragma warning(disable:4355)
#endif

/// Include external headers
#include <OGRE/Ogre.h>

/// Define the dll export qualifier if compiling for Windows
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  #ifdef SKYX_LIB
    #define DllExport __declspec (dllexport)
  #else
    #define DllExport __declspec (dllimport)
  #endif
#else
  #define DllExport
#endif

/// Log macro
#define SkyXLOG(msg) Ogre::LogManager::getSingleton().logMessage("[SkyX] " + \
    Ogre::String(msg));

/// SkyX defines
#define SKYX_VERSION_MAJOR 0
#define SKYX_VERSION_MINOR 3
#define SKYX_VERSION_PATCH 1

#define SKYX_RESOURCE_GROUP "SkyX"

template<typename T>
inline bool equal(const T &_a, const T &_b, const T &_epsilon = 1e-3)
{
  return std::fabs(_a - _b) <= _epsilon;
}

#endif
