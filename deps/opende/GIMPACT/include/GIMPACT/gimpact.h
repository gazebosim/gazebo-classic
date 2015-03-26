#ifndef GIMPACT_H_INCLUDED
#define GIMPACT_H_INCLUDED

/*! \file gimpact.h
\author Francisco León
*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL_GIMPACT
    #ifdef __GNUC__
      #define GIMPACT_VISIBLE __attribute__ ((dllexport))
    #else
      #define GIMPACT_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define GIMPACT_VISIBLE __attribute__ ((dllimport))
    #else
      #define GIMPACT_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define GIMPACT_HIDDEN
#else
  #if __GNUC__ >= 4
    #define GIMPACT_VISIBLE __attribute__ ((visibility ("default")))
    #define GIMPACT_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define GIMPACT_VISIBLE
    #define GIMPACT_HIDDEN
  #endif
#endif

#include "GIMPACT/gim_trimesh.h"

/*! \defgroup GIMPACT_INIT
*/
//! @{
//! Call this for initialize GIMPACT system structures.
GIMPACT_VISIBLE void gimpact_init();
//! Call this for clean GIMPACT system structures.
GIMPACT_VISIBLE void gimpact_terminate();
//! @}
#endif // GIMPACT_H_INCLUDED
