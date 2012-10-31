#ifndef GIM_MATH_H_INCLUDED
#define GIM_MATH_H_INCLUDED

/*! \file gim_math.h
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

#include "config.h"

#include <math.h>
#include <float.h>
#if HAVE_SYS_TYPES_H
#include <sys/types.h>
#elif defined(_MSC_VER)
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
#elif defined(__GNUC__)
#include <inttypes.h>
#else
#error "GIMPACT: Must define int32_t and uint32_t"
#endif


/*! \defgroup BASIC_TYPES
Basic types and constants
Conventions:
Types starting with G
Constants starting with G_
*/
//! @{
/*! Types */
#define GREAL float
#define GINT32 int32_t
#define GUINT32 uint32_t

#define GPTR void*

/*! Constants for integers*/
#define GUINT32_BIT_COUNT 32
#define GUINT32_EXPONENT 5

#define G_FASTMATH 1
#define G_PI 3.14159265358979f
#define G_HALF_PI 1.5707963f
//267948966
#define G_TWO_PI 6.28318530f
//71795864
#define G_ROOT3 1.73205f
#define G_ROOT2 1.41421f
#define G_UINT_INFINITY 65534
#define G_REAL_INFINITY FLT_MAX
#define  G_SIGN_BITMASK      0x80000000
#define G_USE_EPSILON_TEST
#define G_EPSILON 0.0000001f
//! @}

/*! \defgroup MATH_FUNCTIONS
mathematical functions
*/
//! @{
#define G_DEGTORAD(X) ((X)*3.1415926f/180.0f)
#define G_RADTODEG(X) ((X)*180.0f/3.1415926f)

//! Integer representation of a floating-point value.
//#define IR(x)          ((GUINT32&)(x))
#define IR(x)          (static_cast<GUINT32>(x))

//! Signed integer representation of a floating-point value.
//#define SIR(x)          ((GINT32&)(x))
#define SIR(x)          (static_cast<GINT32>(x))

//! Absolute integer representation of a floating-point value
#define AIR(x)          (IR(x)&0x7fffffff)

//! Floating-point representation of an integer value.
//#define FR(x)          ((GREAL&)(x))
#define FR(x)          (static_cast<GREAL>(x))

#define MAX(a,b) ((a)<(b)?(b):(a))
#define MIN(a,b) ((a)>(b)?(b):(a))

#define MAX3(a,b,c) MAX(a,MAX(b,c))
#define MIN3(a,b,c) MIN(a,MIN(b,c))

#define IS_ZERO(value) ((value) < G_EPSILON &&  (value) > -G_EPSILON)

#define IS_NEGATIVE(value) ((value) <= -G_EPSILON)

#define IS_POSISITVE(value) ((value) >= G_EPSILON)

///returns a clamped number
#define CLAMP(number,minval,maxval) ((number)<(minval)?(minval):((number)>(maxval)?(maxval):(number)))

///Swap numbers
#define SWAP_NUMBERS(a,b){ \
    (a) = (a)+(b); \
    (b) = (a)-(b); \
    (a) = (a)-(b); \
}\

#define GIM_INV_SQRT(va,isva)\
{\
    if((va)<=0.0000001f)\
    {\
        (isva) = G_REAL_INFINITY;\
    }\
    else\
    {\
        GREAL _x = (va) * 0.5f;\
        GUINT32 _y = 0x5f3759df - ( IR(va) >> 1);\
        (isva) = FR(_y);\
        (isva) = (isva) * ( 1.5f - ( _x * (isva) * (isva) ) );\
    }\
}\

#define GIM_SQRT(va,sva)\
{\
    GIM_INV_SQRT(va,sva);\
    (sva) = 1.0f/(sva);\
}\

//! Computes 1.0f / sqrtf(x). Comes from Quake3. See http://www.magic-software.com/3DGEDInvSqrt.html
GREAL gim_inv_sqrt(GREAL f);

//! Computes sqrtf(x) faster.
/*!
\sa gim_inv_sqrt
*/
GREAL gim_sqrt(GREAL f);

//!Initializes mathematical functions
void gim_init_math();

//! Generates an unit random
GREAL gim_unit_random();
//! @}

#endif // GIM_MATH_H_INCLUDED
