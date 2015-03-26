/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/* miscellaneous math functions. these are mostly useful for testing */

#ifndef _ODE_MISC_H_
#define _ODE_MISC_H_

#include <ode/common.h>


#ifdef __cplusplus
extern "C" {
#endif


/* return 1 if the random number generator is working. */
ODE_API int dTestRand(void);

/* return next 32 bit random number. this uses a not-very-random linear
 * congruential method.
 */
ODE_API unsigned long dRand(void);

/* get and set the current random number seed. */
ODE_API unsigned long  dRandGetSeed(void);
ODE_API void dRandSetSeed (unsigned long s);

/* return a random integer between 0..n-1. the distribution will get worse
 * as n approaches 2^32.
 */
ODE_API int dRandInt (int n);

/* return a random real number between 0..1 */
ODE_API dReal dRandReal(void);

/* print out a matrix */
#ifdef __cplusplus
ODE_API void dPrintMatrix (const dReal *A, int n, int m, char *fmt = "%10.4f ",
		   FILE *f=stdout);
ODE_API void dPrintIntMatrix (const int *A, int n, int m, char *fmt = "%5d ",
		   FILE *f=stdout);
#else
ODE_API void dPrintMatrix (const dReal *A, int n, int m, char *fmt, FILE *f);
ODE_API void dPrintIntMatrix (const int *A, int n, int m, char *fmt, FILE *f);
#endif

/* make a random vector with entries between +/- range. A has n elements. */
ODE_API void dMakeRandomVector (dReal *A, int n, dReal range);

/* make a random matrix with entries between +/- range. A has size n*m. */
ODE_API void dMakeRandomMatrix (dReal *A, int n, int m, dReal range);

/* clear the upper triangle of a square matrix */
ODE_API void dClearUpperTriangle (dReal *A, int n);

/* return the maximum element difference between the two n*m matrices */
ODE_API dReal dMaxDifference (const dReal *A, const dReal *B, int n, int m);

/* return the maximum element difference between the lower triangle of two
 * n*n matrices */
ODE_API dReal dMaxDifferenceLowerTriangle (const dReal *A, const dReal *B, int n);

/*!
 * \brief dNormalizeAnglePositive
 *
 *        Normalizes the angle to be 0 to 2*M_PI
 *        It takes and returns radians.
 */
static inline dReal dNormalizeAnglePositive(dReal angle)
{
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}


/*!
 * \brief normalize
 *
 * Normalizes the angle from [0, 2*M_PI] to [-M_PI, +M_PI] circle
 * It takes and returns radians.
 *
 */    
static inline dReal dNormalizeAngle(dReal angle)
{
  dReal a = dNormalizeAnglePositive(angle);
  if (a > M_PI)
    a -= 2.0 *M_PI;
  return a;
}

  
/*!
 * \function
 * \brief dShortestAngularDistance
 *
 * Given 2 angles, this returns the shortest angular
 * difference.  The inputs and ouputs are of course radians.
 *
 * The result
 * would always be -pi <= result <= pi.  Adding the result
 * to "from" will always get you an equivelent angle to "to".
 */
static inline dReal dShortestAngularDistance(dReal from, dReal to)
{
  dReal result = dNormalizeAngle(dNormalizeAnglePositive(dNormalizeAnglePositive(to) -
    dNormalizeAnglePositive(from)));

  return result;
}

/*!
 * \function
 * \brief dShortestAngularDistanceUpdate
 *
 * Given 2 angles, this returns the shortest angular
 * difference.  The inputs and ouputs are radians.
 *
 * This function returns (from + delta) where delta is in the range of [-pi, pi].
 * However, if |delta| > tol, then this function simply returns incoming parameter "to".
 *
 */
static inline dReal dShortestAngularDistanceUpdate(dReal from, dReal to, dReal tol = 0.3)
{
  dReal result = dShortestAngularDistance(from, to);

  if (dFabs(result) > tol)
    return to;
  else
    return from + result;
}

#ifdef __cplusplus
}
#endif

#endif
