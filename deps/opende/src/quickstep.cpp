/*************************************************************************
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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
#undef NDEBUG
#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include <ode/misc.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "lcp.h"
#include "util.h"

#include <sys/time.h>

#ifdef SSE
#include <xmmintrin.h>
#define Kf(x) _mm_set_pd((x),(x))
#endif


#undef REPORT_THREAD_TIMING
#define USE_TPROW
#undef TIMING
#undef REPORT_MONITOR
#undef SHOW_CONVERGENCE
//#define LOCAL_STEPPING  // not yet implemented
#undef RECOMPUTE_RMS
#undef USE_1NORM



#ifdef USE_TPROW
// added for threading per constraint rows
#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include "ode/odeinit.h"
#endif

typedef const dReal *dRealPtr;
typedef dReal *dRealMutablePtr;

//***************************************************************************
// configuration

// for the SOR and CG methods:
// uncomment the following line to use warm starting. this definitely
// help for motor-driven joints. unfortunately it appears to hurt
// with high-friction contacts using the SOR method. use with care

//#define WARM_STARTING 1


// for the SOR method:
// uncomment the following line to determine a new constraint-solving
// order for each iteration. however, the qsort per iteration is expensive,
// and the optimal order is somewhat problem dependent.
// @@@ try the leaf->root ordering.

//#define REORDER_CONSTRAINTS 1


// for the SOR method:
// uncomment the following line to randomly reorder constraint rows
// during the solution. depending on the situation, this can help a lot
// or hardly at all, but it doesn't seem to hurt.

#define RANDOMLY_REORDER_CONSTRAINTS 1
#undef LOCK_WHILE_RANDOMLY_REORDER_CONSTRAINTS


// structure for passing variable pointers in SOR_LCP
struct dxSORLCPParameters {
    dxQuickStepParameters *qs;
    int nStart;   // 0
    int nChunkSize;
    int m; // m
    int nb;
    dReal stepsize;
    int* jb;
    const int* findex;
    dRealPtr Ad;
    dRealPtr hi;
    dRealPtr lo;
    dRealPtr Adcfm;
    dRealPtr invI;
    dRealPtr I;
    dRealPtr Ad_precon;
    dRealPtr Adcfm_precon;
    dRealPtr JiMratio;
    dRealMutablePtr vnew;
    dRealMutablePtr b;
    dRealMutablePtr J;
    dRealMutablePtr ac;
    dRealMutablePtr lambda;
    dRealMutablePtr iMJ;
    dRealMutablePtr delta_error ;
    dRealMutablePtr b_precon ;
    dRealMutablePtr J_precon ;
    dRealMutablePtr J_orig ;
    dRealMutablePtr fc ;
#ifdef REORDER_CONSTRAINTS
    dRealMutablePtr last_lambda ;
#endif
};


//****************************************************************************
// special matrix multipliers

// multiply block of B matrix (q x 6) with 12 dReal per row with C vektor (q)
static void Multiply1_12q1 (dReal *A, const dReal *B, const dReal *C, int q)
{
  dIASSERT (q>0 && A && B && C);

  dReal a = 0;
  dReal b = 0;
  dReal c = 0;
  dReal d = 0;
  dReal e = 0;
  dReal f = 0;
  dReal s;

  for(int i=0, k = 0; i<q; k += 12, i++)
  {
    s = C[i]; //C[i] and B[n+k] cannot overlap because its value has been read into a temporary.

    //For the rest of the loop, the only memory dependency (array) is from B[]
    a += B[  k] * s;
    b += B[1+k] * s;
    c += B[2+k] * s;
    d += B[3+k] * s;
    e += B[4+k] * s;
    f += B[5+k] * s;
  }

  A[0] = a;
  A[1] = b;
  A[2] = c;
  A[3] = d;
  A[4] = e;
  A[5] = f;
}

//***************************************************************************
// testing stuff

#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif

//***************************************************************************
// various common computations involving the matrix J

// compute iMJ = inv(M)*J'

static void compute_invM_JT (int m, dRealPtr J, dRealMutablePtr iMJ, int *jb,
  dxBody * const *body, dRealPtr invI)
{
  dRealMutablePtr iMJ_ptr = iMJ;
  dRealPtr J_ptr = J;
  for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal k1 = body[b1]->invMass;
    for (int j=0; j<3; j++) iMJ_ptr[j] = k1*J_ptr[j];
    const dReal *invIrow1 = invI + 12*b1;
    dMultiply0_331 (iMJ_ptr + 3, invIrow1, J_ptr + 3);
    if (b2 >= 0) {
      dReal k2 = body[b2]->invMass;
      for (int j=0; j<3; j++) iMJ_ptr[j+6] = k2*J_ptr[j+6];
      const dReal *invIrow2 = invI + 12*b2;
      dMultiply0_331 (iMJ_ptr + 9, invIrow2, J_ptr + 9);
    }
  }
}

// compute out = inv(M)*J'*in.
#ifdef WARM_STARTING
static void multiply_invM_JT (int m, int nb, dRealMutablePtr iMJ, int *jb,
  dRealPtr in, dRealMutablePtr out)
{
  dSetZero (out,6*nb);
  dRealPtr iMJ_ptr = iMJ;
  for (int i=0; i<m; i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    const dReal in_i = in[i];
    dRealMutablePtr out_ptr = out + b1*6;
    for (int j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in_i;
    iMJ_ptr += 6;
    if (b2 >= 0) {
      out_ptr = out + b2*6;
      for (int j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in_i;
    }
    iMJ_ptr += 6;
  }
}
#endif

// compute out = J*in.

static void multiply_J (int m, dRealPtr J, int *jb,
  dRealPtr in, dRealMutablePtr out)
{
  dRealPtr J_ptr = J;
  for (int i=0; i<m; i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal sum = 0;
    dRealPtr in_ptr = in + b1*6;
    for (int j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
    J_ptr += 6;
    if (b2 >= 0) {
      in_ptr = in + b2*6;
      for (int j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
    }
    J_ptr += 6;
    out[i] = sum;
  }
}


// compute out = (J*inv(M)*J' + cfm)*in.
// use z as an nb*6 temporary.
#ifdef WARM_STARTING
static void multiply_J_invM_JT (int m, int nb, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
  dRealPtr cfm, dRealMutablePtr z, dRealMutablePtr in, dRealMutablePtr out)
{
  multiply_invM_JT (m,nb,iMJ,jb,in,z);
  multiply_J (m,J,jb,z,out);

  // add cfm
  for (int i=0; i<m; i++) out[i] += cfm[i] * in[i];
}
#endif

//***************************************************************************
// conjugate gradient method with jacobi preconditioner
// THIS IS EXPERIMENTAL CODE that doesn't work too well, so it is ifdefed out.
//
// adding CFM seems to be critically important to this method.

#ifdef USE_CG_LCP

static inline dReal dot (int n, dRealPtr x, dRealPtr y)
{
  dReal sum=0;
  for (int i=0; i<n; i++) sum += x[i]*y[i];
  return sum;
}


// x = y + z*alpha

static inline void add (int n, dRealMutablePtr x, dRealPtr y, dRealPtr z, dReal alpha)
{
  for (int i=0; i<n; i++) x[i] = y[i] + z[i]*alpha;
}

static void CG_LCP (dxWorldProcessContext *context,
  int m, int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
  dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr fc, dRealMutablePtr b,
  dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
  dxQuickStepParameters *qs)
{
  const int num_iterations = qs->num_iterations;

  // precompute iMJ = inv(M)*J'
  dReal *iMJ = context->AllocateArray<dReal> (m*12);
  compute_invM_JT (m,J,iMJ,jb,body,invI);

  dReal last_rho = 0;
  dReal *r = context->AllocateArray<dReal> (m);
  dReal *z = context->AllocateArray<dReal> (m);
  dReal *p = context->AllocateArray<dReal> (m);
  dReal *q = context->AllocateArray<dReal> (m);

  // precompute 1 / diagonals of A
  dReal *Ad = context->AllocateArray<dReal> (m);
  dRealPtr iMJ_ptr = iMJ;
  dRealPtr J_ptr = J;
  for (int i=0; i<m; i++) {
    dReal sum = 0;
    for (int j=0; j<6; j++) sum += iMJ_ptr[j] * J_ptr[j];
    if (jb[i*2+1] >= 0) {
      for (int j=6; j<12; j++) sum += iMJ_ptr[j] * J_ptr[j];
    }
    iMJ_ptr += 12;
    J_ptr += 12;
    Ad[i] = REAL(1.0) / (sum + cfm[i]);
  }

#ifdef WARM_STARTING
  // compute residual r = b - A*lambda
  multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,lambda,r);
  for (int k=0; k<m; k++) r[k] = b[k] - r[k];
#else
  dSetZero (lambda,m);
  memcpy (r,b,m*sizeof(dReal));		// residual r = b - A*lambda
#endif

  for (int iteration=0; iteration < num_iterations; iteration++) {
    for (int i=0; i<m; i++) z[i] = r[i]*Ad[i];	// z = inv(M)*r
    dReal rho = dot (m,r,z);		// rho = r'*z

    // @@@
    // we must check for convergence, otherwise rho will go to 0 if
    // we get an exact solution, which will introduce NaNs into the equations.
    if (rho < 1e-10) {
      printf ("CG returned at iteration %d\n",iteration);
      break;
    }

    if (iteration==0) {
      memcpy (p,z,m*sizeof(dReal));	// p = z
    }
    else {
      add (m,p,z,p,rho/last_rho);	// p = z + (rho/last_rho)*p
    }

    // compute q = (J*inv(M)*J')*p
    multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,p,q);

    dReal alpha = rho/dot (m,p,q);		// alpha = rho/(p'*q)
    add (m,lambda,lambda,p,alpha);		// lambda = lambda + alpha*p
    add (m,r,r,q,-alpha);			// r = r - alpha*q
    last_rho = rho;
  }

  // compute fc = inv(M)*J'*lambda
  multiply_invM_JT (m,nb,iMJ,jb,lambda,fc);

#if 0
  // measure solution error
  multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,lambda,r);
  dReal error = 0;
  for (int i=0; i<m; i++) error += dFabs(r[i] - b[i]);
  printf ("lambda error = %10.6e\n",error);
#endif
}

#endif

//***************************************************************************
// SOR-LCP method

// nb is the number of bodies in the body array.
// J is an m*12 matrix of constraint rows
// jb is an array of first and second body numbers for each constraint row
// invI is the global frame inverse inertia for each body (stacked 3x3 matrices)
//
// this returns lambda and fc (the constraint force).
// note: fc is returned as inv(M)*J'*lambda, the constraint force is actually J'*lambda
//
// b, lo and hi are modified on exit

struct IndexError {
#ifdef REORDER_CONSTRAINTS
  dReal error;		// error to sort on
  int findex;
#endif
  int index;		// row index
};


#ifdef REORDER_CONSTRAINTS

static int compare_index_error (const void *a, const void *b)
{
  const IndexError *i1 = (IndexError*) a;
  const IndexError *i2 = (IndexError*) b;
  if (i1->findex < 0 && i2->findex >= 0) return -1;
  if (i1->findex >= 0 && i2->findex < 0) return 1;
  if (i1->error < i2->error) return -1;
  if (i1->error > i2->error) return 1;
  return 0;
}

#endif

void updateVnew(const int nb, dRealPtr invI, dRealPtr ac, dxBody * const * body, dReal stepsize, dRealMutablePtr vnew)
{
  /****************************************************************/
  /* compute vnew per ac update                                   */
  /*   based on all external forces fe, ac                        */
  /*   vnew should have started out same as v(n)                  */
  /*   vnew should end up being v(n+1)                            */
  /*                                                              */
  /*  vnew = v_current + stepsize * invM * f_all                  */
  /*                                                              */
  /****************************************************************/
  {
    const dReal *invIrow = invI;
    dRealMutablePtr vnew_ptr = vnew;
    dxBody *const *const bodyend = body + nb;
    const dReal *ac_ptr = ac;

    for (dxBody *const *bodycurr = body; bodycurr != bodyend; ac_ptr+=6, invIrow += 12, vnew_ptr+=6, bodycurr++) {
      // compute the velocity update:
      // add stepsize * invM * f_all to the body velocity
      dxBody *b = *bodycurr;
      dReal body_invMass_mul_stepsize = stepsize * b->invMass;
      dReal tmp3[3];
      for (int j=0; j<3; j++) {
        // note that (ac) is an acceleration, hence there is
        // add stepsize * ac to the body velocity
        vnew_ptr[j]   = b->lvel[j] + stepsize * ac_ptr[j]   + body_invMass_mul_stepsize * b->facc[j];
        vnew_ptr[j+3] = b->avel[j] + stepsize * ac_ptr[j+3];

        // accumulate step*torques
        tmp3[j] = stepsize*b->tacc[j];
      }
      // vnew = invI * f_all
      dMultiplyAdd0_331 (vnew_ptr+3, invIrow, tmp3);

    }
  }
}

// vnew = J * M * vnew / stepsize
inline static void multiply_JM (const int nb, const int m, dRealPtr J, dRealPtr I, dxBody * const *body, int *jb, dReal stepsize, dRealMutablePtr vnew)
{
  const dReal stepsize1 = dRecip(stepsize);
  dReal *tmp1 = new dReal[nb*6];
  dReal *tmp1curr = tmp1;
  dRealPtr vnewcurr = vnew;
  const dReal *Irow = I;
  dxBody *const *const bodyend = body + nb;
  for (dxBody *const *bodycurr = body; bodycurr != bodyend; vnewcurr+=6, tmp1curr+=6, Irow+=12, bodycurr++) {
    dxBody *b_ptr = *bodycurr;
    dReal body_mass = b_ptr->mass.mass;
    for (int j=0; j<3; j++) tmp1curr[j] = body_mass * vnewcurr[j] * stepsize1;
    dReal tmpa[3];
    for (int j=0; j<3; j++) tmpa[j] = vnewcurr[j+3] * stepsize1;
    dMultiply0_331 (tmp1curr + 3,Irow,tmpa);
  }
  multiply_J (m,J,jb,tmp1,vnew);
  delete tmp1;
  tmp1 = 0;
}

void computeRHSPrecon(dxWorldProcessContext *context, const int m, const int nb, dRealPtr /*invI*/, dRealPtr I, dxBody * const *body, const dReal stepsize1, dRealMutablePtr c, dRealMutablePtr J, int *jb, dRealMutablePtr rhs_precon)
{
    /************************************************************************************/
    /*                                                                                  */
    /*               compute preconditioned rhs                                         */
    /*                                                                                  */
    /*  J J' lambda = J * ( M * (vnew - v) / h + fe )                                   */
    /*                                                                                  */
    /*  initiallly vnew is zero                                                         */
    /*                                                                                  */
    /************************************************************************************/
    // mimic computation of rhs, but do it with J*M*inv(J) prefixed for preconditioned case.
    BEGIN_STATE_SAVE(context, tmp2state) {
      IFTIMING (dTimerNow ("compute rhs_precon"));
      
      // compute the "preconditioned" right hand side `rhs_precon'
      dReal *tmp1 = context->AllocateArray<dReal> (nb*6);
      // this is slightly different than non precon, where M is left multiplied by the pre J terms
      //
      // tmp1 = M*v/h + fe
      //
      dReal *tmp1curr = tmp1;
      const dReal *Irow = I;
      dxBody *const *const bodyend = body + nb;
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; tmp1curr+=6, Irow+=12, bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        dReal body_mass = b_ptr->mass.mass;
        for (int j=0; j<3; j++) tmp1curr[j] = b_ptr->facc[j] +  0* body_mass * b_ptr->lvel[j] * stepsize1;
        dReal tmpa[3];
        for (int j=0; j<3; j++) tmpa[j] = 0* b_ptr->avel[j] * stepsize1;
        dMultiply0_331 (tmp1curr + 3,Irow,tmpa);
        for (int k=0; k<3; k++) tmp1curr[3+k] += b_ptr->tacc[k];
      }
      //
      // rhs_precon = - J * (M*v/h + fe)
      //
      multiply_J (m,J,jb,tmp1,rhs_precon);

      //
      // TODO: rhs_precon = J * inv(M) * c / h - J * (M*v/h + fe)
      //
      // c is the acceleration in the direction of the constraint, i.e. J * accel_inertial_frame
      // so here, we want the forces forces in the constraint directions.
      //
      // analogously, we can view them as either velocity vs. momentum (m*v)
      //  basically one is mass * the other one.
      //
      // The problem here is... everything is ok if velocity/acceleration is zero, but
      //   momentum/forces are NOT zero even if velocity is zero.
      //
      // Question, can we do:  JM .dot. Jv (or JM .dot. c) for momentum/force
      //   in the direction of the constraints?
      //
      for (int i=0; i<m; i++) rhs_precon[i] = 0*c[i]*stepsize1 - rhs_precon[i];


      /*  DEBUG PRINTOUTS
      printf("\n");
      for (int i=0; i<m; i++) printf("c[%d] = %f\n",i,c[i]);
      printf("\n");
      */
    } END_STATE_SAVE(context, tmp2state);

    // complete rhs
    //for (int i=0; i<m; i++) rhs_precon[i] = - rhs_precon[i];
}

static inline dReal dot6(dRealPtr a, dRealPtr b)
{
#ifdef SSE
  __m128d d = _mm_load_pd(a+0) * _mm_load_pd(b+0) + _mm_load_pd(a+2) * _mm_load_pd(b+2) + _mm_load_pd(a+4) * _mm_load_pd(b+4);
  double r[2];
  _mm_store_pd(r, d);
  return r[0] + r[1];
#else
  return a[0] * b[0] +
         a[1] * b[1] +
         a[2] * b[2] +
         a[3] * b[3] +
         a[4] * b[4] +
         a[5] * b[5];
#endif
}

static inline void sum6(dRealMutablePtr a, dReal delta, dRealPtr b)
{
#ifdef SSE
  __m128d __delta = Kf(delta);
  _mm_store_pd(a + 0, _mm_load_pd(a + 0) + __delta * _mm_load_pd(b + 0));
  _mm_store_pd(a + 2, _mm_load_pd(a + 2) + __delta * _mm_load_pd(b + 2));
  _mm_store_pd(a + 4, _mm_load_pd(a + 4) + __delta * _mm_load_pd(b + 4));
#else
  a[0] += delta * b[0];
  a[1] += delta * b[1];
  a[2] += delta * b[2];
  a[3] += delta * b[3];
  a[4] += delta * b[4];
  a[5] += delta * b[5];
#endif
}

static void ComputeRows(
                int /*thread_id*/,
                IndexError* order,
                dxBody* const * /*body*/,
                dxSORLCPParameters params,
                boost::recursive_mutex* /*mutex*/)
{
  struct timeval tv;
  double cur_time;
  gettimeofday(&tv,NULL);
  cur_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("thread %d started at time %f\n",thread_id,cur_time);

  //boost::recursive_mutex::scoped_lock lock(*mutex); // put in ac read/writes?
  dxQuickStepParameters *qs    = params.qs;
  int startRow                 = params.nStart;   // 0
  int nRows                    = params.nChunkSize; // m
  //int m                        = params.m; // m
  //int nb                       = params.nb;
  //dReal stepsize               = params.stepsize;
  int* jb                      = params.jb;
  const int* findex            = params.findex;
  //dRealPtr        Ad           = params.Ad;
  dRealPtr        hi           = params.hi;
  dRealPtr        lo           = params.lo;
  dRealPtr        Adcfm        = params.Adcfm;
  //dRealPtr        invI         = params.invI;
  //dRealPtr        I            = params.I;
  //dRealPtr        Ad_precon    = params.Ad_precon;
  dRealPtr        Adcfm_precon = params.Adcfm_precon;
  //dRealPtr        JiMratio     = params.JiMratio;
  //dRealMutablePtr vnew         = params.vnew;
  dRealMutablePtr b            = params.b;
  dRealMutablePtr J            = params.J;
  dRealMutablePtr ac           = params.ac;
  dRealMutablePtr lambda       = params.lambda;
  dRealMutablePtr iMJ          = params.iMJ;
#ifdef RECOMPUTE_RMS
  dRealMutablePtr delta_error  = params.delta_error;
#endif
  dRealMutablePtr b_precon     = params.b_precon;
  dRealMutablePtr J_precon     = params.J_precon;
  dRealMutablePtr J_orig       = params.J_orig;
  dRealMutablePtr fc           = params.fc;
#ifdef REORDER_CONSTRAINTS
  dRealMutablePtr last_lambda  = params.last_lambda;
#endif

  //printf("iiiiiiiii %d %d %d\n",thread_id,jb[0],jb[1]);
  //for (int i=startRow; i<startRow+nRows; i++) // swap within boundary of our own segment
  //  printf("wwwwwwwwwwwww>id %d start %d n %d  order[%d].index=%d\n",thread_id,startRow,nRows,i,order[i].index);



/*  DEBUG PRINTOUTS
  // print J_orig
  printf("J_orig\n");
  for (int i=0; i < m ; i++) {
    for (int j=0; j < 12 ; j++) {
      printf("	%12.6f",J_orig[i*12+j]);
    }
    printf("\n");
  }
  printf("\n");

  // print J, J_precon (already premultiplied by inverse of diagonal of LHS) and b_precon and b
  printf("J_precon\n");
  for (int i=0; i < m ; i++) {
    for (int j=0; j < 12 ; j++) {
      printf("	%12.6f",J_precon[i*12+j]);
    }
    printf("\n");
  }
  printf("\n");

  printf("J\n");
  for (int i=0; i < m ; i++) {
    for (int j=0; j < 12 ; j++) {
      printf("	%12.6f",J[i*12+j]);
    }
    printf("\n");
  }
  printf("\n");

  printf("b_precon\n");
  for (int i=0; i < m ; i++) printf("	%12.6f",b_precon[i]);
  printf("\n");

  printf("b\n");
  for (int i=0; i < m ; i++) printf("	%12.6f",b[i]);
  printf("\n");
*/


  double rms_error = 0;
  int num_iterations = qs->num_iterations;
  int precon_iterations = qs->precon_iterations;
  double sor_lcp_tolerance = qs->sor_lcp_tolerance;


  // FIME: preconditioning can be defined insdie iterations loop now, becareful to match last iteration with
  //       velocity update
  bool preconditioning;
  for (int iteration=0; iteration < num_iterations + precon_iterations; iteration++) {

    rms_error = 0;

    if (iteration < precon_iterations) preconditioning = true;
    else                               preconditioning = false;

#ifdef REORDER_CONSTRAINTS
    // constraints with findex < 0 always come first.
    if (iteration < 2) {
      // for the first two iterations, solve the constraints in
      // the given order
      IndexError *ordercurr = order+startRow;
      //for (int i = 0; i != m; ordercurr++, i++) { }
      for (int i = startRow; i != startRow+nRows; ordercurr++, i++) {
        ordercurr->error = i;
        ordercurr->findex = findex[i];
        ordercurr->index = i;
      }
    }
    else {
      // sort the constraints so that the ones converging slowest
      // get solved last. use the absolute (not relative) error.
      //for (int i=0; i<m; i++) { }
      for (int i=startRow; i<startRow+nRows; i++) {
        dReal v1 = dFabs (lambda[i]);
        dReal v2 = dFabs (last_lambda[i]);
        dReal max = (v1 > v2) ? v1 : v2;
        if (max > 0) {
          //@@@ relative error: order[i].error = dFabs(lambda[i]-last_lambda[i])/max;
          order[i].error = dFabs(lambda[i]-last_lambda[i]);
        }
        else {
          order[i].error = dInfinity;
        }
        order[i].findex = findex[i];
        order[i].index = i;
      }
    }

    //if (thread_id == 0) for (int i=startRow;i<startRow+nRows;i++) printf("=====> %d %d %d %f %d\n",thread_id,iteration,i,order[i].error,order[i].index);

    //qsort (order,m,sizeof(IndexError),&compare_index_error);
    qsort (order+startRow,nRows,sizeof(IndexError),&compare_index_error);

    //@@@ potential optimization: swap lambda and last_lambda pointers rather
    //    than copying the data. we must make sure lambda is properly
    //    returned to the caller
    //memcpy (last_lambda,lambda,m*sizeof(dReal));
    memcpy (last_lambda+startRow,lambda+startRow,nRows*sizeof(dReal));

    //if (thread_id == 0) for (int i=startRow;i<startRow+nRows;i++) printf("-----> %d %d %d %f %d\n",thread_id,iteration,i,order[i].error,order[i].index);

#endif
#ifdef RANDOMLY_REORDER_CONSTRAINTS
    if ((iteration & 7) == 0) {
      #ifdef LOCK_WHILE_RANDOMLY_REORDER_CONSTRAINTS
        boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every swap
      #endif
      //for (int i=1; i<m; i++) {}   // swap across engire matrix
      //  int swapi = dRandInt(i+1); // swap across engire matrix
      for (int i=startRow+1; i<startRow+nRows; i++) { // swap within boundary of our own segment
        int swapi = dRandInt(i+1-startRow)+startRow; // swap within boundary of our own segment
        //printf("xxxxxxxx>id %d swaping order[%d].index=%d order[%d].index=%d\n",thread_id,i,order[i].index,swapi,order[swapi].index);
        IndexError tmp = order[i];
        order[i] = order[swapi];
        order[swapi] = tmp;
      }

      // {
      //   // verify
      //   boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every row
      //   printf("  random id %d iter %d\n",thread_id,iteration);
      //   for (int i=startRow+1; i<startRow+nRows; i++)
      //     printf(" %5d,",i);
      //   printf("\n");
      //   for (int i=startRow+1; i<startRow+nRows; i++)
      //     printf(" %5d;",(int)order[i].index);
      //   printf("\n");
      // }
    }
#endif

    //dSetZero (delta_error,m);

    dRealMutablePtr ac_ptr1;
    dRealMutablePtr ac_ptr2;
    dRealMutablePtr fc_ptr1;
    dRealMutablePtr fc_ptr2;
    for (int i=startRow; i<startRow+nRows; i++) {
      //boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every row

      // @@@ potential optimization: we could pre-sort J and iMJ, thereby
      //     linearizing access to those arrays. hmmm, this does not seem
      //     like a win, but we should think carefully about our memory
      //     access pattern.

      int index = order[i].index;

      dReal delta;

      {
        int b1 = jb[index*2];
        int b2 = jb[index*2+1];
        ac_ptr1 = ac + 6*b1;
        ac_ptr2 = (b2 >= 0) ? ac + 6*b2 : NULL;
        fc_ptr1 = fc + 6*b1;
        fc_ptr2 = (b2 >= 0) ? fc + 6*b2 : NULL;
      }

      dReal old_lambda = lambda[index];

      if (preconditioning) {
        // modify rhs_precon by adding J * M * vnew / stepsize
        // update delta
        delta = b_precon[index] /* + vnew[index] */ - old_lambda*Adcfm_precon[index];

        // ac is constraint acceleration in the non-precon case,
        //  and fc is the actual constraint force in the precon case
        // J_precon and J differs essentially in Ad and Ad_precon,
        //  Ad is derived from diagonal of J inv(M) J'
        //  Ad_precon is derived from diagonal of J J'
        dRealPtr J_ptr = J_precon + index*12;

        // for preconditioned case, update delta using fc, not ac

        delta -= dot6(fc_ptr1, J_ptr);
        if (fc_ptr2)
          delta -= dot6(fc_ptr2, J_ptr + 6);
      } else {
        delta = b[index] - old_lambda*Adcfm[index];
        // update vnew,
        //updateVnew(nb, invI, ac, body, stepsize, vnew);
        // compute J * M * vnew / stepsize and store in vnew
        //multiply_JM (nb, m, J, I, body, jb, stepsize, vnew);

        dRealPtr J_ptr = J + index*12;
        delta -= dot6(ac_ptr1, J_ptr);
        if (ac_ptr2) {
          delta -= dot6(ac_ptr2, J_ptr + 6);
        }
      }

      {
        dReal hi_act, lo_act;

        // set the limits for this constraint. 
        // this is the place where the QuickStep method differs from the
        // direct LCP solving method, since that method only performs this
        // limit adjustment once per time step, whereas this method performs
        // once per iteration per constraint row.
        // the constraints are ordered so that all lambda[] values needed have
        // already been computed.
        if (findex[index] >= 0) {
          hi_act = dFabs (hi[index] * lambda[findex[index]]);
          lo_act = -hi_act;
        } else {
          hi_act = hi[index];
          lo_act = lo[index];
        }

        // compute lambda and clamp it to [lo,hi].
        // @@@ SSE not a win here
#if 1
        dReal new_lambda = old_lambda + delta;
        if (new_lambda < lo_act) {
          delta = lo_act-old_lambda;
          lambda[index] = lo_act;
        }
        else if (new_lambda > hi_act) {
          delta = hi_act-old_lambda;
          lambda[index] = hi_act;
        }
        else {
          lambda[index] = new_lambda;
        }
#else
        dReal nl = old_lambda + delta;
        _mm_store_sd(&nl, _mm_max_sd(_mm_min_sd(_mm_load_sd(&nl), _mm_load_sd(&hi_act)), _mm_load_sd(&lo_act)));
        lambda[index] = nl;
        delta = nl - old_lambda;
#endif
      }

      // record error
      rms_error += delta*delta;
#ifdef RECOMPUTE_RMS
      delta_error[index] = dFabs(delta);
#endif

      //@@@ a trick that may or may not help
      //dReal ramp = (1-((dReal)(iteration+1)/(dReal)iterations));
      //delta *= ramp;
      
      {
        // no longer need iMJ, just need J' herea
        // to clarify, iMJ is really computed as the inv(M) * J without any further modification
        //if (preconditioning)
        {
          // for preconditioning case, compute fc
          dRealPtr J_ptr = J_orig + index*12; // FIXME: need un-altered unscaled J, not J_precon!!

          // update fc.
          sum6(fc_ptr1, delta, J_ptr);
          if (fc_ptr2)
            sum6(fc_ptr2, delta, J_ptr + 6);
        }
        //else
        {
          // for non-precon case, update ac
          dRealPtr iMJ_ptr = iMJ + index*12;

          // update ac.
          sum6(ac_ptr1, delta, iMJ_ptr);
          if (ac_ptr2)
            sum6(ac_ptr2, delta, iMJ_ptr + 6);
        }
      }
    } // end of for loop on m

// do we need to compute norm across entire solution space (0,m)?
// since local convergence might produce errors in other nodes?
#ifdef RECOMPUTE_RMS
    // recompute rms_error to be sure swap is not corrupting arrays
    rms_error = 0;
    #ifdef USE_1NORM
        //for (int i=startRow; i<startRow+nRows; i++)
        for (int i=0; i<m; i++)
        {
          rms_error = dFabs(delta_error[order[i].index]) > rms_error ? dFabs(delta_error[order[i].index]) : rms_error; // 1norm test
        }
    #else // use 2 norm
        //for (int i=startRow; i<startRow+nRows; i++)
        for (int i=0; i<m; i++)  // use entire solution vector errors
          rms_error += delta_error[order[i].index]*delta_error[order[i].index]; ///(dReal)nRows;
        rms_error = sqrt(rms_error/(dReal)nRows);
    #endif
#else
    rms_error = sqrt(rms_error/(dReal)nRows);
#endif

    //printf("------ %d %d %20.18f\n",thread_id,iteration,rms_error);

    //for (int i=startRow; i<startRow+nRows; i++) printf("debug: %d %f\n",i,delta_error[i]);


    //{
    //  // verify
    //  boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every row
    //  printf("  random id %d iter %d\n",thread_id,iteration);
    //  for (int i=startRow+1; i<startRow+nRows; i++)
    //    printf(" %10d,",i);
    //  printf("\n");
    //  for (int i=startRow+1; i<startRow+nRows; i++)
    //    printf(" %10d;",order[i].index);
    //  printf("\n");
    //  for (int i=startRow+1; i<startRow+nRows; i++)
    //    printf(" %10.8f,",delta_error[i]);
    //  printf("\n%f\n",rms_error);
    //}

#ifdef SHOW_CONVERGENCE
    printf("MONITOR: id: %d iteration: %d error: %20.16f\n",thread_id,iteration,rms_error);
#endif

    if (rms_error < sor_lcp_tolerance)
    {
      #ifdef REPORT_MONITOR
        printf("CONVERGED: id: %d steps: %d rms(%20.18f < %20.18f)\n",thread_id,iteration,rms_error,sor_lcp_tolerance);
      #endif
      if (iteration < precon_iterations) iteration = precon_iterations; // goto non-precon step
      else                               break;                         // finished
    }
    else if (iteration == num_iterations + precon_iterations -1)
    {
      #ifdef REPORT_MONITOR
        printf("WARNING: id: %d did not converge in %d steps, rms(%20.18f > %20.18f)\n",thread_id,num_iterations,rms_error,sor_lcp_tolerance);
      #endif
    }

  } // end of for loop on iterations

  qs->rms_error          = rms_error;

  #ifdef REPORT_THREAD_TIMING
  gettimeofday(&tv,NULL);
  double end_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  printf("      quickstep row thread %d start time %f ended time %f duration %f\n",thread_id,cur_time,end_time,end_time - cur_time);
  #endif
}

static void SOR_LCP (dxWorldProcessContext *context,
  const int m, const int nb, dRealMutablePtr J, dRealMutablePtr J_precon, dRealMutablePtr J_orig, int *jb, dxBody * const *body,
  dRealPtr invI, dRealPtr I, dRealMutablePtr lambda, dRealMutablePtr ac, dRealMutablePtr fc, dRealMutablePtr b, dRealMutablePtr b_precon,
  dRealPtr lo, dRealPtr hi, dRealPtr cfm, const int *findex,
  dxQuickStepParameters *qs,
  dRealMutablePtr /*JiM*/, dRealMutablePtr JiMratio,
#ifdef USE_TPROW
  boost::threadpool::pool* row_threadpool,
#endif
  const dReal stepsize)
{
#ifdef WARM_STARTING
  {
    // for warm starting, this seems to be necessary to prevent
    // jerkiness in motor-driven joints. i have no idea why this works.
    for (int i=0; i<m; i++) lambda[i] *= 0.9;
  }
#else
  dSetZero (lambda,m);
#endif

  // precompute iMJ = inv(M)*J'
  dReal *iMJ = context->AllocateArray<dReal> (m*12);
  compute_invM_JT (m,J,iMJ,jb,body,invI);

  // compute fc=(inv(M)*J')*lambda. we will incrementally maintain fc
  // as we change lambda.
#ifdef WARM_STARTING
  multiply_invM_JT (m,nb,J,jb,lambda,fc);
  multiply_invM_JT (m,nb,iMJ,jb,lambda,ac);
#else
  dSetZero (ac,nb*6);
  dSetZero (fc,nb*6);
#endif

  dReal *Ad = context->AllocateArray<dReal> (m);

  {
    const dReal sor_w = qs->w;		// SOR over-relaxation parameter
    // precompute 1 / diagonals of A
    dRealPtr iMJ_ptr = iMJ;
    dRealPtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
      dReal sum = 0;
      for (int j=0; j<6; j++) sum += iMJ_ptr[j] * J_ptr[j];
      if (jb[i*2+1] >= 0) {
        for (int k=6; k<12; k++) sum += iMJ_ptr[k] * J_ptr[k];
      }
      Ad[i] = sor_w / (sum + cfm[i]);
    }
  }

  // recompute Ad for preconditioned case, Ad_precon is similar to Ad but
  //   whereas Ad is 1 over diagonals of J inv(M) J'
  //    Ad_precon is 1 over diagonals of J J'
  dReal *Ad_precon = context->AllocateArray<dReal> (m);

  {
    const dReal sor_w = qs->w;		// SOR over-relaxation parameter
    // precompute 1 / diagonals of A
    // preconditioned version uses J instead of iMJ
    dRealPtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, i++) {
      dReal sum = 0;
      for (int j=0; j<6; j++) sum += J_ptr[j] * J_ptr[j];
      if (jb[i*2+1] >= 0) {
        for (int k=6; k<12; k++) sum += J_ptr[k] * J_ptr[k];
      }
      Ad_precon[i] = sor_w / (sum + cfm[i]);
    }
  }

  dReal *vnew = context->AllocateArray<dReal> (nb*6);

  /********************************/
  /* allocate for Adcfm           */
  /* which is a mX1 column vector */
  /********************************/
  // compute Adcfm_precon for the preconditioned case
  //   do this first before J gets altered (J's diagonals gets premultiplied by Ad)
  //   and save a copy of J into J_orig
  //   as J becomes J * Ad, J_precon becomes J * Ad_precon
  dReal *Adcfm_precon = context->AllocateArray<dReal> (m);


  {
    // NOTE: This may seem unnecessary but it's indeed an optimization 
    // to move multiplication by Ad[i] and cfm[i] out of iteration loop.

    // scale J_precon and b_precon by Ad
    // copy J_orig
    dRealMutablePtr J_ptr = J;
    dRealMutablePtr J_precon_ptr = J_precon;
    dRealMutablePtr J_orig_ptr = J_orig;
    for (int i=0; i<m; J_ptr += 12, J_precon_ptr += 12, J_orig_ptr += 12, i++) {
      dReal Ad_precon_i = Ad_precon[i];
      for (int j=0; j<12; j++) {
        J_precon_ptr[j] = J_ptr[j] * Ad_precon_i;
        J_orig_ptr[j] = J_ptr[j]; //copy J
      }
      b_precon[i] *= Ad_precon_i;
      // scale Ad by CFM. N.B. this should be done last since it is used above
      Adcfm_precon[i] = Ad_precon_i * cfm[i];
    }
  }

  dReal *Adcfm = context->AllocateArray<dReal> (m);


  {
    // NOTE: This may seem unnecessary but it's indeed an optimization 
    // to move multiplication by Ad[i] and cfm[i] out of iteration loop.

    // scale J and b by Ad
    dRealMutablePtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, i++) {
      dReal Ad_i = Ad[i];
      for (int j=0; j<12; j++) {
        J_ptr[j] *= Ad_i;
      }
      b[i] *= Ad_i;
      // scale Ad by CFM. N.B. this should be done last since it is used above
      Adcfm[i] = Ad_i * cfm[i];
    }
  }


  // order to solve constraint rows in
  IndexError *order = context->AllocateArray<IndexError> (m);

  dReal *delta_error = context->AllocateArray<dReal> (m);

#ifndef REORDER_CONSTRAINTS
  {
    // make sure constraints with findex < 0 come first.
    IndexError *orderhead = order, *ordertail = order + (m - 1);

    // Fill the array from both ends
    for (int i=0; i<m; i++) {
      if (findex[i] < 0) {
        orderhead->index = i; // Place them at the front
        ++orderhead;
      } else {
        ordertail->index = i; // Place them at the end
        --ordertail;
      }
    }
    dIASSERT (orderhead-ordertail==1);
  }
#endif

#ifdef REORDER_CONSTRAINTS
  // the lambda computed at the previous iteration.
  // this is used to measure error for when we are reordering the indexes.
  dReal *last_lambda = context->AllocateArray<dReal> (m);
#endif

  boost::recursive_mutex* mutex = new boost::recursive_mutex();

  // number of chunks must be at least 1
  // (single iteration, through all the constraints)
  int num_chunks = qs->num_chunks > 0 ? qs->num_chunks : 1; // min is 1

  // prepare pointers for threads
  dxSORLCPParameters *params = new dxSORLCPParameters [num_chunks];

  // divide into chunks sequentially
  int chunk = m / num_chunks+1;
  chunk = chunk > 0 ? chunk : 1;
  int thread_id = 0;


  // timing
  struct timeval tv;
  double cur_time;
  gettimeofday(&tv,NULL);
  cur_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("    quickstep start threads at time %f\n",cur_time);


  IFTIMING (dTimerNow ("start pgs rows"));
  for (int i=0; i<m; i+= chunk,thread_id++)
  {
    //for (int ijk=0;ijk<m;ijk++) printf("thread_id> id:%d jb[%d]=%d\n",thread_id,ijk,jb[ijk]);

    int nStart = i - qs->num_overlap < 0 ? 0 : i - qs->num_overlap;
    int nEnd   = i + chunk + qs->num_overlap;
    if (nEnd > m) nEnd = m;
    // if every one reorders constraints, this might just work
    // comment out below if using defaults (0 and m) so every thread runs through all joints
    params[thread_id].qs  = qs ;
    params[thread_id].nStart = nStart;   // 0
    params[thread_id].nChunkSize = nEnd - nStart; // m
    params[thread_id].m = m; // m
    params[thread_id].nb = nb;
    params[thread_id].stepsize = stepsize;
    params[thread_id].jb = jb;
    params[thread_id].findex = findex;
    params[thread_id].Ad = Ad;
    params[thread_id].hi = hi;
    params[thread_id].lo = lo;
    params[thread_id].Adcfm = Adcfm;
    params[thread_id].invI = invI;
    params[thread_id].I= I;
    params[thread_id].Ad_precon = Ad_precon;
    params[thread_id].Adcfm_precon = Adcfm_precon;
    params[thread_id].vnew = vnew;
    params[thread_id].JiMratio = JiMratio;
    params[thread_id].b = b;
    params[thread_id].J = J;
    params[thread_id].ac = ac;
    params[thread_id].lambda = lambda;
    params[thread_id].iMJ = iMJ;
    params[thread_id].delta_error  = delta_error ;
    params[thread_id].b_precon  = b_precon ;
    params[thread_id].J_precon  = J_precon ;
    params[thread_id].J_orig  = J_orig ;
    params[thread_id].fc  = fc ;
#ifdef REORDER_CONSTRAINTS
    params[thread_id].last_lambda  = last_lambda ;
#endif

#ifdef REPORT_MONITOR
    printf("thread summary: id %d i %d m %d chunk %d start %d end %d \n",thread_id,i,m,chunk,nStart,nEnd);
#endif
#ifdef USE_TPROW
    if (row_threadpool && row_threadpool->size() > 0)
      row_threadpool->schedule(boost::bind(ComputeRows,thread_id,order, body, params[thread_id], mutex));
    else //automatically skip threadpool if only 1 thread allocated
      ComputeRows(thread_id,order, body, params[thread_id], mutex);
#else
    ComputeRows(thread_id,order, body, params[thread_id], mutex);
#endif
  }


  // check time for scheduling, this is usually very quick
  //gettimeofday(&tv,NULL);
  //double wait_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("      quickstep done scheduling start time %f stopped time %f duration %f\n",cur_time,wait_time,wait_time - cur_time);

#ifdef USE_TPROW
  IFTIMING (dTimerNow ("wait for threads"));
  if (row_threadpool && row_threadpool->size() > 0)
    row_threadpool->wait();
  IFTIMING (dTimerNow ("threads done"));
#endif



  #ifdef REPORT_THREAD_TIMING
  gettimeofday(&tv,NULL);
  double end_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  printf("    quickstep threads start time %f stopped time %f duration %f\n",cur_time,end_time,end_time - cur_time);
  #endif

  delete [] params;
  delete mutex;
}

struct dJointWithInfo1
{
  dxJoint *joint;
  dxJoint::Info1 info;
};

void dxQuickStepper (dxWorldProcessContext *context, 
  dxWorld *world, dxBody * const *body, int nb,
  dxJoint * const *_joint, int _nj, dReal stepsize)
{
  IFTIMING(dTimerStart("preprocessing"));

  const dReal stepsize1 = dRecip(stepsize);

  {
    // number all bodies in the body list - set their tag values
    for (int i=0; i<nb; i++) body[i]->tag = i;
  }

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
  dReal *invI = context->AllocateArray<dReal> (3*4*nb);
  dReal *I = context->AllocateArray<dReal> (3*4*nb);

  {
    dReal *invIrow = invI;
    dReal *Irow = I;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow += 12, Irow += 12, bodycurr++) {
      dMatrix3 tmp;
      dxBody *b_ptr = *bodycurr;

      // compute inverse inertia tensor in global frame
      dMultiply2_333 (tmp,b_ptr->invI,b_ptr->posr.R);
      dMultiply0_333 (invIrow,b_ptr->posr.R,tmp);

      // also store I for later use by preconditioner
      dMultiply2_333 (tmp,b_ptr->mass.I,b_ptr->posr.R);
      dMultiply0_333 (Irow,b_ptr->posr.R,tmp);

      if (b_ptr->flags & dxBodyGyroscopic) {
        // compute rotational force
        dMultiply0_331 (tmp,Irow,b_ptr->avel);
        dSubtractVectorCross3(b_ptr->tacc,b_ptr->avel,tmp);
      }
    }
  }

  // get the masses for every body
  dReal *invM = context->AllocateArray<dReal> (nb);
  {
    dReal *invMrow = invM;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invMrow++, bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      //*invMrow = b_ptr->mass.mass;
      *invMrow = b_ptr->invMass;

    }
  }


  {
    // add the gravity force to all bodies
    // since gravity does normally have only one component it's more efficient
    // to run three loops for each individual component
    dxBody *const *const bodyend = body + nb;
    dReal gravity_x = world->gravity[0];
    if (gravity_x) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        if ((b_ptr->flags & dxBodyNoGravity)==0) {
          b_ptr->facc[0] += b_ptr->mass.mass * gravity_x;
        }
      }
    }
    dReal gravity_y = world->gravity[1];
    if (gravity_y) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        if ((b_ptr->flags & dxBodyNoGravity)==0) {
          b_ptr->facc[1] += b_ptr->mass.mass * gravity_y;
        }
      }
    }
    dReal gravity_z = world->gravity[2];
    if (gravity_z) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        if ((b_ptr->flags & dxBodyNoGravity)==0) {
          b_ptr->facc[2] += b_ptr->mass.mass * gravity_z;
        }
      }
    }
  }

  // get joint information (m = total constraint dimension, nub = number of unbounded variables).
  // joints with m=0 are inactive and are removed from the joints array
  // entirely, so that the code that follows does not consider them.
  dJointWithInfo1 *const jointiinfos = context->AllocateArray<dJointWithInfo1> (_nj);
  int nj;
  
  {
    dJointWithInfo1 *jicurr = jointiinfos;
    dxJoint *const *const _jend = _joint + _nj;
    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; _jcurr++) {	// jicurr=dest, _jcurr=src
      dxJoint *j = *_jcurr;
      j->getInfo1 (&jicurr->info);
      dIASSERT (jicurr->info.m >= 0 && jicurr->info.m <= 6 && jicurr->info.nub >= 0 && jicurr->info.nub <= jicurr->info.m);
      if (jicurr->info.m > 0) {
        jicurr->joint = j;
        jicurr++;
      }
    }
    nj = jicurr - jointiinfos;
  }

  context->ShrinkArray<dJointWithInfo1>(jointiinfos, _nj, nj);

  int m;
  int mfb; // number of rows of Jacobian we will have to save for joint feedback

  {
    int mcurr = 0, mfbcurr = 0;
    const dJointWithInfo1 *jicurr = jointiinfos;
    const dJointWithInfo1 *const jiend = jicurr + nj;
    for (; jicurr != jiend; jicurr++) {
      int jm = jicurr->info.m;
      mcurr += jm;
      if (jicurr->joint->feedback)
        mfbcurr += jm;
    }

    m = mcurr;
    mfb = mfbcurr;
  }

  // if there are constraints, compute the constraint force
  dReal *J = NULL;
  dReal *J_precon = NULL;
  dReal *J_orig = NULL;
  dReal *JiM = NULL;
  dReal *JiMratio = NULL;
  int *jb = NULL;

  if (m > 0) {
    dReal *cfm, *lo, *hi, *rhs, *rhs_precon, *Jcopy;
    int *findex;

    {
      int mlocal = m;

      const unsigned jelements = mlocal*12;
      J = context->AllocateArray<dReal> (jelements);
      dSetZero (J,jelements);
      J_precon = context->AllocateArray<dReal> (jelements);
      J_orig = context->AllocateArray<dReal> (jelements);

      // create a constraint equation right hand side vector `c', a constraint
      // force mixing vector `cfm', and LCP low and high bound vectors, and an
      // 'findex' vector.
      cfm = context->AllocateArray<dReal> (mlocal);
      dSetValue (cfm,mlocal,world->global_cfm);

      lo = context->AllocateArray<dReal> (mlocal);
      dSetValue (lo,mlocal,-dInfinity);

      hi = context->AllocateArray<dReal> (mlocal);
      dSetValue (hi,mlocal, dInfinity);

      findex = context->AllocateArray<int> (mlocal);
      for (int i=0; i<mlocal; i++) findex[i] = -1;

      const unsigned jbelements = mlocal*2;
      jb = context->AllocateArray<int> (jbelements);

      rhs = context->AllocateArray<dReal> (mlocal);
      rhs_precon = context->AllocateArray<dReal> (mlocal);

      Jcopy = context->AllocateArray<dReal> (mfb*12);
      JiM = context->AllocateArray<dReal> (mlocal*12); // not used
      dSetZero (JiM,jelements);
      // JiMratio is the element-wise maximum ratio between
      //   Ja*inv(Ma) and Jb*inv(Mb) for a joint
      // if the joint is one sided, then we preset it to 1
      JiMratio = context->AllocateArray<dReal> (mlocal); // test variable
      dSetZero (JiMratio,mlocal);
    }

    BEGIN_STATE_SAVE(context, cstate) {
      dReal *c = context->AllocateArray<dReal> (m);
      dSetZero (c, m);

      {
        IFTIMING (dTimerNow ("create J"));
        // get jacobian data from constraints. an m*12 matrix will be created
        // to store the two jacobian blocks from each constraint. it has this
        // format:
        //
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 \    .
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2  )-- jacobian for joint 0, body 1 and body 2 (3 rows)
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 /
        //   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 )--- jacobian for joint 1, body 1 and body 2 (3 rows)
        //   etc...
        //
        //   (lll) = linear jacobian data
        //   (aaa) = angular jacobian data
        //
        dxJoint::Info2 Jinfo;
        Jinfo.rowskip = 12;
        Jinfo.fps = stepsize1;
        Jinfo.erp = world->global_erp;

        dReal *Jcopyrow = Jcopy;
        unsigned ofsi = 0;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; jicurr++) {
          dReal *const Jrow = J + ofsi * 12;
          Jinfo.J1l = Jrow;
          Jinfo.J1a = Jrow + 3;
          Jinfo.J2l = Jrow + 6;
          Jinfo.J2a = Jrow + 9;
          Jinfo.c = c + ofsi;
          Jinfo.cfm = cfm + ofsi;
          Jinfo.lo = lo + ofsi;
          Jinfo.hi = hi + ofsi;
          Jinfo.findex = findex + ofsi;

          // now write all information into J
          dxJoint *joint = jicurr->joint;
          joint->getInfo2 (&Jinfo);

          const int infom = jicurr->info.m;

          // we need a copy of Jacobian for joint feedbacks
          // because it gets destroyed by SOR solver
          // instead of saving all Jacobian, we can save just rows
          // for joints, that requested feedback (which is normally much less)
          if (joint->feedback) {
            const int rowels = infom * 12;
            memcpy(Jcopyrow, Jrow, rowels * sizeof(dReal));
            Jcopyrow += rowels;
          }

          // adjust returned findex values for global index numbering
          int *findex_ofsi = findex + ofsi;
          for (int j=0; j<infom; j++) {
            int fival = findex_ofsi[j];
            if (fival >= 0) 
              findex_ofsi[j] = fival + ofsi;
          }

          ofsi += infom;
        }
      }

      {
        // create an array of body numbers for each joint row
        int *jb_ptr = jb;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; jicurr++) {
          dxJoint *joint = jicurr->joint;
          const int infom = jicurr->info.m;

          int b1 = (joint->node[0].body) ? (joint->node[0].body->tag) : -1;
          int b2 = (joint->node[1].body) ? (joint->node[1].body->tag) : -1;
          for (int j=0; j<infom; j++) {
            jb_ptr[0] = b1;
            jb_ptr[1] = b2;
            jb_ptr += 2;
          }
        }
        dIASSERT (jb_ptr == jb+2*m);
        //printf("jjjjjjjjj %d %d\n",jb[0],jb[1]);
      }





      BEGIN_STATE_SAVE(context, tmp1state) {
        IFTIMING (dTimerNow ("compute rhs"));
        // compute the right hand side `rhs'
        dReal *tmp1 = context->AllocateArray<dReal> (nb*6);
        // put v/h + invM*fe into tmp1
        dReal *tmp1curr = tmp1;
        const dReal *invIrow = invI;
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; tmp1curr+=6, invIrow+=12, bodycurr++) {
          dxBody *b_ptr = *bodycurr;
          dReal body_invMass = b_ptr->invMass;
          for (int j=0; j<3; j++) tmp1curr[j] = b_ptr->facc[j] * body_invMass + b_ptr->lvel[j] * stepsize1;
          dMultiply0_331 (tmp1curr + 3,invIrow,b_ptr->tacc);
          for (int k=0; k<3; k++) tmp1curr[3+k] += b_ptr->avel[k] * stepsize1;
        }

        // put J*tmp1 into rhs
        multiply_J (m,J,jb,tmp1,rhs);
        /*************************************************************/
        /* compute J*inv(M) here JiM, it does not change             */
        /*************************************************************/
        {
          dRealPtr J_ptr = J;
          dRealMutablePtr JiM_ptr = JiM; // intermediate solution storage
          dRealMutablePtr JiMratio_ptr = JiMratio; // intermediate solution storage
          for (int i=0; i<m;J_ptr+=12,JiM_ptr+=12, i++) {

            // compute JiM = J * invM
            int b1 = jb[i*2];
            int b2 = jb[i*2+1];
            dReal k1 = body[b1]->invMass;

            for (int j=0; j<3 ; j++) JiM_ptr[j] = J_ptr[j]*k1;


            const dReal *invI_ptr1 = invI + 12*b1;
            for (int j=0;j<3;j++) {
              JiM_ptr[3+j] = 0;
              for (int k=0;k<3;k++){
                JiM_ptr[3+j] += J_ptr[3+k]*invI_ptr1[k*4+j];
              }
            }

            // preset JiMratio to 1
            JiMratio_ptr[0] = 1.0;

            if (b2 >= 0){
              dReal k2 = body[b2]->invMass;
              for (int j=0; j<3 ; j++) JiM_ptr[j+6] = k2*J_ptr[j+6];
              const dReal *invI_ptr2 = invI + 12*b2;
              for (int j=0;j<3;j++) {
                JiM_ptr[9+j] = 0;
                for (int k=0;k<3;k++) JiM_ptr[9+j] += J_ptr[9+k]*invI_ptr2[k*4+j];
              }

              // check element-wise ratio for JiMratio
              JiMratio_ptr[0] = 1.0;
              for (int j=0;j<3;j++) {
                if  (JiM_ptr[  j] != 0) JiMratio_ptr[0] = std::max(JiMratio_ptr[0],JiM_ptr[6+j]/JiM_ptr[  j]);
                if  (JiM_ptr[3+j] != 0) JiMratio_ptr[0] = std::max(JiMratio_ptr[0],JiM_ptr[9+j]/JiM_ptr[3+j]);
                if  (JiM_ptr[6+j] != 0) JiMratio_ptr[0] = std::max(JiMratio_ptr[0],JiM_ptr[  j]/JiM_ptr[6+j]);
                if  (JiM_ptr[9+j] != 0) JiMratio_ptr[0] = std::max(JiMratio_ptr[0],JiM_ptr[3+j]/JiM_ptr[9+j]);
              }
            }
          }
        }
      
      } END_STATE_SAVE(context, tmp1state);

      // complete rhs
      for (int i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];







      // compute rhs_precon
      computeRHSPrecon(context, m, nb, invI, I, body, stepsize1, c, J, jb, rhs_precon);












      // scale CFM
      for (int j=0; j<m; j++) cfm[j] *= stepsize1;

    } END_STATE_SAVE(context, cstate);

    // load lambda from the value saved on the previous iteration
    dReal *lambda = context->AllocateArray<dReal> (m);

#ifdef WARM_STARTING
    {
      dReal *lambdscurr = lambda;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; jicurr++) {
        int infom = jicurr->info.m;
        memcpy (lambdscurr, jicurr->joint->lambda, infom * sizeof(dReal));
        lambdscurr += infom;
      }
    }
#endif

    dReal *cforce = context->AllocateArray<dReal> (nb*6);
    dReal *caccel = context->AllocateArray<dReal> (nb*6);

    BEGIN_STATE_SAVE(context, lcpstate) {
      IFTIMING (dTimerNow ("solving LCP problem"));
      // solve the LCP problem and get lambda and invM*constraint_force
      SOR_LCP (context,m,nb,J,J_precon,J_orig,jb,body,invI,I,lambda,caccel,cforce,rhs,rhs_precon,lo,hi,cfm,findex,&world->qs,
      JiM,JiMratio,
#ifdef USE_TPROW
               world->row_threadpool,
#endif
               stepsize);

    } END_STATE_SAVE(context, lcpstate);

#ifdef WARM_STARTING
    {
      // save lambda for the next iteration
      //@@@ note that this doesn't work for contact joints yet, as they are
      // recreated every iteration
      const dReal *lambdacurr = lambda;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; jicurr++) {
        int infom = jicurr->info.m;
        memcpy (jicurr->joint->lambda, lambdacurr, infom * sizeof(dReal));
        lambdacurr += infom;
      }
    }
#endif

    // note that the SOR method overwrites rhs and J at this point, so
    // they should not be used again.

    {
      IFTIMING (dTimerNow ("velocity update due to constraint forces"));
      // note that cforce is really not a force but an acceleration, hence there is
      // no premultiplying of invM here (compare to update due to external force 'facc' below)
      //
      // add stepsize * cforce to the body velocity
      // or better named,
      // add stepsize * caccel to the body velocity
      //
      const dReal *caccelcurr = caccel;
      dxBody *const *const bodyend = body + nb;
      // update assuming caccel is already acceleration
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; caccelcurr+=6, bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        for (int j=0; j<3; j++) {
          b_ptr->lvel[j] += stepsize * caccelcurr[j];
          b_ptr->avel[j] += stepsize * caccelcurr[3+j];
        }
      }
    }

    if (mfb > 0) {
      // straightforward computation of joint constraint forces:
      // multiply related lambdas with respective J' block for joints
      // where feedback was requested
      dReal data[6];
      const dReal *lambdacurr = lambda;
      const dReal *Jcopyrow = Jcopy;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; jicurr++) {
        dxJoint *joint = jicurr->joint;
        const int infom = jicurr->info.m;

        if (joint->feedback) {
          dJointFeedback *fb = joint->feedback;
          Multiply1_12q1 (data, Jcopyrow, lambdacurr, infom);
          fb->f1[0] = data[0];
          fb->f1[1] = data[1];
          fb->f1[2] = data[2];
          fb->t1[0] = data[3];
          fb->t1[1] = data[4];
          fb->t1[2] = data[5];

          if (joint->node[1].body)
          {
            Multiply1_12q1 (data, Jcopyrow+6, lambdacurr, infom);
            fb->f2[0] = data[0];
            fb->f2[1] = data[1];
            fb->f2[2] = data[2];
            fb->t2[0] = data[3];
            fb->t2[1] = data[4];
            fb->t2[2] = data[5];
          }
          
          Jcopyrow += infom * 12;
        }
      
        lambdacurr += infom;
      }
    }
  }

  {
    IFTIMING (dTimerNow ("compute velocity update"));
    // compute the velocity update:
    // add stepsize * invM * fe to the body velocity
    const dReal *invIrow = invI;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow += 12, bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      dReal body_invMass_mul_stepsize = stepsize * b_ptr->invMass;
      for (int j=0; j<3; j++) {
        b_ptr->lvel[j] += body_invMass_mul_stepsize * b_ptr->facc[j];
        b_ptr->tacc[j] *= stepsize;
      }
      dMultiplyAdd0_331 (b_ptr->avel, invIrow, b_ptr->tacc);
    }
  }

#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
  if (m > 0) {
    BEGIN_STATE_SAVE(context, velstate) {
      dReal *vel = context->AllocateArray<dReal>(nb*6);

      // check that the updated velocity obeys the constraint (this check needs unmodified J)
      dReal *velcurr = vel;
      dxBody *bodycurr = body, *const bodyend = body + nb;
      for (; bodycurr != bodyend; velcurr += 6, bodycurr++) {
        for (int j=0; j<3; j++) {
          velcurr[j] = bodycurr->lvel[j];
          velcurr[3+j] = bodycurr->avel[j];
        }
      }
      dReal *tmp = context->AllocateArray<dReal> (m);
      multiply_J (m,J,jb,vel,tmp);
      dReal error = 0;
      for (int i=0; i<m; i++) error += dFabs(tmp[i]);
      printf ("velocity error = %10.6e\n",error);
    
    } END_STATE_SAVE(context, velstate)
  }
#endif

  {
    // update the position and orientation from the new linear/angular velocity
    // (over the given timestep)
    IFTIMING (dTimerNow ("update position"));
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      dxStepBody (b_ptr,stepsize);
    }
  }

  {
    IFTIMING (dTimerNow ("tidy up"));
    // zero all force accumulators
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      dSetZero (b_ptr->facc,3);
      dSetZero (b_ptr->tacc,3);
    }
  }

  IFTIMING (dTimerEnd());
  IFTIMING (if (m > 0) dTimerReport (stdout,1));
}

#ifdef USE_CG_LCP
static size_t EstimateGR_LCPMemoryRequirements(int m)
{
  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ
  res += 5 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for r, z, p, q, Ad
  return res;
}
#endif

static size_t EstimateSOR_LCPMemoryRequirements(int m,int nb)
{
  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Ad
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Adcfm
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Ad_precon
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Adcfm_precon
  res += dEFFICIENT_SIZE(sizeof(dReal) * nb*6); // for vnew
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for delta_error
  res += dEFFICIENT_SIZE(sizeof(IndexError) * m); // for order
#ifdef REORDER_CONSTRAINTS
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for last_lambda
#endif
  return res;
}

size_t dxEstimateQuickStepMemoryRequirements (
  dxBody * const * /*body*/, int nb, dxJoint * const *_joint, int _nj)
{
  int nj, m, mfb;

  {
    int njcurr = 0, mcurr = 0, mfbcurr = 0;
    dxJoint::SureMaxInfo info;
    dxJoint *const *const _jend = _joint + _nj;
    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; _jcurr++) {	
      dxJoint *j = *_jcurr;
      j->getSureMaxInfo (&info);
      
      int jm = info.max_m;
      if (jm > 0) {
        njcurr++;

        mcurr += jm;
        if (j->feedback)
          mfbcurr += jm;
      }
    }
    nj = njcurr; m = mcurr; mfb = mfbcurr;
  }

  size_t res = 0;

  res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb); // for invI
  res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb); // for I needed by preconditioner
  res += dEFFICIENT_SIZE(sizeof(dReal) * nb); // for invM

  {
    size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * _nj); // for initial jointiinfos

    size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * nj); // for shrunk jointiinfos
    if (m > 0) {
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J_precon
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J_orig
      sub1_res2 += 4 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for cfm, lo, hi, rhs
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for rhs_precon
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * 2 * m); // for jb            FIXME: shoulbe be 2 not 12?
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * mfb); // for Jcopy
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for JiM
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for JiMratio
      {
        size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for c
        //sub2_res1 += dEFFICIENT_SIZE(sizeof(int)   * 6 * nb); // for JTJfindex
        //sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb * 6 * nb); // for JTJ
        //sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for JTrhs
        sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for invJrhs
        sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for invJrhs_updated
        {
          size_t sub3_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for tmp1
    
          size_t sub3_res2 = 0;

          sub2_res1 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        size_t sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for cforce
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for caccel
        {
          size_t sub3_res1 = EstimateSOR_LCPMemoryRequirements(m,nb); // for SOR_LCP

          size_t sub3_res2 = 0;
#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
          {
            size_t sub4_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for vel
            sub4_res1 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for tmp

            size_t sub4_res2 = 0;

            sub3_res2 += (sub4_res1 >= sub4_res2) ? sub4_res1 : sub4_res2;
          }
#endif
          sub2_res2 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        sub1_res2 += (sub2_res1 >= sub2_res2) ? sub2_res1 : sub2_res2;
      }
    }
    
    res += (sub1_res1 >= sub1_res2) ? sub1_res1 : sub1_res2;
  }

  return res;
}


