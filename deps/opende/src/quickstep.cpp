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
#undef DEBUG_CONVERGENCE_TOLERANCE
#undef SHOW_CONVERGENCE
#define SMOOTH_LAMBDA
#undef USE_1NORM
#undef DEBUG_INERTIA_PROPAGATION
//#define LOCAL_STEPPING  // not yet implemented
//#define PENETRATION_JVERROR_CORRECTION
//#define POST_UPDATE_CONSTRAINT_VIOLATION_CORRECTION

#undef CHECK_VELOCITY_OBEYS_CONSTRAINT


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
// warm starting:
// this definitely help for motor-driven joints.
// unfortunately it appears to hurt with high-friction contacts
// using the SOR method. use with care

// for the SOR method:
// uncomment the following line to determine a new constraint-solving
// order for each iteration. however, the qsort per iteration is expensive,
// and the optimal order is somewhat problem dependent.
// @@@ try the leaf->root ordering.

// #define REORDER_CONSTRAINTS 1

// for the SOR method:
// uncomment the following line to randomly reorder constraint rows
// during the solution. depending on the situation, this can help a lot
// or hardly at all, but it doesn't seem to hurt.

// #define RANDOMLY_REORDER_CONSTRAINTS 1
#undef LOCK_WHILE_RANDOMLY_REORDER_CONSTRAINTS

/// scale SOR for contact to reduce overshoot in solution for contacts
/// \TODO: make this a parameter
#define CONTACT_SOR_SCALE 0.25

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
    dRealPtr hi;
    dRealPtr lo;
    dRealPtr invMOI;
    dRealPtr MOI;
    dRealPtr Ad;
    dRealPtr Adcfm;
    dRealPtr Adcfm_precon;
    dRealMutablePtr rhs;
    dRealMutablePtr rhs_erp;
    dRealMutablePtr J;
    dRealMutablePtr caccel;
    dRealMutablePtr caccel_erp;
    dRealMutablePtr lambda;
    dRealMutablePtr lambda_erp;
    dRealMutablePtr iMJ;
    dRealMutablePtr rhs_precon ;
    dRealMutablePtr J_precon ;
    dRealMutablePtr J_orig ;
    dRealMutablePtr cforce ;
    dRealMutablePtr vnew ;
#ifdef REORDER_CONSTRAINTS
    dRealMutablePtr last_lambda ;
    dRealMutablePtr last_lambda_erp ;
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
  dxBody * const *body, dRealPtr invMOI)
{
  dRealMutablePtr iMJ_ptr = iMJ;
  dRealPtr J_ptr = J;
  for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal k1 = body[b1]->invMass;
    for (int j=0; j<3; j++) iMJ_ptr[j] = k1*J_ptr[j];
    const dReal *invMOIrow1 = invMOI + 12*b1;
    dMultiply0_331 (iMJ_ptr + 3, invMOIrow1, J_ptr + 3);
    if (b2 >= 0) {
      dReal k2 = body[b2]->invMass;
      for (int j=0; j<3; j++) iMJ_ptr[j+6] = k2*J_ptr[j+6];
      const dReal *invMOIrow2 = invMOI + 12*b2;
      dMultiply0_331 (iMJ_ptr + 9, invMOIrow2, J_ptr + 9);
    }
  }
}

// warm starting
// compute out = inv(M)*J'*in.
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
/* not used
// warm starting
static void multiply_J_invM_JT (int m, int nb, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
  dRealPtr cfm, dRealMutablePtr z, dRealMutablePtr in, dRealMutablePtr out)
{
  multiply_invM_JT (m,nb,iMJ,jb,in,z);
  multiply_J (m,J,jb,z,out);

  // add cfm
  for (int i=0; i<m; i++) out[i] += cfm[i] * in[i];
}
*/

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
  dRealPtr invMOI, dRealMutablePtr lambda, dRealMutablePtr cforce, dRealMutablePtr rhs,
  dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
  dxQuickStepParameters *qs)
{
  const int num_iterations = qs->num_iterations;

  // precompute iMJ = inv(M)*J'
  dReal *iMJ = context->AllocateArray<dReal> (m*12);
  compute_invM_JT (m,J,iMJ,jb,body,invMOI);

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

  if (qs->warm_start > 0)
  {
    // warm start
    // compute residual r = rhs - A*lambda
    multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,cforce,lambda,r);
    for (int k=0; k<m; k++) r[k] = rhs[k] - r[k];
  }
  else
  {
    dSetZero (lambda,m);
    memcpy (r,rhs,m*sizeof(dReal));    // residual r = rhs - A*lambda
  }

  for (int iteration=0; iteration < num_iterations; iteration++) {
    for (int i=0; i<m; i++) z[i] = r[i]*Ad[i];  // z = inv(M)*r
    dReal rho = dot (m,r,z);    // rho = r'*z

    // @@@
    // we must check for convergence, otherwise rho will go to 0 if
    // we get an exact solution, which will introduce NaNs into the equations.
    if (rho < 1e-10) {
      printf ("CG returned at iteration %d\n",iteration);
      break;
    }

    if (iteration==0) {
      memcpy (p,z,m*sizeof(dReal));  // p = z
    }
    else {
      add (m,p,z,p,rho/last_rho);  // p = z + (rho/last_rho)*p
    }

    // compute q = (J*inv(M)*J')*p
    multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,cforce,p,q);

    dReal alpha = rho/dot (m,p,q);    // alpha = rho/(p'*q)
    add (m,lambda,lambda,p,alpha);    // lambda = lambda + alpha*p
    add (m,r,r,q,-alpha);      // r = r - alpha*q
    last_rho = rho;
  }

  // compute cforce = inv(M)*J'*lambda
  multiply_invM_JT (m,nb,iMJ,jb,lambda,cforce);

#if 0
  // measure solution error
  multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,cforce,lambda,r);
  dReal error = 0;
  for (int i=0; i<m; i++) error += dFabs(r[i] - rhs[i]);
  printf ("lambda error = %10.6e\n",error);
#endif
}

#endif


struct IndexError {
#ifdef REORDER_CONSTRAINTS
  dReal error;    // error to sort on
  int findex;
#endif
  int index;    // row index
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

void computeRHSPrecon(dxWorldProcessContext *context, const int m, const int nb,
                      dRealPtr MOI, dxBody * const *body,
                      const dReal /*stepsize1*/, dRealMutablePtr /*c*/, dRealMutablePtr J,
                      int *jb, dRealMutablePtr rhs_precon)
{
    /************************************************************************************/
    /*                                                                                  */
    /*               compute preconditioned rhs                                         */
    /*                                                                                  */
    /*  J J' lambda = J * ( M * dv / dt + fe )                                          */
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
      const dReal *MOIrow = MOI;
      dxBody *const *const bodyend = body + nb;
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; tmp1curr+=6, MOIrow+=12, bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        // dReal body_mass = b_ptr->mass.mass;
        for (int j=0; j<3; j++)
          tmp1curr[j] = b_ptr->facc[j]; // +  body_mass * b_ptr->lvel[j] * stepsize1;
        dReal tmpa[3];
        for (int j=0; j<3; j++) tmpa[j] = 0; //b_ptr->avel[j] * stepsize1;
        dMultiply0_331 (tmp1curr + 3,MOIrow,tmpa);
        for (int k=0; k<3; k++) tmp1curr[3+k] += b_ptr->tacc[k];
      }
      //
      // rhs_precon = - J * (M*v/h + fe)
      //
      multiply_J (m,J,jb,tmp1,rhs_precon);

      //
      // no need to add constraint violation correction tterm if we assume acceleration is 0
      //
      for (int i=0; i<m; i++) rhs_precon[i] = - rhs_precon[i];


      /*  DEBUG PRINTOUTS
      printf("\n");
      for (int i=0; i<m; i++) printf("c[%d] = %f\n",i,c[i]);
      printf("\n");
      */
    } END_STATE_SAVE(context, tmp2state);
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
#ifdef SHOW_CONVERGENCE
                int thread_id,
#else
                int /*thread_id*/,
#endif
                IndexError* order,
                dxBody* const * /*body*/,
                dxSORLCPParameters params,
                boost::recursive_mutex* /*mutex*/)
{

  #ifdef REPORT_THREAD_TIMING
  struct timeval tv;
  double cur_time;
  gettimeofday(&tv,NULL);
  cur_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("thread %d started at time %f\n",thread_id,cur_time);
  #endif

  //boost::recursive_mutex::scoped_lock lock(*mutex); // put in caccel read/writes?
  dxQuickStepParameters *qs    = params.qs;
  int startRow                 = params.nStart;   // 0
  int nRows                    = params.nChunkSize; // m
#ifdef USE_1NORM
  int m                        = params.m; // m used for rms error computation
#endif
  // int nb                       = params.nb;
#ifdef PENETRATION_JVERROR_CORRECTION
  dReal stepsize               = params.stepsize;
  dRealMutablePtr vnew         = params.vnew;
#endif
  int* jb                      = params.jb;
  const int* findex            = params.findex;
  dRealPtr        hi           = params.hi;
  dRealPtr        lo           = params.lo;
  dRealPtr        Ad           = params.Ad;
  dRealPtr        Adcfm        = params.Adcfm;
  dRealPtr        Adcfm_precon = params.Adcfm_precon;
  dRealMutablePtr rhs          = params.rhs;
  dRealMutablePtr rhs_erp      = params.rhs_erp;
  dRealMutablePtr J            = params.J;
  dRealMutablePtr caccel       = params.caccel;
  dRealMutablePtr caccel_erp   = params.caccel_erp;
  dRealMutablePtr lambda       = params.lambda;
  dRealMutablePtr lambda_erp   = params.lambda_erp;
  dRealMutablePtr iMJ          = params.iMJ;
  dRealMutablePtr rhs_precon   = params.rhs_precon;
  dRealMutablePtr J_precon     = params.J_precon;
  dRealMutablePtr J_orig       = params.J_orig;
  dRealMutablePtr cforce       = params.cforce;
#ifdef REORDER_CONSTRAINTS
  dRealMutablePtr last_lambda  = params.last_lambda;
  dRealMutablePtr last_lambda_erp  = params.last_lambda_erp;
#endif

  //printf("iiiiiiiii %d %d %d\n",thread_id,jb[0],jb[1]);
  //for (int i=startRow; i<startRow+nRows; i++) // swap within boundary of our own segment
  //  printf("wwwwwwwwwwwww>id %d start %d n %d  order[%d].index=%d\n",thread_id,startRow,nRows,i,order[i].index);



  /*  DEBUG PRINTOUTS
  // print J_orig
  printf("J_orig\n");
  for (int i=startRow; i<startRow+nRows; i++) {
    for (int j=0; j < 12 ; j++) {
      printf("  %12.6f",J_orig[i*12+j]);
    }
    printf("\n");
  }
  printf("\n");

  // print J, J_precon (already premultiplied by inverse of diagonal of LHS) and rhs_precon and rhs
  printf("J_precon\n");
  for (int i=startRow; i<startRow+nRows; i++) {
    for (int j=0; j < 12 ; j++) {
      printf("  %12.6f",J_precon[i*12+j]);
    }
    printf("\n");
  }
  printf("\n");

  printf("J\n");
  for (int i=startRow; i<startRow+nRows; i++) {
    for (int j=0; j < 12 ; j++) {
      printf("  %12.6f",J[i*12+j]);
    }
    printf("\n");
  }
  printf("\n");

  printf("rhs_precon\n");
  for (int i=startRow; i<startRow+nRows; i++)
    printf("  %12.6f",rhs_precon[i]);
  printf("\n");

  printf("rhs\n");
  for (int i=startRow; i<startRow+nRows; i++)
    printf("  %12.6f",rhs[i]);
  printf("\n");
  */

  // m_rms_dlambda[3] keeps track of number of constraint
  // rows per type of constraint.
  // m_rms_dlambda[0]: bilateral constraints (findex = -1)
  // m_rms_dlambda[1]: contact normal constraints (findex = -2)
  // rm_ms_dlambda[2]: friction constraints (findex >= 0)
  int m_rms_dlambda[3];
  m_rms_dlambda[0] = 0;
  m_rms_dlambda[1] = 0;
  m_rms_dlambda[2] = 0;

  // rms of dlambda
  dReal rms_dlambda[4];
  dSetZero(rms_dlambda, 4);
  // rms of b_i - A_ij \lambda_j as we sweep through rows
  dReal rms_error[4];
  dSetZero(rms_error, 4);

  int num_iterations = qs->num_iterations;
  int precon_iterations = qs->precon_iterations;
  dReal sor_lcp_tolerance = qs->sor_lcp_tolerance;
  int friction_iterations = qs->friction_iterations;
  dReal smooth_contacts = qs->smooth_contacts;

#ifdef SHOW_CONVERGENCE
    // show starting lambda
    printf("lambda start: [");
    for (int i=startRow; i<startRow+nRows; i++)
      printf("%f, ", lambda[i]);
    printf("]\n");
#endif

#ifdef PENETRATION_JVERROR_CORRECTION
  dReal Jvnew_final = 0;
#endif
  dRealMutablePtr caccel_ptr1;
  dRealMutablePtr caccel_ptr2;
  dRealMutablePtr caccel_erp_ptr1;
  dRealMutablePtr caccel_erp_ptr2;
  dRealMutablePtr cforce_ptr1;
  dRealMutablePtr cforce_ptr2;
  int total_iterations = precon_iterations + num_iterations + 
    friction_iterations;
  for (int iteration = 0; iteration < total_iterations; ++iteration)
  {
    // reset rms_dlambda at beginning of iteration
    rms_dlambda[2] = 0;
    // reset rms_error at beginning of iteration
    rms_error[2] = 0;
    m_rms_dlambda[2] = 0;
    if (iteration < num_iterations + precon_iterations)
    {
      // skip resetting rms_dlambda and rms_error for bilateral constraints
      // and contact normals during extra friction iterations.
      rms_dlambda[0] = 0;
      rms_dlambda[1] = 0;
      rms_error[0] = 0;
      rms_error[1] = 0;
      m_rms_dlambda[0] = 0;
      m_rms_dlambda[1] = 0;
    }

#ifdef REORDER_CONSTRAINTS //FIXME: do it for lambda_erp and last_lambda_erp
    // constraints with findex < 0 always come first.
    if (iteration < 2) {
      // for the first two iterations, solve the constraints in
      // the given order
      IndexError *ordercurr = order+startRow;
      for (int i = startRow; i != startRow+nRows; ordercurr++, i++) {
        ordercurr->error = i;
        ordercurr->findex = findex[i];
        ordercurr->index = i;
      }
    }
    else {
      // sort the constraints so that the ones converging slowest
      // get solved last. use the absolute (not relative) error.
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

    qsort (order+startRow,nRows,sizeof(IndexError),&compare_index_error);

    //@@@ potential optimization: swap lambda and last_lambda pointers rather
    //    than copying the data. we must make sure lambda is properly
    //    returned to the caller
    memcpy (last_lambda+startRow,lambda+startRow,nRows*sizeof(dReal));

    //if (thread_id == 0) for (int i=startRow;i<startRow+nRows;i++) printf("-----> %d %d %d %f %d\n",thread_id,iteration,i,order[i].error,order[i].index);

#endif
#ifdef RANDOMLY_REORDER_CONSTRAINTS
    if ((iteration & 7) == 0) {
      #ifdef LOCK_WHILE_RANDOMLY_REORDER_CONSTRAINTS
        boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every swap
      #endif
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

#ifdef PENETRATION_JVERROR_CORRECTION
    dRealMutablePtr vnew_ptr1;
    dRealMutablePtr vnew_ptr2;
    const dReal stepsize1 = dRecip(stepsize);
    dReal Jvnew = 0;
#endif
    for (int i=startRow; i<startRow+nRows; i++) {
      //boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every row

      // @@@ potential optimization: we could pre-sort J and iMJ, thereby
      //     linearizing access to those arrays. hmmm, this does not seem
      //     like a win, but we should think carefully about our memory
      //     access pattern.

      int index = order[i].index;
      int constraint_index = findex[index];  // cache for efficiency

      // check if we are doing extra friction_iterations, if so, only solve
      // friction force constraints and nothing else.
      // i.e. skip bilateral and contact normal constraints.
      if (iteration >= (num_iterations + precon_iterations) &&
          constraint_index < 0)
        continue;

      dReal delta,delta_erp;
      dReal delta_precon;

      {
        int b1 = jb[index*2];
        int b2 = jb[index*2+1];
        caccel_ptr1 = caccel + 6*b1;
        caccel_erp_ptr1 = caccel_erp + 6*b1;
        cforce_ptr1 = cforce + 6*b1;
        if (b2 >= 0)
        {
          caccel_ptr2     = caccel + 6*b2;
          caccel_erp_ptr2 = caccel_erp + 6*b2;
          cforce_ptr2     = cforce + 6*b2;
        }
        else
        {
          caccel_ptr2     = NULL;
          caccel_erp_ptr2 = NULL;
          cforce_ptr2     = NULL;
        }
#ifdef PENETRATION_JVERROR_CORRECTION
        vnew_ptr1 = vnew + 6*b1;
        vnew_ptr2 = (b2 >= 0) ? vnew + 6*b2 : NULL;
#endif
      }

      dReal old_lambda        = lambda[index];
      dReal old_lambda_erp    = lambda_erp[index];

      //
      // caccel is the constraint accel in the non-precon case
      // cforce is the constraint force in the     precon case
      // J_precon and J differs essentially in Ad and Ad_precon,
      //  Ad is derived from diagonal of J inv(M) J'
      //  Ad_precon is derived from diagonal of J J'
      //
      // caccel_erp is from the non-precon case with erp turned on
      if (iteration < precon_iterations)
      {
        // preconditioning

        // update delta_precon
        delta_precon = rhs_precon[index] - old_lambda*Adcfm_precon[index];

        dRealPtr J_ptr = J_precon + index*12;

        // for preconditioned case, update delta using cforce, not caccel

        delta_precon -= dot6(cforce_ptr1, J_ptr);
        if (cforce_ptr2)
          delta_precon -= dot6(cforce_ptr2, J_ptr + 6);

        // set the limits for this constraint.
        // this is the place where the QuickStep method differs from the
        // direct LCP solving method, since that method only performs this
        // limit adjustment once per time step, whereas this method performs
        // once per iteration per constraint row.
        // the constraints are ordered so that all lambda[] values needed have
        // already been computed.
        dReal hi_act, lo_act;
        if (constraint_index >= 0) {
          hi_act = dFabs (hi[index] * lambda[constraint_index]);
          lo_act = -hi_act;
        } else {
          hi_act = hi[index];
          lo_act = lo[index];
        }

        // compute lambda and clamp it to [lo,hi].
        // @@@ SSE not a win here
#if 1
        lambda[index] = old_lambda+ delta_precon;
        if (lambda[index] < lo_act) {
          delta_precon = lo_act-old_lambda;
          lambda[index] = lo_act;
        }
        else if (lambda[index] > hi_act) {
          delta_precon = hi_act-old_lambda;
          lambda[index] = hi_act;
        }
#else
        dReal nl = old_lambda+ delta_precon;
        _mm_store_sd(&nl, _mm_max_sd(_mm_min_sd(_mm_load_sd(&nl),
          _mm_load_sd(&hi_act)), _mm_load_sd(&lo_act)));
        lambda[index] = nl;
        delta_precon = nl - old_lambda;
#endif

        // update cforce (this is strictly for the precon case)
        {
          // for preconditioning case, compute cforce
          // FIXME: need un-altered unscaled J, not J_precon!!
          J_ptr = J_orig + index*12;

          // update cforce.
          sum6(cforce_ptr1, delta_precon, J_ptr);
          if (cforce_ptr2)
            sum6(cforce_ptr2, delta_precon, J_ptr + 6);
        }

        // record residual (error) (for the non-erp version)
        // given
        //   dlambda = sor * (b_i - A_ij * lambda_j)/(A_ii + cfm)
        // define scalar Ad:
        //   Ad = sor / (A_ii + cfm)
        // then
        //   dlambda = Ad  * (b_i - A_ij * lambda_j)
        // thus, to get residual from dlambda,
        //   residual = dlambda / Ad
        // or
        //   residual = sqrt(sum( Ad2 * dlambda_i * dlambda_i))
        //   where Ad2 = 1/(Ad * Ad)
        dReal Ad2 = 0.0;
        if (!_dequal(Ad[index], 0.0))
        {
          // Ad[i] = sor_w / (sum + cfm[i]);
          Ad2 = 1.0 / (Ad[index] * Ad[index]);
        }
        else
        {
          // TODO: Usually, this means qs->w (SOR param) is zero.
          // Residual calculation is wrong when SOR (w) is zero
          // Given SOR is rarely 0, we'll set residual as 0 for now.
          // To do this properly, we should compute dlambda without sor
          // then use the Ad without SOR to back out residual.
        }

        dReal delta_precon2 = delta_precon*delta_precon;
        if (constraint_index == -1)  // bilateral
        {
          rms_dlambda[0] += delta_precon2;
          rms_error[0] += delta_precon2*Ad2;
          m_rms_dlambda[0]++;
        }
        else if (constraint_index == -2)  // contact normal
        {
          rms_dlambda[1] += delta_precon2;
          rms_error[1] += delta_precon2*Ad2;
          m_rms_dlambda[1]++;
        }
        else  // friction forces
        {
          rms_dlambda[2] += delta_precon2;
          rms_error[2] += delta_precon2*Ad2;
          m_rms_dlambda[2]++;
        }

        old_lambda_erp = old_lambda;
        lambda_erp[index] = lambda[index];
      }
      else
      {
        // NOTE:
        // for this update, we need not throw away J*v(n+1)/h term from rhs
        //   ...so adding it back, but remember rhs has already been
        //      scaled by Ad_i, so we need to do the same to J*v(n+1)/h
        //      but given that J is already scaled by Ad_i, we don't have
        //      to do it explicitly here

        // delta: erp throttled by info.c_v_max or info.c
        delta =
#ifdef PENETRATION_JVERROR_CORRECTION
               Jvnew_final +
#endif
              rhs[index] - old_lambda*Adcfm[index];
        dRealPtr J_ptr = J + index*12;
        delta -= dot6(caccel_ptr1, J_ptr);
        if (caccel_ptr2)
          delta -= dot6(caccel_ptr2, J_ptr + 6);

        // delta_erp: unthrottled version compute for rhs with custom erp
        // for rhs_erp  note: Adcfm does not have erp because it is on the lhs
        delta_erp = rhs_erp[index] - old_lambda_erp*Adcfm[index];
        delta_erp -= dot6(caccel_erp_ptr1, J_ptr);
        if (caccel_erp_ptr2)
          delta_erp -= dot6(caccel_erp_ptr2, J_ptr + 6);

        // set the limits for this constraint.
        // this is the place where the QuickStep method differs from the
        // direct LCP solving method, since that method only performs this
        // limit adjustment once per time step, whereas this method performs
        // once per iteration per constraint row.
        // the constraints are ordered so that all lambda[] values needed have
        // already been computed.
        dReal hi_act, lo_act;
        dReal hi_act_erp, lo_act_erp;
        if (constraint_index >= 0) {
          // FOR erp throttled by info.c_v_max or info.c
          hi_act = dFabs (hi[index] * lambda[constraint_index]);
          lo_act = -hi_act;
          // for the unthrottled _erp version
          hi_act_erp = dFabs (hi[index] * lambda_erp[constraint_index]);
          lo_act_erp = -hi_act_erp;
        } else {
          // FOR erp throttled by info.c_v_max or info.c
          hi_act = hi[index];
          lo_act = lo[index];
          // for the unthrottled _erp version
          hi_act_erp = hi[index];
          lo_act_erp = lo[index];
        }

        // compute lambda and clamp it to [lo,hi].
        // @@@ SSE not a win here
#if 1
        // FOR erp throttled by info.c_v_max or info.c
        lambda[index] = old_lambda + delta;
        if (lambda[index] < lo_act) {
          delta = lo_act-old_lambda;
          lambda[index] = lo_act;
        }
        else if (lambda[index] > hi_act) {
          delta = hi_act-old_lambda;
          lambda[index] = hi_act;
        }

        // for the unthrottled _erp version
        lambda_erp[index] = old_lambda_erp + delta_erp;
        if (lambda_erp[index] < lo_act_erp) {
          delta_erp = lo_act_erp-old_lambda_erp;
          lambda_erp[index] = lo_act_erp;
        }
        else if (lambda_erp[index] > hi_act_erp) {
          delta_erp = hi_act_erp-old_lambda_erp;
          lambda_erp[index] = hi_act_erp;
        }
#else
        // FOR erp throttled by info.c_v_max or info.c
        dReal nl = old_lambda + delta;
        _mm_store_sd(&nl, _mm_max_sd(_mm_min_sd(_mm_load_sd(&nl), _mm_load_sd(&hi_act)), _mm_load_sd(&lo_act)));
        lambda[index] = nl;
        delta = nl - old_lambda;

        // for the unthrottled _erp version
        dReal nl = old_lambda_erp + delta_erp;
        _mm_store_sd(&nl, _mm_max_sd(_mm_min_sd(_mm_load_sd(&nl), _mm_load_sd(&hi_act)), _mm_load_sd(&lo_act)));
        lambda_erp[index] = nl;
        delta_erp = nl - old_lambda_erp;
#endif

        // option to smooth lambda
#ifdef SMOOTH_LAMBDA
        {
          // smooth delta lambda
          // equivalent to first order artificial dissipation on lambda update.

          // debug smoothing
          // if (i == 0)
          //   printf("rhs[%f] adcfm[%f]: ",rhs[index], Adcfm[index]);
          // if (i == 0)
          //   printf("dlambda iter[%d]: ",iteration);
          // printf(" %f ", lambda[index]-old_lambda);
          // if (i == startRow + nRows - 1)
          //   printf("\n");

          // extra residual smoothing for contact constraints
          // was smoothing both contact normal and friction constraints for VRC
          // if (constraint_index != -1)
          // smooth only lambda for friction directions fails friction_demo.world
          if (constraint_index != -1)
          {
            lambda[index] = (1.0 - smooth_contacts)*lambda[index]
              + smooth_contacts*old_lambda;
            // is filtering lambda_erp necessary?
            // lambda_erp[index] = (1.0 - smooth_contacts)*lambda_erp[index]
            //   + smooth_contacts*old_lambda_erp;
          }
        }
#endif

        // update caccel
        {
          // FOR erp throttled by info.c_v_max or info.c
          dRealPtr iMJ_ptr = iMJ + index*12;

          // update caccel.
          sum6(caccel_ptr1, delta, iMJ_ptr);
          if (caccel_ptr2)
            sum6(caccel_ptr2, delta, iMJ_ptr + 6);

          // update caccel_erp.
          sum6(caccel_erp_ptr1, delta_erp, iMJ_ptr);
          if (caccel_erp_ptr2)
            sum6(caccel_erp_ptr2, delta_erp, iMJ_ptr + 6);

#ifdef PENETRATION_JVERROR_CORRECTION
          // update vnew incrementally
          //   add stepsize * delta_caccel to the body velocity
          //   vnew = vnew + dt * delta_caccel
          sum6(vnew_ptr1, stepsize*delta, iMJ_ptr);;
          if (caccel_ptr2)
            sum6(vnew_ptr2, stepsize*delta, iMJ_ptr + 6);

          // COMPUTE Jvnew = J*vnew/h*Ad
          //   but J is already scaled by Ad, and we multiply by h later
          //   so it's just Jvnew = J*vnew here
          if (iteration >= num_iterations-7) {
            // check for non-contact bilateral constraints only
            // I've set findex to -2 for contact normal constraint
            if (constraint_index == -1) {
              dRealPtr J_ptr = J + index*12;
              Jvnew = dot6(vnew_ptr1,J_ptr);
              if (caccel_ptr2)
                Jvnew += dot6(vnew_ptr2,J_ptr+6);
              // printf("iter [%d] findex [%d] Jvnew [%f] lo [%f] hi [%f]\n",
              //   iteration, constraint_index, Jvnew, lo[index], hi[index]);
            }
          }
          //printf("iter [%d] vnew [%f,%f,%f,%f,%f,%f] Jvnew [%f]\n",
          //       iteration,
          //       vnew_ptr1[0], vnew_ptr1[1], vnew_ptr1[2],
          //       vnew_ptr1[3], vnew_ptr1[4], vnew_ptr1[5],Jvnew);
#endif
        }

        // record residual (error) (for the non-erp version)
        // given
        //   dlambda = sor * (b_i - A_ij * lambda_j)/(A_ii + cfm)
        // define scalar Ad:
        //   Ad = sor / (A_ii + cfm)
        // then
        //   dlambda = Ad  * (b_i - A_ij * lambda_j)
        // thus, to get residual from dlambda,
        //   residual = dlambda / Ad
        // or
        //   residual = sqrt(sum( Ad2 * dlambda_i * dlambda_i))
        //   where Ad2 = 1/(Ad * Ad)
        dReal Ad2 = 0.0;
        if (!_dequal(Ad[index], 0.0))
        {
          // Ad[i] = sor_w / (sum + cfm[i]);
          Ad2 = 1.0 / (Ad[index] * Ad[index]);
        }
        else
        {
          // TODO: Usually, this means qs->w (SOR param) is zero.
          // Residual calculation is wrong when SOR (w) is zero
          // Given SOR is rarely 0, we'll set residual as 0 for now.
          // To do this properly, we should compute dlambda without sor
          // then use the Ad without SOR to back out residual.
        }

        dReal delta2 = delta*delta;
        if (constraint_index == -1)  // bilateral
        {
          rms_dlambda[0] += delta2;
          rms_error[0] += delta2*Ad2;
          m_rms_dlambda[0]++;
        }
        else if (constraint_index == -2)  // contact normal
        {
          rms_dlambda[1] += delta2;
          rms_error[1] += delta2*Ad2;
          m_rms_dlambda[1]++;
        }
        else  // friction forces
        {
          rms_dlambda[2] += delta2;
          rms_error[2] += delta2*Ad2;
          m_rms_dlambda[2]++;
        }
      }

      //@@@ a trick that may or may not help
      //dReal ramp = (1-((dReal)(iteration+1)/(dReal)iterations));
      //delta *= ramp;

    } // end of for loop on m

#ifdef PENETRATION_JVERROR_CORRECTION
    Jvnew_final = Jvnew*stepsize1;
    Jvnew_final = Jvnew_final > 1.0 ? 1.0 : ( Jvnew_final < -1.0 ? -1.0 : Jvnew_final );
#endif

    // DO WE NEED TO COMPUTE NORM ACROSS ENTIRE SOLUTION SPACE (0,m)?
    // since local convergence might produce errors in other nodes?
    dReal dlambda_bilateral_mean = 0.0;
    dReal dlambda_contact_normal_mean = 0.0;
    dReal dlambda_contact_friction_mean = 0.0;
    dReal dlambda_total_mean = 0.0;

    if (m_rms_dlambda[0] > 0)
      dlambda_bilateral_mean        = rms_dlambda[0]/(dReal)m_rms_dlambda[0];
    if (m_rms_dlambda[1] > 0)
      dlambda_contact_normal_mean   = rms_dlambda[1]/(dReal)m_rms_dlambda[1];
    if (m_rms_dlambda[2] > 0)
      dlambda_contact_friction_mean = rms_dlambda[2]/(dReal)m_rms_dlambda[2];
    if (rms_dlambda[0] + rms_dlambda[1] + rms_dlambda[2] > 0)
      dlambda_total_mean = (rms_dlambda[0] + rms_dlambda[1] + rms_dlambda[2])/
        ((dReal)(m_rms_dlambda[0] + m_rms_dlambda[1] + m_rms_dlambda[2]));

    qs->rms_dlambda[0] = sqrt(dlambda_bilateral_mean);
    qs->rms_dlambda[1] = sqrt(dlambda_contact_normal_mean);
    qs->rms_dlambda[2] = sqrt(dlambda_contact_friction_mean);
    qs->rms_dlambda[3] = sqrt(dlambda_total_mean);

    dReal residual_bilateral_mean = 0.0;
    dReal residual_contact_normal_mean = 0.0;
    dReal residual_contact_friction_mean = 0.0;
    dReal residual_total_mean = 0.0;

    if (m_rms_dlambda[0] > 0)
      residual_bilateral_mean        = rms_error[0]/(dReal)m_rms_dlambda[0];
    if (m_rms_dlambda[1] > 0)
      residual_contact_normal_mean   = rms_error[1]/(dReal)m_rms_dlambda[1];
    if (m_rms_dlambda[2] > 0)
      residual_contact_friction_mean = rms_error[2]/(dReal)m_rms_dlambda[2];
    if (rms_error[0] + rms_error[1] + rms_error[2] > 0)
      residual_total_mean = (rms_error[0] + rms_error[1] + rms_error[2])/
        ((dReal)(m_rms_dlambda[0] + m_rms_dlambda[1] + m_rms_dlambda[2]));

    qs->rms_constraint_residual[0] = sqrt(residual_bilateral_mean);
    qs->rms_constraint_residual[1] = sqrt(residual_contact_normal_mean);
    qs->rms_constraint_residual[2] = sqrt(residual_contact_friction_mean);
    qs->rms_constraint_residual[3] = sqrt(residual_total_mean);
    qs->num_contacts = m_rms_dlambda[1];

    // debugging mutex locking
    //{
    //  // verify
    //  boost::recursive_mutex::scoped_lock lock(*mutex); // lock for every row
    //  printf("  random id %d iter %d\n",thread_id,iteration);
    //  for (int i=startRow+1; i<startRow+nRows; i++)
    //    printf(" %10d,",i);
    //  printf("\n");
    //  for (int i=startRow+1; i<startRow+nRows; i++)
    //    printf(" %10d;",order[i].index);
    //  printf("\n%f %f %f\n",
    //    qs->rms_dlambda[0],qs->rms_dlambda[1],qs->rms_dlambda[2]);
    //}

#ifdef SHOW_CONVERGENCE
    /* uncomment for convergence information per row sweep (LOTS OF DATA!)
    printf("MONITOR: thread(%d) iter(%d) rms(%20.18f %20.18f %20.18)f\n",
      thread_id, iteration,
      qs->rms_dlambda[0], qs->rms_dlambda[1], qs->rms_dlambda[2]);

    // print lambda
    for (int i=startRow; i<startRow+nRows; i++)
      printf("%f, ", lambda[i]);
    printf("\n");
    */
#endif

    // option to stop when tolerance has been met
    if (iteration >= precon_iterations &&
        qs->rms_constraint_residual[3] < sor_lcp_tolerance)
    {
      #ifdef DEBUG_CONVERGENCE_TOLERANCE
        printf("CONVERGED: id: %d steps: %d,"
               " rms(%20.18f + %20.18f + %20.18f) < tol(%20.18f)\n",
          thread_id, iteration,
          qs->rms_constraint_residual[0], qs->rms_constraint_residual[1],
          qs->rms_constraint_residual[2],
          sor_lcp_tolerance);
      #endif
      // tolerance satisfied, stop iterating
      break;
    }
    else if (iteration >= total_iterations - 1)
    {
      #ifdef DEBUG_CONVERGENCE_TOLERANCE
        printf("WARNING: id: %d did not converge in %d steps,"
               " rms(%20.18f + %20.18f + %20.18f) > tol(%20.18f)\n",
          thread_id, num_iterations,
          qs->rms_constraint_residual[0], qs->rms_constraint_residual[1],
          qs->rms_constraint_residual[2],
          sor_lcp_tolerance);
      #endif
    }
  } // end of for loop on iterations

#ifdef SHOW_CONVERGENCE
  // show starting lambda
  printf("final lambdas: [");
  for (int i=startRow; i<startRow+nRows; i++)
    printf("%f, ", lambda[i]);
  printf("]\n");
  printf("MONITOR: id: %d steps: %d,"
         " dlambda(%20.18f + %20.18f + %20.18f),"
         " rms(%20.18f + %20.18f + %20.18f) < tol(%20.18f)\n",
    thread_id, total_iterations,
    qs->rms_dlambda[0], qs->rms_dlambda[1], qs->rms_dlambda[2],
    qs->rms_constraint_residual[0], qs->rms_constraint_residual[1],
    qs->rms_constraint_residual[2],
    sor_lcp_tolerance);
#endif
  //printf("vnew: ");
  //for (int i=0; i<6*nb; i++) printf(" %f ",vnew[i]);
  //printf("\n");

  #ifdef REPORT_THREAD_TIMING
  gettimeofday(&tv,NULL);
  double end_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  printf("      quickstep row thread %d start time %f ended time %f duration %f\n",thread_id,cur_time,end_time,end_time - cur_time);
  #endif
}

//***************************************************************************
// SOR_LCP method
//
// nb is the number of bodies in the body array.
// J is an m*12 matrix of constraint rows
// jb is an array of first and second body numbers for each constraint row
// invMOI is the global frame inverse inertia for each body (stacked 3x3 matrices)
//
// this returns lambda and cforce (the constraint force).
// note: cforce is returned as inv(M)*J'*lambda,
//   the constraint force is actually J'*lambda
//
// rhs, lo and hi are modified on exit
//
static void SOR_LCP (dxWorldProcessContext *context,
  const int m, const int nb, dRealMutablePtr J, dRealMutablePtr J_precon, dRealMutablePtr J_orig, dRealMutablePtr vnew, int *jb, dxBody * const *body,
  dRealPtr invMOI, dRealPtr MOI, dRealMutablePtr lambda, dRealMutablePtr lambda_erp,
  dRealMutablePtr caccel, dRealMutablePtr caccel_erp, dRealMutablePtr cforce,
  dRealMutablePtr rhs, dRealMutablePtr rhs_erp, dRealMutablePtr rhs_precon,
  dRealPtr lo, dRealPtr hi, dRealPtr cfm, const int *findex,
  dxQuickStepParameters *qs,
#ifdef USE_TPROW
  boost::threadpool::pool* row_threadpool,
#endif
  const dReal stepsize)
{

  // precompute iMJ = inv(M)*J'
  dReal *iMJ = context->AllocateArray<dReal> (m*12);
  compute_invM_JT (m,J,iMJ,jb,body,invMOI);

  if (qs->warm_start > 0)
  {
    // warm starting
    // compute cforce=(inv(M)*J')*lambda
    if (qs->precon_iterations > 0)
      multiply_invM_JT (m,nb,J,jb,lambda,cforce);

    // re-compute caccel=(inv(M)*J')*lambda with new iMJ
    // seems much better than using stored caccel's
    multiply_invM_JT (m,nb,iMJ,jb,lambda,caccel);
    multiply_invM_JT (m,nb,iMJ,jb,lambda_erp,caccel_erp);
  }
  else
  {
    // no warm starting
    if (qs->precon_iterations > 0)
      dSetZero (cforce,nb*6);
    dSetZero (caccel,nb*6);
    dSetZero (caccel_erp,nb*6);
  }

  dReal *Ad = context->AllocateArray<dReal> (m);

  {
    const dReal sor_w = qs->w;    // SOR over-relaxation parameter
    // precompute 1 / diagonals of A
    dRealPtr iMJ_ptr = iMJ;
    dRealPtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
      dReal sum = 0;
      sum += dot6(iMJ_ptr, J_ptr);
      if (jb[i*2+1] >= 0) {
        sum += dot6(iMJ_ptr+6, J_ptr+6);
      }
      if (findex[i] < 0)
        Ad[i] = sor_w / (sum + cfm[i]);
      else
        Ad[i] = CONTACT_SOR_SCALE * sor_w / (sum + cfm[i]);
    }
  }

  // recompute Ad for preconditioned case, Ad_precon is similar to Ad but
  //   whereas Ad is 1 over diagonals of J inv(M) J'
  //    Ad_precon is 1 over diagonals of J J'
  dReal *Adcfm_precon = NULL;
  if (qs->precon_iterations > 0)
  {
    dReal *Ad_precon = context->AllocateArray<dReal> (m);

    {
      const dReal sor_w = qs->w;    // SOR over-relaxation parameter
      // precompute 1 / diagonals of A
      // preconditioned version uses J instead of iMJ
      dRealPtr J_ptr = J;
      for (int i=0; i<m; J_ptr += 12, i++) {
        dReal sum = 0;
        sum += dot6(J_ptr, J_ptr);
        if (jb[i*2+1] >= 0) {
          sum += dot6(J_ptr+6, J_ptr+6);
        }
        if (findex[i] < 0)
          Ad_precon[i] = sor_w / (sum + cfm[i]);
        else
          Ad_precon[i] = CONTACT_SOR_SCALE * sor_w / (sum + cfm[i]);
      }
    }

    /********************************/
    /* allocate for Adcfm           */
    /* which is a mX1 column vector */
    /********************************/
    // compute Adcfm_precon for the preconditioned case
    //   do this first before J gets altered (J's diagonals gets premultiplied by Ad)
    //   and save a copy of J into J_orig
    //   as J becomes J * Ad, J_precon becomes J * Ad_precon
    Adcfm_precon = context->AllocateArray<dReal> (m);
    {

      // NOTE: This may seem unnecessary but it's indeed an optimization
      // to move multiplication by Ad[i] and cfm[i] out of iteration loop.

      // scale J_precon and rhs_precon by Ad
      // copy J_orig
      dRealMutablePtr J_ptr = J;
      dRealMutablePtr J_orig_ptr = J_orig;
      dRealMutablePtr J_precon_ptr = J_precon;
      for (int i=0; i<m; J_ptr += 12, J_precon_ptr += 12, J_orig_ptr += 12, i++) {
        dReal Ad_precon_i = Ad_precon[i];
        for (int j=0; j<12; j++) {
          J_precon_ptr[j] = J_ptr[j] * Ad_precon_i;
          J_orig_ptr[j] = J_ptr[j]; //copy J
        }
        rhs_precon[i] *= Ad_precon_i;
        // scale Ad by CFM. N.B. this should be done last since it is used above
        Adcfm_precon[i] = Ad_precon_i * cfm[i];
      }
    }
  }

  dReal *Adcfm = context->AllocateArray<dReal> (m);


  {
    // NOTE: This may seem unnecessary but it's indeed an optimization
    // to move multiplication by Ad[i] and cfm[i] out of iteration loop.

    // scale J and rhs by Ad
    dRealMutablePtr J_ptr = J;
    for (int i=0; i<m; J_ptr += 12, i++) {
      dReal Ad_i = Ad[i];
      for (int j=0; j<12; j++) {
        J_ptr[j] *= Ad_i;
      }
      rhs[i] *= Ad_i;
      rhs_erp[i] *= Ad_i;
      // scale Ad by CFM. N.B. this should be done last since it is used above
      Adcfm[i] = Ad_i * cfm[i];
    }
  }


  // order to solve constraint rows in
  IndexError *order = context->AllocateArray<IndexError> (m);
  int *tmpOrder = context->AllocateArray<int> (m);

#ifndef REORDER_CONSTRAINTS
  if (qs->row_reorder1)
  {
    // -1 in front, followed by -2, lastly all the >0
    // Fill the array from both ends
    // where -1 is bilateral, and -2 is friction normal,
    // might be followed by 2 positive tangential indices
    // if friction is not zero.
    // first pass puts -1 in the back
    int front = 0;
    int back = m-1;
    for (int i=0; i<m; ++i) {
      if (findex[i] == -1) {
        tmpOrder[front] = i; // Place them at the front
        ++front;
      } else {
        tmpOrder[back] = i; // Place them at the end
        --back;
      }
    }
    dIASSERT (back - front==1);
    // second pass, put all negatives in the front,
    // should preserver -1 goes before -2,
    // and group all >0 at the end
    front = 0;
    back = m-1;
    for (int i=0; i<m; ++i) {
      if (findex[tmpOrder[i]] < 0) {
        order[front].index = tmpOrder[i]; // Place them at the front
        ++front;
      } else {
        order[back].index = tmpOrder[i]; // Place them at the end
        --back;
      }
    }
    dIASSERT (back - front==1);
  }
  else
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

#ifdef SHOW_CONVERGENCE
    if (0)
    {
      printf("-------------- saved labmdas -------------\n");
      // print current lambdas
      for (int i = 0; i < m; ++i)
      {
        printf("%f, ", lambda[order[i].index]);
      }
      printf("\n");
      for (int i = 0; i < m; ++i)
      {
        printf("%d, ", findex[order[i].index]);
      }
      printf("\n");
      for (int i = 0; i < m; ++i)
      {
        printf("%d, ", order[i].index);
      }
      printf("\n-------------- end of saved labmdas -------------\n");
    }
#endif

#ifdef REORDER_CONSTRAINTS
  // the lambda computed at the previous iteration.
  // this is used to measure error for when we are reordering the indexes.
  dReal *last_lambda = context->AllocateArray<dReal> (m);
  dReal *last_lambda_erp = context->AllocateArray<dReal> (m);
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


  #ifdef REPORT_THREAD_TIMING
  // timing
  struct timeval tv;
  double cur_time;
  gettimeofday(&tv,NULL);
  cur_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("    quickstep start threads at time %f\n",cur_time);
  #endif


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
    params[thread_id].hi = hi;
    params[thread_id].lo = lo;
    params[thread_id].invMOI = invMOI;
    params[thread_id].MOI= MOI;
    params[thread_id].Ad = Ad;
    params[thread_id].Adcfm = Adcfm;
    params[thread_id].Adcfm_precon = Adcfm_precon;
    params[thread_id].rhs = rhs;
    params[thread_id].rhs_erp = rhs_erp;
    params[thread_id].J = J;
    params[thread_id].caccel = caccel;
    params[thread_id].caccel_erp = caccel_erp;
    params[thread_id].lambda = lambda;
    params[thread_id].lambda_erp = lambda_erp;
    params[thread_id].iMJ = iMJ;
    params[thread_id].rhs_precon  = rhs_precon ;
    params[thread_id].J_precon  = J_precon ;
    params[thread_id].J_orig  = J_orig ;
    params[thread_id].cforce  = cforce ;
    params[thread_id].vnew  = vnew ;
#ifdef REORDER_CONSTRAINTS
    params[thread_id].last_lambda  = last_lambda ;
    params[thread_id].last_lambda_erp  = last_lambda_erp ;
#endif

#ifdef DEBUG_CONVERGENCE_TOLERANCE
    printf("thread summary: id %d i %d m %d chunk %d start %d end %d \n",
      thread_id,i,m,chunk,nStart,nEnd);
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

//***************************************************************************
// Modifying inertia along constrained axes without modifying dynamics.
static void DYNAMIC_INERTIA(const int infom, const dxJoint::Info2 &Jinfo, const int b1, const int b2,
                            const dJointWithInfo1 *jicurr,
                            dRealMutablePtr invMOI, dRealMutablePtr MOI)
{
  /// INERTIA PROPAGATION ACROSS CONSTRAINED JOINTS
  for (int j=0; j<infom; j++) {
#ifdef DEBUG_INERTIA_PROPAGATION
    printf("--------JAC---------------\n");
    printf("jacobian [%d] J1l [%f %f %f] J2l [%f %f %f] J1a [%f %f %f]"
           " J2a [%f %f %f]\n", j,
           Jinfo.J1l[0+j*Jinfo.rowskip],
           Jinfo.J1l[1+j*Jinfo.rowskip],
           Jinfo.J1l[2+j*Jinfo.rowskip],
           Jinfo.J2l[0+j*Jinfo.rowskip],
           Jinfo.J2l[1+j*Jinfo.rowskip],
           Jinfo.J2l[2+j*Jinfo.rowskip],
           Jinfo.J1a[0+j*Jinfo.rowskip],
           Jinfo.J1a[1+j*Jinfo.rowskip],
           Jinfo.J1a[2+j*Jinfo.rowskip],
           Jinfo.J2a[0+j*Jinfo.rowskip],
           Jinfo.J2a[1+j*Jinfo.rowskip],
           Jinfo.J2a[2+j*Jinfo.rowskip]);
#endif
    /// \FIXME: For now, implement only for the two non-free axial rotation
    /// constraints for hinge joints.
    /// this only makes sense if joint connects two dynamic bodies (b2 >= 0)
    /// Skip this entire block if we don't want to modify inertia for stability.
    if (b2 >= 0 && jicurr->joint->type() == dJointTypeHinge &&
        (j == 3 || j == 4))
    {
      /// In hinge joint, pure rotational constraint,
      /// J1l and J2l should be zeros, and J1a and J2a should be equal
      /// and opposite to each other.
      /// J1a or J2a indicates the constrained axis direction.
      /// For this implementation, determine constrained axis(s)
      /// direction from J1a for hinge joints.


      /// get the moment of inertia (MOI) for parent and child bodies constrained by J1a and J2a.
      /// MOI and invMOI are already in inertial frame (previously rotated by body.posr.R)
      /// get pointers to our invMOI/MOI matrices
      dReal *invMOI_ptr1 = invMOI + b1 * 12;
      dReal *invMOI_ptr2 = invMOI + b2 * 12;
      dReal *MOI_ptr1 = MOI + b1 * 12;
      dReal *MOI_ptr2 = MOI + b2 * 12;

      // S: unit vector in the constrained axis direction
      // S is the line along which we want to compute MOI
      // FIXME:  check that directions of J1a == J2a
      dVector3 S =
        {Jinfo.J1a[0+j*Jinfo.rowskip],
         Jinfo.J1a[1+j*Jinfo.rowskip],
         Jinfo.J1a[2+j*Jinfo.rowskip] };
      dNormalize3(S);

      // temporary vector used for matrix/vector math
      dVector3 tmp31;

      // compute scalar MOI in line with S for each body
      //   m1 = S' * MOI1 * S
      dMultiply0_133(tmp31, S, MOI_ptr1);
      dReal m1 = dCalcVectorDot3(tmp31, S);

      //   m2 = S' * MOI2 * S
      dMultiply0_133(tmp31, S, MOI_ptr2);
      dReal m2 = dCalcVectorDot3(tmp31, S);

      // identify body with larger inertia
      dReal m_large = m1;
      dReal m_small = m2;
      dReal *MOI_large = MOI_ptr1;
      dReal *MOI_small = MOI_ptr2;
      if (m2 > m1)
      {
        m_large = m2;
        m_small = m1;
        MOI_large = MOI_ptr2;
        MOI_small = MOI_ptr1;
      }

      /// get full axis MOI tensor representing the scalar axis MOI.
      // full MOI tensor for S needs matrix outer product of S:
      //   SS = [ S * S' ]
      dMatrix3 SS = {
           S[0]*S[0], S[0]*S[1], S[0]*S[2], 0,
           S[1]*S[0], S[1]*S[1], S[1]*S[2], 0,
           S[2]*S[0], S[2]*S[1], S[2]*S[2], 0};

#ifdef DEBUG_INERTIA_PROPAGATION
      printf("--------old MOI-----------\n");
      printf("MOI1[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", b1,
        MOI_ptr1[0*4+0],MOI_ptr1[0*4+1],MOI_ptr1[0*4+2],MOI_ptr1[0*4+3],
        MOI_ptr1[1*4+0],MOI_ptr1[1*4+1],MOI_ptr1[1*4+2],MOI_ptr1[1*4+3],
        MOI_ptr1[2*4+0],MOI_ptr1[2*4+1],MOI_ptr1[2*4+2],MOI_ptr1[2*4+3]);
      printf("MOI2[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", b2,
        MOI_ptr2[0*4+0],MOI_ptr2[0*4+1],MOI_ptr2[0*4+2],MOI_ptr2[0*4+3],
        MOI_ptr2[1*4+0],MOI_ptr2[1*4+1],MOI_ptr2[1*4+2],MOI_ptr2[1*4+3],
        MOI_ptr2[2*4+0],MOI_ptr2[2*4+1],MOI_ptr2[2*4+2],MOI_ptr2[2*4+3]);
      printf("--------S VECTORS-----------\n");
      printf("MOI1 b1[%d] S[%f %f %f] = %g\n",b1, S[0], S[1], S[2], m1);
      printf("MOI2 b2[%d] S[%f %f %f] = %g\n",b2, S[0], S[1], S[2], m2);
      printf("--------SS----------------\n");
      printf("SS [%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", b1,
        SS[0*4+0],SS[0*4+1],SS[0*4+2],SS[0*4+3],
        SS[1*4+0],SS[1*4+1],SS[1*4+2],SS[1*4+3],
        SS[2*4+0],SS[2*4+1],SS[2*4+2],SS[2*4+3]);
#endif

      // define maximum ratio of moment of inertia for adjacent bodies
      // ie. m_large / m_small <= moi_ratio_max
      /// \todo make moi_ratio_max adjustable
      /// \todo automatically adjust moi_ratio_max such that
      /// abs sum of off-diagonals remains smaller than the diagonal
      /// for all rows (see comments below about Gauss-Seidel stability).
      // increase moi_ratio_max to skip checks and increase performance
      const dReal moi_ratio_max = 200.0;
      if (m_large > moi_ratio_max * m_small)
      {
        // Large inertia ratio detected, try reducing it.
        // Increase m_small by dm
        //   Reduce m_large by dm
        //   such that (m_large - dm) = moi_ratio_max * (m_small + dm)
        // Intermediate math step:
        //   m_large - moi_ratio_max * m_small = dm * (1 + moi_ratio_max)
        // Then:
        dReal dm = (m_large - moi_ratio_max * m_small) / (1 + moi_ratio_max);

        // This will then be applied to the bodies by multiplying by [S*S']
        //   MOI_large -= dm * SS
        //   MOI_small += dm * SS
        // But first it should be verified that the change will not
        // destabilize the Gauss-Seidel solver.
        // To verify this, the Generalized Line Criterion for Gauss-Seidel
        // is used, which is stated below:
        //
        // The Gauss-Seidel method will converge if for each row of the matrix:
        //   the sum of absolute values of off-diagonal elements
        //   is less than absolute value of the diagonal element.
        //
        // @article{Garcia2003,
        // author = {Garcia, M.V.P. and {Humes Jr.}, C. and Stern, J.M.},
        // doi = {10.1590/S0101-82052003000100006},
        // issn = {0101-8205},
        // journal = {Computational \& Applied Mathematics},
        // number = {1},
        // pages = {91--97},
        // title = {{Generalized line criterion for Gauss-Seidel method}},
        // url = {http://www.ime.usp.br/~jstern/papers/papersJS/ghs03.pdf},
        // volume = {22},
        // year = {2003}
        // }

        // For 3x3 Gauss-Seidel matrix M, this is equivalent to:
        //   abs(M[0][0]) > abs(M[0][1]) + abs(M[0][2])
        //   abs(M[1][1]) > abs(M[1][0]) + abs(M[1][2])
        //   abs(M[2][2]) > abs(M[2][0]) + abs(M[2][1])
        //
        // Since M is a mass matrix, it is positive definite, which implies
        // that the diagonal elements are strictly positive:
        //   M[0][0] > abs(M[0][1]) + abs(M[0][2])
        //   M[1][1] > abs(M[1][0]) + abs(M[1][2])
        //   M[2][2] > abs(M[2][0]) + abs(M[2][1])
        //
        // For extra safety factor define a parameter gamma >= 1 such that:
        //   M[0][0] > gamma * (abs(M[0][1]) + abs(M[0][2]))
        //   M[1][1] > gamma * (abs(M[1][0]) + abs(M[1][2]))
        //   M[2][2] > gamma * (abs(M[2][0]) + abs(M[2][1]))
        const dReal gamma = 1.0;

        int problem = 0;
        for (int row = 0; row < 3; ++row)
        {
          // off-diagonal columns
          int col1 = (row + 1) % 3;
          int col2 = (row + 2) % 3;

          // diagonal index
          int id = row*4 + row;

          // off-diagonal indices
          int iod1 = row*4 + col1;
          int iod2 = row*4 + col2;

          // diagonal element of adjusted M_large
          dReal Md_large = MOI_large[id] - dm*SS[id];

          // sum of absolute values of off-diagonal elements of adjusted M_large
          dReal Mod_large = fabs(MOI_large[iod1] - dm*SS[iod1])
                          + fabs(MOI_large[iod2] - dm*SS[iod2]);

          // diagonal element of adjusted M_small
          dReal Md_small = MOI_small[id] + dm*SS[id];

          // sum of absolute values of off-diagonal elements of adjusted M_small
          dReal Mod_small = fabs(MOI_small[iod1] + dm*SS[iod1])
                          + fabs(MOI_small[iod2] + dm*SS[iod2]);

          if (Md_large <= gamma*Mod_large  ||  Md_small <= gamma*Mod_small)
          {
            problem = 1;
          }
        }

        if (problem == 0)
        {
          // Everything looks good, update the inertia matrices
          for (int i = 0; i < 12; ++i)
          {
            int col = i%4;
            if (col == 3)
            {
              //  set unused terms to zero
              MOI_large[i] = 0;
              MOI_small[i] = 0;
            }
            else
            {
              MOI_large[i] -= dm * SS[i];
              MOI_small[i] += dm * SS[i];
            }
          }
        }

          // Update invMOI by inverting analytically (may not be efficient).
          // try 1981 Ken Miller (http://www.jstor.org/stable/2690437) or
          //   (http://math.stackexchange.com/questions/17776)
          // try taking advantage of symmetry of MOI
          dReal det1 = MOI_ptr1[0*4+0]*(MOI_ptr1[2*4+2]*MOI_ptr1[1*4+1]
                      -MOI_ptr1[2*4+1]*MOI_ptr1[1*4+2])
                      -MOI_ptr1[1*4+0]*(MOI_ptr1[2*4+2]*MOI_ptr1[0*4+1]
                      -MOI_ptr1[2*4+1]*MOI_ptr1[0*4+2])
                      +MOI_ptr1[2*4+0]*(MOI_ptr1[1*4+2]*MOI_ptr1[0*4+1]
                      -MOI_ptr1[1*4+1]*MOI_ptr1[0*4+2]);
          invMOI_ptr1[0*4+0] =  (MOI_ptr1[2*4+2]*MOI_ptr1[1*4+1]
                                -MOI_ptr1[2*4+1]*MOI_ptr1[1*4+2])/det1;
          invMOI_ptr1[0*4+1] = -(MOI_ptr1[2*4+2]*MOI_ptr1[0*4+1]
                                -MOI_ptr1[2*4+1]*MOI_ptr1[0*4+2])/det1;
          invMOI_ptr1[0*4+2] =  (MOI_ptr1[1*4+2]*MOI_ptr1[0*4+1]
                                -MOI_ptr1[1*4+1]*MOI_ptr1[0*4+2])/det1;
          // invMOI_ptr1[0*4+3] = 0.0;
          invMOI_ptr1[1*4+0] = invMOI_ptr1[0*4+1];
          invMOI_ptr1[1*4+1] =  (MOI_ptr1[2*4+2]*MOI_ptr1[0*4+0]
                                -MOI_ptr1[2*4+0]*MOI_ptr1[0*4+2])/det1;
          invMOI_ptr1[1*4+2] = -(MOI_ptr1[1*4+2]*MOI_ptr1[0*4+0]
                                -MOI_ptr1[1*4+0]*MOI_ptr1[0*4+2])/det1;
          // invMOI_ptr1[1*4+3] = 0.0;
          invMOI_ptr1[2*4+0] = invMOI_ptr1[0*4+2];
          invMOI_ptr1[2*4+1] = invMOI_ptr1[1*4+2];
          invMOI_ptr1[2*4+2] =  (MOI_ptr1[1*4+1]*MOI_ptr1[0*4+0]
                                -MOI_ptr1[1*4+0]*MOI_ptr1[0*4+1])/det1;
          // invMOI_ptr1[2*4+3] = 0.0;

          dReal det2 = MOI_ptr2[0*4+0]*(MOI_ptr2[2*4+2]*MOI_ptr2[1*4+1]
                      -MOI_ptr2[2*4+1]*MOI_ptr2[1*4+2])
                      -MOI_ptr2[1*4+0]*(MOI_ptr2[2*4+2]*MOI_ptr2[0*4+1]
                      -MOI_ptr2[2*4+1]*MOI_ptr2[0*4+2])
                      +MOI_ptr2[2*4+0]*(MOI_ptr2[1*4+2]*MOI_ptr2[0*4+1]
                      -MOI_ptr2[1*4+1]*MOI_ptr2[0*4+2]);
          invMOI_ptr2[0*4+0] =  (MOI_ptr2[2*4+2]*MOI_ptr2[1*4+1]
                                -MOI_ptr2[2*4+1]*MOI_ptr2[1*4+2])/det2;
          invMOI_ptr2[0*4+1] = -(MOI_ptr2[2*4+2]*MOI_ptr2[0*4+1]
                                -MOI_ptr2[2*4+1]*MOI_ptr2[0*4+2])/det2;
          invMOI_ptr2[0*4+2] =  (MOI_ptr2[1*4+2]*MOI_ptr2[0*4+1]
                                -MOI_ptr2[1*4+1]*MOI_ptr2[0*4+2])/det2;
          // invMOI_ptr2[0*4+3] = 0.0;
          invMOI_ptr2[1*4+0] = invMOI_ptr2[0*4+1];
          invMOI_ptr2[1*4+1] =  (MOI_ptr2[2*4+2]*MOI_ptr2[0*4+0]
                                -MOI_ptr2[2*4+0]*MOI_ptr2[0*4+2])/det2;
          invMOI_ptr2[1*4+2] = -(MOI_ptr2[1*4+2]*MOI_ptr2[0*4+0]
                                -MOI_ptr2[1*4+0]*MOI_ptr2[0*4+2])/det2;
          // invMOI_ptr2[1*4+3] = 0.0;
          invMOI_ptr2[2*4+0] = invMOI_ptr2[0*4+2];
          invMOI_ptr2[2*4+1] = invMOI_ptr2[1*4+2];
          invMOI_ptr2[2*4+2] =  (MOI_ptr2[1*4+1]*MOI_ptr2[0*4+0]
                                -MOI_ptr2[1*4+0]*MOI_ptr2[0*4+1])/det2;
          // invMOI_ptr2[2*4+3] = 0.0;

  #ifdef DEBUG_INERTIA_PROPAGATION
        printf("---------S Scalars--------\n");
        printf(" original    S1 [%g] S2 [%g]\n", m1, m2);
        printf(" distributed S1 [%g] S2 [%g]\n", m1_new, m2_new);
          printf("----------new MOI---------\n");

          printf("new MOI1[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n",
            b1,
            MOI_ptr1[0*4+0],MOI_ptr1[0*4+1],MOI_ptr1[0*4+2],MOI_ptr1[0*4+3],
            MOI_ptr1[1*4+0],MOI_ptr1[1*4+1],MOI_ptr1[1*4+2],MOI_ptr1[1*4+3],
            MOI_ptr1[2*4+0],MOI_ptr1[2*4+1],MOI_ptr1[2*4+2],MOI_ptr1[2*4+3]);

          // Modify MOI_ptr2
          printf("new MOI2[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n",
            b2,
            MOI_ptr2[0*4+0],MOI_ptr2[0*4+1],MOI_ptr2[0*4+2],MOI_ptr2[0*4+3],
            MOI_ptr2[1*4+0],MOI_ptr2[1*4+1],MOI_ptr2[1*4+2],MOI_ptr2[1*4+3],
            MOI_ptr2[2*4+0],MOI_ptr2[2*4+1],MOI_ptr2[2*4+2],MOI_ptr2[2*4+3]);

          // double check resulting MOI along s
          dMultiply0_133(tmp31, S, MOI_ptr1);
          // scalar body 1 MOI component along vector S
          m1 = dCalcVectorDot3(tmp31, S);
          printf("new MOI1 along S [%f]\n", m1);
          dMultiply0_133(tmp31, S, MOI_ptr2);
          // scalar body 2 MOI component along vector S
          m2 = dCalcVectorDot3(tmp31, S);
          printf("new MOI2 along S [%f]\n", m2);

          /// \todo double check resulting MOI along joint axis and
          /// see that it's the same

          printf("----------new inv---------\n");
          printf("new invMOI1[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n"
                 "[%f %f %f %f]\n", b1,
                 invMOI_ptr1[0*4+0], invMOI_ptr1[0*4+1],
                 invMOI_ptr1[0*4+2], invMOI_ptr1[0*4+3],
                 invMOI_ptr1[1*4+0], invMOI_ptr1[1*4+1],
                 invMOI_ptr1[1*4+2], invMOI_ptr1[1*4+3],
                 invMOI_ptr1[2*4+0], invMOI_ptr1[2*4+1],
                 invMOI_ptr1[2*4+2], invMOI_ptr1[2*4+3]);

          // Modify invMOI_ptr2
          printf("new invMOI2[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n"
                 "[%f %f %f %f]\n", b2,
                 invMOI_ptr2[0*4+0], invMOI_ptr2[0*4+1],
                 invMOI_ptr2[0*4+2], invMOI_ptr2[0*4+3],
                 invMOI_ptr2[1*4+0], invMOI_ptr2[1*4+1],
                 invMOI_ptr2[1*4+2], invMOI_ptr2[1*4+3],
                 invMOI_ptr2[2*4+0], invMOI_ptr2[2*4+1],
                 invMOI_ptr2[2*4+2], invMOI_ptr2[2*4+3]);
  #endif

  #ifdef DEBUG_INERTIA_PROPAGATION
          // check if diagonally-dominant
          if (MOI_ptr1[0*4+0] < dFabs(MOI_ptr1[0*4+1])+dFabs(MOI_ptr1[0*4+2]))
            printf(" * new MOI1 row 1 d[%f] < o[%f, %f]\n",
                   MOI_ptr1[0*4+0],MOI_ptr1[0*4+1], MOI_ptr1[0*4+2]);
          if (MOI_ptr1[1*4+1] < dFabs(MOI_ptr1[1*4+0])+dFabs(MOI_ptr1[1*4+2]))
            printf(" * new MOI1 row 2 d[%f] < o[%f, %f]\n",
                   MOI_ptr1[1*4+1],MOI_ptr1[1*4+0], MOI_ptr1[1*4+2]);
          if (MOI_ptr1[2*4+2] < dFabs(MOI_ptr1[2*4+0])+dFabs(MOI_ptr1[2*4+1]))
            printf(" * new MOI1 row 3 d[%f] < o[%f, %f]\n",
                   MOI_ptr1[2*4+2],MOI_ptr1[2*4+0], MOI_ptr1[2*4+1]);

          if (MOI_ptr2[0*4+0] < dFabs(MOI_ptr2[0*4+1])+dFabs(MOI_ptr2[0*4+2]))
            printf(" * new MOI2 row 1 d[%f] < o[%f, %f]\n",
                   MOI_ptr2[0*4+0],MOI_ptr2[0*4+1], MOI_ptr2[0*4+2]);
          if (MOI_ptr2[1*4+1] < dFabs(MOI_ptr2[1*4+0])+dFabs(MOI_ptr2[1*4+2]))
            printf(" * new MOI2 row 2 d[%f] < o[%f, %f]\n",
                   MOI_ptr2[1*4+1],MOI_ptr2[1*4+0], MOI_ptr2[1*4+2]);
          if (MOI_ptr2[2*4+2] < dFabs(MOI_ptr2[2*4+0])+dFabs(MOI_ptr2[2*4+1]))
            printf(" * new MOI2 row 3 d[%f] < o[%f, %f]\n",
                   MOI_ptr2[2*4+2],MOI_ptr2[2*4+0], MOI_ptr2[2*4+1]);
  #endif
      }
    }
  }
}

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
  // accumulator. MOI and invMOI are a vertical stack of 3x4 matrices, one per body.
  dReal *invMOI = context->AllocateArray<dReal> (3*4*nb);
  dReal *MOI = context->AllocateArray<dReal> (3*4*nb);

  // TODO: possible optimization: move this to inside joint getInfo2, for inertia tweaking
  // update tacc from external force and inertia tensor in inerial frame
  // for now, modify MOI and invMOI after getInfo2 is called
  {
    dReal *invMOIrow = invMOI;
    dReal *MOIrow = MOI;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invMOIrow += 12, MOIrow += 12, bodycurr++) {
      dMatrix3 tmp;
      dxBody *b_ptr = *bodycurr;

      // compute inverse inertia tensor in global frame
      dMultiply2_333 (tmp,b_ptr->invI,b_ptr->posr.R);
      dMultiply0_333 (invMOIrow,b_ptr->posr.R,tmp);

      // also store MOI for later use by preconditioner
      dMultiply2_333 (tmp,b_ptr->mass.I,b_ptr->posr.R);
      dMultiply0_333 (MOIrow,b_ptr->posr.R,tmp);

      if (b_ptr->flags & dxBodyGyroscopic) {
        // compute rotational force
        dMultiply0_331 (tmp,MOIrow,b_ptr->avel);
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
    if (!_dequal(gravity_x, 0.0)) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        if ((b_ptr->flags & dxBodyNoGravity)==0) {
          b_ptr->facc[0] += b_ptr->mass.mass * gravity_x;
        }
      }
    }
    dReal gravity_y = world->gravity[1];
    if (!_dequal(gravity_y, 0.0)) {
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        if ((b_ptr->flags & dxBodyNoGravity)==0) {
          b_ptr->facc[1] += b_ptr->mass.mass * gravity_y;
        }
      }
    }
    dReal gravity_z = world->gravity[2];
    if (!_dequal(gravity_z, 0.0)) {
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
    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; _jcurr++) {  // jicurr=dest, _jcurr=src
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
  int *jb = NULL;
  int *findex;

  dReal *vnew = NULL; // used by PENETRATION_JVERROR_CORRECTION

  dReal *cforce = context->AllocateArray<dReal> (nb*6);
  dReal *caccel = context->AllocateArray<dReal> (nb*6);
  dReal *caccel_erp = context->AllocateArray<dReal> (nb*6);
#ifdef POST_UPDATE_CONSTRAINT_VIOLATION_CORRECTION
  dReal *caccel_corr = context->AllocateArray<dReal> (nb*6);
#endif

  // Get Joint Information, setup Jacobians by calling getInfo2.
  if (m > 0) {
    dReal *cfm, *lo, *hi, *rhs, *rhs_erp, *rhs_precon, *Jcopy;
    dReal *c_v_max;

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

      c_v_max = context->AllocateArray<dReal> (mlocal);
      for (int i=0; i<mlocal; i++) c_v_max[i] = world->contactp.max_vel; // init all to world max surface vel

      const unsigned jbelements = mlocal*2;
      jb = context->AllocateArray<int> (jbelements);

      rhs = context->AllocateArray<dReal> (mlocal);
      rhs_erp = context->AllocateArray<dReal> (mlocal);
      rhs_precon = context->AllocateArray<dReal> (mlocal);

      Jcopy = context->AllocateArray<dReal> (mfb*12);
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

        int *jb_ptr = jb;

        dReal *Jcopyrow = Jcopy;
        unsigned ofsi = 0;
        const dJointWithInfo1 *jicurr = jointiinfos;
        const dJointWithInfo1 *const jiend = jicurr + nj;
        for (; jicurr != jiend; jicurr++) {
          Jinfo.erp = world->global_erp;
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
          Jinfo.c_v_max = c_v_max + ofsi;

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

          // create an array of body numbers for each joint row
          int b1 = (joint->node[0].body) ? (joint->node[0].body->tag) : -1;
          int b2 = (joint->node[1].body) ? (joint->node[1].body->tag) : -1;
          for (int j=0; j<infom; j++) {
            jb_ptr[0] = b1;
            jb_ptr[1] = b2;
            jb_ptr += 2;
          }

          // Modify inertia to keep simulation stable
          if (world->qs.dynamic_inertia_reduction)
            DYNAMIC_INERTIA(infom, Jinfo, b1, b2, jicurr, invMOI, MOI);

          // update index for next joint
          ofsi += infom;

          // double check jb_ptr length
          dIASSERT (jb_ptr == jb+2*m);
        }
      }

      BEGIN_STATE_SAVE(context, tmp1state) {
        IFTIMING (dTimerNow ("compute rhs"));
        // compute the right hand side `rhs'
        dReal *tmp1 = context->AllocateArray<dReal> (nb*6);
        dSetZero(tmp1,nb*6);
        // put v/h + invM*fe into tmp1
        dReal *tmp1curr = tmp1;
        const dReal *invMOIrow = invMOI;
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body;
             bodycurr != bodyend;
             tmp1curr+=6, invMOIrow+=12, bodycurr++) {
          dxBody *b_ptr = *bodycurr;
          dReal body_invMass = b_ptr->invMass;
          for (int j=0; j<3; j++)
            tmp1curr[j] = b_ptr->facc[j]*body_invMass + b_ptr->lvel[j]*stepsize1;
          dMultiply0_331 (tmp1curr + 3,invMOIrow,b_ptr->tacc);
          for (int k=0; k<3; k++) tmp1curr[3+k] += b_ptr->avel[k] * stepsize1;
        }

        // put J*tmp1 into rhs
        multiply_J (m,J,jb,tmp1,rhs);
      } END_STATE_SAVE(context, tmp1state);

      // complete rhs
      for (int i=0; i<m; i++) {
        rhs_erp[i] =      c[i]*stepsize1 - rhs[i];
        if (dFabs(c[i]) > c_v_max[i])
          rhs[i]   =  c_v_max[i]*stepsize1 - rhs[i];
        //if (dFabs(c[i]) > world->contactp.max_vel)
        //  rhs[i]   =  world->contactp.max_vel*stepsize1 - rhs[i];
        else
          rhs[i]   = c[i]*stepsize1 - rhs[i];
      }

      // compute rhs_precon
      if (world->qs.precon_iterations > 0)
        computeRHSPrecon(context,m,nb,MOI,body,stepsize1,c,J,jb,rhs_precon);

      // scale CFM
      for (int j=0; j<m; j++) cfm[j] *= stepsize1;

    } END_STATE_SAVE(context, cstate);

#ifdef PENETRATION_JVERROR_CORRECTION
    // allocate and populate vnew with v(n+1) due to non-constraint forces as the starting value
    vnew = context->AllocateArray<dReal> (nb*6);
    {
      dRealMutablePtr vnewcurr = vnew;
      dxBody* const* bodyend = body + nb;
      const dReal *invMOIrow = invMOI;
      dReal tmp_tacc[3];
      for (dxBody* const* bodycurr = body; bodycurr != bodyend;
           invMOIrow += 12, vnewcurr += 6, bodycurr++) {
        dxBody *b_ptr = *bodycurr;

        // add stepsize * invM * fe to the body velocity
        dReal body_invMass_mul_stepsize = stepsize * b_ptr->invMass;
        for (int j=0; j<3; j++) {
          vnewcurr[j]   = b_ptr->lvel[j] + body_invMass_mul_stepsize * b_ptr->facc[j];
          vnewcurr[j+3] = b_ptr->avel[j];
          tmp_tacc[j]   = b_ptr->tacc[j]*stepsize;
        }
        dMultiplyAdd0_331 (vnewcurr+3, invMOIrow, tmp_tacc);

      }
    }
#endif

    // load lambda from the value saved on the previous iteration
    dReal *lambda = context->AllocateArray<dReal> (m);
    dReal *lambda_erp = context->AllocateArray<dReal> (m);

    // initialize lambda and lambda_erp
    if (world->qs.warm_start > 0)
    {
      // warm starting
      dReal *lambdacurr = lambda;
      dReal *lambda_erpcurr = lambda_erp;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; jicurr++) {
        int infom = jicurr->info.m;
        memcpy (lambdacurr, jicurr->joint->lambda, infom * sizeof(dReal));
        lambdacurr += infom;
        memcpy (lambda_erpcurr, jicurr->joint->lambda_erp, infom * sizeof(dReal));
        lambda_erpcurr += infom;
      }

      // for warm starting, this seems to be necessary to prevent
      // jerkiness in motor-driven joints. i have no idea why this works.
      // also necessary if J condition numbers are high (maybe the same thing).
      for (int i=0; i<m; i++) {
        lambda[i] *= world->qs.warm_start;
        lambda_erp[i] *= world->qs.warm_start;
      }
    }
    else
    {
      dSetZero (lambda,m);
      dSetZero (lambda_erp,m);
    }

    BEGIN_STATE_SAVE(context, lcpstate) {
      IFTIMING (dTimerNow ("solving LCP problem"));
      // solve the LCP problem and get lambda and invM*constraint_force
      SOR_LCP (context,m,nb,J,J_precon,J_orig,vnew,jb,body,
               invMOI,MOI,lambda,lambda_erp,
               caccel,caccel_erp,cforce,
               rhs,rhs_erp,rhs_precon,
               lo,hi,cfm,findex,
               &world->qs,
#ifdef USE_TPROW
               world->row_threadpool,
#endif
               stepsize);

    } END_STATE_SAVE(context, lcpstate);

    if (world->qs.warm_start > 0)
    {
      // warm starting
      // save lambda for the next iteration
      //@@@ note that this doesn't work for contact joints yet, as they are
      // recreated every iteration
      const dReal *lambdacurr = lambda;
      const dReal *lambda_erpcurr = lambda_erp;
      const dJointWithInfo1 *jicurr = jointiinfos;
      const dJointWithInfo1 *const jiend = jicurr + nj;
      for (; jicurr != jiend; jicurr++) {
        int infom = jicurr->info.m;
        memcpy (jicurr->joint->lambda, lambdacurr, infom * sizeof(dReal));
        lambdacurr += infom;
        memcpy (jicurr->joint->lambda_erp, lambda_erpcurr,
          infom * sizeof(dReal));
        lambda_erpcurr += infom;
      }
    }

    // note that the SOR method overwrites rhs and J at this point, so
    // they should not be used again.
    {
      IFTIMING (dTimerNow ("velocity update due to constraint forces"));
      //
      // update new velocity
      // add stepsize * caccel_erp to the body velocity
      //
      const dReal *caccelcurr = caccel_erp;
      dxBody *const *const bodyend = body + nb;
      for (dxBody *const *bodycurr = body; bodycurr != bodyend; caccelcurr+=6, bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        for (int j=0; j<3; j++) {
          b_ptr->lvel[j] += stepsize * caccelcurr[j];
          b_ptr->avel[j] += stepsize * caccelcurr[3+j];
        }
        // printf("caccel [%f %f %f] [%f %f %f]\n"
        //   ,caccelcurr[0] ,caccelcurr[1] ,caccelcurr[2]
        //   ,caccelcurr[3] ,caccelcurr[4] ,caccelcurr[5]);
        // printf("  vel [%f %f %f] [%f %f %f]\n"
        //   ,b_ptr->lvel[0] ,b_ptr->lvel[1] ,b_ptr->lvel[2]
        //   ,b_ptr->avel[0] ,b_ptr->avel[1] ,b_ptr->avel[2]);
      }
    }

    if (mfb > 0) {
      // force feedback without erp is better
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
    const dReal *invMOIrow = invMOI;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invMOIrow += 12, bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      dReal body_invMass_mul_stepsize = stepsize * b_ptr->invMass;
      for (int j=0; j<3; j++) {
        b_ptr->lvel[j] += body_invMass_mul_stepsize * b_ptr->facc[j];
        b_ptr->tacc[j] *= stepsize;
      }
      dMultiplyAdd0_331 (b_ptr->avel, invMOIrow, b_ptr->tacc);
      // printf("fe [%f %f %f] [%f %f %f]\n"
      //   ,b_ptr->facc[0] ,b_ptr->facc[1] ,b_ptr->facc[2]
      //   ,b_ptr->tacc[0] ,b_ptr->tacc[1] ,b_ptr->tacc[2]);
      /* DEBUG PRINTOUTS
      printf("uncorrect vel [%f %f %f] [%f %f %f]\n"
        ,b_ptr->lvel[0] ,b_ptr->lvel[1] ,b_ptr->lvel[2]
        ,b_ptr->avel[0] ,b_ptr->avel[1] ,b_ptr->avel[2]);
      */
    }
  }

#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
  if (m > 0) {
    BEGIN_STATE_SAVE(context, rmsstate) {
      dReal *vel = context->AllocateArray<dReal>(nb*6);

      // CHECK THAT THE UPDATED VELOCITY OBEYS THE CONSTRAINT
      //  (this check needs unmodified J)
      //  put residual into tmp
      dRealMutablePtr velcurr = vel;
      dxBody* const* bodyend = body + nb;
      for (dxBody* const* bodycurr = body; bodycurr != bodyend; velcurr += 6, bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        for (int j=0; j<3; j++) {
          velcurr[j]   = b_ptr->lvel[j];
          velcurr[3+j] = b_ptr->avel[j];
        }
      }
      dReal *tmp = context->AllocateArray<dReal> (m);
      multiply_J (m,J,jb,vel,tmp);

      int m_Jv_bilateral = 0;
      int m_Jv_contact = 0;
      int m_Jv_friction = 0;
      dReal Jv_bilateral = 0;
      dReal Jv_contact = 0;
      dReal Jv_friction = 0;
      for (int i=0; i<m; i++)
      {
        if (findex[i] == -1)
        {
          m_Jv_bilateral++;
          Jv_bilateral += dFabs(tmp[i])*dFabs(tmp[i]);
        }
        else if (findex[i] == -2)
        {
          // contact error includes joint limits
          Jv_contact += dFabs(tmp[i])*dFabs(tmp[i]);
          m_Jv_contact++;
        }
        else if (findex[i] >= 0)
        {
          m_Jv_friction++;
          Jv_friction += dFabs(tmp[i])*dFabs(tmp[i]);
        }

        // Note: This is not a good measure of constraint error
        // for soft contact, as Jv is not necessarily zero here.
        // Better measure is compute the residual.  \\\ TODO
      }
      // printf ("error = %10.6e %10.6e %10.6e\n",
      //   error, Jv_bilateral, Jv_contact);
      // world->qs.rms_constraint_residual[0] = sqrt(error/(dReal)m);

      dReal residual_bilateral_mean = 0.0;
      dReal residual_contact_normal_mean = 0.0;
      dReal residual_contact_friction_mean = 0.0;
      dReal residual_total_mean = 0.0;

      if (m_Jv_bilateral > 0)
        residual_bilateral_mean        = Jv_bilateral/(dReal)m_Jv_bilateral;
      if (m_Jv_contact > 0)
        residual_contact_normal_mean   = Jv_contact/(dReal)m_Jv_contact;
      if (m_Jv_friction > 0)
        residual_contact_friction_mean = Jv_friction/(dReal)m_Jv_friction;
      if (m_Jv_bilateral + m_Jv_contact + m_Jv_friction > 0)
        residual_total_mean = (Jv_bilateral + Jv_contact + Jv_friction)/
          ((dReal)(m_Jv_bilateral + m_Jv_contact + m_Jv_friction));

      world->qs.rms_constraint_residual[0] = sqrt(residual_bilateral_mean);
      world->qs.rms_constraint_residual[1] = sqrt(residual_contact_normal_mean);
      world->qs.rms_constraint_residual[2] =
        sqrt(residual_contact_friction_mean);
      world->qs.rms_constraint_residual[3] = sqrt(residual_total_mean);
      world->qs.num_contacts = m_Jv_contact;
    } END_STATE_SAVE(context, rmsstate);
  }
#endif
  {
    // update the position and orientation from the new linear/angular velocity
    // (over the given timestep)
    IFTIMING (dTimerNow ("update position"));
    const dReal *caccelcurr = caccel;
    dxBody *const *const bodyend = body + nb;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend;
         caccelcurr += 6, ++bodycurr)
    {
      dxBody *b_ptr = *bodycurr;
      {
        // sum all forces (external and constraint) into facc and tacc
        // so dBodyGetForce and dBodyGetTorque returns total force and torque
        // on the body
        dReal cf[6];
        cf[0] = b_ptr->mass.mass * caccelcurr[0];
        cf[1] = b_ptr->mass.mass * caccelcurr[1];
        cf[2] = b_ptr->mass.mass * caccelcurr[2];
        dMultiply0_331 (cf+3, b_ptr->mass.I, caccelcurr+3);
        for (unsigned int j = 0; j < 3; ++j)
        {
          b_ptr->facc[j] += cf[j];
          b_ptr->tacc[j] += cf[3+j];
        }
      }
      dxStepBody (b_ptr,stepsize);
    }
  }

  // revert lvel and avel with the non-erp version of caccel
  if (m > 0) {
    dReal erp_removal = 1.00;
    IFTIMING (dTimerNow ("velocity update due to constraint forces"));
    // remove caccel_erp
    const dReal *caccel_erp_curr = caccel_erp;
    const dReal *caccel_curr = caccel;
    dxBody *const *const bodyend = body + nb;
    int debug_count = 0;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend;
         caccel_curr+=6, caccel_erp_curr+=6, bodycurr++, debug_count++) {
      dxBody *b_ptr = *bodycurr;
      for (int j=0; j<3; j++) {
        // dReal v0 = b_ptr->lvel[j];
        // dReal a0 = b_ptr->avel[j];
        dReal dv = erp_removal * stepsize *
          (caccel_curr[j]   - caccel_erp_curr[j]);
        dReal da = erp_removal * stepsize *
          (caccel_curr[3+j] - caccel_erp_curr[3+j]);

        /* default v removal
        */
        b_ptr->lvel[j] += dv;
        b_ptr->avel[j] += da;
        /* think about minimize J*v somehow without SORLCP...
        */
        /* minimize final velocity test 1,
        if (v0 * dv < 0) {
          if (fabs(v0) < fabs(dv))
            b_ptr->lvel[j] = 0.0;
          else
            b_ptr->lvel[j] += dv;
        }
        if (a0 * da < 0) {
          if (fabs(a0) < fabs(da))
            b_ptr->avel[j] = 0.0;
          else
            b_ptr->avel[j] += da;
        }
        */

        /*  DEBUG PRINTOUTS, total forces/accel on a body
        printf("nb[%d] m[%d] b[%d] i[%d] v[%f] dv[%f] vf[%f] a[%f] da[%f] af[%f] debug[%f - %f][%f - %f]\n"
               ,nb, m, debug_count, j, v0, dv, b_ptr->lvel[j]
                 , a0, da, b_ptr->avel[j]
               ,caccel_curr[j], caccel_erp_curr[j]
               ,caccel_curr[3+j], caccel_erp_curr[3+j]);
        */
      }
      /*  DEBUG PRINTOUTS
      printf("corrected vel [%f %f %f] [%f %f %f]\n",
        b_ptr->lvel[0], b_ptr->lvel[1], b_ptr->lvel[2],
        b_ptr->avel[0], b_ptr->avel[1], b_ptr->avel[2]);
      */
    }

#ifdef POST_UPDATE_CONSTRAINT_VIOLATION_CORRECTION
    // ADD CACCEL CORRECTION FROM VELOCITY CONSTRAINT VIOLATION
    BEGIN_STATE_SAVE(context, velstate) {
      dReal *vel = context->AllocateArray<dReal>(nb*6);

      // CHECK THAT THE UPDATED VELOCITY OBEYS THE CONSTRAINT
      //  (this check needs unmodified J)
      //  put residual into tmp
      dRealMutablePtr velcurr = vel;
      //dxBody* const* bodyend = body + nb;
      for (dxBody* const* bodycurr = body; bodycurr != bodyend; velcurr += 6, bodycurr++) {
        dxBody *b_ptr = *bodycurr;
        for (int j=0; j<3; j++) {
          velcurr[j]   = b_ptr->lvel[j];
          velcurr[3+j] = b_ptr->avel[j];
        }
      }
      dReal *tmp = context->AllocateArray<dReal> (m);
      multiply_J (m,J,jb,vel,tmp);

      // DIRECTLY ADD THE CONSTRAINT VIOLATION TERM (TMP) TO VELOCITY UPDATE
      // add correction term dlambda = J*v(n+1)/dt
      // and caccel += dt*invM*JT*dlambda (dt's cancel out)
      dReal *iMJ = context->AllocateArray<dReal> (m*12);
      compute_invM_JT (m,J,iMJ,jb,body,invMOI);
      // compute caccel_corr=(inv(M)*J')*dlambda, correction term
      // as we change lambda.
      multiply_invM_JT (m,nb,iMJ,jb,tmp,caccel_corr);

    } END_STATE_SAVE(context, velstate);

    // ADD CACCEL CORRECTION FROM VELOCITY CONSTRAINT VIOLATION
    caccelcurr = caccel;
    const dReal* caccel_corrcurr = caccel_corr;
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; caccel_corrcurr+=6, bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      for (int j=0; j<3; j++) {
        b_ptr->lvel[j] += erp_removal * stepsize * caccel_corrcurr[j];
        b_ptr->avel[j] += erp_removal * stepsize * caccel_corrcurr[3+j];
      }
    }
#endif


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

static size_t EstimateSOR_LCPMemoryRequirements(int m,int /*nb*/)
{
  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Ad
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Adcfm
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Ad_precon
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for Adcfm_precon
  res += dEFFICIENT_SIZE(sizeof(IndexError) * m); // for order
  res += dEFFICIENT_SIZE(sizeof(int) * m); // for tmpOrder
#ifdef REORDER_CONSTRAINTS
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for last_lambda
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for last_lambda_erp
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

  // for invMOI
  res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb);
  // for MOI (inertia) needed by preconditioner
  res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb);
  res += dEFFICIENT_SIZE(sizeof(dReal) * nb); // for invM

  {
    // for initial jointiinfos
    size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * _nj);

    // for shrunk jointiinfos
    size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * nj);
    if (m > 0) {
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J_precon
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J_orig
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for vnew
      sub1_res2 += 3 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for cfm, lo, hi
      sub1_res2 += 2 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for rhs, rhs_erp
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for rhs_precon
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * 2 * m); // for jb
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
      sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for c_v_max
      sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * mfb); // for Jcopy
      {
        size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for c
        {
          size_t sub3_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for tmp1

          size_t sub3_res2 = 0;

          sub2_res1 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
        }

        size_t sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda_erp
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for cforce
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for caccel
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for caccel_erp
#ifdef POST_UPDATE_CONSTRAINT_VIOLATION_CORRECTION
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for caccel_corr
        sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for vel
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for tmp
        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ
#endif
        {
          // for SOR_LCP
          size_t sub3_res1 = EstimateSOR_LCPMemoryRequirements(m,nb);

          size_t sub3_res2 = 0;
#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
          {
            // for vel
            size_t sub4_res1 = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb);
            sub4_res1 += dEFFICIENT_SIZE(sizeof(dReal) * m); // for tmp
            sub4_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ

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


