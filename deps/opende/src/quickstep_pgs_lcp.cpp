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
#include "util.h"
#include <sys/time.h>
#include "quickstep_util.h"
#include "quickstep_pgs_lcp.h"

using namespace ode;
 
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

        delta_precon -= quickstep::dot6(cforce_ptr1, J_ptr);
        if (cforce_ptr2)
          delta_precon -= quickstep::dot6(cforce_ptr2, J_ptr + 6);

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
          quickstep::sum6(cforce_ptr1, delta_precon, J_ptr);
          if (cforce_ptr2)
            quickstep::sum6(cforce_ptr2, delta_precon, J_ptr + 6);
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
        delta -= quickstep::dot6(caccel_ptr1, J_ptr);
        if (caccel_ptr2)
          delta -= quickstep::dot6(caccel_ptr2, J_ptr + 6);

        // delta_erp: unthrottled version compute for rhs with custom erp
        // for rhs_erp  note: Adcfm does not have erp because it is on the lhs
        delta_erp = rhs_erp[index] - old_lambda_erp*Adcfm[index];
        delta_erp -= quickstep::dot6(caccel_erp_ptr1, J_ptr);
        if (caccel_erp_ptr2)
          delta_erp -= quickstep::dot6(caccel_erp_ptr2, J_ptr + 6);

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
          quickstep::sum6(caccel_ptr1, delta, iMJ_ptr);
          if (caccel_ptr2)
            quickstep::sum6(caccel_ptr2, delta, iMJ_ptr + 6);

          // update caccel_erp.
          quickstep::sum6(caccel_erp_ptr1, delta_erp, iMJ_ptr);
          if (caccel_erp_ptr2)
            quickstep::sum6(caccel_erp_ptr2, delta_erp, iMJ_ptr + 6);

#ifdef PENETRATION_JVERROR_CORRECTION
          // update vnew incrementally
          //   add stepsize * delta_caccel to the body velocity
          //   vnew = vnew + dt * delta_caccel
          quickstep::sum6(vnew_ptr1, stepsize*delta, iMJ_ptr);;
          if (caccel_ptr2)
            quickstep::sum6(vnew_ptr2, stepsize*delta, iMJ_ptr + 6);

          // COMPUTE Jvnew = J*vnew/h*Ad
          //   but J is already scaled by Ad, and we multiply by h later
          //   so it's just Jvnew = J*vnew here
          if (iteration >= num_iterations-7) {
            // check for non-contact bilateral constraints only
            // I've set findex to -2 for contact normal constraint
            if (constraint_index == -1) {
              dRealPtr J_ptr = J + index*12;
              Jvnew = quickstep::dot6(vnew_ptr1,J_ptr);
              if (caccel_ptr2)
                Jvnew += quickstep::dot6(vnew_ptr2,J_ptr+6);
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
// PGS_LCP method
// this returns lambda and cforce (the constraint force).
// note: cforce is returned as inv(M)*J'*lambda,
//   the constraint force is actually J'*lambda
//
// rhs, lo and hi are modified on exit
//
void quickstep::PGS_LCP (dxWorldProcessContext *context,
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

size_t quickstep::EstimatePGS_LCPMemoryRequirements(int m,int /*nb*/)
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

