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
#include <thread>

#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/objects.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include <ode/misc.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"

#ifndef _WIN32
  #include <sys/time.h>
#endif

#include "quickstep_util.h"
#include "quickstep_pgs_lcp.h"
#ifndef TIMING
#ifdef HDF5_INSTRUMENT
#define DUMP
std::vector<dReal> errors;
#endif   //HDF5_INSTRUMENT
#endif   //timing

using namespace ode;

static void* ComputeRows(void *p)
{
  dxPGSLCPParameters *params = (dxPGSLCPParameters *)p;

  #ifdef REPORT_THREAD_TIMING
  int thread_id                 = params->thread_id;
  struct timeval tv;
  double cur_time;
  gettimeofday(&tv,NULL);
  cur_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  // printf("thread %d started at time %f\n",thread_id,cur_time);
  #endif

  IndexError* order             = params->order;
  dxBody* const* body           = params->body;
#ifdef RANDOMLY_REORDER_CONSTRAINTS
#ifdef LOCK_WHILE_RANDOMLY_REORDER_CONSTRAINTS
  boost::recursive_mutex* mutex = params->mutex;
#endif
#endif
  bool inline_position_correction = params->inline_position_correction;
  bool position_correction_thread = params->position_correction_thread;

  dxQuickStepParameters *qs    = params->qs;
  int startRow                 = params->nStart;   // 0
  int nRows                    = params->nChunkSize; // m
#ifdef USE_1NORM
  int m                        = params->m; // m used for rms error computation
#endif
  int nb                       = params->nb;
#ifdef PENETRATION_JVERROR_CORRECTION
  dReal stepsize               = params->stepsize;
  dRealMutablePtr vnew         = params->vnew;
#endif
  int* jb                      = params->jb;
  const int* findex            = params->findex;
  bool skip_friction           = params->skip_friction;
  dRealPtr        hi           = params->hi;
  dRealPtr        lo           = params->lo;
  dRealPtr        Ad           = params->Ad;
  dRealPtr        Adcfm        = params->Adcfm;
  dRealPtr        Adcfm_precon = params->Adcfm_precon;
  dRealPtr        J            = params->J;
  dRealPtr        iMJ          = params->iMJ;
  dRealPtr        rhs_precon   = params->rhs_precon;
  dRealPtr        J_precon     = params->J_precon;
  dRealPtr        J_orig       = params->J_orig;
  dRealMutablePtr cforce       = params->cforce;

  dRealPtr        rhs          = params->rhs;
  dRealMutablePtr caccel       = params->caccel;
  dRealMutablePtr lambda       = params->lambda;

  /// THREAD_POSITION_CORRECTION
  dRealPtr rhs_erp             = params->rhs_erp;
  dRealMutablePtr caccel_erp   = params->caccel_erp;
  dRealMutablePtr lambda_erp   = params->lambda_erp;

#ifdef REORDER_CONSTRAINTS
  dRealMutablePtr last_lambda  = params->last_lambda;
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
  dReal pgs_lcp_tolerance = qs->pgs_lcp_tolerance;
  int friction_iterations = qs->friction_iterations;
  Friction_Model friction_model = qs->friction_model;
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
#ifdef HDF5_INSTRUMENT
  errors.resize(num_iterations + precon_iterations + friction_iterations);
#endif
  dRealMutablePtr caccel_ptr1;
  dRealMutablePtr caccel_ptr2;

  /// THREAD_POSITION_CORRECTION
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


      dReal delta = 0;
      dReal delta_precon = 0;

      // THREAD_POSITION_CORRECTION
      dReal delta_erp = 0;
      // precon does not support split position correction right now.
      // dReal delta_precon_erp = 0;

      // setup pointers
      int b1 = jb[index*2];
      int b2 = jb[index*2+1];

      // for precon
      {
        cforce_ptr1 = cforce + 6*b1;
        if (b2 >= 0)
        {
          cforce_ptr2     = cforce + 6*b2;
        }
        else
        {
          cforce_ptr2     = NULL;
        }
      }

      // for non-precon
      {
        caccel_ptr1 = caccel + 6*b1;
        if (b2 >= 0)
        {
          caccel_ptr2     = caccel + 6*b2;
        }
        else
        {
          caccel_ptr2     = NULL;
        }

        if (inline_position_correction)
        {
          caccel_erp_ptr1 = caccel_erp + 6*b1;
          if (b2 >= 0)
          {
            caccel_erp_ptr2     = caccel_erp + 6*b2;
          }
          else
          {
            caccel_erp_ptr2     = NULL;
          }
        }
      }
      dReal old_lambda        = lambda[index];

      /// THREAD_POSITION_CORRECTION
      dReal old_lambda_erp;
      if (inline_position_correction)
        old_lambda_erp    = lambda_erp[index];

#ifdef PENETRATION_JVERROR_CORRECTION
      // 4/4 optional pointers for jverror correction
      vnew_ptr1 = vnew + 6*b1;
      vnew_ptr2 = (b2 >= 0) ? vnew + 6*b2 : NULL;
#endif


      //
      // caccel is the constraint accel in the non-precon case
      // cforce is the constraint force in the     precon case
      // J_precon and J differs essentially in Ad and Ad_precon,
      //  Ad is derived from diagonal of J inv(M) J'
      //  Ad_precon is derived from diagonal of J J'
      //
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
        // @@@ SSE is used to speed up vector math
        // operations with gcc compiler when defined
        // but SSE is not a win here, #undef for now
#undef SSE_CLAMP
#ifndef SSE_CLAMP
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

        // initialize position correction terms (_erp) with precon results
        if (inline_position_correction)
        {
          old_lambda_erp = old_lambda;
          lambda_erp[index] = lambda[index];
        }
      }
      else
      {
        if (!skip_friction || constraint_index < 0)
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

          if (inline_position_correction)
          {
            delta_erp = rhs_erp[index] - old_lambda_erp*Adcfm[index];
            delta_erp -= quickstep::dot6(caccel_erp_ptr1, J_ptr);
            if (caccel_ptr2)
              delta_erp -= quickstep::dot6(caccel_erp_ptr2, J_ptr + 6);
          }

        // set the limits for this constraint.
        // this is the place where the QuickStep method differs from the
        // direct LCP solving method, since that method only performs this
        // limit adjustment once per time step, whereas this method performs
        // once per iteration per constraint row.
        // the constraints are ordered so that all lambda[] values needed have
        // already been computed.
        dReal hi_act, lo_act;
        /// THREAD_POSITION_CORRECTION
        dReal hi_act_erp, lo_act_erp;
        if (constraint_index >= 0)
        {
          // printf ("index diff %d\n", index - constraint_index);
          if (index - constraint_index >= 3)
          {
            // torsinal friction should have been added as the third row from
            // contact normal constraint
            // this_is_torsional_friction
            hi_act = dFabs (hi[index] * lambda[constraint_index]);
            lo_act = -hi_act;
            if (inline_position_correction)
            {
              hi_act_erp = dFabs (hi[index] * lambda_erp[constraint_index]);
              lo_act_erp = -hi_act_erp;
            }
            // printf("lo %f hi %f\n", lo[index], hi[index]);
          }
          else
          {
            // deal with non-torsional frictions
            if (friction_model == pyramid_friction)
            {
              // FOR erp throttled by info.c_v_max or info.c
              hi_act = dFabs (hi[index] * lambda[constraint_index]);
              lo_act = -hi_act;
              if (inline_position_correction)
              {
                hi_act_erp = dFabs (hi[index] * lambda_erp[constraint_index]);
                lo_act_erp = -hi_act_erp;
              }
            }
            else if (friction_model == cone_friction)
            {
              quickstep::dxConeFrictionModel(lo_act, hi_act, lo_act_erp, hi_act_erp, jb, J_orig, index,
                  constraint_index, startRow, nRows, nb, body, i, order, findex, NULL, hi, lambda, lambda_erp);
            }
            else if(friction_model == box_friction)
            {
              hi_act = hi[index];
              lo_act = -hi_act;
              hi_act_erp = hi[index];
              lo_act_erp = -hi_act_erp;
            }
            else
            {
                // initialize the hi and lo to get rid of warnings
                hi_act = dInfinity;
                lo_act = -dInfinity;
                hi_act_erp = dInfinity;
                lo_act_erp = -dInfinity;
                dMessage (d_ERR_UASSERT, "internal error, undefined friction model");
            }
          }
        }
        else
        {
          // FOR erp throttled by info.c_v_max or info.c
          hi_act = hi[index];
          lo_act = lo[index];
          if (inline_position_correction)
          {
            hi_act_erp = hi[index];
            lo_act_erp = lo[index];
          }
        }
        // compute lambda and clamp it to [lo,hi].
        // @@@ SSE not a win here
#undef SSE_CLAMP
#ifndef SSE_CLAMP
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

          if (inline_position_correction)
          {
            lambda_erp[index] = old_lambda_erp + delta_erp;
            if (lambda_erp[index] < lo_act_erp) {
              delta_erp = lo_act_erp-old_lambda_erp;
              lambda_erp[index] = lo_act_erp;
            }
            else if (lambda_erp[index] > hi_act_erp) {
              delta_erp = hi_act_erp-old_lambda_erp;
              lambda_erp[index] = hi_act_erp;
            }
          }
#else
          // FOR erp throttled by info.c_v_max or info.c
          dReal nl = old_lambda + delta;
          _mm_store_sd(&nl,
                       _mm_max_sd(_mm_min_sd(_mm_load_sd(&nl),
                       _mm_load_sd(&hi_act)),
                       _mm_load_sd(&lo_act)));
          lambda[index] = nl;
          delta = nl - old_lambda;

          if (inline_position_correction)
          {
            dReal nl_erp = old_lambda_erp + delta_erp;
            _mm_store_sd(&nl_erp,
                         _mm_max_sd(_mm_min_sd(_mm_load_sd(&nl_erp),
                         _mm_load_sd(&hi_act_erp)),
                         _mm_load_sd(&lo_act_erp)));
            lambda_erp[index] = nl_erp;
            delta_erp = nl_erp - old_lambda_erp;
          }
#endif

          // option to smooth lambda
#ifdef SMOOTH_LAMBDA
          // skip smoothing for the position correction thread
          if (!position_correction_thread)
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
            }

            // if (inline_position_correction)
            // {
            //   /// not smoothing lambda_erp
            // }
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

            if (inline_position_correction)
            {
              quickstep::sum6(caccel_erp_ptr1, delta_erp, iMJ_ptr);
              if (caccel_erp_ptr2)
                quickstep::sum6(caccel_erp_ptr2, delta_erp, iMJ_ptr + 6);
            }
          }
        }  // end of skip friction check

#ifdef PENETRATION_JVERROR_CORRECTION
        {
          // FOR erp throttled by info.c_v_max or info.c
          dRealPtr iMJ_ptr = iMJ + index*12;
          // update vnew incrementally
          //   add stepsize * delta_caccel to the body velocity
          //   vnew = vnew + dt * delta_caccel
          quickstep::sum6(vnew_ptr1, stepsize*delta, iMJ_ptr);
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
        }
#endif


        //////////////////////////////////////////////////////
        // record residual (error) (for the non-erp version)
        //////////////////////////////////////////////////////
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
      } // end of non-precon

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

#ifdef HDF5_INSTRUMENT
    errors[iteration] = residual_total_mean;
#endif
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
        qs->rms_constraint_residual[3] < pgs_lcp_tolerance)
    {
      #ifdef DEBUG_CONVERGENCE_TOLERANCE
        printf("CONVERGED: id: %d steps: %d,"
               " rms(%20.18f + %20.18f + %20.18f) < tol(%20.18f)\n",
          thread_id, iteration,
          qs->rms_constraint_residual[0], qs->rms_constraint_residual[1],
          qs->rms_constraint_residual[2],
          pgs_lcp_tolerance);
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
          pgs_lcp_tolerance);
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
    pgs_lcp_tolerance);
#endif
  //printf("vnew: ");
  //for (int i=0; i<6*nb; i++) printf(" %f ",vnew[i]);
  //printf("\n");

  #ifdef REPORT_THREAD_TIMING
  gettimeofday(&tv,NULL);
  double end_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  printf("      quickstep row thread %d start time %f ended time %f duration %f\n",thread_id,cur_time,end_time,end_time - cur_time);
  #endif

  if (position_correction_thread)
    IFTIMING (dTimerNow ("ComputeRows_erp ends"));
  else
    IFTIMING (dTimerNow ("ComputeRows ends"));

  // doing below seems to slow things down
  // if (position_correction_thread)
  //   pthread_exit(NULL);

#ifdef HDF5_INSTRUMENT
  IFDUMP(h5_write_errors(DATA_FILE, errors.data(), errors.size()));
#endif

  return NULL;
}

//***************************************************************************
// PGS_LCP method was previously SOR_LCP
//
// this returns lambda and cforce (the constraint force).
// note: cforce is returned as inv(M)*J'*lambda,
//   the constraint force is actually J'*lambda
//
// rhs, lo and hi are modified on exit
//
void quickstep::PGS_LCP (dxWorldProcessContext *context,
  const int m, const int nb, dRealMutablePtr J, dRealMutablePtr J_precon,
  dRealMutablePtr J_orig,
#ifdef PENETRATION_JVERROR_CORRECTION
  dRealMutablePtr vnew,
  const dReal stepsize,
#endif
  int *jb, dxBody * const *body,
  dRealPtr invMOI, dRealPtr MOI, dRealMutablePtr lambda,
  dRealMutablePtr lambda_erp,
  dRealMutablePtr caccel, dRealMutablePtr caccel_erp, dRealMutablePtr cforce,
  dRealMutablePtr rhs, dRealMutablePtr rhs_erp, dRealMutablePtr rhs_precon,
  dRealPtr lo, dRealPtr hi, dRealPtr cfm, const int *findex,
  dxQuickStepParameters *qs
#ifdef USE_TPROW
  , boost::threadpool::pool* row_threadpool
#endif
  )
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
      sum += quickstep::dot6(iMJ_ptr, J_ptr);
      if (jb[i*2+1] >= 0) {
        sum += quickstep::dot6(iMJ_ptr+6, J_ptr+6);
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
        sum += quickstep::dot6(J_ptr, J_ptr);
        if (jb[i*2+1] >= 0) {
          sum += quickstep::dot6(J_ptr+6, J_ptr+6);
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

  boost::recursive_mutex* mutex =
    context->AllocateArray<boost::recursive_mutex>(1);

  // number of chunks must be at least 1
  // (single iteration, through all the constraints)
  int num_chunks = qs->num_chunks > 0 ? qs->num_chunks : 1; // min is 1

  // divide into chunks sequentially
  int chunk = m / num_chunks+1;
  chunk = chunk > 0 ? chunk : 1;
  int thread_id = 0;

  // prepare pointers for threads
  // params for solution with correction (_erp) term
  dxPGSLCPParameters *params_erp = NULL;
  if (qs->thread_position_correction)
    params_erp = context->AllocateArray<dxPGSLCPParameters>(num_chunks);

  // params for solution without correction (_erp) term
  dxPGSLCPParameters *params =
    context->AllocateArray<dxPGSLCPParameters>(num_chunks);

#ifdef REPORT_THREAD_TIMING
  // timing
  struct timeval tv;
  double cur_time;
  gettimeofday(&tv,NULL);
  cur_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("    quickstep start threads at time %f\n",cur_time);
#endif

  // this is the main computational loop for PGS
  // here we iterate over each row and make progressive updates

  // to do split impulse / position projection correction,
  //   - solve lambda     and caccel     using rhs
  //   - solve lambda_erp and caccel_erp using rhs_erp
  // these two solves can be performed simultaneously.

  IFTIMING (dTimerNow ("start pgs rows"));
  for (int i=0; i<m; i+= chunk,thread_id++)
  {
    // for (int ijk=0;ijk<m;ijk++)
    //   printf("thread_id> id:%d jb[%d]=%d\n",thread_id,ijk,jb[ijk]);

    int nStart = i - qs->num_overlap < 0 ? 0 : i - qs->num_overlap;
    int nEnd   = i + chunk + qs->num_overlap;
    if (nEnd > m) nEnd = m;

    std::thread params_erp_thread;

    if (qs->thread_position_correction && params_erp != NULL)
    {
      // setup params for ComputeRows
      IFTIMING (dTimerNow ("start pgs_erp rows"));
      //////////////////////////////////////////////////////
      /// repeat for position projection
      /// setup params_erp for ComputeRows
      //////////////////////////////////////////////////////
      params_erp[thread_id].thread_id = thread_id;
      params_erp[thread_id].order     = order;
      params_erp[thread_id].body      = body;
      params_erp[thread_id].mutex     = mutex;
      params_erp[thread_id].inline_position_correction = false;
      params_erp[thread_id].position_correction_thread = true;
#ifdef PENETRATION_JVERROR_CORRECTION
      params_erp[thread_id].stepsize = stepsize;
      params_erp[thread_id].vnew  = vnew_erp;  /// \TODO need to allocate vnew_erp
#endif
      params_erp[thread_id].qs  = qs;
      // if every one reorders constraints, this might just work
      // comment out below if using defaults (0 and m) so every
      // thread runs through all joints
      params_erp[thread_id].nStart = nStart;   // 0
      params_erp[thread_id].nChunkSize = nEnd - nStart; // m
      params_erp[thread_id].m = m; // m
      params_erp[thread_id].nb = nb;
      params_erp[thread_id].jb = jb;
      params_erp[thread_id].findex = findex;
      params_erp[thread_id].skip_friction = false;  // might be a save, but need to test more
      params_erp[thread_id].hi = hi;
      params_erp[thread_id].lo = lo;
      params_erp[thread_id].invMOI = invMOI;
      params_erp[thread_id].MOI= MOI;
      params_erp[thread_id].Ad = Ad;
      params_erp[thread_id].Adcfm = Adcfm;
      params_erp[thread_id].Adcfm_precon = Adcfm_precon;
      params_erp[thread_id].J = J;
      params_erp[thread_id].iMJ = iMJ;
      params_erp[thread_id].rhs_precon  = rhs_precon;
      params_erp[thread_id].J_precon  = J_precon;
      params_erp[thread_id].J_orig  = J_orig;
      params_erp[thread_id].cforce  = cforce;

      params_erp[thread_id].rhs = rhs_erp;
      params_erp[thread_id].caccel = caccel_erp;
      params_erp[thread_id].lambda = lambda_erp;

#ifdef REORDER_CONSTRAINTS
      params_erp[thread_id].last_lambda  = last_lambda_erp;
#endif

#ifdef DEBUG_CONVERGENCE_TOLERANCE
      printf("thread summary: id %d i %d m %d chunk %d start %d end %d \n",
        thread_id,i,m,chunk,nStart,nEnd);
#endif

#ifdef USE_TPROW
      if (row_threadpool && row_threadpool->size() > 1)
      {
        // skip threadpool if less than 2 threads allocated
        // printf("threading out for params_erp\n");
        row_threadpool->schedule(boost::bind(*ComputeRows, (void*)(&params_erp[thread_id])));
      }
      else
      {
        params_erp_thread = std::thread(*ComputeRows,
          (void*)(&(params_erp[thread_id])));
      }
#else
      params_erp_thread = std::thread(*ComputeRows,
        (void*)(&(params_erp[thread_id])));
#endif
    }


    // setup params for ComputeRows non_erp
    params[thread_id].thread_id = thread_id;
    params[thread_id].order     = order;
    params[thread_id].body      = body;
    params[thread_id].mutex     = mutex;
    params[thread_id].inline_position_correction = !qs->thread_position_correction;
    params[thread_id].position_correction_thread = false;
#ifdef PENETRATION_JVERROR_CORRECTION
    params[thread_id].stepsize = stepsize;
    params[thread_id].vnew  = vnew;
#endif
    params[thread_id].qs  = qs;
    // if every one reorders constraints, this might just work
    // comment out below if using defaults (0 and m) so every
    // thread runs through all joints
    params[thread_id].nStart = nStart;   // 0
    params[thread_id].nChunkSize = nEnd - nStart; // m
    params[thread_id].m = m; // m
    params[thread_id].nb = nb;
    params[thread_id].jb = jb;
    params[thread_id].findex = findex;
    params[thread_id].skip_friction = false;
    params[thread_id].hi = hi;
    params[thread_id].lo = lo;
    params[thread_id].invMOI = invMOI;
    params[thread_id].MOI= MOI;
    params[thread_id].Ad = Ad;
    params[thread_id].Adcfm = Adcfm;
    params[thread_id].Adcfm_precon = Adcfm_precon;
    params[thread_id].J = J;
    params[thread_id].iMJ = iMJ;
    params[thread_id].rhs_precon  = rhs_precon;
    params[thread_id].J_precon  = J_precon;
    params[thread_id].J_orig  = J_orig;
    params[thread_id].cforce  = cforce;

    params[thread_id].rhs = rhs;
    params[thread_id].caccel = caccel;
    params[thread_id].lambda = lambda;

    if (!qs->thread_position_correction)
    {
      /// if running without thread_position_correction, compute both in
      /// the same loop
      params[thread_id].rhs_erp = rhs_erp;
      params[thread_id].caccel_erp = caccel_erp;
      params[thread_id].lambda_erp = lambda_erp;
    }

#ifdef REORDER_CONSTRAINTS
    params[thread_id].last_lambda  = last_lambda;
#endif

#ifdef DEBUG_CONVERGENCE_TOLERANCE
    printf("thread summary: id %d i %d m %d chunk %d start %d end %d \n",
      thread_id,i,m,chunk,nStart,nEnd);
#endif
#ifdef USE_TPROW
    if (row_threadpool && row_threadpool->size() > 0)
    {
      // skip threadpool if less than 2 threads allocated
      // printf("threading out for params\n");
      row_threadpool->schedule(boost::bind(*ComputeRows, (void*)(&(params[thread_id]))));
    }
    else
      ComputeRows((void*)(&(params[thread_id])));
#else
    ComputeRows((void*)(&(params[thread_id])));
#endif

    if (qs->thread_position_correction && params_erp_thread.joinable())
    {
      IFTIMING (dTimerNow ("wait for params_erp threads"));
      params_erp_thread.join();
      IFTIMING (dTimerNow ("params_erp threads done"));
    }
  }


  // check time for scheduling, this is usually very quick
  //gettimeofday(&tv,NULL);
  //double wait_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  //printf("      quickstep done scheduling start time %f stopped time %f duration %f\n",
  //       cur_time,wait_time,wait_time - cur_time);

#ifdef USE_TPROW
  IFTIMING (dTimerNow ("wait for threads"));
  if (row_threadpool && row_threadpool->size() > 0)
    row_threadpool->wait();
  IFTIMING (dTimerNow ("threads done"));
#endif


  #ifdef REPORT_THREAD_TIMING
  gettimeofday(&tv,NULL);
  double end_time = (double)tv.tv_sec + (double)tv.tv_usec / 1.e6;
  printf("    quickstep threads start time %f stopped time %f duration %f\n",
         cur_time,end_time,end_time - cur_time);
  #endif
}

void quickstep::dxConeFrictionModel(dReal& lo_act, dReal& hi_act, dReal& lo_act_erp, dReal& hi_act_erp,
    int *jb, dRealPtr J_orig, int index, int constraint_index, int startRow, int nRows,
    const int nb, dxBody * const *body, int i, const IndexError *order, const int *findex,
    dRealPtr /*lo*/, dRealPtr hi, dRealMutablePtr lambda, dRealMutablePtr lambda_erp)
{
  // This computes the corresponding hi_act and lo_act for friction constraints.
  // For each contact, we have lambda_n, lambda_f1, and lambda_f2.
  // Now couple the two friction and to satisfy the cone Coulomb friction  model
  // tangential velocity at the contact frame:
  // v_f1 = J_f1 * v
  // v_f2 = J_f2 * v
  // v_f1 = J(0:5)*v_b1_global + J(6:11)*v_b2_global,
  // v_f2 = J2(0:5) * v_b1_global + J2(6:11)*v_b2_global
  // v = sqrt(v_f1^2 + v_f2^2);
  //
  // if (v < eps)
  //   lo_act_f1  = 0;
  //   hi_act_f1  = 0;
  //   lo_act_f2  = 0;
  //   hi_act_f2  = 0;
  // else
  //   hi_act_f1 =  abs(v_f1) / v  *  (mu * lambda_n);
  //   lo_act_f1 = - lo_act_f1;
  //   hi_act_f2 =  abs(v_f2) / v  *  (mu * lambda_n);
  //   lo_act_f2 = - lo_act_f2;
  // end
  //
  dReal v_f1, v_f2, v;
  v_f1 = 0.0; v_f2 = 0.0, v = 0.0;
  int b1 = jb[index*2];
  int b2 = jb[index*2+1];
  int bodycounter = 0;
  int ivel=0;

  // J_ptr has been scaled! use J_orig_ptr here!
  dRealPtr J_orig_ptr = J_orig + index*12;
  dReal body1_vel[6];
  dReal body2_vel[6];
  dSetZero(body1_vel, 6);
  dSetZero(body2_vel, 6);
  dxBody *const *const bodyend = body + nb;
  for (dxBody *const *bodycurr = body; bodycurr != bodyend; bodycurr++, bodycounter++)
  {
    dxBody *b_ptr = *bodycurr;
    // first direction
    if (b1 >= 0)
    {
      if (bodycounter == b1)
      {
        for (ivel = 0; ivel < 3; ivel++)
        {
          body1_vel[ivel] = b_ptr->lvel[ivel];
          body1_vel[ivel+3] = b_ptr->avel[ivel];
        }
      }
    }
    if (b2 >= 0)
    {
      if (bodycounter == b2)
      {
        for (ivel = 0; ivel < 3; ivel++)
        {
          body2_vel[ivel] = b_ptr->lvel[ivel];
          body2_vel[ivel+3] = b_ptr->avel[ivel];
        }
      }
    }
  }

  //startRow, nRows;
  int previndex, nextindex, prev_constraint_index, next_constraint_index;
  if (i == startRow)
  {
    prev_constraint_index = -100;
    next_constraint_index = constraint_index;
  }
  else if (i == startRow+nRows-1)
  {
    prev_constraint_index = constraint_index;
    next_constraint_index = -100;
  }
  else
  {
    previndex = order[i-1].index;
    nextindex = order[i+1].index;
    prev_constraint_index = findex[previndex];
    next_constraint_index = findex[nextindex];
  }

  // use previndex and nextindex to see if this is part of the same
  // contact constraint.  The problem is that with torsional friction,
  // there are 3 consecutive constraints sharing the same constraint index.
  // But we want to ignore anything to do with the third torsional
  // friction constraint row here.
  // So we need to check against previous constraint row first:
  if (constraint_index == prev_constraint_index)
  {
    dRealPtr J_prev_ptr =  J_orig + index*12 - 12;
    v_f1 = quickstep::dot6(J_prev_ptr, body1_vel) + quickstep::dot6(J_prev_ptr, body2_vel);
    v_f2 = quickstep::dot6(J_orig_ptr, body1_vel) + quickstep::dot6(J_orig_ptr, body2_vel);
  }
  else if (constraint_index == next_constraint_index)
  {
    // body1 was always the 1st body in the body pair
    dRealPtr J_next_ptr =  J_orig + index*12 + 12;
    v_f1 = quickstep::dot6(J_orig_ptr, body1_vel) + quickstep::dot6(J_orig_ptr+6, body2_vel);
    v_f2 = quickstep::dot6(J_next_ptr, body1_vel) + quickstep::dot6(J_next_ptr+6, body2_vel);
  }
  else
  {
    dMessage (d_ERR_LCP, "constraint index error");
  }


  v = sqrt(v_f1*v_f1 + v_f2*v_f2);
  if (dFabs(v) < 1e-18)
  {
    hi_act = 0.0; lo_act = 0.0;
  }
  else
  {
    if (constraint_index == next_constraint_index)
    {
      // first direction  ---> corresponds to the primary friction direction
      hi_act = (dFabs(v_f1) / v) * dFabs (hi[index] * lambda[constraint_index]);
      lo_act = -hi_act;
      hi_act_erp = (dFabs(v_f1) / v) * dFabs (hi[index] * lambda_erp[constraint_index]);
      lo_act_erp = -hi_act;
    }
    else if (constraint_index == prev_constraint_index)
    {
      // second-direction  ---> corresponds to the secondary friction direction
      hi_act = (dFabs(v_f2) / v) * dFabs (hi[index] * lambda[constraint_index]);
      lo_act = -hi_act;
      hi_act_erp = (dFabs(v_f2) / v) * dFabs (hi[index] * lambda_erp[constraint_index]);
      lo_act_erp = -hi_act;
    }
    else
    {
      dMessage (d_ERR_LCP, "constraint index error");
    }
  } // if-else (abs(v)< eps)
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
  res += dEFFICIENT_SIZE(sizeof(dxPGSLCPParameters) * m); // for params_erp
  res += dEFFICIENT_SIZE(sizeof(dxPGSLCPParameters) * m); // for params
  res += dEFFICIENT_SIZE(sizeof(boost::recursive_mutex)); // for mutex
  return res;
}

