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
#include "quickstep_util.h"
#include "quickstep_cg_lcp.h"
#include "quickstep_pgs_lcp.h"

using namespace ode;
using namespace quickstep;

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
      PGS_LCP (context,m,nb,J,J_precon,J_orig,vnew,jb,body,
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
          size_t sub3_res1 = EstimatePGS_LCPMemoryRequirements(m,nb);

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

