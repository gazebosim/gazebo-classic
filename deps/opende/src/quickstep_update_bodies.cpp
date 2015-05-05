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
#if !defined(WIN32)
#include <sys/time.h>
#endif
#include "quickstep_util.h"
#include "quickstep_update_bodies.h"
using namespace ode;

// Update the velocity, position of bodies
void quickstep::dxUpdateBodies(
#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
  dxWorldProcessContext *context,
  const int *findex,
  dRealMutablePtr const cforce,
  dxQuickStepParameters *qs,
#endif
  const int m, const int mfb, dxBody *const *body, int nb,
  dJointWithInfo1 *const jointiinfos, int nj, const dReal stepsize,
  dRealMutablePtr const lambda, dRealMutablePtr const  caccel,
  dRealMutablePtr const caccel_erp, dRealPtr Jcopy,  dRealPtr invMOI)
{
  // note that the PGS method overwrites rhs and J at this point, so
  // they should not be used again.
  if(m>0){
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
    dReal tmp_tacc[3];
    for (dxBody *const *bodycurr = body; bodycurr != bodyend; invMOIrow += 12, bodycurr++) {
      dxBody *b_ptr = *bodycurr;
      dReal body_invMass_mul_stepsize = stepsize * b_ptr->invMass;
      for (int j=0; j<3; j++) {
        b_ptr->lvel[j] += body_invMass_mul_stepsize * b_ptr->facc[j];
        tmp_tacc[j] = b_ptr->tacc[j] * stepsize;
      }
      dMultiplyAdd0_331 (b_ptr->avel, invMOIrow, tmp_tacc);
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
        /* think about minimize J*v somehow without PGSLCP...
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

size_t quickstep::dxUpdateBodiesMemoryRequirements(int m, int nb)
{
  // for vel
  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb);
  res += dEFFICIENT_SIZE(sizeof(dReal) * m); // for tmp
  res += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ
  return res;
}
