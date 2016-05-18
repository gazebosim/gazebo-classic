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


#include <gazebo/ode/misc.h>
#include "config.h"
#include "screw.h"
#include "joint_internal.h"

// screw

dxJointScrew::dxJointScrew( dxWorld *w ) :
        dxJoint( w )
{
    dSetZero( anchor1, 4 );
    dSetZero( anchor2, 4 );
    dSetZero( axis1, 4 );
    axis1[0] = 1;
    dSetZero( axis2, 4 );
    axis2[0] = 1;
    dSetZero( qrel, 4 );
    limot.init( world );
    cumulative_angle = 0;
    thread_pitch = 1.0;  // rad/m (3141.6 rad/m is about 0.002 m/rev)
}


void
dxJointScrew::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 6;
}


void
dxJointScrew::getInfo1( dxJoint::Info1 *info )
{
    info->nub = 5;

    info->m = 5;

    // if proper joint limits are specified

    // see if we're at a joint limit for the rotational hinge
    if ( limot.lostop <= limot.histop )
    {
        dReal angle = getHingeAngle( node[0].body,
                                     node[1].body,
                                     axis1, qrel );
        // from angle, update cumulative_angle, which does not wrap
        cumulative_angle =
          dShortestAngularDistanceUpdate(cumulative_angle,angle);

        // printf("angle: %f lo[%f] hi[%f]\n", cumulative_angle,
        //   limot.lostop, limot.histop);

        if ( limot.testRotationalLimit( cumulative_angle ) )
            info->m = 6;

        // FIXME: reset does not reset joint angle if more than 2pi
        // printf("cumulative angle: %f  lo: %f  hi:  %f test: %f\n",
        //   cumulative_angle, limot.lostop, limot.histop,
        //   limot.testRotationalLimit( cumulative_angle ));
    }

    /* uncommnet to enforce slider joint limit
    // see if we're at a joint limit for the slider
    limot.limit = 0;
    if ( ( limot.lostop > -dInfinity || limot.histop < dInfinity ) &&
            limot.lostop <= limot.histop )
    {
        // measure joint position
        dReal pos = dJointGetScrewPosition ( this );

        // printf("pos: %f lo[%f] hi[%f]\n", pos, limot.lostop, limot.histop);

        if ( pos <= limot.lostop )
        {
            limot.limit = 1;
            limot.limit_err = pos - limot.lostop;
            info->m = 6;
        }
        else if ( pos >= limot.histop )
        {
            limot.limit = 2;
            limot.limit_err = pos - limot.histop;
            info->m = 6;
        }
    }
    */
}


void
dxJointScrew::getInfo2( dxJoint::Info2 *info )
{
    // Added by OSRF
    //
    // Screw Constraint Overview 
    //
    // make 5 constraint rows.
    // given screw axis, first create two orthogonal axis p and q.
    // row 1: linear constraint along p
    // row 2: linear constraint along q
    // row 3: screw constraint about user specified axis
    // row 4: rotational constraint about p
    // row 5: rotational constraint about q

    // Added by OSRF
    // If joint values of erp and cfm are negative, then ignore them.
    // info->erp, info->cfm already have the global values from quickstep
    if (this->erp >= 0)
      info->erp = erp;
    if (this->cfm >= 0)
    {
      info->cfm[0] = cfm;
      info->cfm[1] = cfm;
      info->cfm[2] = cfm;
      info->cfm[3] = cfm;
      info->cfm[4] = cfm;
    }

    // constraint rows 1 to 3
    {
      // pull out pos and R for both bodies. also get the `connection'
      // vector pos2-pos1.
      dReal *pos1, *pos2, *R1, *R2;
      dVector3 cgdiff;
      cgdiff[0] = cgdiff[1] = cgdiff[2] = 0;
      pos1 = node[0].body->posr.pos;
      R1 = node[0].body->posr.R;
      if ( node[1].body )
      {
          pos2 = node[1].body->posr.pos;
          R2 = node[1].body->posr.R;
          for (int i = 0; i < 3; ++i )
          {
              // store distance between cg's in cgdiff
              cgdiff[i] = pos2[i] - pos1[i];
          }
      }
      else
      {
          pos2 = 0;
          R2 = 0;
      }



      // compute error for screw due to drift
      dReal lin_disp; // linear displacement
      dReal lin_err; // linear displacement
      {
        // get linear disp for screw
        // get axis1 in global coordinates
        dVector3 ax1, q;
        dMultiply0_331 ( ax1, node[0].body->posr.R, axis1 );
        if ( node[1].body )
        {
            // get body2 + offset point in global coordinates
            dMultiply0_331 ( q, node[1].body->posr.R, offset );
            //printf("debug offset q[%f %f %f] p0[%f %f %f] p1[%f %f %f] \t",
            //  q[0],q[1],q[2],
            //  node[0].body->posr.pos[0], node[0].body->posr.pos[1],
            //  node[0].body->posr.pos[2],
            //  node[1].body->posr.pos[0], node[1].body->posr.pos[1],
            //  node[1].body->posr.pos[2]);
            for ( int ii = 0; ii < 3; ++ii )
              q[ii] = node[0].body->posr.pos[ii] - q[ii] -
                node[1].body->posr.pos[ii];
        }
        else
        {
            q[0] = node[0].body->posr.pos[0] - offset[0];
            q[1] = node[0].body->posr.pos[1] - offset[1];
            q[2] = node[0].body->posr.pos[2] - offset[2];
        }
        lin_disp = dCalcVectorDot3 ( ax1, q );

        // linear error should be length scaled, BUT
        if (dFabs(thread_pitch) > 1.0)
        {
          // constraint is written in length scale, so
          // linear error is length scaled.
          lin_err = -(lin_disp-cumulative_angle/thread_pitch);
        }
        else
        {
          // here the entire constraint equation, including lin_err
          // is multiplied by thread_pitch for |thread_pitch| less than 1.0
          // for added numerical stability,
          lin_err = -(thread_pitch*lin_disp-cumulative_angle);
        }
        // printf("lin disp: %f lin err: %f\n", lin_disp, lin_err);
      }



      int s0 = 0 * info->rowskip;
      int s1 = 1 * info->rowskip;
      int s2 = 2 * info->rowskip;

      // remaining two rows. we want: vel2 = vel1 + w1 x cgdiff ... but this would
      // result in three equations, so we project along the planespace vectors
      // so that sliding along the slider axis is disregarded. for symmetry we
      // also substitute (w1+w2)/2 for w1, as w1 is supposed to equal w2.

      // ax1 is axis1 converted to body1 frame
      dVector3 ax1;
      dMultiply0_331 ( ax1, R1, axis1 );

      // p and q are vectors perpendicular to ax1 in body1 frame
      dVector3 p, q;
      dPlaneSpace ( ax1, p, q );

      // linear constraints for the hinge joint
      // perpendicular to the sliding axis direction.
      for (int i = 0; i < 3; ++i ) info->J1l[s0+i] = p[i];
      for (int i = 0; i < 3; ++i ) info->J1l[s1+i] = q[i];

      // if p and q do not pass through body CG's,
      // we need to add angular constraints to balance out the forces
      // from these linear constraints.  See below:

      // a1 and a2 are axis vectors in the body frame
      // (whereas anchor1 and anchor2 are in world frame).
      // anchor1 is the vector from CG to joint anchor in world frame.
      dVector3 a1, a2;
      dMultiply0_331( a1, R1, anchor1 );

      // tmpp is a vector perpendicular to a1 and p in body frame,
      // it is the direction of the angular constraint that will
      // cancel out moment generated by linear constraint p
      // if p does not pass through CG.
      {
        dVector3 tmpp;
        dCalcVectorCross3(tmpp, p, a1);
        for (int i = 0; i < 3; ++i ) info->J1a[s0+i] = -tmpp[i];
      }

      // tmpq is similar to tmpp, but for q.
      {
        dVector3 tmpq;
        dCalcVectorCross3(tmpq, q, a1);
        for (int i = 0; i < 3; ++i ) info->J1a[s1+i] = -tmpq[i];
      }

      // screw constraint:
      // now constrain the sliding axis by rotation of the other body
      if (dFabs(thread_pitch) > 1.0)
      {
        for (int i = 0; i < 3; ++i ) info->J1l[s2+i] = ax1[i];
        for (int i = 0; i < 3; ++i ) info->J1a[s2+i] = -ax1[i]/thread_pitch;
      }
      else
      {
        // here the entire constraint equation, including lin_err
        // is multiplied by thread_pitch for |thread_pitch| less than 1.0
        // for added numerical stability,
        for (int i = 0; i < 3; ++i ) info->J1l[s2+i] = ax1[i]*thread_pitch;
        for (int i = 0; i < 3; ++i ) info->J1a[s2+i] = -ax1[i];
      }

      // repeat above for child body if one exists
      if ( node[1].body )
      {
        // linear constraints for s0 and s1
        for (int i = 0; i < 3; ++i ) info->J2l[s0+i] = -p[i];
        for (int i = 0; i < 3; ++i ) info->J2l[s1+i] = -q[i];

        // angular compensation if p and q do not pass through CG
        dMultiply0_331( a2, R2, anchor2 );
        dVector3 tmpp;
        dCalcVectorCross3(tmpp, p, a2);
        for (int i = 0; i < 3; ++i ) info->J2a[s0+i] = tmpp[i];
        dVector3 tmpq;
        dCalcVectorCross3(tmpq, q, a2);
        for (int i = 0; i < 3; ++i ) info->J2a[s1+i] = tmpq[i];

        // screw constraint:
        // constrain the sliding axis by rotation of the other body
        if (dFabs(thread_pitch) > 1.0)
        {
          for (int i = 0; i < 3; ++i ) info->J2a[s2+i] =  ax1[i]/thread_pitch;
          for (int i = 0; i < 3; ++i ) info->J2l[s2+i] = -ax1[i];
        }
        else
        {
          // here the entire constraint equation, including lin_err
          // is multiplied by thread_pitch for |thread_pitch| less than 1.0
          // for added numerical stability,
          for (int i = 0; i < 3; ++i ) info->J2a[s2+i] =  ax1[i];
          for (int i = 0; i < 3; ++i ) info->J2l[s2+i] = -ax1[i]*thread_pitch;
        }
      }

      // debug
      // printf ("anchor1 %f %f %f\n", anchor1[0], anchor1[1], anchor1[2]);
      // printf ("a1 %f %f %f\n", a1[0], a1[1], a1[2]);
      // printf ("ax1 %f %f %f\n", ax1[0], ax1[1], ax1[2]);
      // printf ("tmpp %f %f %f\n", tmpp[0], tmpp[1], tmpp[2]);
      // printf ("p %f %f %f\n", p[0], p[1], p[2]);
      // printf ("q %f %f %f\n", q[0], q[1], q[2]);
      // printf ("J1a[s0] %f %f %f\n", info->J1a[s0+0], info->J1a[s0+1],
      //   info->J1a[s0+2]);
      // printf ("J1a[s1] %f %f %f\n", info->J1a[s1+0], info->J1a[s1+1],
      //   info->J1a[s1+2]);
      // info->J1a[s0+0] = 1;
      // info->J1a[s0+1] = 0;
      // info->J1a[s0+2] = 0;
      // info->J1a[s1+0] = -1;
      // info->J1a[s1+1] = 0;
      // info->J1a[s1+2] = 0;
      // printf("screw err lin[%f], ang[%f], diff[%f] tp[%f]\n",
      //   thread_pitch*lin_disp, cumulative_angle, lin_err, thread_pitch);

      // compute last two elements of right hand side. we want to align the offset
      // point (in body 2's frame) with the center of body 1.
      dReal k = info->fps * info->erp;
      if ( node[1].body )
      {
          // dVector3 ofs;  // offset point in global coordinates
          // dMultiply0_331 ( ofs, R2, offset );
          // for (int i = 0; i < 3; ++i ) cgdiff[i] += ofs[i];

          // error between body anchors
          dVector3 error12;
          for (int i = 0; i < 3; ++i)
            error12[i] = a2[i] + node[1].body->posr.pos[i] - a1[i] -
              node[0].body->posr.pos[i];

          // error in the p direction is error12 dot p
          info->c[0] = k * (dCalcVectorDot3(error12, p));
          // error in the q direction is error12 dot p
          info->c[1] = k * (dCalcVectorDot3(error12, q));
          // interpenetration error for screw constraint
          info->c[2] = k * lin_err;
      }
      else
      {
          // debug
          // printf ("anchor1 %f %f %f\n", anchor1[0], anchor1[1], anchor1[2]);
          // printf ("anchor2 %f %f %f\n", anchor2[0], anchor2[1], anchor2[2]);
          // printf ("a1 %f %f %f\n", a1[0], a1[1], a1[2]);
          // printf ("p1 %f %f %f\n",
          //   node[0].body->posr.pos[0],
          //   node[0].body->posr.pos[1],
          //   node[0].body->posr.pos[2]);

          // error of body's anchor
          dVector3 error1;
          for (int i = 0; i < 3; ++i) error1[i] = anchor2[i] - a1[i] -
            node[0].body->posr.pos[i];
          // printf ("error1 %f %f %f\n", error1[0], error1[1], error1[2]);

          // error in the p direction is error1 dot p
          info->c[0] = k * (dCalcVectorDot3(error1, p));
          // error in the q direction
          info->c[1] = k * (dCalcVectorDot3(error1, q));
          // interpenetration error for screw constraint
          info->c[2] = k * lin_err;

          if ( flags & dJOINT_REVERSE )
              for (int i = 0; i < 3; ++i ) ax1[i] = -ax1[i];
      }

      // uncommnet to enforce slider joint limit
      // limot.addLimot ( this, info, 5, ax1, 0 );
    }

    // constraint rows 4 and 5
    {
      // set the two hinge rows. the screw axis should be the only unconstrained
      // rotational axis, the angular velocity of the two bodies perpendicular to
      // the hinge axis should be equal. thus the constraint equations are
      //    p*w1 - p*w2 = 0
      //    q*w1 - q*w2 = 0
      // where p and q are unit vectors normal to the hinge axis, and w1 and w2
      // are the angular velocity vectors of the two bodies.

      dVector3 ax1;  // length 1 joint axis in global coordinates, from 1st body
      dVector3 p, q; // plane space vectors for ax1
      dMultiply0_331( ax1, node[0].body->posr.R, axis1 );
      dPlaneSpace( ax1, p, q );

      int s3 = 3 * info->rowskip;
      int s4 = 4 * info->rowskip;

      info->J1a[s3+0] = p[0];
      info->J1a[s3+1] = p[1];
      info->J1a[s3+2] = p[2];
      info->J1a[s4+0] = q[0];
      info->J1a[s4+1] = q[1];
      info->J1a[s4+2] = q[2];

      if ( node[1].body )
      {
          info->J2a[s3+0] = -p[0];
          info->J2a[s3+1] = -p[1];
          info->J2a[s3+2] = -p[2];
          info->J2a[s4+0] = -q[0];
          info->J2a[s4+1] = -q[1];
          info->J2a[s4+2] = -q[2];
      }

      // compute the right hand side of the constraint equation. set relative
      // body velocities along p and q to bring the screw back into alignment.
      // if ax1,ax2 are the unit length screw axes as computed from body1 and
      // body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
      // if `theta' is the angle between ax1 and ax2, we need an angular velocity
      // along u to cover angle erp*theta in one step :
      //   |angular_velocity| = angle/time = erp*theta / stepsize
      //                      = (erp*fps) * theta
      //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
      //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
      // ...as ax1 and ax2 are unit length. if theta is smallish,
      // theta ~= sin(theta), so
      //    angular_velocity  = (erp*fps) * (ax1 x ax2)
      // ax1 x ax2 is in the plane space of ax1, so we project the angular
      // velocity to p and q to find the right hand side.

      dVector3 ax2, b;
      if ( node[1].body )
      {
          dMultiply0_331( ax2, node[1].body->posr.R, axis2 );
      }
      else
      {
          ax2[0] = axis2[0];
          ax2[1] = axis2[1];
          ax2[2] = axis2[2];
      }
      dCalcVectorCross3( b, ax1, ax2 );
      dReal k = info->fps * info->erp;
      info->c[3] = k * dCalcVectorDot3( b, p );
      info->c[4] = k * dCalcVectorDot3( b, q );

      // enforcing rotation joint limit
      limot.addLimot( this, info, 5, ax1, 1 );
    }
}



void dJointSetScrewAnchor( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Screw );
    setAnchors( joint, x, y, z, joint->anchor1, joint->anchor2 );
    joint->computeInitialRelativeRotation();
}


void dJointSetScrewAnchorDelta( dJointID j, dReal x, dReal y, dReal z,
  dReal dx, dReal dy, dReal dz )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Screw );

    if ( joint->node[0].body )
    {
        dReal q[4];
        q[0] = x - joint->node[0].body->posr.pos[0];
        q[1] = y - joint->node[0].body->posr.pos[1];
        q[2] = z - joint->node[0].body->posr.pos[2];
        q[3] = 0;
        dMultiply1_331( joint->anchor1, joint->node[0].body->posr.R, q );

        if ( joint->node[1].body )
        {
            q[0] = x - joint->node[1].body->posr.pos[0];
            q[1] = y - joint->node[1].body->posr.pos[1];
            q[2] = z - joint->node[1].body->posr.pos[2];
            q[3] = 0;
            dMultiply1_331( joint->anchor2, joint->node[1].body->posr.R, q );
        }
        else
        {
            // Move the relative displacement between the passive body and the
            //  anchor in the same direction as the passive body has just moved
            joint->anchor2[0] = x + dx;
            joint->anchor2[1] = y + dy;
            joint->anchor2[2] = z + dz;
        }
    }
    joint->anchor1[3] = 0;
    joint->anchor2[3] = 0;

    joint->computeInitialRelativeRotation();
}



void dJointSetScrewAxis( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Screw );
    setAxes( joint, x, y, z, joint->axis1, joint->axis2 );
    joint->computeOffset();
    joint->computeInitialRelativeRotation();
}

void dJointSetScrewAxisOffset( dJointID j, dReal x, dReal y, dReal z, dReal dangle )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Screw );
    setAxes( joint, x, y, z, joint->axis1, joint->axis2 );
    joint->computeInitialRelativeRotation();

    if ( joint->flags & dJOINT_REVERSE ) dangle = -dangle;

    dQuaternion qAngle, qOffset;
    dQFromAxisAndAngle(qAngle, x, y, z, dangle);
    dQMultiply3(qOffset, qAngle, joint->qrel);
    joint->qrel[0] = qOffset[0];
    joint->qrel[1] = qOffset[1];
    joint->qrel[2] = qOffset[2];
    joint->qrel[3] = qOffset[3];
}



void dJointGetScrewAnchor( dJointID j, dVector3 result )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, Screw );
    if ( joint->flags & dJOINT_REVERSE )
        getAnchor2( joint, result, joint->anchor2 );
    else
        getAnchor( joint, result, joint->anchor1 );
}


void dJointGetScrewAnchor2( dJointID j, dVector3 result )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, Screw );
    if ( joint->flags & dJOINT_REVERSE )
        getAnchor( joint, result, joint->anchor1 );
    else
        getAnchor2( joint, result, joint->anchor2 );
}


void dJointGetScrewAxis( dJointID j, dVector3 result )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, Screw );
    getAxis( joint, result, joint->axis1 );
}


void dJointSetScrewParam( dJointID j, int parameter, dReal value )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Screw );
    switch (parameter)
    {
      case dParamERP:
        joint->erp = value;
        break;
      case dParamCFM:
        joint->cfm = value;
        // dParamCFM label is also used for normal_cfm
        joint->limot.set( parameter, value );
        break;
      default:
        joint->limot.set( parameter, value );
        break;
    }
}


dReal dJointGetScrewParam( dJointID j, int parameter )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Screw );
    switch (parameter)
    {
      case dParamERP:
        return joint->erp;
      case dParamCFM:
        return joint->cfm;
      default:
        return joint->limot.get( parameter );
    }
}


dReal dJointGetScrewAngle( dJointID j )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dAASSERT( joint );
    checktype( joint, Screw );
    if ( joint->node[0].body )
    {
        dReal ang = getHingeAngle( joint->node[0].body,
                                   joint->node[1].body,
                                   joint->axis1,
                                   joint->qrel );
        // from angle, update cumulative_angle, which does not wrap
        dReal joint_angle =
          dShortestAngularDistanceUpdate(joint->cumulative_angle,ang);
        if ( joint->flags & dJOINT_REVERSE )
            return -joint_angle;
        else
            return joint_angle;
    }
    else return 0;
}


dReal dJointGetScrewAngleRate( dJointID j )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dAASSERT( joint );
    checktype( joint, Screw );
    if ( joint->node[0].body )
    {
        dVector3 axis;
        dMultiply0_331( axis, joint->node[0].body->posr.R, joint->axis1 );
        dReal rate = dCalcVectorDot3( axis, joint->node[0].body->avel );
        if (joint->node[1].body)
          rate -= dCalcVectorDot3( axis, joint->node[1].body->avel );
        if (joint->flags & dJOINT_REVERSE)
          rate = - rate;
        return rate;
    }
    else return 0;
}


dReal dJointGetScrewPosition ( dJointID j )
{
    dxJointScrew* joint = ( dxJointScrew* ) j;
    dUASSERT ( joint, "bad joint argument" );
    checktype ( joint, Screw );

    // get axis1 in global coordinates
    dVector3 ax1, q;
    if (!joint->node[0].body)
      return 0;

    dMultiply0_331 ( ax1, joint->node[0].body->posr.R, joint->axis1 );

    if (joint->node[1].body)
    {
        // get body2 + offset point in global coordinates
        dMultiply0_331 ( q, joint->node[1].body->posr.R, joint->offset );
        for (int i = 0; i < 3; ++i )
            q[i] = joint->node[0].body->posr.pos[i]
                   - q[i]
                   - joint->node[1].body->posr.pos[i];
    }
    else
    {
        q[0] = joint->node[0].body->posr.pos[0] - joint->offset[0];
        q[1] = joint->node[0].body->posr.pos[1] - joint->offset[1];
        q[2] = joint->node[0].body->posr.pos[2] - joint->offset[2];

        if ( joint->flags & dJOINT_REVERSE )
        {
            // N.B. it could have been simplier to only inverse the sign of
            //      the dCalcVectorDot3 result but this case is exceptional and
            //      doing the check for all case can decrease the performance.
            ax1[0] = -ax1[0];
            ax1[1] = -ax1[1];
            ax1[2] = -ax1[2];
        }
    }

    return dCalcVectorDot3 ( ax1, q );
}


dReal dJointGetScrewPositionRate ( dJointID j )
{
    dxJointScrew* joint = ( dxJointScrew* ) j;
    dUASSERT ( joint, "bad joint argument" );
    checktype ( joint, Screw );

    // get axis1 in global coordinates
    dVector3 ax1;
    dMultiply0_331 ( ax1, joint->node[0].body->posr.R, joint->axis1 );

    if ( joint->node[1].body )
    {
        return dCalcVectorDot3 ( ax1, joint->node[0].body->lvel ) -
               dCalcVectorDot3 ( ax1, joint->node[1].body->lvel );
    }
    else
    {
        dReal rate = dCalcVectorDot3 ( ax1, joint->node[0].body->lvel );
        if ( joint->flags & dJOINT_REVERSE ) rate = - rate;
        return rate;
    }
}

void dJointAddScrewTorque( dJointID j, dReal torque )
{
    dxJointScrew* joint = ( dxJointScrew* )j;
    dVector3 axis;
    dAASSERT( joint );
    checktype( joint, Screw );

    if ( joint->flags & dJOINT_REVERSE )
        torque = -torque;

    getAxis( joint, axis, joint->axis1 );
    axis[0] *= torque;
    axis[1] *= torque;
    axis[2] *= torque;

    if ( joint->node[0].body != 0 )
        dBodyAddTorque( joint->node[0].body, axis[0], axis[1], axis[2] );
    if ( joint->node[1].body != 0 )
        dBodyAddTorque( joint->node[1].body, -axis[0], -axis[1], -axis[2] );
}

void dJointAddScrewForce ( dJointID j, dReal force )
{
    dxJointScrew* joint = ( dxJointScrew* ) j;
    dVector3 axis;
    dUASSERT ( joint, "bad joint argument" );
    checktype ( joint, Screw );

    if ( joint->flags & dJOINT_REVERSE )
        force -= force;

    getAxis ( joint, axis, joint->axis1 );
    axis[0] *= force;
    axis[1] *= force;
    axis[2] *= force;

    if ( joint->node[0].body != 0 )
        dBodyAddForce ( joint->node[0].body, axis[0], axis[1], axis[2] );
    if ( joint->node[1].body != 0 )
        dBodyAddForce ( joint->node[1].body, -axis[0], -axis[1], -axis[2] );

    if ( joint->node[0].body != 0 && joint->node[1].body != 0 )
    {
        // linear torque decoupling:
        // we have to compensate the torque, that this screw force may generate
        // if body centers are not aligned along the screw axis

        dVector3 ltd; // Linear Torque Decoupling vector (a torque)

        dVector3 cgdiff;
        cgdiff[0] = REAL ( 0.5 ) * ( joint->node[1].body->posr.pos[0] -
          joint->node[0].body->posr.pos[0] );
        cgdiff[1] = REAL ( 0.5 ) * ( joint->node[1].body->posr.pos[1] -
          joint->node[0].body->posr.pos[1] );
        cgdiff[2] = REAL ( 0.5 ) * ( joint->node[1].body->posr.pos[2] -
          joint->node[0].body->posr.pos[2] );
        dCalcVectorCross3( ltd, cgdiff, axis );

        dBodyAddTorque ( joint->node[0].body, ltd[0], ltd[1], ltd[2] );
        dBodyAddTorque ( joint->node[1].body, ltd[0], ltd[1], ltd[2] );
    }
}



dJointType
dxJointScrew::type() const
{
    return dJointTypeScrew;
}



size_t
dxJointScrew::size() const
{
    return sizeof( *this );
}


void
dxJointScrew::setRelativeValues()
{
    dVector3 vec;
    dJointGetScrewAnchor(this, vec);
    setAnchors( this, vec[0], vec[1], vec[2], anchor1, anchor2 );

    dJointGetScrewAxis(this, vec);
    setAxes( this,  vec[0], vec[1], vec[2], axis1, axis2 );
    computeInitialRelativeRotation();
}


/// Compute initial relative rotation body1 -> body2, or env -> body1
void
dxJointScrew::computeInitialRelativeRotation()
{
    if ( node[0].body )
    {
        if ( node[1].body )
        {
            dQMultiply1( qrel, node[0].body->q, node[1].body->q );
        }
        else
        {
            // set qrel to the transpose of the first body q
            qrel[0] =  node[0].body->q[0];
            qrel[1] = -node[0].body->q[1];
            qrel[2] = -node[0].body->q[2];
            qrel[3] = -node[0].body->q[3];
        }
    }
}

/// Compute center of body1 w.r.t body 2
void
dxJointScrew::computeOffset()
{
    if ( node[1].body )
    {
        dVector3 cgdiff;
        cgdiff[0] = node[0].body->posr.pos[0] - node[1].body->posr.pos[0];
        cgdiff[1] = node[0].body->posr.pos[1] - node[1].body->posr.pos[1];
        cgdiff[2] = node[0].body->posr.pos[2] - node[1].body->posr.pos[2];

        dMultiply1_331 ( offset, node[1].body->posr.R, cgdiff );
    }
    else if ( node[0].body )
    {
        offset[0] = node[0].body->posr.pos[0];
        offset[1] = node[0].body->posr.pos[1];
        offset[2] = node[0].body->posr.pos[2];
    }
}

