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


#include <ode/misc.h>
#include <ode/odeconfig.h>
#include "config.h"
#include "gearbox.h"
#include "joint_internal.h"

/*
 * Double Hinge joint
 */

dxJointGearbox::dxJointGearbox(dxWorld* w) :
    dxJoint(w)
{
    ratio = 1.0;
    flags |= dJOINT_TWOBODIES;
    dSetZero( q_initial_1, 4 );
    dSetZero( q_initial_2, 4 );
    cumulative_angle1 = 0.0;
    cumulative_angle2 = 0.0;
}


void
dxJointGearbox::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 1;  // make this 2?
}


void
dxJointGearbox::getInfo1( dxJoint::Info1* info )
{
    info->m = 1;  // make this 2?
    info->nub = 1;  // make this 2?
}


void
dxJointGearbox::getInfo2( dxJoint::Info2* info )
{
    dVector3 globalAxis1, globalAxis2;

    dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], globalAxis1);
    dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], globalAxis2);

    double ang1 = getHingeAngle(
      refBody1, node[0].body, axis1, q_initial_1);
    double ang2 = getHingeAngle(
      refBody2, node[1].body, axis2, q_initial_2);

    cumulative_angle1 = dShortestAngularDistanceUpdate(cumulative_angle1,ang1);
    cumulative_angle2 = dShortestAngularDistanceUpdate(cumulative_angle2,ang2);

    double err = dShortestAngularDistance(
     cumulative_angle1, -ratio * cumulative_angle2);

    // FIXME: error calculation is not amenable to reset of poses,
    // cumulative angles might snap to wrong angular value.

    // Debug
    printf("a1(%f, %f, %f) a2(%f, %f, %f)\n",
      axis1[0], axis1[1], axis1[2],
      axis2[0], axis2[1], axis2[2]);
    printf("    global a1(%f, %f, %f) a2(%f, %f, %f)\n",
      globalAxis1[0], globalAxis1[1], globalAxis1[2],
      globalAxis2[0], globalAxis2[1], globalAxis2[2]);
    printf("    a1(%f) a1cum(%f) a2(%f) a2cum(%f) err(%f)\n",
      ang1, cumulative_angle1, ang2, cumulative_angle2, err);

    // this constraint is true if the assembly is stationary, but
    // if the whole assembly is rotating in space, the constraint
    // needs to be modified, but how?
    // J1a * q1 = rotation speed of body 1 in inertial frame
    // J2a * q2 = rotation speed of body 2 in inertial frame
    // now the reference body:
    // J1a * q1_ref_body = inertial rotation speed of body 1's refernce body
    // J2a * q2_ref_body = inertial rotation speed of body 2's refernce body
    // so the whole equation should be
    //            J1a * q1 - J1a * q1_ref_body 
    // + ratio * (J2a * q2 - J2a * q2_ref_body) = c
    // can this be accomplished with just J1a and J2a?
    // Current equation assumes q1_ref_body and q2_ref_body are zeros.
    // What if we moved ref_body stuff to the right hand side?
    //            J1a * q1 + ratio * J2a * q2
    // = c  + J1a * q1_ref_body + ratio * J2a * q2_ref_body
    //
    // below is not possible unless we can manipulate b1 and b2
    // in quickstep.cpp.
    // how about doing this in 2 rows:
    //            J1a * q1 - J1a * q1_ref_body
    // = ratio * (J2a * q2 - J2a * q2_ref_body) + c1
    // and
    //   ratio * (J2a * q2 - J2a * q2_ref_body)
    // =          J1a * q1 - J1a * q1_ref_body  + c2
    //
    // or this:
    //   J1a * q1          + ratio * J2a * q2
    // = J1a * q1_ref_body + ratio * J2a * q2_ref_body + c1
    // and
    //   J1a * q1_ref_body + ratio * J2a * q2_ref_body
    // = J1a * q1          + ratio * J2a * q2          + c1

    info->J1a[0] = globalAxis1[0];
    info->J1a[1] = globalAxis1[1];
    info->J1a[2] = globalAxis1[2];
    
    info->J2a[0] = ratio * globalAxis2[0];
    info->J2a[1] = ratio * globalAxis2[1];
    info->J2a[2] = ratio * globalAxis2[2];

    dReal k = info->fps * info->erp;
    info->c[0] = 0*-k * err +
                info->J1a[0] * refBody1->avel[0] +
                info->J1a[1] * refBody1->avel[1] +
                info->J1a[2] * refBody1->avel[2] +
                info->J2a[0] * refBody2->avel[0] +
                info->J2a[1] * refBody2->avel[1] +
                info->J2a[2] * refBody2->avel[2];
    // info->c[0] =
    //           +(info->J1a[0] * refBody1->avel[0] +
    //             info->J1a[1] * refBody1->avel[1] +
    //             info->J1a[2] * refBody1->avel[2])
    //           +(info->J2a[0] * refBody2->avel[0] +
    //             info->J2a[1] * refBody2->avel[1] +
    //             info->J2a[2] * refBody2->avel[2]);


    // dVector3 d;
    // dAddScaledVectors3(d, node[0].body->avel, node[1].body->avel,
    // 		       1.0, ratio);

    // printf("d: %f\n", dCalcVectorDot3(globalAxis1, d));
    info->findex[0] = -2;
}

void dJointSetGearboxAxis1( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointGearbox* joint = static_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    dBodyVectorFromWorld(joint->node[0].body, x, y, z, joint->axis1);
    dNormalize3(joint->axis1);
}

void dJointSetGearboxAxis2( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointGearbox* joint = static_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    dBodyVectorFromWorld(joint->node[1].body, x, y, z, joint->axis2);
    dNormalize3(joint->axis2);
}

void dJointGetGearboxAxis1( dJointID j, dVector3 result )
{
    dxJointGearbox* joint = static_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    dBodyVectorToWorld(joint->node[0].body,
                       joint->axis1[0], joint->axis1[1], joint->axis1[2],
                       result);
}

void dJointGetGearboxAxis2( dJointID j, dVector3 result )
{
    dxJointGearbox* joint = static_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    dBodyVectorToWorld(joint->node[0].body,
                       joint->axis2[0], joint->axis2[1], joint->axis2[2],
                       result);
}

void dJointSetGearboxReferenceBody( dJointID j, dBodyID b )
{
    dJointSetGearboxReferenceBody1(j, b);
    dJointSetGearboxReferenceBody2(j, b);
}

void dJointSetGearboxReferenceBody1( dJointID j, dBodyID b )
{
    dxJointGearbox* joint = dynamic_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( b, "bad body argument" );

    joint->refBody1 = b;

    if ( joint->node[0].body )
    {
      if ( b )
      {
        // q_initial_1 = node[0]q * inv(refBody1q)
        dQMultiply1(joint->q_initial_1, joint->refBody1->q,
                    joint->node[0].body->q);

        // printf("%f %f %f %f\n", b->q[0], b->q[1], b->q[2], b->q[3]);
        // printf("node[0]: %f %f %f %f | rel: %f %f %f %f\n",
        //   joint->node[0].body->q[0], joint->node[0].body->q[1],
        //   joint->node[0].body->q[2], joint->node[0].body->q[3],
        //   joint->q_initial_1[0], joint->q_initial_1[1],
        //   joint->q_initial_1[2], joint->q_initial_1[3]);
      }
      else
      {
        // set q_initial_1 to the transpose of the first body q
        joint->q_initial_1[0] = joint->node[0].body->q[0];
        joint->q_initial_1[1] = joint->node[0].body->q[1];
        joint->q_initial_1[2] = joint->node[0].body->q[2];
        joint->q_initial_1[3] = joint->node[0].body->q[3];
      }
    }
    else
    {
      if ( b )
      {
        // set q_initial_1 to the transpose of the first body q
        joint->q_initial_1[0] =   joint->refBody1->q[0];
        joint->q_initial_1[1] = - joint->refBody1->q[1];
        joint->q_initial_1[2] = - joint->refBody1->q[2];
        joint->q_initial_1[3] = - joint->refBody1->q[3];
      }
      else
      {
        // both refBody1 and node[0].body are null, nothing happens
      }
    }
}

void dJointSetGearboxReferenceBody2( dJointID j, dBodyID b )
{
    dxJointGearbox* joint = dynamic_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( b, "bad body argument" );

    joint->refBody2 = b;

    if ( joint->node[1].body )
    {
      if ( b )
      {
        dQMultiply1(joint->q_initial_2, joint->refBody2->q,
                    joint->node[1].body->q);

        // printf("node[0]: %f %f %f %f | rel: %f %f %f %f\n",
        //   joint->node[1].body->q[0], joint->node[1].body->q[1],
        //   joint->node[1].body->q[2], joint->node[1].body->q[3],
        //   joint->q_initial_2[0], joint->q_initial_2[1],
        //   joint->q_initial_2[2], joint->q_initial_2[3]);
      }
      else
      {
        // set q_initial_2 to the transpose of the first body q
        joint->q_initial_2[0] = joint->node[1].body->q[0];
        joint->q_initial_2[1] = joint->node[1].body->q[1];
        joint->q_initial_2[2] = joint->node[1].body->q[2];
        joint->q_initial_2[3] = joint->node[1].body->q[3];
      }
    }
    else
    {
      if ( b )
      {
        // set q_initial_2 to the transpose of the second body q
        joint->q_initial_2[0] =   joint->refBody2->q[0];
        joint->q_initial_2[1] = - joint->refBody2->q[1];
        joint->q_initial_2[2] = - joint->refBody2->q[2];
        joint->q_initial_2[3] = - joint->refBody2->q[3];
      }
      else
      {
        // both refBody2 and node[1].body are null, nothing happens
      }
    }
}

void dJointSetGearboxRatio( dJointID j, dReal value )
{
    dxJointGearbox* joint = dynamic_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    joint->ratio = value;
}

dReal dJointGetGearboxRatio( dJointID j )
{
    dxJointGearbox* joint = dynamic_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->ratio;
}

void dJointSetGearboxParam( dJointID j, int parameter, dReal value )
{
    dxJointGearbox* joint = static_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    switch ( parameter ) {
        case dParamCFM:
            joint->cfm = value;
            break;
        case dParamERP:
            joint->erp = value;
            break;
        default:
            dUASSERT( false, "unknown joint parameter" );
            break;
    }
}


dReal dJointGetGearboxParam( dJointID j, int parameter )
{
    dxJointGearbox* joint = static_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );

    switch ( parameter ) {
        case dParamCFM:
            return joint->cfm;
        case dParamERP:
            return joint->erp;
        default:
            return 0;
    }
}

dJointType
dxJointGearbox::type() const
{
    return dJointTypeGearbox;
}

size_t
dxJointGearbox::size() const
{
    return sizeof( *this );
}
