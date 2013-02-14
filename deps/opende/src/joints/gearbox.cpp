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


#include <ode/odeconfig.h>
#include "config.h"
#include "gearbox.h"
#include "joint_internal.h"


//****************************************************************************
// helper function: shortest_angular_distance implementation

  /*!
   * \brief normalize_angle_positive
   *
   *        Normalizes the angle to be 0 to 2*M_PI
   *        It takes and returns radians.
   */
  static inline double normalize_angle_positive(double angle)
  {
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
  }


  /*!
   * \brief normalize
   *
   * Normalizes the angle to be -M_PI circle to +M_PI circle
   * It takes and returns radians.
   *
   */
  static inline double normalize_angle(double angle)
  {
    double a = normalize_angle_positive(angle);
    if (a > M_PI)
      a -= 2.0 *M_PI;
    return a;
  }


  /*!
   * \function
   * \brief shortest_angular_distance
   *
   * Given 2 angles, this returns the shortest angular
   * difference.  The inputs and ouputs are of course radians.
   *
   * The result
   * would always be -pi <= result <= pi.  Adding the result
   * to "from" will always get you an equivelent angle to "to".
   */

  static inline double shortest_angular_distance(double from, double to)
  {
    double result = normalize_angle_positive(normalize_angle_positive(to) - normalize_angle_positive(from));

    if (result > M_PI)
      // If the result > 180,
      // It's shorter the other way.
      result = -(2.0*M_PI - result);

    return normalize_angle(result);
  }



//****************************************************************************

/*
 * Double Hinge joint
 */

dxJointGearbox::dxJointGearbox(dxWorld* w) :
    dxJointDHinge(w)
{
    ratio = 1.0;
    flags |= dJOINT_TWOBODIES;
    dSetZero( qrel1, 4 );
    dSetZero( qrel2, 4 );
    cumulative_angle1 = 0;
    cumulative_angle2 = 0;
}


void
dxJointGearbox::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 5;
}


void
dxJointGearbox::getInfo1( dxJoint::Info1* info )
{
    info->m = 1;
    info->nub = 1;
}


void
dxJointGearbox::getInfo2( dxJoint::Info2* info )
{
    dVector3 globalAxis1, globalAxis2;

    dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], globalAxis1);
    dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], globalAxis2);

    double ang1 = getHingeAngle(refBody,node[0].body,axis1,qrel1);
    cumulative_angle1 = cumulative_angle1
                     + shortest_angular_distance(cumulative_angle1,ang1);

    double ang2 = getHingeAngle(refBody,node[1].body,axis2,qrel2);
    cumulative_angle2 = cumulative_angle2
                     + shortest_angular_distance(cumulative_angle2,ang2);

    double err = shortest_angular_distance(cumulative_angle1,
      -ratio * cumulative_angle2);

    // printf("a1(%f) a2(%f) e(%f)\n", ang1, ang2, err);

    info->J1a[0] = globalAxis1[0];
    info->J1a[1] = globalAxis1[1];
    info->J1a[2] = globalAxis1[2];
    
    info->J2a[0] = ratio * globalAxis2[0];
    info->J2a[1] = ratio * globalAxis2[1];
    info->J2a[2] = ratio * globalAxis2[2];
    
    dReal k = info->fps * info->erp;
    info->c[0] = -k * err;

    // dVector3 d;
    // dAddScaledVectors3(d, node[0].body->avel, node[1].body->avel,
    // 		       1.0, ratio);

    // printf("d: %f\n", dCalcVectorDot3(globalAxis1, d));
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
    dxJointGearbox* joint = dynamic_cast<dxJointGearbox*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( b, "bad body argument" );

    joint->refBody = b;

    if ( joint->node[0].body )
    {
        dQMultiply1( joint->qrel1, joint->refBody->q, joint->node[0].body->q );
    }
    else
    {
        // set qrel1 to the transpose of the first body q
        joint->qrel1[0] =   joint->refBody->q[0];
        joint->qrel1[1] = - joint->refBody->q[1];
        joint->qrel1[2] = - joint->refBody->q[2];
        joint->qrel1[3] = - joint->refBody->q[3];
    }

    if ( joint->node[1].body )
    {
        dQMultiply1( joint->qrel2, joint->refBody->q, joint->node[1].body->q );
    }
    else
    {
        // set qrel2 to the transpose of the first body q
        joint->qrel2[0] =   joint->refBody->q[0];
        joint->qrel2[1] = - joint->refBody->q[1];
        joint->qrel2[2] = - joint->refBody->q[2];
        joint->qrel2[3] = - joint->refBody->q[3];
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
