/************************************************************************************

PublicHeader:   OVR.h
Filename    :   OVR_SensorFilter.cpp
Content     :   Basic filtering of sensor data
Created     :   March 7, 2013
Authors     :   Steve LaValle, Anna Yershova, Max Katsev

Copyright   :   Copyright 2012 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "OVR_SensorFilter.h"

namespace OVR {

Vector3f SensorFilter::Median() const
{
    int half_window = Count / 2;
    float* sortx = (float*) OVR_ALLOC(Count * sizeof(float));
    float* sorty = (float*) OVR_ALLOC(Count * sizeof(float));
    float* sortz = (float*) OVR_ALLOC(Count * sizeof(float));
    float resultx = 0.0f, resulty = 0.0f, resultz = 0.0f;

    for (int i = 0; i < Count; i++) 
    {
        sortx[i] = Elements[i].x;
        sorty[i] = Elements[i].y;
        sortz[i] = Elements[i].z;
    }
    for (int j = 0; j <= half_window; j++) 
    {
        int minx = j;
        int miny = j;
        int minz = j;
        for (int k = j + 1; k < Count; k++) 
        {
            if (sortx[k] < sortx[minx]) minx = k;
            if (sorty[k] < sorty[miny]) miny = k;
            if (sortz[k] < sortz[minz]) minz = k;
        }
        const float tempx = sortx[j];
        const float tempy = sorty[j];
        const float tempz = sortz[j];
        sortx[j] = sortx[minx];
        sortx[minx] = tempx;

        sorty[j] = sorty[miny];
        sorty[miny] = tempy;

        sortz[j] = sortz[minz];
        sortz[minz] = tempz;
    }
    resultx = sortx[half_window];
    resulty = sorty[half_window];
    resultz = sortz[half_window];

    OVR_FREE(sortx);
    OVR_FREE(sorty);
    OVR_FREE(sortz);

    return Vector3f(resultx, resulty, resultz);
}

//  Only the diagonal of the covariance matrix.
Vector3f SensorFilter::Variance() const
{
    Vector3f mean = Mean();
    Vector3f total = Vector3f(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < Count; i++) 
    {
        total.x += (Elements[i].x - mean.x) * (Elements[i].x - mean.x);
        total.y += (Elements[i].y - mean.y) * (Elements[i].y - mean.y);
        total.z += (Elements[i].z - mean.z) * (Elements[i].z - mean.z);
    }
    return total / (float) Count;
}

// Should be a 3x3 matrix returned, but OVR_math.h doesn't have one
Matrix4f SensorFilter::Covariance() const
{
    Vector3f mean = Mean();
    Matrix4f total = Matrix4f(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    for (int i = 0; i < Count; i++) 
    {
        total.M[0][0] += (Elements[i].x - mean.x) * (Elements[i].x - mean.x);
        total.M[1][0] += (Elements[i].y - mean.y) * (Elements[i].x - mean.x);
        total.M[2][0] += (Elements[i].z - mean.z) * (Elements[i].x - mean.x);
        total.M[1][1] += (Elements[i].y - mean.y) * (Elements[i].y - mean.y);
        total.M[2][1] += (Elements[i].z - mean.z) * (Elements[i].y - mean.y);
        total.M[2][2] += (Elements[i].z - mean.z) * (Elements[i].z - mean.z);
    }
    total.M[0][1] = total.M[1][0];
    total.M[0][2] = total.M[2][0];
    total.M[1][2] = total.M[2][1];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            total.M[i][j] *= 1.0f / Count;
    return total;
}

Vector3f SensorFilter::PearsonCoefficient() const
{
    Matrix4f cov = Covariance();
    Vector3f pearson = Vector3f();
    pearson.x = cov.M[0][1]/(sqrt(cov.M[0][0])*sqrt(cov.M[1][1]));
    pearson.y = cov.M[1][2]/(sqrt(cov.M[1][1])*sqrt(cov.M[2][2]));
    pearson.z = cov.M[2][0]/(sqrt(cov.M[2][2])*sqrt(cov.M[0][0]));

    return pearson;
}

} //namespace OVR
