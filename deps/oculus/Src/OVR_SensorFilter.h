/************************************************************************************

PublicHeader:   OVR.h
Filename    :   OVR_SensorFilter.h
Content     :   Basic filtering of sensor data
Created     :   March 7, 2013
Authors     :   Steve LaValle, Anna Yershova, Max Katsev

Copyright   :   Copyright 2012 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef OVR_SensorFilter_h
#define OVR_SensorFilter_h

#include "Kernel/OVR_Math.h"


namespace OVR {

// A simple circular buffer data structure that stores last N elements in an array
template <typename T>
class CircularBuffer
{
protected:
    enum
    {
        DefaultFilterCapacity = 20
    };

    int         LastIdx;                    // The index of the last element that was added to the buffer
    int         Capacity;                   // The buffer size (maximum number of elements)
    int         Count;                      // Number of elements in the filter
    T*          Elements;

public:
    CircularBuffer(int capacity = DefaultFilterCapacity) 
        : LastIdx(-1), Capacity(capacity), Count(0)
    {
        Elements = (T*)OVR_ALLOC(capacity * sizeof(T));
        for (int i = 0; i < Capacity; i++)
            Elements[i] = T();
    }

    ~CircularBuffer() {
        OVR_FREE(Elements);
    }

private:
    // Make the class non-copyable
    CircularBuffer(const CircularBuffer& other);
    CircularBuffer& operator=(const CircularBuffer& other);

public:
    // Add a new element to the filter
    void AddElement (const T &e)
    {
        LastIdx = (LastIdx + 1) % Capacity;
        Elements[LastIdx] = e;
        if (Count < Capacity)
            Count++;
    }

    // Get element i.  0 is the most recent, 1 is one step ago, 2 is two steps ago, ...
    T GetPrev(int i = 0) const
    {
		OVR_ASSERT(i >= 0);
        if (i >= Count) // return 0 if the filter doesn't have enough elements
            return T();
        int idx = (LastIdx - i);
        if (idx < 0) // Fix the wraparound case
            idx += Capacity;
		OVR_ASSERT(idx >= 0); // Multiple wraparounds not allowed
        return Elements[idx];
    }
};

// A base class for filters that maintains a buffer of sensor data taken over time and implements
// various simple filters, most of which are linear functions of the data history.
// Maintains the running sum of its elements for better performance on large capacity values
template <typename T>
class SensorFilterBase : public CircularBuffer<T>
{
protected:
    T RunningTotal;               // Cached sum of the elements

public:
    SensorFilterBase(int capacity = CircularBuffer<T>::DefaultFilterCapacity) : CircularBuffer<T>(capacity), RunningTotal() { };

    // Add a new element to the filter
    // Updates the running sum value
    void AddElement (const T &e)
    {
        int NextIdx = (this->LastIdx + 1) % this->Capacity;
        RunningTotal += (e - this->Elements[NextIdx]);
        CircularBuffer<T>::AddElement(e);
        if (this->LastIdx == 0)
        {
            // update the cached total to avoid error accumulation
            RunningTotal = T();
            for (int i = 0; i < this->Count; i++)
                RunningTotal += this->Elements[i];
        } 
    }

    // Simple statistics
    T Total() const 
    { 
        return RunningTotal; 
    }

    T Mean() const
    {
        return (this->Count == 0) ? T() : (Total() / (float) this->Count);
    }

    // A popular family of smoothing filters and smoothed derivatives
    T SavitzkyGolaySmooth8() const
    {
        OVR_ASSERT(this->Capacity >= 8);
        return this->GetPrev(0)*0.41667f +
                this->GetPrev(1)*0.33333f +
                this->GetPrev(2)*0.25f +
                this->GetPrev(3)*0.16667f +
                this->GetPrev(4)*0.08333f -
                this->GetPrev(6)*0.08333f -
                this->GetPrev(7)*0.16667f;
    }

    T SavitzkyGolayDerivative4() const
    {
        OVR_ASSERT(this->Capacity >= 4);
        return this->GetPrev(0)*0.3f +
                this->GetPrev(1)*0.1f -
                this->GetPrev(2)*0.1f -
                this->GetPrev(3)*0.3f;
    }

    T SavitzkyGolayDerivative5() const
    {
            OVR_ASSERT(this->Capacity >= 5);
            return this->GetPrev(0)*0.2f +
                    this->GetPrev(1)*0.1f -
                    this->GetPrev(3)*0.1f -
                    this->GetPrev(4)*0.2f;
    }

    T SavitzkyGolayDerivative12() const
    {
        OVR_ASSERT(this->Capacity >= 12);
        return this->GetPrev(0)*0.03846f +
                this->GetPrev(1)*0.03147f +
                this->GetPrev(2)*0.02448f +
                this->GetPrev(3)*0.01748f +
                this->GetPrev(4)*0.01049f +
                this->GetPrev(5)*0.0035f -
                this->GetPrev(6)*0.0035f -
                this->GetPrev(7)*0.01049f -
                this->GetPrev(8)*0.01748f -
                this->GetPrev(9)*0.02448f -
                this->GetPrev(10)*0.03147f -
                this->GetPrev(11)*0.03846f;
    } 

    T SavitzkyGolayDerivativeN(int n) const
    {    
        OVR_ASSERT(this->Capacity >= n);
        int m = (n-1)/2;
        T result = T();
        for (int k = 1; k <= m; k++) 
        {
            int ind1 = m - k;
            int ind2 = n - m + k - 1;
            result += (this->GetPrev(ind1) - this->GetPrev(ind2)) * (float) k;
        }
        float coef = 3.0f/(m*(m+1.0f)*(2.0f*m+1.0f));
        result = result*coef;
        return result;
    }
};

// This class maintains a buffer of sensor data taken over time and implements
// various simple filters, most of which are linear functions of the data history.
class SensorFilter : public SensorFilterBase<Vector3f>
{
public:
    SensorFilter(int capacity = DefaultFilterCapacity) : SensorFilterBase<Vector3f>(capacity) { };

    // Simple statistics
    Vector3f Median() const;
    Vector3f Variance() const; // The diagonal of covariance matrix
    Matrix4f Covariance() const;
    Vector3f PearsonCoefficient() const;
};

} //namespace OVR

#endif // OVR_SensorFilter_h
