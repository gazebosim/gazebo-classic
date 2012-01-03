#ifndef CUDA_MATH_H
#define CUDA_MATH_H

#include <stdio.h>

#include "parallel_common.h"

template <typename T> struct vec3         { typedef float   Type; typedef float* PtrType; }; // dummy
template <>           struct vec3<float>  { typedef float3  Type; typedef float3* PtrType; };
template <>           struct vec3<double> { typedef double3 Type; typedef double3* PtrType; };

template <typename T> struct vec4         { typedef float   Type; typedef float* PtrType; }; // dummy
template <>           struct vec4<float>  { typedef float4  Type; typedef float4* PtrType; };
template <>           struct vec4<double> { typedef double4 Type; typedef double4* PtrType; };

template <typename T>
inline dxDevice T readAndReplace(T* buffer, const T& element) {
  T value = *buffer;
  *buffer = element;
  return value;
}

inline dxHost dxDevice void add_assign_volatile(volatile float3& a, float3& b, volatile float3& c) {
  a.x = b.x = b.x + c.x;
  a.y = b.y = b.y + c.y;
  a.z = b.z = b.z + c.z;
}
inline dxHost dxDevice void add_assign_volatile(volatile double3& a, double3& b, volatile double3& c) {
  a.x = b.x = b.x + c.x;
  a.y = b.y = b.y + c.y;
  a.z = b.z = b.z + c.z;
}

inline dxHost dxDevice void add_assign_volatile(volatile float4& a, float4& b, volatile float4& c) {
  a.x = b.x = b.x + c.x;
  a.y = b.y = b.y + c.y;
  a.z = b.z = b.z + c.z;
}
inline dxHost dxDevice void add_assign_volatile(volatile double4& a, double4& b, volatile double4& c) {
  a.x = b.x = b.x + c.x;
  a.y = b.y = b.y + c.y;
  a.z = b.z = b.z + c.z;
}

inline dxHost dxDevice void assign_volatile(volatile float3& a, float3& b) {
  a.x = b.x; a.y = b.y; a.z = b.z;
}
inline dxHost dxDevice void assign_volatile(volatile double3& a, double3& b) {
  a.x = b.x; a.y = b.y; a.z = b.z;
}

inline dxHost dxDevice void make_zero(float3& a) {
  a.x = a.y = a.z = 0.0f;
}
inline dxHost dxDevice void make_zero(double3& a) {
  a.x = a.y = a.z = 0.0;
}
inline dxHost dxDevice void make_zero(float4& a) {
  a.x = a.y = a.z = a.w = 0.0f;
}
inline dxHost dxDevice void make_zero(double4& a) {
  a.x = a.y = a.z = a.w = 0.0;
}

#ifndef __CUDACC__
#include <math.h>

inline float fminf(float a, float b) throw()
{
  return a < b ? a : b;
}

inline float fmaxf(float a, float b) throw()
{
  return a < b ? a : b;
}

inline int max(int a, int b)
{
  return a > b ? a : b;
}

inline int min(int a, int b)
{
  return a < b ? a : b;
}

#else

#ifdef CUDA_ATOMICSUPPORT
template <>
dxDevice inline float readAndReplace<float>(float* buffer, const float& element) {
  return atomicExch(buffer, element);
}
#endif

#endif

// float functions
////////////////////////////////////////////////////////////////////////////////

// clamp
inline dxDevice dxHost float clamp(float f, float a, float b)
{
  return fmaxf(a, fminf(f, b));
}

// clamp
inline dxDevice dxHost double clamp(double f, double a, double b)
{
  return fmax(a, fmin(f, b));
}

// int2 functions
////////////////////////////////////////////////////////////////////////////////

// negate
inline dxHost dxDevice int2 operator-(int2 &a)
{
  return make_int2(-a.x, -a.y);
}

// addition
inline dxHost dxDevice int2 operator+(int2 a, int2 b)
{
  return make_int2(a.x + b.x, a.y + b.y);
}
inline dxHost dxDevice void operator+=(int2 &a, int2 b)
{
  a.x += b.x; a.y += b.y;
}

// subtract
inline dxHost dxDevice int2 operator-(int2 a, int2 b)
{
  return make_int2(a.x - b.x, a.y - b.y);
}
inline dxHost dxDevice void operator-=(int2 &a, int2 b)
{
  a.x -= b.x; a.y -= b.y;
}

// multiply
inline dxHost dxDevice int2 operator*(int2 a, int2 b)
{
  return make_int2(a.x * b.x, a.y * b.y);
}
inline dxHost dxDevice int2 operator*(int2 a, int s)
{
  return make_int2(a.x * s, a.y * s);
}
inline dxHost dxDevice int2 operator*(int s, int2 a)
{
  return make_int2(a.x * s, a.y * s);
}
inline dxHost dxDevice void operator*=(int2 &a, int s)
{
  a.x *= s; a.y *= s;
}

// float3 functions
////////////////////////////////////////////////////////////////////////////////

// additional constructors
inline dxHost dxDevice float3 make_float3(float s)
{
  return make_float3(s, s, s);
}
inline dxHost dxDevice float3 make_float3(float4 a)
{
  return make_float3(a.x, a.y, a.z);  // discards w
}
inline dxHost dxDevice float3 make_float3(int3 a)
{
  return make_float3(float(a.x), float(a.y), float(a.z));
}

inline dxHost dxDevice double3 make_double3(double s)
{
  return make_double3(s, s, s);
}

inline dxHost dxDevice double3 make_double3(double4 a)
{
  return make_double3(a.x, a.y, a.z);  // discards w
}
inline dxHost dxDevice double3 make_double3(int3 a)
{
  return make_double3(double(a.x), double(a.y), double(a.z));
}

// negate
inline dxHost dxDevice float3 operator-(float3 &a)
{
  return make_float3(-a.x, -a.y, -a.z);
}

// min
static __inline__ dxHost dxDevice float3 fminf(float3 a, float3 b)
{
  return make_float3(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z));
}

// max
static __inline__ dxHost dxDevice float3 fmaxf(float3 a, float3 b)
{
  return make_float3(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z));
}

// addition
inline dxHost dxDevice float3 operator+(float3 a, float3 b)
{
  return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline dxHost dxDevice double3 operator+(double3 a, double3 b)
{
  return make_double3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline dxHost dxDevice float3 operator+(float3 a, float b)
{
  return make_float3(a.x + b, a.y + b, a.z + b);
}
inline dxHost dxDevice double3 operator+(double3 a, double b)
{
  return make_double3(a.x + b, a.y + b, a.z + b);
}
inline dxHost dxDevice void operator+=(float3 &a, float3 b)
{
  a.x += b.x; a.y += b.y; a.z += b.z;
}
inline dxHost dxDevice void operator+=(double3 &a, double3 b)
{
  a.x += b.x; a.y += b.y; a.z += b.z;
}

// subtract
inline dxHost dxDevice float3 operator-(float3 a, float3 b)
{
  return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline dxHost dxDevice float3 operator-(float3 a, float b)
{
  return make_float3(a.x - b, a.y - b, a.z - b);
}
inline dxHost dxDevice void operator-=(float3 &a, float3 b)
{
  a.x -= b.x; a.y -= b.y; a.z -= b.z;
}

// multiply
inline dxHost dxDevice float3 operator*(float3 a, float3 b)
{
  return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline dxHost dxDevice float3 operator*(float3 a, float s)
{
  return make_float3(a.x * s, a.y * s, a.z * s);
}
inline dxHost dxDevice float3 operator*(float s, float3 a)
{
  return make_float3(a.x * s, a.y * s, a.z * s);
}
inline dxHost dxDevice void operator*=(float3 &a, float s)
{
  a.x *= s; a.y *= s; a.z *= s;
}
inline dxHost dxDevice void operator*=(double3 &a, double s)
{
  a.x *= s; a.y *= s; a.z *= s;
}

// divide
inline dxHost dxDevice float3 operator/(float3 a, float3 b)
{
  return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}
inline dxHost dxDevice float3 operator/(float3 a, float s)
{
  float inv = 1.0f / s;
  return a * inv;
}
inline dxHost dxDevice float3 operator/(float s, float3 a)
{
  float inv = 1.0f / s;
  return a * inv;
}
inline dxHost dxDevice void operator/=(float3 &a, float s)
{
  float inv = 1.0f / s;
  a *= inv;
}

// clamp
inline dxDevice dxHost float3 clamp(float3 v, float a, float b)
{
  return make_float3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}

inline dxDevice dxHost float3 clamp(float3 v, float3 a, float3 b)
{
  return make_float3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}

// dot product
inline dxHost dxDevice float dot(const float3& a, const float3& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline dxHost dxDevice double dot(const double3& a, const double3& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
// dot product
inline dxHost dxDevice float dot(const float3& a, const float4& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline dxHost dxDevice double dot(const double3& a, const double4& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
// dot product
inline dxHost dxDevice float dot(const float4& a, const float4& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline dxHost dxDevice double dot(const double4& a, const double4& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// cross product
inline dxHost dxDevice float3 cross(float3 a, float3 b)
{
  return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

// length
inline dxHost dxDevice float length(float3 v)
{
  return sqrtf(dot(v, v));
}

// normalize
inline dxHost dxDevice float3 normalize(float3 v)
{
  float invLen = 1.0f / sqrtf(dot(v, v));
  return v * invLen;
}

// floor
inline dxHost dxDevice float3 floor(const float3 v)
{
  return make_float3(floor(v.x), floor(v.y), floor(v.z));
}

// float4 functions
////////////////////////////////////////////////////////////////////////////////

// additional constructors
inline dxHost dxDevice float4 make_float4(float s)
{
  return make_float4(s, s, s, s);
}
inline dxHost dxDevice float4 make_float4(float3 a)
{
  return make_float4(a.x, a.y, a.z, 0.0f);
}
inline dxHost dxDevice float4 make_float4(float3 a, float w)
{
  return make_float4(a.x, a.y, a.z, w);
}
inline dxHost dxDevice float4 make_float4(const float& a, const float& b, const float& c)
{
  return make_float4((float)a, (float)b, (float)c);
}
inline dxHost dxDevice float4 make_float4(int4 a)
{
  return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

inline dxHost dxDevice double4 make_double4(double s)
{
  return make_double4(s, s, s, s);
}
inline dxHost dxDevice double4 make_double4(double3 a)
{
  return make_double4(a.x, a.y, a.z, 0.0f);
}
inline dxHost dxDevice double4 make_double4(double3 a, double w)
{
  return make_double4(a.x, a.y, a.z, w);
}
inline dxHost dxDevice double4 make_double4(const double& a, const double& b, const double& c)
{
  return make_double4((double)a, (double)b, (double)c);
}
inline dxHost dxDevice double4 make_double4(int4 a)
{
  return make_double4(double(a.x), double(a.y), double(a.z), double(a.w));
}
inline dxHost dxDevice double4 make_fdouble4(double s)
{
  double4 d;
  d.x = s;
  d.y = s;
  d.z = s;
  d.w = s;
  float* f;
  //f = reinterpret_cast<float4*>(&d);
  f = (float*)(&(d.x)); *f = (float)s;
  f = (float*)(&(d.y)); *f = (float)s;
  f = (float*)(&(d.z)); *f = (float)s;
  f = (float*)(&(d.w)); *f = (float)s;
  return d;
}


// negate
inline dxHost dxDevice float4 operator-(float4 &a)
{
  return make_float4(-a.x, -a.y, -a.z, -a.w);
}

// min
static __inline__ dxHost dxDevice float4 fminf(float4 a, float4 b)
{
  return make_float4(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z), fminf(a.w,b.w));
}

// max
static __inline__ dxHost dxDevice float4 fmaxf(float4 a, float4 b)
{
  return make_float4(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z), fmaxf(a.w,b.w));
}

// addition
inline dxHost dxDevice float4 operator+(float4 a, float4 b)
{
  return make_float4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline dxHost dxDevice double4 operator+(double4 a, double4 b)
{
  return make_double4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline dxHost dxDevice void operator+=(float4 &a, float4 b)
{
  a.x += b.x; a.y += b.y; a.z += b.z; a.w += b.w;
}
inline dxHost dxDevice void operator+=(double4 &a, double4 b)
{
  a.x += b.x; a.y += b.y; a.z += b.z; a.w += b.w;
}

// subtract
inline dxHost dxDevice float4 operator-(float4 a, float4 b)
{
  return make_float4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline dxHost dxDevice void operator-=(float4 &a, float4 b)
{
  a.x -= b.x; a.y -= b.y; a.z -= b.z; a.w -= b.w;
}

// multiply
template <typename T> inline dxHost dxDevice typename vec4<T>::Type operator*(typename vec4<T>::Type a, T s)
{
  return make_vec4(a.x * s, a.y * s, a.z * s, a.w * s);
}
inline dxHost dxDevice float4 operator*(float s, float4 a)
{
  return make_float4(a.x * s, a.y * s, a.z * s, a.w * s);
}
inline dxHost dxDevice void operator*=(float4 &a, float s)
{
  a.x *= s; a.y *= s; a.z *= s; a.w *= s;
}
inline dxHost dxDevice void operator*=(double4 &a, double s)
{
  a.x *= s; a.y *= s; a.z *= s; a.w *= s;
}

// divide
inline dxHost dxDevice float4 operator/(float4 a, float4 b)
{
  return make_float4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}
inline dxHost dxDevice float4 operator/(float4 a, float s)
{
  float inv = 1.0f / s;
  return a * inv;
}
inline dxHost dxDevice float4 operator/(float s, float4 a)
{
  float inv = 1.0f / s;
  return a * inv;
}
inline dxHost dxDevice void operator/=(float4 &a, float s)
{
  float inv = 1.0f / s;
  a *= inv;
}

// clamp
inline dxDevice dxHost float4 clamp(float4 v, float a, float b)
{
  return make_float4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}

inline dxDevice dxHost float4 clamp(float4 v, float4 a, float4 b)
{
  return make_float4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

// dot product
template <typename T> inline dxHost dxDevice T dot(typename vec4<T>::Type a, typename vec4<T>::Type b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// length
inline dxHost dxDevice float length(float4 r)
{
  return sqrtf(dot<float>(r, r));
}

// normalize
inline dxHost dxDevice float4 normalize(float4 v)
{
  float invLen = 1.0f / sqrtf(dot<float>(v, v));
  return v * invLen;
}

// floor
inline dxHost dxDevice float4 floor(const float4 v)
{
  return make_float4(floor(v.x), floor(v.y), floor(v.z), floor(v.w));
}

inline dxHost dxDevice vec3<float>::Type make_vec3(float a, float b, float c) {
  return make_float3(a,b,c);
}

inline dxHost dxDevice vec4<float>::Type make_vec4(const float& a, const float& b, const float& c) {
  return make_float4(a,b,c,(float)0.0);
}

inline dxHost dxDevice vec4<double>::Type make_vec4(const double& a, const double& b, const double& c) {
  return make_double4(a,b,c,(double)0.0);
}

inline dxHost dxDevice vec4<float>::Type make_vec4(float a, float b, float c, float d) {
  return make_float4(a,b,c,d);
}

inline dxHost dxDevice vec4<double>::Type make_vec4(double a, double b, double c, double d) {
  return make_double4(a,b,c,d);
}
inline dxHost dxDevice vec3<double>::Type make_vec3(double a, double b, double c) {
  return make_double3(a,b,c);
}

inline dxHost dxDevice vec4<float>::Type make_vec4( float3 a ) { return make_float4(a); }
inline dxHost dxDevice vec4<double>::Type make_vec4( double3 a ) { return make_double4(a); }

inline dxHost dxDevice vec4<float>::Type make_vec4( float a ) { return make_float4(a); }
inline dxHost dxDevice vec4<double>::Type make_vec4( double a ) { return make_double4(a); }

inline dxHost dxDevice vec3<float>::Type make_vec3( float4 a ) { return make_float3(a); }
inline dxHost dxDevice vec3<double>::Type make_vec3( double4 a ) { return make_double3(a); }

inline dxHost dxDevice vec3<float>::Type make_vec3( float a ) { return make_float3(a); }
inline dxHost dxDevice vec3<double>::Type make_vec3( double a ) { return make_double3(a); }


#endif
