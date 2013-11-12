#if !defined(__VECTOR_TYPES_H__)
#define __VECTOR_TYPES_H__

#ifndef __CUDACC__
    #define __builtin_align__(a)
    #define __inline__ inline
    #define __host__
    #define __device__
#else
    #define __builtin_align__(a) __align__(a)
    #define __inline__ inline
#endif

struct char1{
    signed char x;
};

struct uchar1{
    unsigned char x;
};

struct __builtin_align__(2) char2{
    signed char x, y;
};

struct __builtin_align__(2) uchar2{
    unsigned char x, y;
};

struct char3{
    signed char x, y, z;
};

struct uchar3{
    unsigned char x, y, z;
};

struct __builtin_align__(4) char4{
    signed char x, y, z, w;
};

struct __builtin_align__(4) uchar4{
    unsigned char x, y, z, w;
};

struct short1{
    short x;
};

struct ushort1{
    unsigned short x;
};

struct __builtin_align__(4) short2{
    short x, y;
};

struct __builtin_align__(4) ushort2{
    unsigned short x, y;
};

struct short3{
    short x, y, z;
};

struct ushort3{
    unsigned short x, y, z;
};

struct __builtin_align__(8) short4{
    short x, y, z, w;
};

struct __builtin_align__(8) ushort4{
    unsigned short x, y, z, w;
};

struct int1{
    int x;
};

struct uint1{
    unsigned int x;
};

struct __builtin_align__(8) int2{
    int x, y;
};

struct __builtin_align__(8) uint2{
    unsigned int x, y;
};

struct int3{
    int x, y, z;
};

struct uint3{
    unsigned int x, y, z;
};

struct __builtin_align__(16) int4{
    int x, y, z, w;
};

struct __builtin_align__(16) uint4{
    unsigned int x, y, z, w;
};

struct long1{
    long int x;
};

struct ulong1{
    unsigned long x;
};


struct
    #ifdef _WIN32
        __builtin_align__(8)
    #else /* _WIN32 */
        __builtin_align__(2*sizeof(long int))
    #endif /* _WIN32 */

long2{
    long int x, y;
};


struct
    #ifdef _WIN32
       __builtin_align__(8)
    #else /* _WIN32 */
       __builtin_align__(2*sizeof(unsigned long int))
    #endif /* _WIN32 */

ulong2{
    unsigned long int x, y;
};

#ifndef __LP64__

struct long3{
    long int x, y, z;
};

struct ulong3{
    unsigned long int x, y, z;
};

struct __builtin_align__(16) long4{
    long int x, y, z, w;
};

struct __builtin_align__(16) ulong4{
    unsigned long int x, y, z, w;
};

#endif /* !__LP64__ */


struct float1{
    float x;
};

struct __builtin_align__(8) float2{
    float x, y;
};

struct float3{
    float x, y, z;
};

struct __builtin_align__(16) float4{
    float x, y, z, w;
};

struct longlong1{
    long long int x;
};

struct ulonglong1{
    unsigned long long int x;
};

struct __builtin_align__(16) longlong2{
    long long int x, y;
};

struct __builtin_align__(16) ulonglong2{
    unsigned long long int x, y;
};

struct double1{
    double x;
};

struct __builtin_align__(16) double2{
    double x, y;
};

struct double3{
    double x, y, z;
};

struct __builtin_align__(32) double4{
    double x, y, z, w;
};

/*******************************************************************************
*                                                                              *
*                                                                              *
*                                                                              *
*******************************************************************************/


typedef struct char1 char1;

typedef struct uchar1 uchar1;

typedef struct char2 char2;

typedef struct uchar2 uchar2;

typedef struct char3 char3;

typedef struct uchar3 uchar3;

typedef struct char4 char4;

typedef struct uchar4 uchar4;

typedef struct short1 short1;

typedef struct ushort1 ushort1;

typedef struct short2 short2;

typedef struct ushort2 ushort2;

typedef struct short3 short3;

typedef struct ushort3 ushort3;

typedef struct short4 short4;

typedef struct ushort4 ushort4;

typedef struct int1 int1;

typedef struct uint1 uint1;

typedef struct int2 int2;

typedef struct uint2 uint2;

typedef struct int3 int3;

typedef struct uint3 uint3;

typedef struct int4 int4;

typedef struct uint4 uint4;

typedef struct long1 long1;

typedef struct ulong1 ulong1;

typedef struct long2 long2;

typedef struct ulong2 ulong2;

typedef struct long3 long3;

typedef struct ulong3 ulong3;

typedef struct long4 long4;

typedef struct ulong4 ulong4;

typedef struct float1 float1;

typedef struct float2 float2;

typedef struct float3 float3;

typedef struct float4 float4;

typedef struct longlong1 longlong1;

typedef struct ulonglong1 ulonglong1;

typedef struct longlong2 longlong2;

typedef struct ulonglong2 ulonglong2;

typedef struct double1 double1;

typedef struct double2 double2;

typedef struct double3 double3;

typedef struct double4 double4;

/*******************************************************************************
*                                                                              *
*                                                                              *
*                                                                              *
*******************************************************************************/


typedef struct dim3 dim3;


struct dim3
{
  unsigned int x, y, z;
#ifdef __cplusplus
 dim3(unsigned int x = 1, unsigned int y = 1, unsigned int z = 1) : x(x), y(y), z(z) {}
 dim3(uint3 v) : x(v.x), y(v.y), z(v.z) {}
  operator uint3(void) { uint3 t; t.x = x; t.y = y; t.z = z; return t; }
#endif
};




static __inline__ __host__ __device__ char1 make_char1(signed char x){
  char1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ uchar1 make_uchar1(unsigned char x){
  uchar1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ char2 make_char2(signed char x, signed char y){
  char2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ uchar2 make_uchar2(unsigned char x, unsigned char y){
  uchar2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ char3 make_char3(signed char x, signed char y, signed char z){
  char3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ uchar3 make_uchar3(unsigned char x, unsigned char y, unsigned char z){
  uchar3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ char4 make_char4(signed char x, signed char y, signed char z, signed char w){
  char4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ uchar4 make_uchar4(unsigned char x, unsigned char y, unsigned char z, unsigned char w){
  uchar4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ short1 make_short1(short x){
  short1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ ushort1 make_ushort1(unsigned short x){
  ushort1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ short2 make_short2(short x, short y){
  short2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ ushort2 make_ushort2(unsigned short x, unsigned short y){
  ushort2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ short3 make_short3(short x,short y, short z){
  short3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ ushort3 make_ushort3(unsigned short x, unsigned short y, unsigned short z){
  ushort3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ short4 make_short4(short x, short y, short z, short w){
  short4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ ushort4 make_ushort4(unsigned short x, unsigned short y, unsigned short z, unsigned short w){
  ushort4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ int1 make_int1(int x){
  int1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ uint1 make_uint1(unsigned int x){
  uint1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ int2 make_int2(int x, int y){
  int2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ uint2 make_uint2(unsigned int x, unsigned int y){
  uint2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ int3 make_int3(int x, int y, int z){
  int3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ uint3 make_uint3(unsigned int x, unsigned int y, unsigned int z){
  uint3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ int4 make_int4(int x, int y, int z, int w){
  int4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ uint4 make_uint4(unsigned int x, unsigned int y, unsigned int z, unsigned int w){
  uint4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ long1 make_long1(long int x){
  long1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ ulong1 make_ulong1(unsigned long int x){
  ulong1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ long2 make_long2(long int x, long int y){
  long2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ ulong2 make_ulong2(unsigned long int x, unsigned long int y){
  ulong2 t; t.x = x; t.y = y; return t;
}

#ifndef __LP64__

static __inline__ __host__ __device__ long3 make_long3(long int x, long int y, long int z){
  long3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ ulong3 make_ulong3(unsigned long int x, unsigned long int y, unsigned long int z){
  ulong3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ long4 make_long4(long int x, long int y, long int z, long int w){
  long4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ ulong4 make_ulong4(unsigned long int x, unsigned long int y, unsigned long int z, unsigned long int w){
  ulong4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

#endif /* !__LP64__ */

static __inline__ __host__ __device__ float1 make_float1(float x){
  float1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ float2 make_float2(float x, float y){
  float2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ float3 make_float3(float x, float y, float z){
  float3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ float4 make_float4(float x, float y, float z, float w){
  float4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static __inline__ __host__ __device__ longlong1 make_longlong1(long long int x){
  longlong1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ ulonglong1 make_ulonglong1(unsigned long long int x){
  ulonglong1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ longlong2 make_longlong2(long long int x, long long int y){
  longlong2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ ulonglong2 make_ulonglong2(unsigned long long int x, unsigned long long int y){
  ulonglong2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ double1 make_double1(double x){
  double1 t; t.x = x; return t;
}

static __inline__ __host__ __device__ double2 make_double2(double x, double y){
  double2 t; t.x = x; t.y = y; return t;
}

static __inline__ __host__ __device__ double3 make_double3(double x, double y, double z){
  double3 t; t.x = x; t.y = y; t.z = z; return t;
}

static __inline__ __host__ __device__ double4 make_double4(double x, double y, double z, double w){
  double4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

#endif /* !__VECTOR_TYPES_H__ */
