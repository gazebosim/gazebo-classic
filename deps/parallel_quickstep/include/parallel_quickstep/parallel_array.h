#ifndef PARALLEL_ARRAY_H
#define PARALLEL_ARRAY_H

#include <parallel_common.h>
#include <parallel_memory.h>

#include <stdio.h>
#include <cstring>

namespace parallel_utils
{

namespace ArrayTypes
{
  //! Types of parallel arrays
  enum ArrayType { HOST_ARRAY, DEVICE_ARRAY };
}
typedef ArrayTypes::ArrayType ArrayType;


#define BUFFER_MULTIPLIER 1.5              /**< Specifies the growth factor for the buffer */
#define BUFFER_MAX_THRESHOLD_MULTIPLIER .9 /**< Specifies the growth increase threshold for the buffer */
#define BUFFER_MIN_THRESHOLD_MULTIPLIER .2 /**< Specifies the growth decrease threshold for the buffer */

//! Encapsulates allocation/deallocation and access to arrays on both the host and device
template <typename Type, typename RType, ParallelType PType>
class ParallelArray
{
  typedef MemManager<Type,PType> PMemManager;
  typedef typename PMemManager::mem_flags MemFlags;

public:

  /**
   * @brief Constructs on instance of a parallely array according the arguments
   *
   * @param type The type of array, either host or device
   * @param size The size (in elements, not bytes) of the the array
   * @param flags The flags to be used by the memory manager
   */
  ParallelArray( ArrayType type = ArrayTypes::DEVICE_ARRAY, ArraySize size = 0, MemFlags flags = 0 )
      : type_(type), size_(size), internalSize_(0), flags_(flags), buffer_(NULL)
  {
    realloc( );
  }

  /**
   * Destroys the array, freeing memory appropriately
   */  virtual ~ParallelArray()
  {
    dealloc( );
  }

  /**
   * Sets the size of the array, reallocating memory appropriately.  No guarantees on data preservation
   *
   * @param size The number of elements fot the new array
   */
  void setSize( ArraySize size )
  {
    if( size_ != size )
    {
      size_ = size;
      realloc();
    }
  }

  /**
   * Sets flags governing memory allocation and memory transfer specific for the parallel type
   *
   * @param flags The flags to be used by the memory manager
   */
  void setFlags( MemFlags flags )
  {
    if( flags_ != flags )
    {
      flags_ = flags;
      realloc();
    }
  }

  /**
   * Prints the array to the console. Utility only.
   *
   * @param s The string prefix
   * @param stride The stride between elements in the buffer to print
   */
  void print( const char* s, int stride = 1 ) const
  {
    printf("\n %s { ", s);
    for(ArraySize i = 0; i < getSize(); i += stride)
      printVal(buffer_[i]);
    printf("}\n");
  }

  inline void printVal(const int &a) const{ printf("%d ", a); }
  inline void printVal(const int4 &a) const { printf("(%d,%d,%d,%d) ", a.x, a.y, a.z, a.w); }
  inline void printVal(const float &a) const{ printf("%f ", a); }
  inline void printVal(const float3 &a) const { printf("(%f,%f,%f) ", a.x, a.y, a.z); }
  inline void printVal(const float4 &a) const { printf("(%f,%f,%f,%f) ", a.x, a.y, a.z, a.w); }
  inline void printVal(const double &a) const { printf("%f ", a); }
  inline void printVal(const double3 &a) const { printf("(%f,%f,%f) ", a.x, a.y, a.z); }
  inline void printVal(const double4 &a) const { printf("(%f,%f,%f,%f) ", a.x, a.y, a.z, a.w); }

  inline bool isHostArray( ) const { return type_ == ArrayTypes::HOST_ARRAY; }
  inline bool isDeviceArray( ) const { return type_ == ArrayTypes::DEVICE_ARRAY; }
  inline ArrayType getType( ) const { return type_; }
  inline ArraySize getByteSize( ) const { return getByteSize( size_ ); }
  inline ArraySize getByteSize( ArraySize arraySize ) const { return arraySize * sizeof(Type); }
  inline ArraySize getSize( ) const { return size_; }
  inline ArraySize getInternalSize( ) const { return internalSize_; }
  inline ArraySize getInternalByteSize( ) const { return getByteSize( internalSize_ ); }
  inline MemFlags getFlags( ) const { return flags_; }
  inline RType getBuffer( ) { return buffer_; }
  inline RType getBuffer( ArraySize offset ) { return offsetBuffer<RType>(buffer_, offset); }

protected:

  virtual void realloc( )
  {
    if( size_ < internalSize_ * BUFFER_MIN_THRESHOLD_MULTIPLIER ||
        size_ > internalSize_ * BUFFER_MAX_THRESHOLD_MULTIPLIER ) {
      dealloc();
      internalSize_ = size_ * BUFFER_MULTIPLIER;
      if( isHostArray() ) PMemManager::allocateHost((void**)&this->buffer_, getInternalByteSize(), flags_ );
      else PMemManager::allocateDevice( (typename PMemManager::device_init_type)&this->buffer_, getInternalByteSize() );
    }
  }

  virtual void dealloc( )
  {
    if( internalSize_ == 0 ) return;
    if( isHostArray() ) PMemManager::freeHost( this->buffer_ );
    else PMemManager::freeDevice( this->buffer_ );
    this->buffer_ = NULL;
  }

private:

  ArrayType type_;                      /**< The type of the array, eithe host or device */
  ArraySize size_;                      /**< The number of elements in the array */
  ArraySize internalSize_;              /**< The internal size of the array, may be more than the reported size */
  MemFlags flags_;                      /**< Flags governing array processing by the memory manager */

  RType buffer_;                        /**< Handle to the actual memory */

};

//! Wraps both a host and device ParallArray and exposes methods for synchronizing between the two
template <class Type, ParallelType PType>
class ParallelHDArray
{
  typedef MemManager<Type,PType> PMemManager;
  typedef typename PMemManager::device_type DType;
  typedef typename PMemManager::host_type HType;
  typedef typename PMemManager::stream_type SType;
  typedef typename PMemManager::mem_flags MemFlags;

public:

  /**
   * Constructs an instance of the class, initializing both host and device arrays
   *
   * @param size The number of elements
   * @param flags Flags governing memory operations
   * @param stream
   */
  ParallelHDArray( ArraySize size = 0, MemFlags flags = 0, SType stream = 0)
      : stream_(stream), hostArray_( ArrayTypes::HOST_ARRAY, size, flags ), deviceArray_( ArrayTypes::DEVICE_ARRAY, size, flags )
  {
  }

  /**
   * Destroys the object, freeing both host and device arrays
   */
  virtual ~ParallelHDArray()
  {
  }

  /**
   * Copies data from the device to the host
   *
   * @param offset The offset into the array from which to copy
   * @param length The number of elements to copy
   * @param copyType The type of copy, synchronous or asynchronous
   */
  inline void syncToHost( ArraySize offset, ArraySize length, CopyType copyType = CopyTypes::COPY_DEFAULT )
  {
    if( getSize( ) <= 0 ) return;
    PMemManager::copyToHost( getHostBuffer( offset ), getDeviceBuffer( offset ), getByteSize( length ), copyType, stream_);
  }

  /**
   * Copies data form the host to the device
   *
   * @param offset The offset into the array from which to copy
   * @param length The number of elements to copy
   * @param copyType The type of copy, synchronous or asynchronous
   */
  inline void syncToDevice( ArraySize offset, ArraySize length, CopyType copyType  = CopyTypes::COPY_DEFAULT )
  {
    if( getSize( ) <= 0 ) return;
    PMemManager::copyToDevice( getDeviceBuffer( offset ), getHostBuffer( offset ), getByteSize( length ), copyType, stream_);
  }

  /**
   * Copies size number of elements from the device to the host
   *
   * @param copyType The type of copy, synchronous or asynchronous
   */
  inline void syncToHost( CopyType copyType = CopyTypes::COPY_DEFAULT ) { syncToHost(0, getSize(), copyType ); }

  /**
   * Copies size number of elements from the host to the device
   *
   * @param copyType The type of copy, synchronous or asynchronous
   */
  inline void syncToDevice( CopyType copyType = CopyTypes::COPY_DEFAULT ) { syncToDevice(0, getSize(), copyType ); }

  /**
   * Sets the size of the host and device arrays
   *
   * @param size The size of the arrays
   */
  void setSize( ArraySize size )
  {
    hostArray_.setSize( size );
    deviceArray_.setSize( size );
  }

  /**
   * Sets the flags of the host and device arrays
   *
   * @param flags The flags to set
   */
  void setFlags( MemFlags flags )
  {
    hostArray_.setFlags( flags );
    deviceArray_.setFlags( flags );
  }

  /**
   * Prints the contents of the host array
   *
   * @param s The prefix
   * @param stride The stride between elements
   */
  void print( const char * s, int stride = 1 ) const  { hostArray_.print( s, stride ); }

  inline ArraySize getByteSize( ) const { return hostArray_.getByteSize(); }
  inline ArraySize getByteSize( ArraySize arraySize ) const { return hostArray_.getByteSize( arraySize ); }
  inline ArraySize getSize( ) const { return hostArray_.getSize(); }
  inline MemFlags getFlags( ) const { return hostArray_.getFlags(); }
  inline HType getHostBuffer( ) { return getHostBuffer( 0 ); }
  inline DType getDeviceBuffer( ) { return getDeviceBuffer( 0 ); }
  inline HType getHostBuffer( ArraySize offset ) { return hostArray_.getBuffer( offset ); }
  inline DType getDeviceBuffer( ArraySize offset ) { return deviceArray_.getBuffer( offset ); }

protected:

private:

  SType stream_;                                /**< The type of stream, specific to the underlying architecture */

  ParallelArray<Type,HType,PType> hostArray_;   /**< Handle toe the host side ParallelArray */
  ParallelArray<Type,DType,PType> deviceArray_; /**< Handle to the device side ParallelArray */

};

}

#endif
