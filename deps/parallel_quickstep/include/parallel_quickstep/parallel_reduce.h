#ifndef PARALLEL_REDUCE_H
#define PARALLEL_REDUCE_H

#include "parallel_common.h"

namespace parallel_ode
{

namespace ReduceTypes
{
  //! The type of reduction approach
  enum ReduceType { REDUCE_NONE,
                    REDUCE_SEQUENTIAL,
                    REDUCE_STRIDED,
                    REDUCE_COMPACT,
                    DEFAULT_REDUCE_TYPE = REDUCE_STRIDED };
}
typedef ReduceTypes::ReduceType ReduceType;

//! The base strategy class
/**
 *  This class is little more than a wrapper that assists the reduction kernel in determing the stride
 * and offsets between body reduction entries in the reduction buffer.
 */
class ReduceStrategy
{
public:

  /**
   * @brief Constructs an instance of the base strategy
   */
  ReduceStrategy( )
      : bodyAlignment_( ParallelOptions::DEFAULTALIGN ),
      bodySize_( 0 ),
      bodySizeWithReduction_(0),
      bodyStride_(0),
      bodyOffsetStride_(0),
      bClearReduceBuffers_(true) { }

  /**
   * @brief Destroys the strategy
   */
  virtual ~ReduceStrategy( ) { }

  /**
   * @brief Virtual function that initializes the strategy.  Essentially, this is where the strides/sizes are deteremined.
   *
   * @param bodySize The number of bodies
   * @param maxBodyRepetitionCount The maximum number of times a body is repeated for all batches
   * @param batchRepetitionCount The maximum number of times a body is repeated in each batch
   */
  virtual void initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount );

  /**
   * @brief Virtual function indicating the type of strategy
   *
   * @return The type of strategy
   */
  virtual ReduceType getType( ) const { return ReduceTypes::REDUCE_NONE; }

  /**
   * @brief Sets the alignment for the number of bodies, e.g., 16/32/64
   *
   * @param bodyAlignment The body alignment
   */
  inline void setBodyAlignment( int bodyAlignment ) { bodyAlignment_ = bodyAlignment; }

  /**
   * @brief Sets whether the reduction kernel should clear the reduction buffer after the reduce operation
   *
   * @param clearBuffers Boolean indicating whether to clear or not
   */
  inline void setClearReduceBuffers( bool clearBuffers ) { bClearReduceBuffers_ = clearBuffers; }

  /**
   * @brief Computes the final index for a given body index into the body reduction buffer
   *
   * @param baseBodyIndex The base index for the given body
   * @param bodyOffset The repetition number of this particular body within the batch
   *
   * @return
   */
  inline int getFinalIndex( int baseBodyIndex, int bodyOffset ) { return baseBodyIndex + (bodyOffset * bodyOffsetStride_); }

  inline int getBodyAlignment( ) const { return bodyAlignment_; }
  inline int getBodySize( ) const { return bodySize_; }
  inline int getBodySizeWithReduction( ) const { return bodySizeWithReduction_; }
  inline int getBodyStride( ) const { return bodyStride_; }
  inline int getBodyOffsetStride( ) const { return bodyOffsetStride_; }

  inline bool clearReduceBuffers( ) { return bClearReduceBuffers_; }

  inline void print( ) {
    printf("BSize,BSizeReduce,BStride,BOffsetStride = %d, %d, %d, %d\n", bodySize_, bodySizeWithReduction_, bodyStride_, bodyOffsetStride_);
  }

protected:
  inline void setBodySize( int bodySize ) { bodySize_ = bodySize; }
  inline void setBodySizeWithReduction( int bodySize ) { bodySizeWithReduction_ = bodySize; }
  inline void setBodyStride( int bodyStride ) { bodyStride_ = bodyStride; }
  inline void setBodyOffsetStride( int bodyStride ) { bodyOffsetStride_ = bodyStride; }

private:

  int bodyAlignment_;                   /**< # of bodies per memory aligned stride ( 128 bytes typically ) */
  int bodySize_;                        /**< Total # of bodies */
  int bodySizeWithReduction_;           /**< Total # of body reduction entries */
  int bodyStride_;                      /**< Stride between successive bodies in the reduction buffer */
  int bodyOffsetStride_;                /**< Stride between entires for the same body in the reduction buffer */

  bool bClearReduceBuffers_;            /**< Indicates whether the reduction kernel should zero out the reduction buffers */

};

//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Sequential reduction strategy.
 *
 * Repeated body entries within a batch are ordered sequentialy, e.g. AAAAAAAA**BBBBBBBBB**CCCCCCCC
 *   where * indicates padding for alignment
 */
class SequentialReduceStrategy : public ReduceStrategy
{
public:

  SequentialReduceStrategy( ) : ReduceStrategy( ) { }
  virtual ~SequentialReduceStrategy( ) { }

  virtual void initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount );
  virtual ReduceType getType( ) const { return ReduceTypes::REDUCE_SEQUENTIAL; }
};

//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Strided reduction strategy.
 *
 * Repeated body entries within a batch are strided, e.g. ABC**ABC**ABC**ABC**ABC**ABC
 *    where * indicates padding for alignment
 */
class StridedReduceStrategy : public ReduceStrategy
{

public:

  StridedReduceStrategy( ) : ReduceStrategy( ) { }
  virtual ~StridedReduceStrategy( ) { }

  virtual void initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount );
  virtual ReduceType getType( ) const { return ReduceTypes::REDUCE_STRIDED; }
};

//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Compact reduction strategy.
 *
 * Repeated body entries within a batch are strided, e.g. AAAAAAAABBBBBBBBBCCCCCCCC
 *    and there is no padding between different bodies
 */
class CompactReduceStrategy : public ReduceStrategy
{

public:

  CompactReduceStrategy( ) : ReduceStrategy( ) { }
  virtual ~CompactReduceStrategy( ) { }

  virtual void initialize( int bodySize, int maxBodyRepetitionCount, const IntVector& batchRepetitionCount );
  virtual ReduceType getType( ) const { return ReduceTypes::REDUCE_COMPACT; }
};

//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Handles the creation of reduction strategies according to a desired type and alignment
 */
class ReduceStrategyFactory
{
public:
  /**
   * @brief Creates a reduction strategy according the given arguments
   *
   * @param reduceType The type of reduction
   * @param bClearBuffers Whether to clear the reduction buffers after each reduce operation
   * @param bodyAlignment The number of bodies per aligned memory stride
   *
   * @return A pointer to the reduction strategy
   */
  static ReduceStrategy* create( ReduceType reduceType, bool bClearBuffers = true, int bodyAlignment = ParallelOptions::DEFAULTALIGN )
  {
    ReduceStrategy* newStrategy = NULL;

    switch( reduceType )
    {
      case ReduceTypes::REDUCE_SEQUENTIAL:
        newStrategy = new SequentialReduceStrategy( );
        break;
      case ReduceTypes::REDUCE_STRIDED:
        newStrategy = new StridedReduceStrategy( );
        break;
      case ReduceTypes::REDUCE_COMPACT:
        newStrategy = new CompactReduceStrategy( );
        break;
      case ReduceTypes::REDUCE_NONE:
      default:
        newStrategy = new ReduceStrategy( );
        break;
    }

    if( newStrategy ) {
      newStrategy->setBodyAlignment( bodyAlignment );
      newStrategy->setClearReduceBuffers( bClearBuffers );
    }

    return newStrategy;
  }
};

}

#endif
