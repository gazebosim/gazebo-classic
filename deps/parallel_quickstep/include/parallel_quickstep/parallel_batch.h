#ifndef PARALLEL_BATCH_H
#define PARALLEL_BATCH_H

#include "parallel_common.h"

namespace parallel_ode
{

namespace BatchTypes
{
  //! The type of batching approach
  enum BatchType { BATCH_RANDOM,
                   BATCH_GREEDY,
                   BATCH_COLORED,
                   DEFAULT_BATCH_TYPE = BATCH_RANDOM };
}
typedef BatchTypes::BatchType BatchType;

//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Base class for all batch strategy implementations. Defaults to random batch assignment.
 *
 */
class BatchStrategy
{
public:

  typedef IntVector BatchVector;

  /**
   * @brief Constructs on instance of the base strategy
   *
   * @param maxBatches The maximum number of batches to be supported
   */
  BatchStrategy( int maxBatches = ParallelOptions::MAXBATCHES )
      :  maxBatches_( maxBatches ),
         bAlign_( false ),
         alignment_( ParallelOptions::DEFAULTALIGN ) {

  }

  /**
   * @brief Destroys the strategy instance
   */
  virtual ~BatchStrategy( ) { }

  /**
   * @brief Batches the constraints
   *
   * @param pairList The original list of body pairs from ODE, per constraint
   * @param numConstraints The number of constraints
   * @param numBodies The number of bodies
   * @param constraintIndices The final permuted ordering of the constraints
   * @param bodyRepetitionCount0 The repetition count for each body-constraint for the first index of pairList
   * @param bodyRepetitionCount1 The repetition count for each body-constraint for the second index of pairList
   * @param maxBodyRepetitionCountInBatch The maximum # times a body is repeated in each batch
   * @param batchIndices The indices of each batch into the global array of constraints
   * @param batchSizes The number of constraints per batch
   *
   * @return
   */
  virtual int batch( const int* pairList, const int numConstraints, const int numBodies,
                     BatchVector& constraintIndices,
                     BatchVector& bodyRepetitionCount0,
                     BatchVector& bodyRepetitionCount1,
                     BatchVector& maxBodyRepetitionCountInBatch,
                     BatchVector& batchIndices,
                     BatchVector& batchSizes );

  inline int getMaxBatches() const { return maxBatches_; }
  inline void setMaxBatches( int maxBatches ) { maxBatches_ = maxBatches_; }

  inline bool isAligning() const { return bAlign_; }
  inline void setAlign( bool bAlign ) { bAlign_ = bAlign; }

  inline int getAlignment() const { return alignment_; }
  inline void setAlignment( int alignment ) { alignment_ = alignment; }

protected:

  /**
   * @brief Computes batch indices and body repetition counts
   *
   * @return The maximum number of times a body is repeated over all batches
   */
  int baseBatch( const int* pairList,
                 const BatchVector& constraintIndices,
                 const BatchVector& batchSizes,
                 BatchVector& batchIndices,
                 BatchVector& bodyRepetitionCount0,
                 BatchVector& bodyRepetitionCount1,
                 BatchVector& maxBodyRepetitionCountInBatch );

  /**
   * @brief Computes the body repetition counts
   *
   * @return The maximum number of times a body is repeated over all batches
   */
  int batchRepetitionCount( const int* pairList,
                            const BatchVector& constraintIndices,
                            const BatchVector& batchIndices,
                            BatchVector& bodyRepetitionCount0,
                            BatchVector& bodyRepetitionCount1,
                            BatchVector& maxBodyRepetitionCountInBatch );

  /**
   * @brief Computes batch indices given batch sizes
   *
   * @param batchSizes The size of each batch
   * @param batchIndices The computed indices for each batch
   * @param bAlign Whether to align the indices for each batch
   * @param alignment The alignment width if align is enabled
   */
  static void batchIndicesFromBatchSizes( const BatchVector& batchSizes, BatchVector& batchIndices, bool bAlign, int alignment );

private:

  int maxBatches_;                      /**< The maximum number of batches */
  bool bAlign_;                         /**< Whether to align the batch indices */
  int alignment_;                       /**< The alignment width for batch index alignment */
};

//////////////////////////////////////////////////////////////////////////////////////////////

//* A greedy batch strategy
/**
 *  Greedily assigns constraints to batches if a given constraint has not used a body involved in the constraint.
 */
class GreedyBatchStrategy : public BatchStrategy
{

public:

  /**
   * @brief Constructs on instance of the greedy strategy
   *
   * @param maxBatches The maximum number of batches
   */
  GreedyBatchStrategy( int maxBatches = ParallelOptions::MAXBATCHES ) : BatchStrategy( maxBatches ) { }

  /**
   * @brief Greedily assigns constraints to batches.  If the maximum number of batches is reached and all constraints
   * have not been assigned, the process resets the "used body" indicator and continues assignment
   */
  virtual int batch( const int* pairList, const int numPairs, const int numBodies,
                     BatchVector& bodyRepetitionCount0,
                     BatchVector& bodyRepetitionCount1,
                     BatchVector& maxBodyRepetitionCountInBatch,
                     BatchVector& constraintIndices,
                     BatchVector& batchIndices,
                     BatchVector& batchSizes );

};

//////////////////////////////////////////////////////////////////////////////////////////////

//* A coloring batch strategy
/**
 *  An approximate approach to coloring edges for constraint batch assignment.
 */
class ColoringBatchStrategy : public BatchStrategy
{

public:

  /**
   * @brief Creates an instance of the coloring strategy
   *
   * @param maxBatches The maximum number of batches
   */
  ColoringBatchStrategy( int maxBatches =  ParallelOptions::MAXBATCHES ) : BatchStrategy( maxBatches ) { }

  /**
   * @brief A rough attempt at an approximate edge coloring of the constraints.  Very slow
   */
  virtual int batch( const int* pairList, const int numPairs, const int numBodies,
                     BatchVector& bodyRepetitionCount0,
                     BatchVector& bodyRepetitionCount1,
                     BatchVector& maxBodyRepetitionCountInBatch,
                     BatchVector& constraintIndices,
                     BatchVector& batchIndices,
                     BatchVector& batchSizes );

};

//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Handles the creation of batch strategies according to a desired # of batches and alignment
 */
class BatchStrategyFactory
{
public:

  /**
   * @brief Creates a batch straegy according to the given arguments
   *
   * @param batchType The type of batch strategy
   * @param numBatches The maximum number of batches
   * @param bAlign Whether to align the batch index assignment
   * @param alignment The alignment width
   *
   * @return A pointer to the batch strategy
   */
  static BatchStrategy* create( BatchType batchType, int numBatches, bool bAlign = false, int alignment = ParallelOptions::DEFAULTALIGN )
  {
    BatchStrategy* newStrategy = NULL;
    switch(batchType)
    {
      case BatchTypes::BATCH_RANDOM:
        newStrategy = new BatchStrategy( numBatches );
        break;
      case BatchTypes::BATCH_GREEDY:
        newStrategy = new GreedyBatchStrategy( numBatches );
        break;
      case BatchTypes::BATCH_COLORED:
      default:
        newStrategy = new ColoringBatchStrategy( numBatches );
        break;
    }
    newStrategy->setAlign( bAlign );
    newStrategy->setAlignment( alignment );
    return newStrategy;
  }
};

}

#endif
