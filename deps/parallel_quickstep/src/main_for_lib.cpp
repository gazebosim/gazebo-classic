#include <parallel_common.h>

template <typename T, ParallelType PType> int parallelReduceTest();

int main( int argc, char **argv )
{
#ifdef CUDA_DOUBLESUPPORT
  return parallelReduceTest<dReal,ParallelTypes::PARALLEL_TYPE>();
#else
  return parallelReduceTest<float,ParallelTypes::PARALLEL_TYPE>();
#endif
};
